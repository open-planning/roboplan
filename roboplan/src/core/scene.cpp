#include <stdexcept>

#include <tl/expected.hpp>

#include <pinocchio/collision/broadphase.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <yaml-cpp/yaml.h>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_utils.hpp>

namespace {

/// @brief Tolerance for the norm of continuous joint values (in the form cos(theta), sin(theta))
/// to be on the unit circle.
constexpr double kUnitCircleTol = 1.0e-6;

}  // namespace

namespace roboplan {

std::string readFile(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("File not found: " + path.string());
  }
  auto size = std::filesystem::file_size(path);
  std::string content(size, '\0');
  std::ifstream in(path, std::ios::binary);
  in.read(&content[0], size);
  return content;
}

Scene::Scene(const std::string& name, const std::filesystem::path& urdf_path,
             const std::filesystem::path& srdf_path,
             const std::vector<std::filesystem::path>& package_paths,
             const std::filesystem::path& yaml_config_path)
    : Scene(name, readFile(urdf_path), readFile(srdf_path), package_paths, yaml_config_path) {}

Scene::Scene(const std::string& name, const std::string& urdf, const std::string& srdf,
             const std::vector<std::filesystem::path>& package_paths,
             const std::filesystem::path& yaml_config_path)
    : name_{name} {
  // Convert the vector of package paths to string to be compatible with
  // Pinocchio.
  std::vector<std::string> package_paths_str;
  package_paths_str.reserve(package_paths.size());
  for (const auto& path : package_paths) {
    package_paths_str.push_back(std::string(path));
  }

  // Start by creating a Pinocchio model with mimic support to parse all the relationships.
  pinocchio::Model mimic_model;
  pinocchio::urdf::buildModelFromXML(urdf, mimic_model, /*verbose*/ false, /*mimic*/ true);

  YAML::Node yaml_config;
  if (!yaml_config_path.empty() && !std::filesystem::is_directory(yaml_config_path)) {
    yaml_config = YAML::LoadFile(yaml_config_path);
  }

  // Initialize the RNG to be pseudorandom. You can use setRngSeed() to fix this.
  std::random_device rd;
  rng_gen_ = std::mt19937(rd());

  // Create additional robot information.
  size_t q_idx = 0;
  size_t v_idx = 0;
  joint_names_.reserve(mimic_model.njoints - 1);
  actuated_joint_names_.reserve(mimic_model.njoints - mimic_model.mimicking_joints.size() - 1);
  for (int idx = 1; idx < mimic_model.njoints; ++idx) {  // omits "universe" joint.
    const auto& joint_name = mimic_model.names.at(idx);
    joint_names_.push_back(joint_name);

    const auto& joint = mimic_model.joints.at(idx);
    if (joint.shortname() == "JointModelMimic") {
      // If the joint is a mimic joint, do nothing for now.
      // The information will be extracted later.
      continue;
    }
    actuated_joint_names_.push_back(joint_name);

    if (!kPinocchioJointTypeMap.contains(joint.shortname())) {
      throw std::runtime_error("Joint '" + joint_name + "' was parsed as a joint of type '" +
                               joint.shortname() + "' but this is not in the RoboPlan joint map.");
    }
    auto info = JointInfo(kPinocchioJointTypeMap.at(joint.shortname()));
    for (int idx = 0; idx < joint.nq(); ++idx) {
      info.limits.min_position[idx] = mimic_model.lowerPositionLimit(q_idx);
      info.limits.max_position[idx] = mimic_model.upperPositionLimit(q_idx);
      ++q_idx;
    }

    std::optional<YAML::Node> maybe_acc_limits;
    std::optional<YAML::Node> maybe_jerk_limits;
    if (yaml_config["joint_limits"] && yaml_config["joint_limits"][joint_name]) {
      const auto& limits_config = yaml_config["joint_limits"][joint_name];
      if (limits_config["max_acceleration"]) {
        maybe_acc_limits = limits_config["max_acceleration"];
        if (!maybe_acc_limits->IsSequence() ||
            (maybe_acc_limits->size() != static_cast<size_t>(joint.nv()))) {
          throw std::runtime_error("Acceleration limits for joint '" + joint_name +
                                   "' must be a sequence of size " + std::to_string(joint.nv()) +
                                   ".");
        }
      }
      if (limits_config["max_jerk"]) {
        maybe_jerk_limits = limits_config["max_jerk"];
        if (!maybe_jerk_limits->IsSequence() ||
            (maybe_jerk_limits->size() != static_cast<size_t>(joint.nv()))) {
          throw std::runtime_error("Jerk limits for joint '" + joint_name +
                                   "' must be a sequence of size " + std::to_string(joint.nv()) +
                                   ".");
        }
      }
    }
    for (int idx = 0; idx < joint.nv(); ++idx) {
      info.limits.max_velocity[idx] = mimic_model.velocityLimit(v_idx);
      if (maybe_acc_limits) {
        info.limits.max_acceleration[idx] = maybe_acc_limits.value()[idx].as<double>();
      }
      if (maybe_jerk_limits) {
        info.limits.max_jerk[idx] = maybe_jerk_limits.value()[idx].as<double>();
      }
      ++v_idx;
    }
    joint_info_map_.emplace(joint_name, info);
  }

  // Add the mimic joint information once all the other joints have been parsed.
  const auto num_mimics = mimic_model.mimicked_joints.size();
  for (size_t idx = 0; idx < num_mimics; ++idx) {
    const auto mimicking_idx = mimic_model.mimicking_joints[idx];
    const auto& mimicking_joint_name = mimic_model.names[mimicking_idx];
    const auto& mimicking_joint = mimic_model.joints[mimicking_idx];

    const auto mimicked_idx = mimic_model.mimicked_joints[idx];
    const auto& mimicked_joint_name = mimic_model.names[mimicked_idx];

    auto* mimic_joint = boost::get<pinocchio::JointModelMimic>(&mimicking_joint);
    const auto mimicked_joint_type = joint_info_map_.at(mimicked_joint_name).type;
    auto info = JointInfo(mimicked_joint_type);
    info.mimic_info = JointMimicInfo{
        .mimicked_joint_name = mimicked_joint_name,
        .scaling = mimic_joint->scaling(),
        .offset = mimic_joint->offset(),
    };
    joint_info_map_.emplace(mimicking_joint_name, info);
  }

  // Replace the model with its non-mimic version.
  pinocchio::urdf::buildModelFromXML(urdf, model_);
  pinocchio::urdf::buildGeom(model_, std::istringstream(urdf), pinocchio::COLLISION,
                             collision_model_, package_paths_str);
  collision_model_.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairsFromXML(model_, collision_model_, srdf);

  // Create auxiliary model info
  frame_map_ = createFrameMap(model_);
  joint_group_info_map_ = createJointGroupInfo(model_, srdf);

  model_data_ = pinocchio::Data(model_);
  collision_model_data_ = pinocchio::GeometryData(collision_model_);

  // Initialize the current state of the scene.
  cur_state_ = JointConfiguration{.joint_names = joint_names_,
                                  .positions = pinocchio::neutral(model_),
                                  .velocities = Eigen::VectorXd::Zero(model_.nv),
                                  .accelerations = Eigen::VectorXd::Zero(model_.nv)};
}

tl::expected<JointInfo, std::string> Scene::getJointInfo(const std::string& joint_name) const {
  auto it = joint_info_map_.find(joint_name);
  if (it == joint_info_map_.end()) {
    return tl::make_unexpected("Joint '" + joint_name + "' is not in the scene.");
  }
  return it->second;
}

double Scene::configurationDistance(const Eigen::VectorXd& q_start,
                                    const Eigen::VectorXd& q_end) const {
  return pinocchio::distance(model_, q_start, q_end);
}

void Scene::setRngSeed(unsigned int seed) { rng_gen_ = std::mt19937(seed); }

Eigen::VectorXd Scene::randomPositions() {
  Eigen::VectorXd positions(model_.nq);
  for (const auto& joint_name : joint_names_) {
    const auto& info = joint_info_map_.at(joint_name);
    if (info.mimic_info) {
      continue;  // Skip mimic joints as they are set later.
    }

    const auto q_idx = model_.idx_qs[model_.getJointId(joint_name)];
    if (info.type == JointType::CONTINUOUS) {
      // Special case for continuous joints, since the format is [cos(theta), sin(theta)].
      const auto angle = std::uniform_real_distribution<double>(-M_PI, M_PI)(rng_gen_);
      positions(q_idx) = std::cos(angle);
      positions(q_idx + 1) = std::sin(angle);
    } else {
      // Generic case.
      for (size_t idx = 0; idx < info.num_position_dofs; ++idx) {
        const auto& lo = info.limits.min_position[idx];
        const auto& hi = info.limits.max_position[idx];
        positions(q_idx) = std::uniform_real_distribution<double>(lo, hi)(rng_gen_);
      }
    }
  }

  applyMimics(positions);
  return positions;
}

std::optional<Eigen::VectorXd> Scene::randomCollisionFreePositions(size_t max_samples) {
  for (size_t idx = 0; idx < max_samples; ++idx) {
    const auto positions = randomPositions();
    if (!hasCollisions(positions)) {
      return positions;
    }
  }
  return std::nullopt;
}

bool Scene::hasCollisions(const Eigen::VectorXd& q) const {
  pinocchio::updateGeometryPlacements(model_, model_data_, collision_model_, collision_model_data_,
                                      q);
  return pinocchio::computeCollisions(model_, model_data_, collision_model_, collision_model_data_,
                                      q,
                                      /* stop_at_first_collision*/ true);
}

bool Scene::isValidPose(const Eigen::VectorXd& q) const {
  size_t q_idx = 0;
  for (const auto& joint_name : joint_names_) {
    const auto& info = joint_info_map_.at(joint_name);
    if (info.mimic_info) {
      ++q_idx;
      continue;  // Skip over mimic joints since we validate their parent.
    }

    switch (info.type) {
    // TODO: Validate multi-DOF joints.
    case JointType::CONTINUOUS:
      // Unbounded so always valid, but check whether the representation is on the unit circle.
      if (std::abs(std::pow(q(q_idx), 2) + std::pow(q(q_idx + 1), 2) - 1.0) > kUnitCircleTol) {
        return false;
      }
      q_idx += 2;
      break;
    default:
      for (size_t idx = 0; idx < info.num_position_dofs; ++idx) {
        const auto& lo = info.limits.min_position[idx];
        const auto& hi = info.limits.max_position[idx];
        if (q(q_idx) < lo || q(q_idx) > hi) {
          return false;
        }
        ++q_idx;
      }
    }
  }
  return true;
}

void Scene::applyMimics(Eigen::VectorXd& q) const {
  for (const auto& [joint_name, joint_info] : joint_info_map_) {
    if (!joint_info.mimic_info) {
      continue;
    }
    const auto& mimic_info = joint_info.mimic_info.value();

    const auto mimicking_idx = model_.getJointId(joint_name);
    const auto mimicking_idx_q = model_.idx_qs[mimicking_idx];

    const auto mimicked_idx = model_.getJointId(mimic_info.mimicked_joint_name);
    const auto mimicked_idx_q = model_.idx_qs[mimicked_idx];

    if (joint_info.type == JointType::CONTINUOUS) {
      const auto mimicked_angle = std::atan2(q(mimicked_idx_q + 1), q(mimicked_idx_q));
      const auto mimicking_angle = mimicked_angle * mimic_info.scaling + mimic_info.offset;
      q(mimicking_idx_q) = std::cos(mimicking_angle);
      q(mimicking_idx_q + 1) = std::sin(mimicking_angle);
    } else {  // Prismatic or revolute, which are single-DOF.
      q(mimicking_idx_q) = q(mimicked_idx_q) * mimic_info.scaling + mimic_info.offset;
    }
  }
}

Eigen::VectorXd Scene::toFullJointPositions(const std::string& group_name,
                                            const Eigen::VectorXd& q) const {
  Eigen::VectorXd q_out = cur_state_.positions;

  const auto maybe_group_info = getJointGroupInfo(group_name);
  if (!maybe_group_info) {
    throw std::runtime_error("Failed to get full joint positions: " + maybe_group_info.error());
  }
  const auto& q_indices = maybe_group_info.value().q_indices;
  if (q_indices.size() != q.size()) {
    throw std::runtime_error("Failed to get full joint positions: Joint group '" + group_name +
                             "' has nq=" + std::to_string(q_indices.size()) +
                             " but the input positions is of size " + std::to_string(q.size()) +
                             ".");
  }

  q_out(q_indices) = q;
  applyMimics(q_out);
  return q_out;
}

Eigen::VectorXd Scene::interpolate(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                                   const double fraction) const {
  return pinocchio::interpolate(model_, q_start, q_end, fraction);
}

Eigen::Matrix4d Scene::forwardKinematics(const Eigen::VectorXd& q,
                                         const std::string& frame_name) const {
  // TODO: Need to add all sorts of validation here.
  const auto maybe_frame_id = getFrameId(frame_name);
  if (!maybe_frame_id) {
    throw std::runtime_error("Failed to get frame ID: " + maybe_frame_id.error());
  }
  const auto frame_id = maybe_frame_id.value();

  pinocchio::forwardKinematics(model_, model_data_, q);
  pinocchio::updateFramePlacement(model_, model_data_, frame_id);
  return model_data_.oMf.at(frame_id);
}

tl::expected<pinocchio::FrameIndex, std::string> Scene::getFrameId(const std::string& name) const {
  auto it = frame_map_.find(name);
  if (it == frame_map_.end()) {
    return tl::make_unexpected("Frame name '" + name + "' not found in frame_map_.");
  }
  return it->second;
}

tl::expected<JointGroupInfo, std::string> Scene::getJointGroupInfo(const std::string& name) const {
  auto it = joint_group_info_map_.find(name);
  if (it == joint_group_info_map_.end()) {
    return tl::make_unexpected("Group name '" + name + "' not found in joint_group_info_map_.");
  }
  return it->second;
}

Eigen::VectorXi Scene::getJointPositionIndices(const std::vector<std::string>& joint_names) const {
  std::vector<int> q_indices;
  for (const auto& joint_name : joint_names) {
    const auto joint_id = model_.getJointId(joint_name);
    const auto idx_start = model_.idx_qs[joint_id];
    for (int dof = 0; dof < model_.joints[joint_id].nq(); ++dof) {
      q_indices.push_back(idx_start + dof);
    }
  }
  return Eigen::VectorXi::Map(q_indices.data(), q_indices.size());
}

tl::expected<void, std::string> Scene::addBoxGeometry(const std::string& name,
                                                      const std::string& parent_frame,
                                                      const Box& box, const Eigen::Matrix4d& tform,
                                                      const Eigen::Vector4d& color) {
  const auto maybe_parent_frame_id = getFrameId(parent_frame);
  if (!maybe_parent_frame_id) {
    return tl::make_unexpected("Failed to add box: " + maybe_parent_frame_id.error());
  }
  const auto& parent_frame_id = maybe_parent_frame_id.value();
  const auto parent_joint_id = model_.frames.at(parent_frame_id).parentJoint;

  pinocchio::GeometryObject geom_obj{name, parent_frame_id, parent_joint_id, pinocchio::SE3(tform),
                                     box.geom_ptr};
  geom_obj.meshColor = color;
  return addGeometry(geom_obj);
}

tl::expected<void, std::string> Scene::addSphereGeometry(const std::string& name,
                                                         const std::string& parent_frame,
                                                         const Sphere& sphere,
                                                         const Eigen::Matrix4d& tform,
                                                         const Eigen::Vector4d& color) {
  const auto maybe_parent_frame_id = getFrameId(parent_frame);
  if (!maybe_parent_frame_id) {
    return tl::make_unexpected("Failed to add sphere: " + maybe_parent_frame_id.error());
  }
  const auto& parent_frame_id = maybe_parent_frame_id.value();
  const auto parent_joint_id = model_.frames.at(parent_frame_id).parentJoint;

  pinocchio::GeometryObject geom_obj{name, parent_frame_id, parent_joint_id, pinocchio::SE3(tform),
                                     sphere.geom_ptr};
  geom_obj.meshColor = color;
  return addGeometry(geom_obj);
}

tl::expected<void, std::string> Scene::addGeometry(const pinocchio::GeometryObject& geom_obj) {
  auto it = collision_geometry_map_.find(geom_obj.name);
  if (it != collision_geometry_map_.end()) {
    return tl::make_unexpected("Object '" + geom_obj.name +
                               "' already exists in the scene. Cannot add.");
  }

  const auto collision_geom_idx = collision_model_.addGeometryObject(geom_obj, model_);
  collision_geometry_map_[geom_obj.name] = collision_geom_idx;

  // Add all collision pairs
  // TODO: Allow specifying filtered geometries
  for (size_t idx = 0; idx < collision_model_.ngeoms; ++idx) {
    if (idx == collision_geom_idx) {
      continue;  // Don't add a self-collision pair.
    }
    collision_model_.addCollisionPair(pinocchio::CollisionPair(idx, collision_geom_idx));
  }

  collision_model_data_ = pinocchio::GeometryData(collision_model_);
  return {};
}

tl::expected<void, std::string> Scene::updateGeometryPlacement(const std::string& name,
                                                               const std::string& parent_frame,
                                                               Eigen::Matrix4d& tform) {
  auto it = collision_geometry_map_.find(name);
  if (it == collision_geometry_map_.end()) {
    return tl::make_unexpected("Could not find object '" + name + "' to update.");
  }
  const auto& collision_geom_idx = it->second;

  const auto parent_frame_id = getFrameId(parent_frame);
  if (!parent_frame_id) {
    return tl::make_unexpected(parent_frame_id.error());
  }

  auto& collision_geom = collision_model_.geometryObjects[collision_geom_idx];
  collision_geom.parentFrame = parent_frame_id.value();
  collision_geom.placement = pinocchio::SE3(tform);
  return {};
}

tl::expected<void, std::string> Scene::removeGeometry(const std::string& name) {
  auto it = collision_geometry_map_.find(name);
  if (it == collision_geometry_map_.end()) {
    return tl::make_unexpected("Could not find object '" + name + "' to remove.");
  }
  collision_model_.removeGeometryObject(name);
  collision_geometry_map_.erase(name);
  collision_model_data_ = pinocchio::GeometryData(collision_model_);
  return {};
}

std::ostream& operator<<(std::ostream& os, const Scene& scene) {
  os << "Scene: " << scene.name_ << "\n";
  os << "Joint names: ";
  for (const auto& joint_name : scene.joint_names_) {
    os << joint_name << " ";
  }
  os << "\n";
  os << "Joint information:\n";
  for (const auto& joint_name : scene.joint_names_) {
    const auto& info = scene.joint_info_map_.at(joint_name);
    os << "  " << joint_name << ":\n";
    if (info.mimic_info) {
      os << "    mimics " << info.mimic_info->mimicked_joint_name << "\n";
      os << "    scaling: " << info.mimic_info->scaling;
      os << ", offset: " << info.mimic_info->offset << "\n";
    } else {
      const auto& limits = info.limits;
      os << "    min positions: " << limits.min_position.transpose() << "\n";
      os << "    max positions: " << limits.max_position.transpose() << "\n";
      os << "    max velocity: " << limits.max_velocity.transpose() << "\n";
      os << "    max acceleration: " << limits.max_acceleration.transpose() << "\n";
      os << "    max jerk: " << limits.max_jerk.transpose() << "\n";
    }
  }
  os << "Joint group information:\n";
  for (const auto& [group_name, group_info] : scene.joint_group_info_map_) {
    os << "  [" << group_name << "] " << group_info;
  }
  os << "State:\n";
  os << "  positions: " << scene.cur_state_.positions.transpose() << "\n";
  os << "  velocities: " << scene.cur_state_.velocities.transpose() << "\n";
  os << "  accelerations: " << scene.cur_state_.accelerations.transpose() << "\n";
  return os;
}

}  // namespace roboplan
