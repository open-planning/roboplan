#include <stdexcept>

#include <tl/expected.hpp>

#include <pinocchio/collision/broadphase.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <yaml-cpp/yaml.h>

#include <roboplan/core/scene.hpp>

namespace {

const std::map<std::string, roboplan::JointType> kPinocchioJointTypeMap = {
    {"JointModelPX", roboplan::JointType::PRISMATIC},
    {"JointModelPY", roboplan::JointType::PRISMATIC},
    {"JointModelPZ", roboplan::JointType::PRISMATIC},
    {"JointModelRX", roboplan::JointType::REVOLUTE},
    {"JointModelRY", roboplan::JointType::REVOLUTE},
    {"JointModelRZ", roboplan::JointType::REVOLUTE},
    {"JointModelPlanar", roboplan::JointType::PLANAR},
    {"JointModelFreeFlyer", roboplan::JointType::FLOATING},
};

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

  // Build the Pinocchio models and default data from XML strings.
  pinocchio::urdf::buildModelFromXML(urdf, model_);

  pinocchio::urdf::buildGeom(model_, std::istringstream(urdf), pinocchio::COLLISION,
                             collision_model_, package_paths_str);
  collision_model_.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairsFromXML(model_, collision_model_, srdf);

  model_data_ = pinocchio::Data(model_);
  collision_model_data_ = pinocchio::GeometryData(collision_model_);

  YAML::Node yaml_config;
  if (!yaml_config_path.empty() && !std::filesystem::is_directory(yaml_config_path)) {
    yaml_config = YAML::LoadFile(yaml_config_path);
  }

  // Initialize the RNG to be pseudorandom. You can use setRngSeed() to fix
  // this.
  std::random_device rd;
  rng_gen_ = std::mt19937(rd());

  // Create additional robot information.
  size_t q_idx = 0;
  size_t v_idx = 0;
  joint_names_.reserve(model_.njoints - 1);
  for (int idx = 1; idx < model_.njoints; ++idx) {  // omits "universe" joint.
    const auto joint_name = model_.names.at(idx);
    joint_names_.push_back(joint_name);

    const auto& joint = model_.joints.at(idx);
    auto info = JointInfo(kPinocchioJointTypeMap.at(joint.shortname()));
    for (int idx = 0; idx < joint.nq(); ++idx) {
      info.limits.min_position[idx] = model_.lowerPositionLimit(q_idx);
      info.limits.max_position[idx] = model_.upperPositionLimit(q_idx);
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
      info.limits.max_velocity[idx] = model_.velocityLimit(v_idx);
      if (maybe_acc_limits) {
        info.limits.max_acceleration[idx] = maybe_acc_limits.value()[idx].as<double>();
      }
      if (maybe_jerk_limits) {
        info.limits.max_jerk[idx] = maybe_jerk_limits.value()[idx].as<double>();
      }
      ++v_idx;
    }
    joint_info_.emplace(joint_name, info);
  }

  createFrameMap(model_);

  // Initialize the current state of the scene.
  cur_state_ = JointConfiguration{.joint_names = joint_names_,
                                  .positions = pinocchio::neutral(model_),
                                  .velocities = Eigen::VectorXd::Zero(model_.nv),
                                  .accelerations = Eigen::VectorXd::Zero(model_.nv)};
}

double Scene::configurationDistance(const Eigen::VectorXd& q_start,
                                    const Eigen::VectorXd& q_end) const {
  return pinocchio::distance(model_, q_start, q_end);
}

void Scene::setRngSeed(unsigned int seed) { rng_gen_ = std::mt19937(seed); }

Eigen::VectorXd Scene::randomPositions() {
  Eigen::VectorXd positions(model_.nq);
  int q_idx = 0;
  for (const auto& joint_name : joint_names_) {
    const auto& info = joint_info_.at(joint_name);
    for (size_t idx = 0; idx < info.num_position_dofs; ++idx) {
      const auto& lo = info.limits.min_position[idx];
      const auto& hi = info.limits.max_position[idx];
      positions(q_idx) = std::uniform_real_distribution<double>(lo, hi)(rng_gen_);
      ++q_idx;
    }
  }
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
  return pinocchio::computeCollisions(model_, model_data_, collision_model_, collision_model_data_,
                                      q,
                                      /* stop_at_first_collision*/ true);
}

bool Scene::isValidPose(const Eigen::VectorXd& q) const {
  size_t q_idx = 0;
  for (const auto& joint_name : joint_names_) {
    const auto& info = joint_info_.at(joint_name);
    for (size_t idx = 0; idx < info.num_position_dofs; ++idx) {
      const auto& lo = info.limits.min_position[idx];
      const auto& hi = info.limits.max_position[idx];
      if (q(q_idx) < lo || q(q_idx) > hi) {
        return false;
      }
      ++q_idx;
    }
  }
  return true;
}

Eigen::VectorXd Scene::interpolate(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                                   const double fraction) const {
  return pinocchio::interpolate(model_, q_start, q_end, fraction);
}

Eigen::Matrix4d Scene::forwardKinematics(const Eigen::VectorXd& q,
                                         const std::string& frame_name) const {
  // TODO: Need to add all sorts of validation here.
  pinocchio::framesForwardKinematics(model_, model_data_, q);
  auto frame_id = getFrameId(frame_name);
  if (!frame_id) {
    throw std::runtime_error("Failed to get frame ID: " + frame_id.error());
  }
  return model_data_.oMf[frame_id.value()];
}

std::ostream& operator<<(std::ostream& os, const Scene& scene) {
  os << "Scene: " << scene.name_ << "\n";
  os << "Joint names: ";
  for (const auto& joint_name : scene.joint_names_) {
    os << joint_name << " ";
  }
  os << "\n";
  os << "Joint limits:\n";
  for (const auto& joint_name : scene.joint_names_) {
    const auto& limits = scene.joint_info_.at(joint_name).limits;
    os << "  " << joint_name << ":\n";
    os << "    min positions: " << limits.min_position.transpose() << "\n";
    os << "    max positions: " << limits.max_position.transpose() << "\n";
    os << "    max velocity: " << limits.max_velocity.transpose() << "\n";
    os << "    max acceleration: " << limits.max_acceleration.transpose() << "\n";
    os << "    max jerk: " << limits.max_jerk.transpose() << "\n";
  }
  os << "State:\n";
  os << "  positions: " << scene.cur_state_.positions.transpose() << "\n";
  os << "  velocities: " << scene.cur_state_.velocities.transpose() << "\n";
  os << "  accelerations: " << scene.cur_state_.accelerations.transpose() << "\n";
  return os;
}

void Scene::createFrameMap(const pinocchio::Model& model) {
  frame_map_.clear();  // Clear existing map if needed
  for (int i = 1; i < model.nframes; ++i) {
    const auto& frame = model.frames[i];
    frame_map_[frame.name] = model.getFrameId(frame.name);
  }
}

tl::expected<pinocchio::FrameIndex, std::string> Scene::getFrameId(const std::string& name) const {
  auto it = frame_map_.find(name);
  if (it == frame_map_.end()) {
    return tl::make_unexpected("Frame name '" + name + "' not found in frame_map_.");
  }

  return it->second;
}

}  // namespace roboplan
