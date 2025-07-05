#include <pinocchio/collision/broadphase.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

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

Scene::Scene(const std::string& name, const std::filesystem::path& urdf_path,
             const std::filesystem::path& srdf_path,
             const std::vector<std::filesystem::path>& package_paths)
    : name_{name} {
  // Convert the vector of package paths to string to be compatible with
  // Pinocchio.
  std::vector<std::string> package_paths_str;
  package_paths_str.reserve(package_paths.size());
  for (const auto& path : package_paths) {
    package_paths_str.push_back(std::string(path));
  }

  // Build the Pinocchio models and default data.
  pinocchio::urdf::buildModel(urdf_path, model_);

  pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, collision_model_,
                             package_paths_str);
  collision_model_.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model_, collision_model_, srdf_path);

  model_data_ = pinocchio::Data(model_);
  collision_model_data_ = pinocchio::GeometryData(collision_model_);

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
    for (int idx = 0; idx < joint.nv(); ++idx) {
      info.limits.max_velocity[idx] = model_.velocityLimit(v_idx);
      ++v_idx;
    }
    joint_info_.emplace(joint_name, info);
  }

  // Initialize the current state of the scene.
  cur_state_ = JointConfiguration{.joint_names = joint_names_,
                                  .positions = pinocchio::neutral(model_),
                                  .velocities = Eigen::VectorXd::Zero(model_.nv),
                                  .accelerations = Eigen::VectorXd::Zero(model_.nv)};
}

double Scene::configurationDistance(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end) {
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

bool Scene::hasCollisions(const Eigen::VectorXd& q) {
  return pinocchio::computeCollisions(model_, model_data_, collision_model_, collision_model_data_,
                                      q,
                                      /* stop_at_first_collision*/ true);
}

bool Scene::hasCollisionsAlongPath(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                                   const double min_step_size) {

  const auto distance = configurationDistance(q_start, q_end);

  // Special case for short paths (also handles division by zero in the next case).
  const bool collision_at_endpoints = hasCollisions(q_start) || hasCollisions(q_end);
  if (distance <= min_step_size) {
    return collision_at_endpoints;
  }

  // In the general case, check the first and last points, then all the intermediate ones.
  const auto num_steps = static_cast<size_t>(std::ceil(distance / min_step_size)) + 1;
  if (collision_at_endpoints) {
    return true;
  }
  for (size_t idx = 1; idx <= num_steps - 1; ++idx) {
    const auto fraction = static_cast<double>(idx) / static_cast<double>(num_steps);
    if (hasCollisions(pinocchio::interpolate(model_, q_start, q_end, fraction))) {
      return true;
    }
  }
  return false;
}

bool Scene::isValidPose(const Eigen::VectorXd& q) {
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
    os << "    velocity: " << limits.max_velocity.transpose() << "\n";
  }
  os << "State:\n";
  os << "  positions: " << scene.cur_state_.positions.transpose() << "\n";
  os << "  velocities: " << scene.cur_state_.velocities.transpose() << "\n";
  os << "  accelerations: " << scene.cur_state_.accelerations.transpose() << "\n";
  return os;
}

}  // namespace roboplan
