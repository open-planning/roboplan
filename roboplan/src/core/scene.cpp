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

} // namespace

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

  // Build the Pinocchio models.
  pinocchio::urdf::buildModel(urdf_path, model_);

  pinocchio::GeometryModel visual_model;
  pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::VISUAL, visual_model,
                             package_paths_str);

  pinocchio::GeometryModel collision_model;
  pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION,
                             collision_model, package_paths_str);
  collision_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model_, collision_model, srdf_path);

  // Initialize the RNG to be pseudorandom. You can use setRngSeed() to fix
  // this.
  std::random_device rd;
  rng_gen_ = std::mt19937(rd());

  // Create additional robot information.
  size_t q_idx = 0;
  size_t v_idx = 0;
  joint_names_.reserve(model_.njoints - 1);
  for (int idx = 1; idx < model_.njoints; ++idx) { // omits "universe" joint.
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
  cur_state_ =
      JointConfiguration{.joint_names = joint_names_,
                         .positions = pinocchio::neutral(model_),
                         .velocities = Eigen::VectorXd::Zero(model_.nv),
                         .accelerations = Eigen::VectorXd::Zero(model_.nv)};
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
      positions(q_idx) =
          std::uniform_real_distribution<double>(lo, hi)(rng_gen_);
      ++q_idx;
    }
  }
  return positions;
}

void Scene::print() {
  std::cout << "Scene : " << name_ << "\n";
  std::cout << "Joint names: ";
  for (const auto& joint_name : joint_names_) {
    std::cout << joint_name << " ";
  }
  std::cout << "\n";
  std::cout << "Joint limits:\n";
  for (const auto& joint_name : joint_names_) {
    const auto& limits = joint_info_.at(joint_name).limits;
    std::cout << "  " << joint_name << ":\n";
    std::cout << "    min positions: " << limits.min_position.transpose()
              << "\n";
    std::cout << "    max positions: " << limits.max_position.transpose()
              << "\n";
    std::cout << "    velocity: " << limits.max_velocity.transpose() << "\n";
  }
  std::cout << "State:\n";
  std::cout << "  positions: " << cur_state_.positions.transpose() << "\n";
  std::cout << "  velocities: " << cur_state_.velocities.transpose() << "\n";
  std::cout << "  accelerations: " << cur_state_.accelerations.transpose()
            << "\n";
}

} // namespace roboplan
