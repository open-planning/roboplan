#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <roboplan/core/scene.hpp>

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

  /// Create additional information to track the state of the robot.
  std::vector<std::string> joint_names;
  joint_names.reserve(model_.njoints - 1);
  for (int idx = 1; idx < model_.njoints; ++idx) { // omits "universe" joint.
    joint_names.push_back(model_.names.at(idx));
  }
  cur_state_ =
      JointConfiguration{.joint_names = joint_names,
                         .positions = pinocchio::neutral(model_),
                         .velocities = Eigen::VectorXd::Zero(model_.nv),
                         .accelerations = Eigen::VectorXd::Zero(model_.nv)};
}

void Scene::print() {
  std::cout << "Scene " << name_ << " has " << model_.nq << " DOF.\n";
  std::cout << "Joint names: ";
  for (const auto& joint_name : cur_state_.joint_names) {
    std::cout << joint_name << " ";
  }
  std::cout << "\n";
  std::cout << "q: " << cur_state_.positions.transpose() << "\n";
  std::cout << "v: " << cur_state_.velocities.transpose() << "\n";
  std::cout << "a: " << cur_state_.accelerations.transpose() << "\n";
}

} // namespace roboplan
