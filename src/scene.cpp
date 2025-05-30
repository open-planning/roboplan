#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <roboplan/scene.hpp>

namespace roboplan {

Scene::Scene(const std::filesystem::path& urdf_path,
             const std::filesystem::path& srdf_path) {
  const std::vector<std::string> package_paths{
      "/home/sebastian/workspace/roboplan_ws/src/roboplan"};

  pinocchio::urdf::buildModel(urdf_path, model_);

  pinocchio::GeometryModel visual_model;
  pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::VISUAL, visual_model,
                             package_paths);

  pinocchio::GeometryModel collision_model;
  pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION,
                             collision_model, package_paths);
  collision_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model_, collision_model, srdf_path);

  std::cout << "Built a Pinocchio model with " << std::to_string(model_.nq)
            << " DOFs\n";

  std::vector<std::string> joint_names;
  joint_names.reserve(model_.njoints - 1);
  for (int idx = 1; idx < model_.njoints; ++idx) {
    joint_names.push_back(model_.names.at(idx));
  }
  cur_state_ =
      JointConfiguration{.joint_names = joint_names,
                         .positions = pinocchio::neutral(model_),
                         .velocities = Eigen::VectorXd::Zero(model_.nv),
                         .accelerations = Eigen::VectorXd::Zero(model_.nv)};
}

void Scene::print() {
  std::cout << "Scene has " << model_.nq << "DOF.\n";
  std::cout << "Joint names: ";
  for (const auto& joint_name : cur_state_.joint_names) {
    std::cout << joint_name << " ";
  }
  std::cout << "\n";
  std::cout << "q: " << cur_state_.positions << "\n";
  std::cout << "v: " << cur_state_.velocities << "\n";
  std::cout << "a: " << cur_state_.accelerations << "\n";
}

} // namespace roboplan
