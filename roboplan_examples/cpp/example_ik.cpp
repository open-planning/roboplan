#include <filesystem>
#include <memory>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

using namespace roboplan;

int main(int /*argc*/, char* /*argv*/[]) {
  // Set up the scene
  const auto share_prefix = example_models::get_package_share_dir();
  const auto model_prefix = share_prefix / "roboplan_example_models" / "models";
  const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
  const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
  const std::vector<std::filesystem::path> package_paths = {share_prefix};

  auto scene = std::make_shared<Scene>("example_ik_scene", urdf_path, srdf_path, package_paths);

  // Set up and solve IK
  SimpleIkOptions options;
  options.group_name = "arm";
  options.step_size = 0.25;
  options.max_linear_error_norm = 0.001;
  options.max_angular_error_norm = 0.001;
  options.check_collisions = true;
  auto ik_solver = SimpleIk(scene, options);

  auto model = scene->getModel();
  auto q_tgt = pinocchio::neutral(model);
  q_tgt[0] += 0.1;
  q_tgt[2] -= 0.1;
  q_tgt[4] -= 0.25;

  const auto goal = CartesianConfiguration{
      .base_frame = "base",
      .tip_frame = "tool0",
      .tform = scene->forwardKinematics(q_tgt, "tool0"),
  };

  JointConfiguration start;
  start.positions = pinocchio::neutral(model);

  JointConfiguration solution;
  const auto success = ik_solver.solveIk(goal, start, solution);

  if (!success) {
    std::cout << "IK solve failed!\n";
    return 0;
  }

  std::cout << "IK solve succeeded\n  Result: " << solution.positions.transpose() << "\n";

  return 0;
}
