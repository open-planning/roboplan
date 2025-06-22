#include <filesystem>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

using namespace roboplan;

int main(int /*argc*/, char* /*argv*/[]) {

  const auto share_prefix = roboplan_example_models::get_package_share_dir();

  // Set up the scene
  const auto urdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.urdf";
  const auto srdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.srdf";
  const std::vector<std::filesystem::path> package_paths = {share_prefix};

  auto scene = Scene("test_ik_scene", urdf_path, srdf_path, package_paths);

  // Set up and solve IK
  SimpleIkOptions options;
  options.step_size = 0.05;
  auto ik_solver = SimpleIk(scene, options);

  pinocchio::Data data(scene.getModel());
  auto q_tgt = pinocchio::neutral(scene.getModel());
  q_tgt[0] += 0.1;
  q_tgt[2] -= 0.1;
  q_tgt[4] -= 0.25;

  pinocchio::framesForwardKinematics(scene.getModel(), data, q_tgt);
  const auto init_tform = data.oMf[scene.getModel().getFrameId("tool0")];

  const auto goal = CartesianConfiguration{
      .base_frame = "base",
      .tip_frame = "tool0",
      .tform = init_tform.toHomogeneousMatrix(),
  };

  JointConfiguration start;
  start.positions = pinocchio::neutral(scene.getModel());

  JointConfiguration solution;
  const auto success = ik_solver.solveIk(goal, start, solution);

  if (!success) {
    std::cout << "IK solve failed!\n";
    return 0;
  }

  std::cout << "IK solve succeeded\n  Result: " << solution.positions.transpose() << "\n";

  return 0;
}
