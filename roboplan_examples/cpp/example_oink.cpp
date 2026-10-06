#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include <pinocchio/algorithm/kinematics.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/configuration.hpp>
#include <roboplan_oink/tasks/frame.hpp>

using namespace roboplan;

int main(int /*argc*/, char* /*argv*/[]) {
  // Set up the scene
  const auto share_prefix = example_models::get_package_share_dir();
  const auto model_prefix = share_prefix / "roboplan_example_models" / "models";
  const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
  const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
  const std::vector<std::filesystem::path> package_paths = {share_prefix};

  const auto description = loadUrdfSceneDescription(urdf_path, package_paths);
  auto scene = Scene("example_optimal_ik_scene", description);
  if (const auto imported = scene.importSrdf(loadTextFile(srdf_path)); !imported) {
    throw std::runtime_error(imported.error());
  }

  // Set up target configuration
  const auto& model = scene.getModel();
  auto q_tgt = pinocchio::neutral(model);
  q_tgt[0] += 0.1;
  q_tgt[2] -= 0.1;
  q_tgt[4] -= 0.25;

  // Define goal pose
  const auto goal = CartesianConfiguration{
      .base_frame = "base",
      .tip_frame = "tool0",
      .tform = scene.forwardKinematics(q_tgt, "tool0", "base"),
  };

  // Set initial configuration
  auto q_start = pinocchio::neutral(model);
  scene.setJointPositions(q_start);

  // Update forward kinematics in scene data
  scene.forwardKinematics(q_start, "tool0");

  // Create Oink instance
  Oink oink(scene, "arm");
  const auto num_variables = oink.v_indices.size();

  // Create a FrameTask (high priority)
  FrameTaskOptions frame_options{
      .position_cost = 1.0,
      .orientation_cost = 1.0,
      .task_gain = 1.0,
      .lm_damping = 0.01,
  };
  auto frame_task = std::make_shared<FrameTask>(oink, scene, goal, frame_options);

  // Create a ConfigurationTask to regularize toward start configuration (low priority)
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Constant(num_variables, 0.1);
  ConfigurationTaskOptions config_options{
      .task_gain = 1.0,
      .lm_damping = 0.0,
      .priority = 2,
  };
  auto config_task = std::make_shared<ConfigurationTask>(oink, q_start(oink.q_indices),
                                                         joint_weights, config_options);

  std::vector<std::shared_ptr<Task>> tasks = {frame_task, config_task};

  // Create position limit constraint
  auto position_limit = std::make_shared<PositionLimit>(oink, 1.0);  // gain = 1.0
  std::vector<std::shared_ptr<Constraints>> constraints = {position_limit};

  // Solve IK with constraints
  Eigen::VectorXd delta_q = Eigen::VectorXd::Zero(num_variables);
  auto result = oink.solveIk(scene, tasks, constraints, {}, delta_q);

  if (!result.has_value()) {
    std::cout << "IK solve failed: " << result.error() << "\n";
    return 1;
  }

  std::cout << "IK solve succeeded!\n";
  std::cout << "  Start config: " << q_start.transpose() << "\n";
  std::cout << "  Delta q:      " << delta_q.transpose() << "\n";
  std::cout << "  Final config: " << (q_start + delta_q).transpose() << "\n";
  std::cout << "  Target config:" << q_tgt.transpose() << "\n";

  return 0;
}
