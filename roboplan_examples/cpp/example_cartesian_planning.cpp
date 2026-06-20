#include <filesystem>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>
#include <roboplan_example_models/resources.hpp>

using namespace roboplan;

int main(int /*argc*/, char* /*argv*/[]) {
  // Set up the scene with the UR5 model.
  const auto share_prefix = example_models::get_package_share_dir();
  const auto model_prefix = share_prefix / "roboplan_example_models" / "models";
  const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
  const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
  const std::vector<std::filesystem::path> package_paths = {share_prefix};

  auto scene =
      std::make_shared<Scene>("example_cartesian_scene", urdf_path, srdf_path, package_paths);

  // Use the current configuration as the IK seed (also the path's start pose).
  JointConfiguration q_start;
  q_start.positions = scene->getCurrentJointPositions();

  // Build a simple straight-line path of 3 waypoints that translates the tool 5 cm along the
  // base-frame X axis, starting at the current tool pose.
  const std::string base_frame = "base";
  const std::string tip_frame = "tool0";
  const Eigen::Matrix4d start = scene->forwardKinematics(q_start.positions, tip_frame, base_frame);
  std::vector<Eigen::Matrix4d> waypoints;
  for (int i = 0; i < 3; ++i) {
    Eigen::Matrix4d pose = start;
    pose(0, 3) += i * 0.025;  // 0, 2.5 cm, 5 cm
    waypoints.push_back(pose);
  }
  const CartesianPath path({base_frame}, {tip_frame}, {waypoints});

  // Plan the joint trajectory that traces the path under a bounded-acceleration Cartesian feedrate
  // profile (the default Bounded speed mode).
  CartesianPlannerOptions options;
  options.group_name = "arm";
  options.dt = 0.01;
  options.max_linear_speed = 0.05;

  CartesianPathPlanner planner(scene, options);
  const auto result = planner.plan(path, q_start);
  if (!result.has_value()) {
    std::cout << "Cartesian planning failed: " << result.error() << "\n";
    return 1;
  }

  const JointTrajectory& traj = *result;
  const auto [peak_velocity_ratio, peak_acceleration_ratio] = planner.computePeakLimitRatios(traj);
  std::cout << "Cartesian planning succeeded!\n";
  std::cout << "  Trajectory samples:  " << traj.times.size() << "\n";
  std::cout << "  Trajectory duration: " << traj.times.back() << " s\n";
  std::cout << "  Achieved path length: " << planner.computeAchievedPathLength(traj, path)
            << " m\n";
  std::cout << "  Peak velocity / limit:     " << peak_velocity_ratio << "\n";
  std::cout << "  Peak acceleration / limit: " << peak_acceleration_ratio << "\n";

  return 0;
}
