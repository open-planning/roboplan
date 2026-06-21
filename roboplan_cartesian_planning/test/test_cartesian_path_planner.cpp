#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/configuration.hpp>
#include <roboplan_oink/tasks/frame.hpp>

namespace roboplan {

namespace {
constexpr char kGroup[] = "arm";
constexpr char kBaseFrame[] = "base";
constexpr char kTipFrame[] = "tool0";
}  // namespace

class CartesianPlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    // Load the YAML config so the joint acceleration limits are populated (they back the
    // peak-acceleration-ratio checks and the TOPP-RA re-timing).
    const auto yaml_config_path = model_prefix / "ur_robot_model" / "ur5_config.yaml";
    const std::vector<std::filesystem::path> package_paths = {
        example_models::get_package_share_dir()};
    scene_ = std::make_shared<Scene>("test_scene", urdf_path, srdf_path, package_paths,
                                     yaml_config_path);
  }

  /// @brief Builds a single-frame straight-line CartesianPath of `num_waypoints` points
  /// starting at the current tool pose and translating by `delta` (meters) per step
  /// along the world X axis of the base frame.
  CartesianPath makeLinePath(const Eigen::VectorXd& q_start_full, int num_waypoints, double delta) {
    const Eigen::Matrix4d start = scene_->forwardKinematics(q_start_full, kTipFrame, kBaseFrame);
    std::vector<Eigen::Matrix4d> waypoints;
    for (int i = 0; i < num_waypoints; ++i) {
      Eigen::Matrix4d pose = start;
      pose(0, 3) += i * delta;
      waypoints.push_back(pose);
    }
    return CartesianPath({kBaseFrame}, {kTipFrame}, {waypoints});
  }

public:
  std::shared_ptr<Scene> scene_;
};

TEST_F(CartesianPlannerTest, TracesStraightLineWithinTolerance) {
  CartesianPlannerOptions options;
  options.group_name = kGroup;
  options.dt = 0.01;
  options.max_linear_speed = 0.05;
  options.max_position_error = 0.005;
  options.max_orientation_error = 0.01;

  CartesianPathPlanner planner(scene_, options);

  JointConfiguration q_start;
  q_start.positions = scene_->getCurrentJointPositions();

  // A short, clearly reachable line (5 cm total) from the current pose.
  const CartesianPath path = makeLinePath(q_start.positions, 3, 0.025);

  const auto result = planner.plan(path, q_start);
  ASSERT_TRUE(result.has_value()) << result.error();

  const JointTrajectory& traj = *result;
  ASSERT_GE(traj.positions.size(), 2u);
  EXPECT_EQ(traj.times.size(), traj.positions.size());
  EXPECT_EQ(traj.velocities.size(), traj.positions.size());
  EXPECT_EQ(traj.accelerations.size(), traj.positions.size());

  // Times must be strictly increasing.
  for (size_t i = 1; i < traj.times.size(); ++i) {
    EXPECT_GT(traj.times.at(i), traj.times.at(i - 1));
  }

  // The final configuration must reach the final waypoint within tolerance.
  const Eigen::VectorXd q_full_final = scene_->toFullJointPositions(kGroup, traj.positions.back());
  const Eigen::Matrix4d fk_final = scene_->forwardKinematics(q_full_final, kTipFrame, kBaseFrame);
  const Eigen::Matrix4d& goal = path.tforms.at(0).back();
  const double final_position_error = (fk_final.block<3, 1>(0, 3) - goal.block<3, 1>(0, 3)).norm();
  EXPECT_LE(final_position_error, options.max_position_error + 1e-6);

  // The achieved path length should be close to the commanded 5 cm line.
  EXPECT_NEAR(planner.computeAchievedPathLength(traj, path), 0.05, 0.01);
}

TEST_F(CartesianPlannerTest, RespectsJointVelocityAndPositionLimits) {
  CartesianPlannerOptions options;
  options.group_name = kGroup;
  options.dt = 0.01;
  // Command an aggressive speed so the velocity limits are actively binding.
  options.max_linear_speed = 0.5;
  options.max_position_error = 0.01;
  options.max_orientation_error = 0.05;

  CartesianPathPlanner planner(scene_, options);

  JointConfiguration q_start;
  q_start.positions = scene_->getCurrentJointPositions();
  const CartesianPath path = makeLinePath(q_start.positions, 4, 0.05);

  const auto result = planner.plan(path, q_start);
  ASSERT_TRUE(result.has_value()) << result.error();

  const JointTrajectory& traj = *result;
  const auto velocity_limits = scene_->getVelocityLimitVectors(kGroup);
  ASSERT_TRUE(velocity_limits.has_value());
  const Eigen::VectorXd v_max = velocity_limits->second.cwiseAbs();

  // Allow a small relative slack for the QP's constraint-satisfaction tolerance.
  const Eigen::ArrayXd velocity_bound = v_max.array() * 1.01 + 1e-6;
  for (const auto& velocity : traj.velocities) {
    ASSERT_EQ(velocity.size(), v_max.size());
    EXPECT_TRUE((velocity.array().abs() <= velocity_bound).all())
        << "peak |q_dot| = " << velocity.array().abs().maxCoeff() << " exceeds limit "
        << v_max.maxCoeff();
  }

  // Every configuration along the trajectory must respect joint position limits.
  for (const auto& group_positions : traj.positions) {
    const Eigen::VectorXd q_full = scene_->toFullJointPositions(kGroup, group_positions);
    EXPECT_TRUE(scene_->isValidConfiguration(q_full));
  }
}

TEST_F(CartesianPlannerTest, TracesMultiFramePathDefault) {
  CartesianPlannerOptions options;
  options.group_name = kGroup;
  options.dt = 0.01;
  options.max_linear_speed = 0.05;
  options.max_position_error = 0.005;
  options.max_orientation_error = 0.01;
  CartesianPathPlanner planner(scene_, options);

  JointConfiguration q_start;
  q_start.positions = scene_->getCurrentJointPositions();

  // A coordinated two-end-effector path: trace the same tip frame against two identical,
  // trivially consistent references. This exercises the multi-frame plumbing while staying
  // kinematically feasible for a single arm.
  CartesianPath path = makeLinePath(q_start.positions, 3, 0.025);
  path.base_frames.push_back(kBaseFrame);
  path.tip_frames.push_back(kTipFrame);
  path.tforms.push_back(path.tforms.at(0));

  const auto result = planner.plan(path, q_start);
  ASSERT_TRUE(result.has_value()) << result.error();

  // The final configuration must reach the final waypoint within tolerance.
  const Eigen::VectorXd q_full_final =
      scene_->toFullJointPositions(kGroup, result->positions.back());
  const Eigen::Matrix4d fk_final = scene_->forwardKinematics(q_full_final, kTipFrame, kBaseFrame);
  const Eigen::Matrix4d& goal = path.tforms.at(0).back();
  const double final_position_error = (fk_final.block<3, 1>(0, 3) - goal.block<3, 1>(0, 3)).norm();
  EXPECT_LE(final_position_error, options.max_position_error + 1e-6);
}

TEST_F(CartesianPlannerTest, RejectsMismatchedFramePath) {
  CartesianPlannerOptions options;
  options.group_name = kGroup;
  CartesianPathPlanner planner(scene_, options);

  JointConfiguration q_start;
  q_start.positions = scene_->getCurrentJointPositions();

  // Add a base/tip frame without a matching transform list: an inconsistent path.
  CartesianPath path = makeLinePath(q_start.positions, 2, 0.02);
  path.base_frames.push_back(kBaseFrame);
  path.tip_frames.push_back(kTipFrame);

  const auto result = planner.plan(path, q_start);
  ASSERT_FALSE(result.has_value());
}

TEST_F(CartesianPlannerTest, CustomComponentsTracesLine) {
  CartesianPlannerOptions options;
  options.group_name = kGroup;
  options.dt = 0.01;
  options.max_linear_speed = 0.05;
  options.max_position_error = 0.005;
  options.max_orientation_error = 0.01;

  JointConfiguration q_start;
  q_start.positions = scene_->getCurrentJointPositions();
  const CartesianPath path = makeLinePath(q_start.positions, 3, 0.025);

  // Build a caller-owned Oink setup mirroring the planner's built-in default.
  auto oink = std::make_shared<Oink>(*scene_, kGroup);

  CartesianConfiguration target;
  target.base_frame = "";  // Target is interpreted in the world frame.
  target.tip_frame = kTipFrame;
  target.tform = scene_->forwardKinematics(q_start.positions, kTipFrame);
  FrameTaskOptions frame_options;
  frame_options.priority = 1;
  auto frame_task = std::make_shared<FrameTask>(*oink, *scene_, target, frame_options);

  const Eigen::VectorXd joint_weights = Eigen::VectorXd::Constant(oink->num_variables, 0.05);
  ConfigurationTaskOptions config_options;
  config_options.priority = 2;
  const Eigen::VectorXd target_q = q_start.positions(oink->q_indices);
  auto config_task =
      std::make_shared<ConfigurationTask>(*oink, target_q, joint_weights, config_options);

  const auto velocity_limits = scene_->getVelocityLimitVectors(kGroup);
  ASSERT_TRUE(velocity_limits.has_value());
  const Eigen::VectorXd v_max = velocity_limits->second.cwiseAbs();

  CartesianPlannerComponents components;
  components.oink = oink;
  components.tracking_tasks = {frame_task};
  components.extra_tasks = {config_task};
  components.constraints = {std::make_shared<VelocityLimit>(*oink, options.dt, v_max),
                            std::make_shared<PositionLimit>(*oink, 1.0)};

  CartesianPathPlanner planner(scene_, options, components);
  const auto result = planner.plan(path, q_start);
  ASSERT_TRUE(result.has_value()) << result.error();

  const Eigen::VectorXd q_full_final =
      scene_->toFullJointPositions(kGroup, result->positions.back());
  const Eigen::Matrix4d fk_final = scene_->forwardKinematics(q_full_final, kTipFrame, kBaseFrame);
  const Eigen::Matrix4d& goal = path.tforms.at(0).back();
  const double final_position_error = (fk_final.block<3, 1>(0, 3) - goal.block<3, 1>(0, 3)).norm();
  EXPECT_LE(final_position_error, options.max_position_error + 1e-6);
}

TEST_F(CartesianPlannerTest, CustomComponentsConstructorValidatesInputs) {
  CartesianPlannerOptions options;
  options.group_name = kGroup;

  // Null Oink is rejected.
  CartesianPlannerComponents components;
  EXPECT_THROW(CartesianPathPlanner(scene_, options, components), std::runtime_error);

  // Empty tracking tasks are rejected.
  components.oink = std::make_shared<Oink>(*scene_, kGroup);
  EXPECT_THROW(CartesianPathPlanner(scene_, options, components), std::runtime_error);

  // A null tracking-task entry is rejected.
  components.tracking_tasks = {nullptr};
  EXPECT_THROW(CartesianPathPlanner(scene_, options, components), std::runtime_error);
}

TEST_F(CartesianPlannerTest, CustomComponentsRejectsFrameCountMismatch) {
  CartesianPlannerOptions options;
  options.group_name = kGroup;

  JointConfiguration q_start;
  q_start.positions = scene_->getCurrentJointPositions();

  auto oink = std::make_shared<Oink>(*scene_, kGroup);
  CartesianConfiguration target;
  target.base_frame = "";
  target.tip_frame = kTipFrame;
  target.tform = scene_->forwardKinematics(q_start.positions, kTipFrame);
  FrameTaskOptions frame_options;
  frame_options.priority = 1;
  auto frame_task = std::make_shared<FrameTask>(*oink, *scene_, target, frame_options);

  CartesianPlannerComponents components;
  components.oink = oink;
  components.tracking_tasks = {frame_task};  // one tracking task

  CartesianPathPlanner planner(scene_, options, components);

  // A two-end-effector path with only one tracking task must be rejected.
  CartesianPath path = makeLinePath(q_start.positions, 2, 0.02);
  path.base_frames.push_back(kBaseFrame);
  path.tip_frames.push_back(kTipFrame);
  path.tforms.push_back(path.tforms.at(0));

  const auto result = planner.plan(path, q_start);
  ASSERT_FALSE(result.has_value());
}

TEST_F(CartesianPlannerTest, TimeOptimalModeRespectsVelocityAndAccelerationLimits) {
  JointConfiguration q_start;
  q_start.positions = scene_->getCurrentJointPositions();
  const CartesianPath path = makeLinePath(q_start.positions, 4, 0.05);

  CartesianPlannerOptions time_optimal_options;
  time_optimal_options.group_name = kGroup;
  time_optimal_options.max_position_error = 0.01;
  time_optimal_options.max_orientation_error = 0.05;
  time_optimal_options.speed_mode = CartesianSpeedMode::TimeOptimal;
  CartesianPathPlanner planner(scene_, time_optimal_options);
  const auto time_optimal_result = planner.plan(path, q_start);
  ASSERT_TRUE(time_optimal_result.has_value()) << time_optimal_result.error();

  // Allow a small slack for spline/discretization and the QP tolerance.
  const auto [peak_velocity_ratio, peak_acceleration_ratio] =
      planner.computePeakLimitRatios(*time_optimal_result);
  EXPECT_LE(peak_velocity_ratio, 1.1);
  EXPECT_LE(peak_acceleration_ratio, 1.1);
  EXPECT_GE(time_optimal_result->positions.size(), 2u);
}

TEST_F(CartesianPlannerTest, BoundedModeBoundsAccelerationAndStartsStopsAtRest) {
  CartesianPlannerOptions options;
  options.group_name = kGroup;
  options.dt = 0.01;
  // Aggressive commanded speed so the acceleration profile and joint-limit throttle are active.
  options.max_linear_speed = 0.5;
  options.max_linear_acceleration = 0.5;
  options.max_angular_acceleration = 2.5;
  options.max_position_error = 0.01;
  options.max_orientation_error = 0.05;
  options.speed_mode = CartesianSpeedMode::Bounded;

  CartesianPathPlanner planner(scene_, options);

  JointConfiguration q_start;
  q_start.positions = scene_->getCurrentJointPositions();
  const CartesianPath path = makeLinePath(q_start.positions, 4, 0.05);

  const auto result = planner.plan(path, q_start);
  ASSERT_TRUE(result.has_value()) << result.error();

  const JointTrajectory& traj = *result;
  ASSERT_GE(traj.velocities.size(), 3u);

  // The trapezoidal profile ramps from rest and back to rest, so the first and last joint
  // velocities should be (near) zero.
  EXPECT_LT(traj.velocities.front().cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_LT(traj.velocities.back().cwiseAbs().maxCoeff(), 0.05);

  // The bounded-acceleration profile plus the global slow-down retry should keep the peak joint
  // acceleration near its limit, unlike the old constant-speed trace which ignored acceleration
  // entirely. Allow slack for the finite-difference accelerations and the retry's accept tolerance.
  const auto [peak_velocity_ratio, peak_acceleration_ratio] = planner.computePeakLimitRatios(traj);
  EXPECT_LE(peak_acceleration_ratio, 1.1);
  EXPECT_LE(peak_velocity_ratio, 1.1);
}

TEST_F(CartesianPlannerTest, RejectsBadSeedSize) {
  CartesianPlannerOptions options;
  options.group_name = kGroup;
  CartesianPathPlanner planner(scene_, options);

  JointConfiguration q_start;
  q_start.positions = Eigen::VectorXd::Zero(3);  // wrong size
  const CartesianPath path = makeLinePath(scene_->getCurrentJointPositions(), 2, 0.02);

  const auto result = planner.plan(path, q_start);
  ASSERT_FALSE(result.has_value());
}

}  // namespace roboplan
