#include <gtest/gtest.h>
#include <limits>
#include <memory>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <roboplan/core/pose_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/barriers/position_barrier.hpp>
#include <roboplan_oink/barriers/self_collision_barrier.hpp>
#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/constraints/relative_pose.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/configuration.hpp>
#include <roboplan_oink/tasks/frame.hpp>
#include <test_utils.hpp>

namespace roboplan {

class SolveIterativeIkTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    const auto description =
        loadUrdfSceneDescription(model_prefix / "ur_robot_model" / "ur5_gripper.urdf",
                                 {example_models::get_package_share_dir()});
    scene_ = std::make_shared<Scene>("test_scene", description);
    scene_->importJointLimitsFromConfig(
        loadJointLimitsConfig(model_prefix / "ur_robot_model" / "ur5_config.yaml"));
    ASSERT_TRUE(
        scene_->importSrdf(loadTextFile(model_prefix / "ur_robot_model" / "ur5_gripper.srdf")));
    oink_ = std::make_shared<Oink>(*scene_);
    nv_ = oink_->num_variables;
    q_ = Eigen::VectorXd::Zero(scene_->getModel().nq);
    q_.head(6) << 0.3, -1.2, 1.4, -1.0, -1.5, 0.2;
    scene_->setJointPositions(q_);
    limits_ = {std::make_shared<PositionLimit>(*oink_)};
  }

  /// @brief A FrameTask targeting the pose of `frame` at `q_goal`.
  std::shared_ptr<FrameTask> frameTask(const Eigen::VectorXd& q_goal,
                                       const std::string& frame = "tool0",
                                       const FrameTaskOptions& options = {}) {
    const CartesianConfiguration goal{
        .base_frame = "", .tip_frame = frame, .tform = scene_->forwardKinematics(q_goal, frame)};
    return std::make_shared<FrameTask>(*oink_, *scene_, goal, options);
  }

  /// @brief Checks that `solution` puts `frame` at its pose at `q_goal`.
  void expectReaches(const tl::expected<Eigen::VectorXd, std::string>& solution,
                     const Eigen::VectorXd& q_goal, const std::string& frame = "tool0") {
    ASSERT_TRUE(solution) << solution.error();
    const auto [position_error, orientation_error] = poseError(
        scene_->forwardKinematics(*solution, frame), scene_->forwardKinematics(q_goal, frame));
    EXPECT_LE(position_error, 1e-3);
    EXPECT_LE(orientation_error, 1e-3);
  }

  std::shared_ptr<Scene> scene_;
  std::shared_ptr<Oink> oink_;
  int nv_ = 0;
  Eigen::VectorXd q_;
  std::vector<std::shared_ptr<Constraints>> limits_;
};

TEST_F(SolveIterativeIkTest, ReachesGoal) {
  Eigen::VectorXd q_goal = q_;
  q_goal.head(6).array() += 0.2;
  expectReaches(oink_->solveIterativeIk(q_, {frameTask(q_goal)}, {}, limits_, {}), q_goal);
}

TEST_F(SolveIterativeIkTest, PositionLimitPullsStartBackInsideCustomLimits) {
  // Custom limits 0.3 rad around q_, with the start 0.2 rad past the upper limit on joint 0.
  const Eigen::VectorXd lower = q_.head(nv_).array() - 0.3;
  const Eigen::VectorXd upper = q_.head(nv_).array() + 0.3;
  Eigen::VectorXd q_start = q_;
  q_start[0] = upper[0] + 0.2;
  Eigen::VectorXd q_goal = q_;
  q_goal.head(6).array() += 0.1;

  auto limit = std::make_shared<PositionLimit>(*oink_, 1.0, lower, upper, /*max_pullback*/ 0.05);
  const auto solution = oink_->solveIterativeIk(q_start, {frameTask(q_goal)}, {}, {limit}, {},
                                                IterativeSolveOptions{.max_restarts = 0});
  expectReaches(solution, q_goal);
  EXPECT_TRUE((solution->array() >= lower.array() - 1e-4).all());
  EXPECT_TRUE((solution->array() <= upper.array() + 1e-4).all());
}

TEST_F(SolveIterativeIkTest, ConfigurationTaskConvergesToTarget) {
  Eigen::VectorXd q_goal = q_;
  q_goal.head(3) += Eigen::Vector3d(0.3, -0.2, 0.4);
  auto task = std::make_shared<ConfigurationTask>(*oink_, q_goal(oink_->q_indices),
                                                  Eigen::VectorXd::Ones(nv_));
  const auto solution = oink_->solveIterativeIk(q_, {task}, {}, {}, {});
  ASSERT_TRUE(solution) << solution.error();
  EXPECT_LT((*solution - q_goal).norm(), 1e-3);
}

// A posture task holding the start pose fights the goal at equal priority, but is confined to the
// goal's nullspace at priority 2. Uses the 7-DOF Kinova so that nullspace is not empty; on a 6-DOF
// arm the posture task would simply be skipped.
TEST(SolveIterativeIkKinovaTest, ExtraTaskYieldsToGoalOnlyAtLowerPriority) {
  const auto model_dir = example_models::get_package_models_dir() / "kinova_robot_model";
  Scene scene("test_scene", loadUrdfSceneDescription(model_dir / "kinova_robotiq.urdf",
                                                     {example_models::get_package_share_dir()}));
  scene.importJointLimitsFromConfig(
      loadJointLimitsConfig(model_dir / "kinova_robotiq_config.yaml"));
  ASSERT_TRUE(scene.importSrdf(loadTextFile(model_dir / "kinova_robotiq.srdf")));
  const std::string group = "manipulator";
  const std::string frame = "end_effector_link";
  Oink oink(scene, group);

  // The SRDF "Home" pose and a goal 0.2 rad away on every arm joint, integrated from neutral so
  // the continuous joints get their cos/sin form.
  Eigen::VectorXd home(7);
  home << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57;
  const Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(scene.getModel().nv);
  const Eigen::VectorXd q_start = scene.integrate(pinocchio::neutral(scene.getModel()),
                                                  scene.toFullJointVelocities(group, home, zero_v));
  const Eigen::VectorXd q_goal = scene.integrate(
      q_start, scene.toFullJointVelocities(group, Eigen::VectorXd::Constant(7, 0.2), zero_v));
  const Eigen::Matrix4d goal_pose = scene.forwardKinematics(q_goal, frame);
  auto frame_task = std::make_shared<FrameTask>(
      oink, scene,
      CartesianConfiguration{.base_frame = "", .tip_frame = frame, .tform = goal_pose});

  for (const int priority : {1, 2}) {
    auto posture =
        std::make_shared<ConfigurationTask>(oink, q_start(oink.q_indices), Eigen::VectorXd::Ones(7),
                                            ConfigurationTaskOptions{.priority = priority});
    const auto solution = oink.solveIterativeIk(q_start, {frame_task}, {posture}, {}, {},
                                                IterativeSolveOptions{.max_restarts = 0});
    if (priority == 1) {
      EXPECT_FALSE(solution);
    } else {
      ASSERT_TRUE(solution) << solution.error();
      const auto [position_error, orientation_error] =
          poseError(scene.forwardKinematics(*solution, frame), goal_pose);
      EXPECT_LE(position_error, 1e-3);
      EXPECT_LE(orientation_error, 1e-3);
    }
  }
}

TEST_F(SolveIterativeIkTest, HoldsRelativePoseConstraint) {
  // Lock the wrist to the forearm, then ask for a tool pose reachable with the first three joints.
  const Eigen::Matrix4d T_ab = scene_->forwardKinematics(q_, "forearm_link").inverse() *
                               scene_->forwardKinematics(q_, "tool0");
  Eigen::VectorXd q_goal = q_;
  q_goal.head(3) += Eigen::Vector3d(0.2, 0.2, -0.2);
  for (const double tolerance : {0.0, 0.02}) {
    auto constraint = std::make_shared<RelativePoseConstraint>(
        *oink_, *scene_, "forearm_link", "tool0", T_ab, 1.0, Eigen::Vector3d::Constant(tolerance),
        Eigen::Vector3d::Constant(tolerance));
    const auto solution = oink_->solveIterativeIk(q_, {frameTask(q_goal)}, {}, {constraint}, {},
                                                  IterativeSolveOptions{.max_iters = 200});
    ASSERT_TRUE(solution) << "tolerance " << tolerance << ": " << solution.error();
    expectReaches(solution, q_goal);
    scene_->setJointPositions(*solution);
    EXPECT_LE(constraint->computeViolation(posed(*oink_, *scene_)).norm(), 1e-4)
        << "tolerance " << tolerance;
  }
}

TEST_F(SolveIterativeIkTest, LowerPriorityTaskDoesNotFightConstraint) {
  // The wrist is in the forearm task's nullspace, but holding the tool orientation couples it back
  // to the forearm. A priority-2 configuration task holding the wrist still must not drag the
  // forearm off its target through the constraint.
  Eigen::VectorXd q_goal = q_;
  q_goal.head(3) += Eigen::Vector3d(0.2, 0.2, -0.2);
  auto posture = std::make_shared<ConfigurationTask>(*oink_, q_(oink_->q_indices),
                                                     Eigen::VectorXd::Constant(nv_, 0.05),
                                                     ConfigurationTaskOptions{.priority = 2});
  for (const double tolerance : {0.0, 0.01}) {
    auto constraint = std::make_shared<RelativePoseConstraint>(
        *oink_, *scene_, "base_link", "tool0",
        scene_->forwardKinematics(q_, "base_link").inverse() *
            scene_->forwardKinematics(q_, "tool0"),
        1.0, Eigen::Vector3d::Constant(10.0), Eigen::Vector3d::Constant(tolerance));
    const auto solution = oink_->solveIterativeIk(
        q_,
        {frameTask(q_goal, "forearm_link",
                   FrameTaskOptions{.position_tolerance = 1e-5, .orientation_tolerance = 1e-5})},
        {posture}, {constraint}, {});
    ASSERT_TRUE(solution) << "tolerance " << tolerance << ": " << solution.error();
    const Eigen::Vector3d error =
        scene_->forwardKinematics(*solution, "forearm_link").topRightCorner<3, 1>() -
        scene_->forwardKinematics(q_goal, "forearm_link").topRightCorner<3, 1>();
    EXPECT_LT(error.norm(), 1e-5) << "tolerance " << tolerance;
  }
}

TEST_F(SolveIterativeIkTest, InfiniteToleranceAxesStayInLowerPriorityNullspace) {
  // Only the tool orientation is constrained, so a priority-2 task can still move the tool position
  // even though the constraint ranks above all tasks.
  const double inf = std::numeric_limits<double>::infinity();
  auto constraint = std::make_shared<RelativePoseConstraint>(
      *oink_, *scene_, "base_link", "tool0",
      scene_->forwardKinematics(q_, "base_link").inverse() * scene_->forwardKinematics(q_, "tool0"),
      1.0, Eigen::Vector3d::Constant(inf), Eigen::Vector3d::Zero());
  CartesianConfiguration tool_goal{
      .base_frame = "", .tip_frame = "tool0", .tform = scene_->forwardKinematics(q_, "tool0")};
  tool_goal.tform(2, 3) += 0.05;
  auto tool_task = std::make_shared<FrameTask>(
      *oink_, *scene_, tool_goal, FrameTaskOptions{.orientation_cost = 0.0, .priority = 2});

  const auto solution = oink_->solveIterativeIk(q_, {frameTask(q_, "shoulder_link"), tool_task}, {},
                                                {constraint}, {});
  ASSERT_TRUE(solution) << solution.error();
  const Eigen::Vector3d error =
      scene_->forwardKinematics(*solution, "tool0").topRightCorner<3, 1>() -
      tool_goal.tform.topRightCorner<3, 1>();
  EXPECT_LT(error.norm(), 1e-3);
}

TEST_F(SolveIterativeIkTest, ReachesGoalWithBarriers) {
  const double dt = 0.1;
  Eigen::VectorXd q_goal = q_;
  q_goal.head(6).array() += 0.2;
  std::vector<std::shared_ptr<Constraints>> constraints = {
      std::make_shared<VelocityLimit>(*oink_, dt, Eigen::VectorXd::Constant(nv_, 1.0))};
  std::vector<std::shared_ptr<Barrier>> barriers = {
      std::make_shared<PositionBarrier>(*oink_, *scene_, "tool0", Eigen::Vector3d::Constant(-2.0),
                                        Eigen::Vector3d::Constant(2.0), dt),
      // The safe-displacement term biases every step toward a safe motion, which would hold the
      // solution short of the goal, so it is turned off here.
      std::make_shared<SelfCollisionBarrier>(
          *oink_, *scene_, dt,
          SelfCollisionBarrierOptions{
              .n_collision_pairs = 4, .gain = 5.0, .safe_displacement_gain = 0.0, .d_min = 0.02})};
  // Collision distances make each step slow, so give the solve more time than the default.
  const auto solution =
      oink_->solveIterativeIk(q_, {frameTask(q_goal)}, {}, constraints, barriers,
                              IterativeSolveOptions{.max_time = 10.0, .check_collisions = true});
  ASSERT_TRUE(solution) << solution.error();
  expectReaches(solution, q_goal);
  EXPECT_FALSE(scene_->hasCollisions(*solution));
}

TEST_F(SolveIterativeIkTest, RestartsFromBadSeed) {
  // Seed the solver folded onto itself, far from a goal it can reach, and let the restarts find it.
  Eigen::VectorXd q_goal = q_;
  q_goal.head(6) << 1.5, -0.5, 0.5, -0.5, 1.0, 0.0;
  Eigen::VectorXd q_bad = Eigen::VectorXd::Zero(q_.size());
  q_bad.head(6) << 0.0, -3.0, 3.0, -3.0, 0.0, 0.0;

  oink_->setRngSeed(42);
  expectReaches(oink_->solveIterativeIk(
                    q_bad, {frameTask(q_goal)}, {}, limits_, {},
                    IterativeSolveOptions{.max_iters = 20, .max_time = 1.0, .max_restarts = 20}),
                q_goal);
}

TEST_F(SolveIterativeIkTest, UnreachableGoalFails) {
  auto task = frameTask(q_);
  Eigen::Matrix4d target = task->target_pose.tform;
  target(2, 3) += 10.0;
  task->setTargetFrameTransform(target);
  EXPECT_FALSE(
      oink_->solveIterativeIk(q_, {task}, {}, {}, {}, IterativeSolveOptions{.max_restarts = 1}));
}

}  // namespace roboplan
