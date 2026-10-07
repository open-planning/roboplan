#include <gtest/gtest.h>
#include <limits>
#include <memory>

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

TEST_F(SolveIterativeIkTest, ConfigurationTaskConvergesToTarget) {
  Eigen::VectorXd q_goal = q_;
  q_goal.head(3) += Eigen::Vector3d(0.3, -0.2, 0.4);
  auto task = std::make_shared<ConfigurationTask>(*oink_, q_goal(oink_->q_indices),
                                                  Eigen::VectorXd::Ones(nv_));
  const auto solution = oink_->solveIterativeIk(q_, {task}, {}, {}, {});
  ASSERT_TRUE(solution) << solution.error();
  EXPECT_LT((*solution - q_goal).norm(), 1e-3);
}

TEST_F(SolveIterativeIkTest, ExtraTaskYieldsToGoalOnlyAtLowerPriority) {
  // A posture task holding the start pose fights the goal at equal priority, but is confined to
  // the goal's nullspace at priority 2.
  Eigen::VectorXd q_goal = q_;
  q_goal.head(6).array() += 0.2;
  for (const int priority : {1, 2}) {
    auto posture = std::make_shared<ConfigurationTask>(
        *oink_, q_(oink_->q_indices), Eigen::VectorXd::Ones(nv_),
        ConfigurationTaskOptions{.priority = priority});
    const auto solution = oink_->solveIterativeIk(q_, {frameTask(q_goal)}, {posture}, {}, {},
                                                  IterativeSolveOptions{.max_restarts = 0});
    if (priority == 1) {
      EXPECT_FALSE(solution);
    } else {
      expectReaches(solution, q_goal);
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
        *oink_, *scene_, "forearm_link", "tool0", T_ab, Eigen::Vector3d::Constant(tolerance),
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
        Eigen::Vector3d::Constant(10.0), Eigen::Vector3d::Constant(tolerance));
    const auto solution =
        oink_->solveIterativeIk(q_, {frameTask(q_goal, "forearm_link")}, {posture}, {constraint},
                                {}, IterativeSolveOptions{.max_task_error_norm = 1e-5});
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
      Eigen::Vector3d::Constant(inf), Eigen::Vector3d::Zero());
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

  oink_->getContext().setRngSeed(42);
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
