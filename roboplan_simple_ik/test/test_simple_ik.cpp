#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

namespace roboplan {

constexpr auto kGroupName = "arm";
constexpr auto kBaseFrame = "base";
constexpr auto kTipFrame = "tool0";

// The solver's default time budget is too tight to reliably converge on a slow
// or loaded machine (e.g. a debug CI build), so give it generous headroom.
constexpr double kMaxSolveTime = 1.0;

class RoboPlanSimpleIkTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const std::vector<std::filesystem::path> package_paths = {
        example_models::get_package_share_dir()};
    scene = std::make_shared<Scene>("test_scene", urdf_path, srdf_path, package_paths);
  }

  // Builds a goal pose by running forward kinematics on a known configuration.
  CartesianConfiguration reachableGoal() const {
    const Eigen::VectorXd q_group{{0.0, -1.0, 1.0, -1.5, -1.5, 0.0}};
    const auto q = scene->toFullJointPositions(kGroupName, q_group);
    CartesianConfiguration goal;
    goal.base_frame = kBaseFrame;
    goal.tip_frame = kTipFrame;
    goal.tform = scene->forwardKinematics(q, kTipFrame, kBaseFrame);
    return goal;
  }

public:
  // No default constructors, so must be pointers.
  std::shared_ptr<Scene> scene;
};

TEST_F(RoboPlanSimpleIkTest, SolveIk) {
  // Happy path: solve for a reachable target.
  scene->setRngSeed(286);

  SimpleIkOptions options;
  options.group_name = kGroupName;
  options.max_time = kMaxSolveTime;
  auto ik = std::make_unique<SimpleIk>(scene, options);

  const auto goal = reachableGoal();
  JointConfiguration start;
  start.positions = Eigen::VectorXd::Zero(6);

  JointConfiguration solution;
  ASSERT_TRUE(ik->solveIk(goal, start, solution));

  // The solution must actually reach the goal pose.
  const auto q_solution = scene->toFullJointPositions(kGroupName, solution.positions);
  const auto achieved = scene->forwardKinematics(q_solution, kTipFrame, kBaseFrame);
  EXPECT_TRUE(achieved.isApprox(goal.tform, 1e-2));
}

TEST_F(RoboPlanSimpleIkTest, InvalidGroupName) {
  // An unknown joint group must fail to even construct the solver.
  SimpleIkOptions options;
  options.group_name = "not_a_group";
  EXPECT_THROW(SimpleIk(scene, options), std::runtime_error);
}

TEST_F(RoboPlanSimpleIkTest, InvalidTipFrame) {
  // An unknown tip frame must fail when solving.
  SimpleIkOptions options;
  options.group_name = kGroupName;
  auto ik = std::make_unique<SimpleIk>(scene, options);

  CartesianConfiguration goal;
  goal.base_frame = kBaseFrame;
  goal.tip_frame = "not_a_frame";

  JointConfiguration start;
  start.positions = Eigen::VectorXd::Zero(6);
  JointConfiguration solution;
  EXPECT_THROW(ik->solveIk(goal, start, solution), std::runtime_error);
}

TEST_F(RoboPlanSimpleIkTest, CollisionChecking) {
  // The same reachable target succeeds when collisions are ignored, but fails
  // once an obstacle makes every candidate configuration collide.
  scene->setRngSeed(286);

  const auto goal = reachableGoal();
  JointConfiguration start;
  start.positions = Eigen::VectorXd::Zero(6);
  JointConfiguration solution;

  // Without collision checking, the solver reaches the target.
  SimpleIkOptions options;
  options.group_name = kGroupName;
  options.check_collisions = false;
  options.max_restarts = 0;
  options.max_time = kMaxSolveTime;
  auto ik = std::make_unique<SimpleIk>(scene, options);
  ASSERT_TRUE(ik->solveIk(goal, start, solution));

  // Wrap the whole robot in a box so any configuration is in collision.
  ASSERT_TRUE(scene
                  ->addBoxGeometry("wall", "universe", Box(4.0, 4.0, 4.0),
                                   Eigen::Matrix4d::Identity(), Eigen::Vector4d(0.5, 0.5, 0.5, 0.5))
                  .has_value());

  options.check_collisions = true;
  ik = std::make_unique<SimpleIk>(scene, options);
  EXPECT_FALSE(ik->solveIk(goal, start, solution));
}

}  // namespace roboplan
