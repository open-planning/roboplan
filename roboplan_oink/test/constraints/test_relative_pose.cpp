#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <stdexcept>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/relative_pose.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/configuration.hpp>
#include <roboplan_oink/tasks/frame.hpp>
#include <test_utils.hpp>

namespace roboplan {

class RelativePoseConstraintTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    const auto description =
        loadUrdfSceneDescription(model_prefix / "ur_robot_model" / "ur5_gripper.urdf",
                                 {example_models::get_package_share_dir()});
    scene_ = std::make_shared<Scene>("test_scene", description);
    oink_ = std::make_shared<Oink>(*scene_);
    q_ = Eigen::VectorXd::Zero(scene_->getModel().nq);
    q_.head(6) << 0.3, -1.2, 1.4, -1.0, -1.5, 0.2;
    scene_->setJointPositions(q_);
  }

  std::shared_ptr<Scene> scene_;
  std::shared_ptr<Oink> oink_;
  Eigen::VectorXd q_;
};

TEST_F(RelativePoseConstraintTest, InvalidFrameThrows) {
  EXPECT_THROW(RelativePoseConstraint(*oink_, *scene_, "forearm_link", "not_a_frame",
                                      Eigen::Matrix4d::Identity()),
               std::runtime_error);
}

TEST_F(RelativePoseConstraintTest, JacobianMatchesFiniteDifference) {
  // An arbitrary target so the error (and the Jacobian) are nontrivial.
  Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
  target.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  target.topRightCorner<3, 1>() << 0.1, -0.2, 0.3;
  RelativePoseConstraint constraint(*oink_, *scene_, "upper_arm_link", "tool0", target);

  const int nv = oink_->num_variables;
  Eigen::MatrixXd A(6, nv);
  Eigen::VectorXd lower(6), upper(6);
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), A, lower, upper));
  const Eigen::Matrix<double, 6, 1> e0 = constraint.computeError(posed(*oink_, *scene_));
  EXPECT_TRUE((-lower).isApprox(e0));

  const Eigen::VectorXd dq = 1e-6 * Eigen::VectorXd::Random(nv);
  scene_->setJointPositions(pinocchio::integrate(scene_->getModel(), q_, dq));
  const Eigen::Matrix<double, 6, 1> e1 = constraint.computeError(posed(*oink_, *scene_));
  EXPECT_TRUE((e1 - e0).isApprox(A * dq, 1e-4)) << (e1 - e0).transpose() << "\n"
                                                << (A * dq).transpose();
}

TEST_F(RelativePoseConstraintTest, HoldsRelativePoseWithinTolerance) {
  for (const double tolerance : {0.0, 0.02}) {
    scene_->setJointPositions(q_);
    const Eigen::Matrix4d initial_tool = scene_->forwardKinematics(q_, "tool0");
    const Eigen::Matrix4d T_ab =
        scene_->forwardKinematics(q_, "forearm_link").inverse() * initial_tool;
    auto constraint = std::make_shared<RelativePoseConstraint>(
        *oink_, *scene_, "forearm_link", "tool0", T_ab, Eigen::Vector3d::Constant(tolerance),
        Eigen::Vector3d::Constant(tolerance));

    // Drag tool0 sideways; the wrist must stay locked to the forearm (up to the tolerance).
    CartesianConfiguration goal;
    goal.tip_frame = "tool0";
    goal.tform = initial_tool;
    goal.tform(1, 3) += 0.1;
    auto frame_task = std::make_shared<FrameTask>(
        *oink_, *scene_, goal, FrameTaskOptions{.orientation_cost = 0.0, .task_gain = 0.2});
    std::vector<std::shared_ptr<Task>> tasks = {frame_task};
    std::vector<std::shared_ptr<Constraints>> constraints = {constraint};

    Eigen::VectorXd q = q_;
    Eigen::VectorXd delta_q(oink_->num_variables);
    for (int i = 0; i < 100; ++i) {
      ASSERT_TRUE(oink_->solveIk(q, tasks, constraints, {}, delta_q, 1e-6));
      q = pinocchio::integrate(scene_->getModel(), q, delta_q);
      scene_->setJointPositions(q);
      const Eigen::Matrix<double, 6, 1> error = constraint->computeError(posed(*oink_, *scene_));
      ASSERT_LE(error.cwiseAbs().maxCoeff(), tolerance + 1e-3) << "tolerance " << tolerance;
    }
    const Eigen::Vector3d moved = scene_->forwardKinematics(q, "tool0").topRightCorner<3, 1>() -
                                  initial_tool.topRightCorner<3, 1>();
    EXPECT_GT(moved.y(), 0.05) << "tolerance " << tolerance;
  }
}

TEST_F(RelativePoseConstraintTest, LowerPriorityTaskDoesNotFightConstraint) {
  // The wrist is in the forearm task's nullspace, but holding the tool orientation couples it back
  // to the forearm. A priority-2 configuration task holding the wrist still must not drag the
  // forearm off its target through the constraint.
  Eigen::VectorXd q_goal = q_;
  q_goal.head(3) += Eigen::Vector3d(0.2, 0.2, -0.2);
  CartesianConfiguration goal;
  goal.tip_frame = "forearm_link";
  goal.tform = scene_->forwardKinematics(q_goal, "forearm_link");
  std::vector<std::shared_ptr<Task>> tasks = {
      std::make_shared<FrameTask>(*oink_, *scene_, goal, FrameTaskOptions{}),
      std::make_shared<ConfigurationTask>(*oink_, q_,
                                          Eigen::VectorXd::Constant(oink_->num_variables, 0.05),
                                          ConfigurationTaskOptions{.priority = 2})};

  for (const double tolerance : {0.0, 0.01}) {
    std::vector<std::shared_ptr<Constraints>> constraints = {
        std::make_shared<RelativePoseConstraint>(
            *oink_, *scene_, "base_link", "tool0",
            scene_->forwardKinematics(q_, "base_link").inverse() *
                scene_->forwardKinematics(q_, "tool0"),
            Eigen::Vector3d::Constant(10.0), Eigen::Vector3d::Constant(tolerance))};
    Eigen::VectorXd q = q_;
    Eigen::VectorXd delta_q(oink_->num_variables);
    for (int i = 0; i < 20; ++i) {
      ASSERT_TRUE(oink_->solveIk(q, tasks, constraints, {}, delta_q, 1e-12));
      q = pinocchio::integrate(scene_->getModel(), q, delta_q);
    }
    const Eigen::Vector3d error =
        scene_->forwardKinematics(q, "forearm_link").topRightCorner<3, 1>() -
        goal.tform.topRightCorner<3, 1>();
    EXPECT_LT(error.norm(), 1e-5) << "tolerance " << tolerance;
  }
}

TEST_F(RelativePoseConstraintTest, InfiniteToleranceAxesStayInLowerPriorityNullspace) {
  // Only the tool orientation is constrained, so a priority-2 task can still move the tool position
  // even though the constraint ranks above all tasks.
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<std::shared_ptr<Constraints>> constraints = {std::make_shared<RelativePoseConstraint>(
      *oink_, *scene_, "base_link", "tool0",
      scene_->forwardKinematics(q_, "base_link").inverse() * scene_->forwardKinematics(q_, "tool0"),
      Eigen::Vector3d::Constant(inf), Eigen::Vector3d::Zero())};

  CartesianConfiguration shoulder_goal;
  shoulder_goal.tip_frame = "shoulder_link";
  shoulder_goal.tform = scene_->forwardKinematics(q_, "shoulder_link");
  CartesianConfiguration tool_goal;
  tool_goal.tip_frame = "tool0";
  tool_goal.tform = scene_->forwardKinematics(q_, "tool0");
  tool_goal.tform(2, 3) += 0.05;
  std::vector<std::shared_ptr<Task>> tasks = {
      std::make_shared<FrameTask>(*oink_, *scene_, shoulder_goal, FrameTaskOptions{}),
      std::make_shared<FrameTask>(*oink_, *scene_, tool_goal,
                                  FrameTaskOptions{.orientation_cost = 0.0, .priority = 2})};

  Eigen::VectorXd q = q_;
  Eigen::VectorXd delta_q(oink_->num_variables);
  for (int i = 0; i < 50; ++i) {
    ASSERT_TRUE(oink_->solveIk(q, tasks, constraints, {}, delta_q, 1e-12));
    q = pinocchio::integrate(scene_->getModel(), q, delta_q);
  }
  const Eigen::Vector3d error = scene_->forwardKinematics(q, "tool0").topRightCorner<3, 1>() -
                                tool_goal.tform.topRightCorner<3, 1>();
  EXPECT_LT(error.norm(), 1e-3);
}

}  // namespace roboplan
