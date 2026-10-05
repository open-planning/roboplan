#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/relative_pose.hpp>
#include <roboplan_oink/optimal_ik.hpp>
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

}  // namespace roboplan
