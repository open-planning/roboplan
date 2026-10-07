#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/relative_pose.hpp>
#include <roboplan_oink/optimal_ik.hpp>
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
  const Eigen::Matrix<double, 6, 1> e0 = constraint.computeViolation(posed(*oink_, *scene_));
  EXPECT_TRUE((-lower).isApprox(e0));

  const Eigen::VectorXd dq = 1e-6 * Eigen::VectorXd::Random(nv);
  scene_->setJointPositions(pinocchio::integrate(scene_->getModel(), q_, dq));
  const Eigen::Matrix<double, 6, 1> e1 = constraint.computeViolation(posed(*oink_, *scene_));
  EXPECT_TRUE((e1 - e0).isApprox(A * dq, 1e-4)) << (e1 - e0).transpose() << "\n"
                                                << (A * dq).transpose();
}

}  // namespace roboplan
