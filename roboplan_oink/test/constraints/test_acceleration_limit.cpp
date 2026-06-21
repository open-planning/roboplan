#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>

#include <OsqpEigen/OsqpEigen.h>
#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/acceleration_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

class AccelerationLimitTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    urdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    srdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    package_paths_ = {example_models::get_package_share_dir()};
    yaml_config_path_ = model_prefix / "ur_robot_model" / "ur5_config.yaml";

    scene_ = std::make_shared<Scene>("test_scene", urdf_path_, srdf_path_, package_paths_,
                                     yaml_config_path_);
    oink_ = std::make_shared<Oink>(*scene_);

    const auto& model = scene_->getModel();
    num_variables_ = model.nv;

    // Use a mid-range configuration so the braking-distance term does not bind.
    Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
    scene_->setJointPositions(q);
  }

  std::filesystem::path urdf_path_;
  std::filesystem::path srdf_path_;
  std::vector<std::filesystem::path> package_paths_;
  std::filesystem::path yaml_config_path_;
  std::shared_ptr<Scene> scene_;
  std::shared_ptr<Oink> oink_;
  int num_variables_;
};

TEST_F(AccelerationLimitTest, Construction) {
  double dt = 0.01;
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_) * 5.0;

  AccelerationLimit constraint(*oink_, dt, a_max);

  EXPECT_EQ(constraint.dt, dt);
  EXPECT_EQ(constraint.a_max.size(), num_variables_);
  EXPECT_TRUE(constraint.a_max.isApprox(a_max));
  EXPECT_EQ(constraint.Delta_q_prev.size(), num_variables_);
  EXPECT_TRUE(constraint.Delta_q_prev.isApprox(Eigen::VectorXd::Zero(num_variables_)));
}

TEST_F(AccelerationLimitTest, GetNumConstraints) {
  AccelerationLimit constraint(*oink_, 0.01, Eigen::VectorXd::Ones(num_variables_) * 5.0);
  EXPECT_EQ(constraint.getNumConstraints(*scene_), num_variables_);
}

TEST_F(AccelerationLimitTest, ConstraintMatrixIsIdentity) {
  AccelerationLimit constraint(*oink_, 0.01, Eigen::VectorXd::Ones(num_variables_) * 5.0);

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(*scene_, G, lower, upper).has_value());

  EXPECT_TRUE(G.isApprox(Eigen::MatrixXd::Identity(num_variables_, num_variables_)));
}

// From rest (Delta_q_prev = 0), the bound should be the symmetric acceleration box a_max*dt²
// (assuming the braking-distance term is looser, which holds at this mid-range configuration).
TEST_F(AccelerationLimitTest, BoundsFromRest) {
  double dt = 0.01;
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_) * 5.0;
  AccelerationLimit constraint(*oink_, dt, a_max);

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(*scene_, G, lower, upper).has_value());

  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_NEAR(upper(i), a_max(i) * dt * dt, 1e-12);
    EXPECT_NEAR(lower(i), -a_max(i) * dt * dt, 1e-12);
  }
}

// The acceleration box is centered on the previous displacement: with Delta_q_prev = d,
// the admissible displacement is [d - a_max*dt², d + a_max*dt²].
TEST_F(AccelerationLimitTest, BoundsCenteredOnPreviousDisplacement) {
  double dt = 0.01;
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_) * 5.0;
  AccelerationLimit constraint(*oink_, dt, a_max);

  // Previous velocity of 0.5 rad/s on every joint -> Delta_q_prev = 0.5 * dt.
  Eigen::VectorXd v_prev = Eigen::VectorXd::Constant(num_variables_, 0.5);
  constraint.setLastVelocity(v_prev);
  EXPECT_TRUE(constraint.Delta_q_prev.isApprox(v_prev * dt));

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(*scene_, G, lower, upper).has_value());

  const double box = a_max(0) * dt * dt;
  const double d = 0.5 * dt;
  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_NEAR(upper(i), d + box, 1e-12);
    EXPECT_NEAR(lower(i), -(box - d), 1e-12);  // = d - box
  }
}

// reset() clears the previous displacement back to the rest case.
TEST_F(AccelerationLimitTest, ResetClearsPreviousDisplacement) {
  AccelerationLimit constraint(*oink_, 0.01, Eigen::VectorXd::Ones(num_variables_) * 5.0);
  constraint.setLastVelocity(Eigen::VectorXd::Constant(num_variables_, 1.0));
  ASSERT_FALSE(constraint.Delta_q_prev.isApprox(Eigen::VectorXd::Zero(num_variables_)));

  constraint.reset();
  EXPECT_TRUE(constraint.Delta_q_prev.isApprox(Eigen::VectorXd::Zero(num_variables_)));
}

// An infinite acceleration limit leaves the joint unconstrained (bounds clamped to OSQP INFTY).
TEST_F(AccelerationLimitTest, InfiniteLimitIsUnconstrained) {
  Eigen::VectorXd a_max =
      Eigen::VectorXd::Constant(num_variables_, std::numeric_limits<double>::infinity());
  AccelerationLimit constraint(*oink_, 0.01, a_max);

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(*scene_, G, lower, upper).has_value());

  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_GE(upper(i), OsqpEigen::INFTY);
    EXPECT_LE(lower(i), -OsqpEigen::INFTY);
  }
}

// Near the upper position limit, the braking-distance term should clamp the upper bound below
// the pure acceleration box.
TEST_F(AccelerationLimitTest, BrakingDistanceLimitsApproachToBound) {
  const auto limits = scene_->getPositionLimitVectors("", /*collapsed*/ true);
  ASSERT_TRUE(limits.has_value());
  const Eigen::VectorXd& q_max = limits->second;

  // Place the first joint just inside its upper limit (if it has a finite one).
  int test_joint = -1;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(q_max(i))) {
      q(i) = q_max(i) - 1e-4;  // 0.1 mrad from the limit
      test_joint = i;
      break;
    }
  }
  ASSERT_GE(test_joint, 0) << "Model has no finite upper position limit to test braking.";
  scene_->setJointPositions(q);

  double dt = 0.01;
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_) * 5.0;
  AccelerationLimit constraint(*oink_, dt, a_max);

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(*scene_, G, lower, upper).has_value());

  const double accel_box = a_max(test_joint) * dt * dt;
  const double brake = dt * std::sqrt(2.0 * a_max(test_joint) * 1e-4);
  EXPECT_LT(upper(test_joint), accel_box);
  EXPECT_NEAR(upper(test_joint), brake, 1e-12);
}

TEST_F(AccelerationLimitTest, InvalidDtThrows) {
  EXPECT_THROW(
      { AccelerationLimit(*oink_, 0.0, Eigen::VectorXd::Ones(num_variables_)); },
      std::invalid_argument);
}

TEST_F(AccelerationLimitTest, MismatchedAMaxSizeThrows) {
  EXPECT_THROW(
      { AccelerationLimit(*oink_, 0.01, Eigen::VectorXd::Ones(num_variables_ - 1)); },
      std::invalid_argument);
}

TEST_F(AccelerationLimitTest, NegativeAMaxThrows) {
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_);
  a_max(2) = -1.0;
  EXPECT_THROW({ AccelerationLimit(*oink_, 0.01, a_max); }, std::invalid_argument);
}

TEST_F(AccelerationLimitTest, MismatchedVPrevSizeThrows) {
  AccelerationLimit constraint(*oink_, 0.01, Eigen::VectorXd::Ones(num_variables_));
  EXPECT_THROW(
      { constraint.setLastVelocity(Eigen::VectorXd::Ones(num_variables_ - 1)); },
      std::invalid_argument);
}

TEST_F(AccelerationLimitTest, MismatchedWorkspaceSize) {
  AccelerationLimit constraint(*oink_, 0.01, Eigen::VectorXd::Ones(num_variables_));

  Eigen::MatrixXd G(num_variables_ - 1, num_variables_);
  Eigen::VectorXd lower(num_variables_ - 1), upper(num_variables_ - 1);
  auto result = constraint.computeQpConstraints(*scene_, G, lower, upper);

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("size mismatch") != std::string::npos);
}

}  // namespace roboplan
