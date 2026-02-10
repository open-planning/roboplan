#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/position_limit.hpp>

namespace roboplan {

class PositionLimitTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Use UR5 robot for testing
    const auto model_prefix = example_models::get_package_models_dir();
    urdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    srdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    package_paths_ = {example_models::get_package_share_dir()};
    yaml_config_path_ = model_prefix / "ur_robot_model" / "ur5_config.yaml";

    scene_ = std::make_shared<Scene>("test_scene", urdf_path_, srdf_path_, package_paths_,
                                     yaml_config_path_);

    const auto& model = scene_->getModel();
    num_variables_ = model.nv;

    // Set initial configuration at zero (middle of joint ranges)
    Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
    scene_->setJointPositions(q);
  }

  std::filesystem::path urdf_path_;
  std::filesystem::path srdf_path_;
  std::vector<std::filesystem::path> package_paths_;
  std::filesystem::path yaml_config_path_;
  std::shared_ptr<Scene> scene_;
  int num_variables_;
};

// Test position limit construction
TEST_F(PositionLimitTest, Construction) {
  // Test default gain
  PositionLimit constraint1(num_variables_);
  EXPECT_EQ(constraint1.config_limit_gain, 1.0);

  // Test custom gain
  PositionLimit constraint2(num_variables_, 0.5);
  EXPECT_EQ(constraint2.config_limit_gain, 0.5);
}

// Test getNumConstraints returns correct value
TEST_F(PositionLimitTest, GetNumConstraints) {
  PositionLimit constraint(num_variables_, 1.0);

  int num_constraints = constraint.getNumConstraints(*scene_);
  EXPECT_EQ(num_constraints, num_variables_);
}

// Test constraint matrix dimensions
TEST_F(PositionLimitTest, ConstraintMatrixDimensions) {
  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  EXPECT_EQ(constraint_matrix.rows(), num_variables_);
  EXPECT_EQ(constraint_matrix.cols(), num_variables_);
  EXPECT_EQ(lower_bounds.size(), num_variables_);
  EXPECT_EQ(upper_bounds.size(), num_variables_);
}

// Test constraint matrix is identity
TEST_F(PositionLimitTest, ConstraintMatrixIsIdentity) {
  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Constraint matrix should be identity for box constraints
  Eigen::MatrixXd expected_identity = Eigen::MatrixXd::Identity(num_variables_, num_variables_);
  EXPECT_TRUE(constraint_matrix.isApprox(expected_identity));
}

// Test bounds at centered configuration
TEST_F(PositionLimitTest, BoundsAtCenter) {
  const auto& model = scene_->getModel();

  // Set configuration at the center of joint limits
  Eigen::VectorXd q = (model.lowerPositionLimit + model.upperPositionLimit) / 2.0;
  scene_->setJointPositions(q);

  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // At center, distances to upper and lower limits should be equal
  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(model.lowerPositionLimit[i]) && std::isfinite(model.upperPositionLimit[i])) {
      EXPECT_NEAR(upper_bounds[i], -lower_bounds[i], 1e-6);
    }
  }
}

// Test bounds near upper limit
TEST_F(PositionLimitTest, BoundsNearUpperLimit) {
  const auto& model = scene_->getModel();

  // Set configuration near upper limit (90% of the way)
  Eigen::VectorXd q =
      model.lowerPositionLimit + 0.9 * (model.upperPositionLimit - model.lowerPositionLimit);
  scene_->setJointPositions(q);

  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Upper bounds should be smaller than lower bounds (less room to move up)
  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(model.upperPositionLimit[i])) {
      EXPECT_LT(upper_bounds[i], -lower_bounds[i]);
    }
  }
}

// Test bounds near lower limit
TEST_F(PositionLimitTest, BoundsNearLowerLimit) {
  const auto& model = scene_->getModel();

  // Set configuration near lower limit (10% of the way)
  Eigen::VectorXd q =
      model.lowerPositionLimit + 0.1 * (model.upperPositionLimit - model.lowerPositionLimit);
  scene_->setJointPositions(q);

  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Lower bounds should be smaller (in magnitude) than upper bounds
  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(model.lowerPositionLimit[i])) {
      EXPECT_GT(upper_bounds[i], -lower_bounds[i]);
    }
  }
}

// Test gain parameter effect
TEST_F(PositionLimitTest, GainEffect) {
  const auto& model = scene_->getModel();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  PositionLimit constraint1(num_variables_, 0.5);  // Conservative gain
  PositionLimit constraint2(num_variables_, 1.0);  // Aggressive gain

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds1(num_variables_);
  Eigen::VectorXd upper_bounds1(num_variables_);
  Eigen::VectorXd lower_bounds2(num_variables_);
  Eigen::VectorXd upper_bounds2(num_variables_);

  ASSERT_TRUE(
      constraint1.computeQpConstraints(*scene_, constraint_matrix, lower_bounds1, upper_bounds1)
          .has_value());
  ASSERT_TRUE(
      constraint2.computeQpConstraints(*scene_, constraint_matrix, lower_bounds2, upper_bounds2)
          .has_value());

  // Constraint with gain 0.5 should have tighter bounds (50% of full range)
  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(model.lowerPositionLimit[i]) && std::isfinite(model.upperPositionLimit[i])) {
      EXPECT_NEAR(upper_bounds1[i], 0.5 * upper_bounds2[i], 1e-6);
      EXPECT_NEAR(lower_bounds1[i], 0.5 * lower_bounds2[i], 1e-6);
    }
  }
}

// Test with gain = 0 (no motion allowed)
TEST_F(PositionLimitTest, ZeroGain) {
  PositionLimit constraint(num_variables_, 0.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Both bounds should be zero (no motion allowed)
  EXPECT_TRUE(upper_bounds.isApprox(Eigen::VectorXd::Zero(num_variables_)));
  EXPECT_TRUE(lower_bounds.isApprox(Eigen::VectorXd::Zero(num_variables_)));
}

// Test infinite joint limits are clamped
TEST_F(PositionLimitTest, InfiniteJointLimits) {
  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Check that no bounds are infinite (they should be clamped to OSQP_INFTY)
  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_TRUE(std::isfinite(lower_bounds[i]));
    EXPECT_TRUE(std::isfinite(upper_bounds[i]));
  }
}

// Test bounds prevent exceeding upper limit
TEST_F(PositionLimitTest, PreventExceedingUpperLimit) {
  const auto& model = scene_->getModel();

  // Start at a configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // If we move by the maximum allowed upper bound, we shouldn't exceed joint limits
  Eigen::VectorXd q_new = q + upper_bounds;

  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(model.upperPositionLimit[i])) {
      EXPECT_LE(q_new[i], model.upperPositionLimit[i] + 1e-6);
    }
  }
}

// Test bounds prevent exceeding lower limit
TEST_F(PositionLimitTest, PreventExceedingLowerLimit) {
  const auto& model = scene_->getModel();

  // Start at a configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // If we move by the minimum allowed lower bound, we shouldn't go below joint limits
  Eigen::VectorXd q_new = q + lower_bounds;

  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(model.lowerPositionLimit[i])) {
      EXPECT_GE(q_new[i], model.lowerPositionLimit[i] - 1e-6);
    }
  }
}

// Test error handling for mismatched workspace size
TEST_F(PositionLimitTest, MismatchedWorkspaceSize) {
  PositionLimit constraint(num_variables_, 1.0);

  // Create workspace with wrong dimensions
  Eigen::MatrixXd constraint_matrix(num_variables_ - 1, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_ - 1);
  Eigen::VectorXd upper_bounds(num_variables_ - 1);

  auto result =
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds);

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("size mismatch") != std::string::npos);
}

// Test modification of gain parameter
TEST_F(PositionLimitTest, ModifyGain) {
  PositionLimit constraint(num_variables_, 1.0);

  // Change gain
  constraint.config_limit_gain = 0.3;

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Bounds should reflect new gain
  const auto& model = scene_->getModel();
  const auto& q = scene_->getCurrentJointPositions();

  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(model.upperPositionLimit[i])) {
      double expected_upper = 0.3 * (model.upperPositionLimit[i] - q[i]);
      EXPECT_NEAR(upper_bounds[i], expected_upper, 1e-6);
    }
  }
}

// Test at exact joint limit
TEST_F(PositionLimitTest, AtJointLimit) {
  const auto& model = scene_->getModel();

  // Set configuration at upper limit
  Eigen::VectorXd q = model.upperPositionLimit;
  scene_->setJointPositions(q);

  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Upper bounds should be zero or very small (no room to move up)
  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(model.upperPositionLimit[i])) {
      EXPECT_NEAR(upper_bounds[i], 0.0, 1e-6);
    }
  }
}

// Test different configurations produce different bounds
TEST_F(PositionLimitTest, DifferentConfigurationsDifferentBounds) {
  PositionLimit constraint(num_variables_, 1.0);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds1(num_variables_);
  Eigen::VectorXd upper_bounds1(num_variables_);
  Eigen::VectorXd lower_bounds2(num_variables_);
  Eigen::VectorXd upper_bounds2(num_variables_);

  // Configuration 1
  Eigen::VectorXd q1 = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q1);
  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds1, upper_bounds1)
          .has_value());

  // Configuration 2 (different)
  Eigen::VectorXd q2 = Eigen::VectorXd::Ones(num_variables_) * 0.5;
  scene_->setJointPositions(q2);
  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds2, upper_bounds2)
          .has_value());

  // Bounds should be different
  EXPECT_FALSE(upper_bounds1.isApprox(upper_bounds2));
  EXPECT_FALSE(lower_bounds1.isApprox(lower_bounds2));
}

}  // namespace roboplan
