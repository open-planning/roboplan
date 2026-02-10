#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>

namespace roboplan {

class VelocityLimitTest : public ::testing::Test {
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

    // Set initial configuration
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

// Test velocity limit construction
TEST_F(VelocityLimitTest, Construction) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;

  VelocityLimit constraint(num_variables_, dt, v_max);

  EXPECT_EQ(constraint.dt, dt);
  EXPECT_EQ(constraint.v_max.size(), num_variables_);
  EXPECT_TRUE(constraint.v_max.isApprox(v_max));
}

// Test getNumConstraints returns correct value
TEST_F(VelocityLimitTest, GetNumConstraints) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;

  VelocityLimit constraint(num_variables_, dt, v_max);

  int num_constraints = constraint.getNumConstraints(*scene_);
  EXPECT_EQ(num_constraints, num_variables_);
}

// Test constraint matrix dimensions
TEST_F(VelocityLimitTest, ConstraintMatrixDimensions) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;

  VelocityLimit constraint(num_variables_, dt, v_max);

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
TEST_F(VelocityLimitTest, ConstraintMatrixIsIdentity) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;

  VelocityLimit constraint(num_variables_, dt, v_max);

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

// Test bounds are symmetric
TEST_F(VelocityLimitTest, BoundsAreSymmetric) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;

  VelocityLimit constraint(num_variables_, dt, v_max);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Lower bounds should be negative of upper bounds
  EXPECT_TRUE(lower_bounds.isApprox(-upper_bounds));
}

// Test bounds are scaled by dt and v_max
TEST_F(VelocityLimitTest, BoundsScaling) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 2.0;

  VelocityLimit constraint(num_variables_, dt, v_max);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Expected bounds: +/- dt * v_max
  Eigen::VectorXd expected_upper = dt * v_max;
  Eigen::VectorXd expected_lower = -dt * v_max;

  EXPECT_TRUE(upper_bounds.isApprox(expected_upper));
  EXPECT_TRUE(lower_bounds.isApprox(expected_lower));
}

// Test different velocity limits per joint
TEST_F(VelocityLimitTest, PerJointLimits) {
  double dt = 0.01;
  Eigen::VectorXd v_max(num_variables_);
  v_max << 1.0, 2.0, 3.0, 1.5, 2.5, 0.5;  // Different limit for each joint

  VelocityLimit constraint(num_variables_, dt, v_max);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Check each joint has correct bounds
  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_NEAR(upper_bounds[i], dt * v_max[i], 1e-10);
    EXPECT_NEAR(lower_bounds[i], -dt * v_max[i], 1e-10);
  }
}

// Test with zero velocity limit
TEST_F(VelocityLimitTest, ZeroVelocityLimit) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Zero(num_variables_);

  VelocityLimit constraint(num_variables_, dt, v_max);

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

// Test with very small timestep
TEST_F(VelocityLimitTest, SmallTimestep) {
  double dt = 1e-6;  // Very small timestep
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;

  VelocityLimit constraint(num_variables_, dt, v_max);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Bounds should be very small
  EXPECT_LT(upper_bounds.maxCoeff(), 1e-5);
  EXPECT_GT(lower_bounds.minCoeff(), -1e-5);
}

// Test with large timestep
TEST_F(VelocityLimitTest, LargeTimestep) {
  double dt = 1.0;  // 1 second timestep
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 0.5;

  VelocityLimit constraint(num_variables_, dt, v_max);

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Bounds should be dt * v_max = 1.0 * 0.5 = 0.5
  EXPECT_TRUE(upper_bounds.isApprox(Eigen::VectorXd::Constant(num_variables_, 0.5)));
  EXPECT_TRUE(lower_bounds.isApprox(Eigen::VectorXd::Constant(num_variables_, -0.5)));
}

// Test error handling for mismatched v_max size
TEST_F(VelocityLimitTest, MismatchedVMaxSize) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_ - 1);  // Wrong size

  // Constructor should throw std::invalid_argument due to size mismatch
  EXPECT_THROW({ VelocityLimit constraint(num_variables_, dt, v_max); }, std::invalid_argument);
}

// Test error handling for mismatched workspace size
TEST_F(VelocityLimitTest, MismatchedWorkspaceSize) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_);

  VelocityLimit constraint(num_variables_, dt, v_max);

  // Create workspace with wrong dimensions
  Eigen::MatrixXd constraint_matrix(num_variables_ - 1, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_ - 1);
  Eigen::VectorXd upper_bounds(num_variables_ - 1);

  auto result =
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds);

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("size mismatch") != std::string::npos);
}

// Test modification of dt parameter
TEST_F(VelocityLimitTest, ModifyDt) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;

  VelocityLimit constraint(num_variables_, dt, v_max);

  // Change dt
  constraint.dt = 0.02;

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Bounds should reflect new dt
  EXPECT_TRUE(upper_bounds.isApprox(Eigen::VectorXd::Constant(num_variables_, 0.02)));
  EXPECT_TRUE(lower_bounds.isApprox(Eigen::VectorXd::Constant(num_variables_, -0.02)));
}

// Test modification of v_max parameter
TEST_F(VelocityLimitTest, ModifyVMax) {
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;

  VelocityLimit constraint(num_variables_, dt, v_max);

  // Change v_max
  constraint.v_max = Eigen::VectorXd::Ones(num_variables_) * 2.0;

  Eigen::MatrixXd constraint_matrix(num_variables_, num_variables_);
  Eigen::VectorXd lower_bounds(num_variables_);
  Eigen::VectorXd upper_bounds(num_variables_);

  ASSERT_TRUE(
      constraint.computeQpConstraints(*scene_, constraint_matrix, lower_bounds, upper_bounds)
          .has_value());

  // Bounds should reflect new v_max
  EXPECT_TRUE(upper_bounds.isApprox(Eigen::VectorXd::Constant(num_variables_, 0.02)));
  EXPECT_TRUE(lower_bounds.isApprox(Eigen::VectorXd::Constant(num_variables_, -0.02)));
}

}  // namespace roboplan
