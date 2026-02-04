#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/tasks/configuration.hpp>

namespace roboplan {

class ConfigurationTaskTest : public ::testing::Test {
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
    nq_ = model.nq;
    nv_ = model.nv;

    // Set a non-zero initial configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nq_);
    q[0] = 0.5;   // shoulder_pan_joint
    q[1] = -0.5;  // shoulder_lift_joint
    q[2] = 1.0;   // elbow_joint
    scene_->setJointPositions(q);
  }

  std::filesystem::path urdf_path_;
  std::filesystem::path srdf_path_;
  std::vector<std::filesystem::path> package_paths_;
  std::filesystem::path yaml_config_path_;
  std::shared_ptr<Scene> scene_;
  int nq_;
  int nv_;
};

// Test configuration task construction
TEST_F(ConfigurationTaskTest, Construction) {
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);

  // Test default construction
  ConfigurationTask task1(target_q, joint_weights);
  EXPECT_EQ(task1.target_q.size(), nq_);
  EXPECT_EQ(task1.joint_weights.size(), nv_);
  EXPECT_EQ(task1.gain, 1.0);
  EXPECT_EQ(task1.lm_damping, 0.0);

  // Test construction with custom options
  ConfigurationTaskOptions options{
      .task_gain = 0.8,
      .lm_damping = 0.01,
  };
  ConfigurationTask task2(target_q, joint_weights, options);
  EXPECT_EQ(task2.gain, 0.8);
  EXPECT_EQ(task2.lm_damping, 0.01);
}

// Test that negative joint weights throw
TEST_F(ConfigurationTaskTest, NegativeWeightThrows) {
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);
  joint_weights(2) = -1.0;  // Negative weight

  EXPECT_THROW({ ConfigurationTask task(target_q, joint_weights); }, std::invalid_argument);
}

// Test error computation at current configuration (zero error)
TEST_F(ConfigurationTaskTest, ErrorAtCurrentConfig) {
  // Get current configuration and use it as target
  const Eigen::VectorXd& q = scene_->getCurrentJointPositions();
  Eigen::VectorXd target_q = q;
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);

  ConfigurationTask task(target_q, joint_weights);

  // Compute error
  auto result = task.computeError(*scene_);

  ASSERT_TRUE(result.has_value()) << "computeError failed: " << result.error();
  EXPECT_EQ(task.error_container.size(), nv_);

  // Error should be close to zero
  EXPECT_NEAR(task.error_container.norm(), 0.0, 1e-10);
}

// Test error computation with offset
TEST_F(ConfigurationTaskTest, ErrorWithOffset) {
  // Get current configuration
  const Eigen::VectorXd& q = scene_->getCurrentJointPositions();

  // Create target with offset
  Eigen::VectorXd target_q = q;
  target_q(0) += 0.1;  // 0.1 rad offset on first joint
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);

  ConfigurationTask task(target_q, joint_weights);

  // Compute error
  auto result = task.computeError(*scene_);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(task.error_container.size(), nv_);

  // Error points FROM current TO target, so when target > current, error > 0
  // Error on first joint should be approximately +0.1
  EXPECT_NEAR(task.error_container(0), 0.1, 1e-10);

  // Other joints should have zero error
  for (int i = 1; i < nv_; ++i) {
    EXPECT_NEAR(task.error_container(i), 0.0, 1e-10);
  }
}

// Test Jacobian computation dimensions and identity
TEST_F(ConfigurationTaskTest, JacobianIsIdentity) {
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);

  ConfigurationTask task(target_q, joint_weights);

  auto result = task.computeJacobian(*scene_);

  ASSERT_TRUE(result.has_value()) << "computeJacobian failed: " << result.error();
  EXPECT_EQ(task.jacobian_container.rows(), nv_);
  EXPECT_EQ(task.jacobian_container.cols(), nv_);

  // Jacobian should be negative identity
  Eigen::MatrixXd expected = -Eigen::MatrixXd::Identity(nv_, nv_);
  EXPECT_TRUE(task.jacobian_container.isApprox(expected, 1e-6))
      << "Jacobian is not negative identity" << std::endl
      << "Computed:\n"
      << task.jacobian_container << std::endl
      << "Expected:\n"
      << expected << std::endl;
}

// Test QP objective computation
TEST_F(ConfigurationTaskTest, QpObjectiveComputation) {
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);

  ConfigurationTaskOptions options{.lm_damping = 0.01};
  ConfigurationTask task(target_q, joint_weights, options);

  // Compute QP objective matrices (this internally calls computeJacobian and computeError)
  Eigen::SparseMatrix<double> H(nv_, nv_);
  Eigen::VectorXd c(nv_);
  auto result = task.computeQpObjective(*scene_, H, c);

  ASSERT_TRUE(result.has_value());

  // H should be positive semi-definite (diagonal elements >= 0)
  for (int i = 0; i < nv_; ++i) {
    EXPECT_GE(H.coeff(i, i), 0.0);
  }

  // H should be symmetric
  Eigen::MatrixXd H_dense = Eigen::MatrixXd(H);
  EXPECT_TRUE(H_dense.isApprox(H_dense.transpose(), 1e-10));
}

// Test weight matrix with per-joint weights
TEST_F(ConfigurationTaskTest, WeightMatrixPerJoint) {
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(nq_);

  // Create non-uniform weights
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Zero(nv_);
  joint_weights(0) = 10.0;  // High weight on first joint
  joint_weights(1) = 1.0;   // Normal weight on second
  joint_weights(2) = 0.1;   // Low weight on third
  // Remaining joints have zero weight

  ConfigurationTask task(target_q, joint_weights);

  // Check weight matrix diagonal
  const Eigen::MatrixXd& W = task.weight;
  EXPECT_EQ(W.rows(), nv_);
  EXPECT_EQ(W.cols(), nv_);

  // Verify diagonal elements: sqrt(joint_weight)
  EXPECT_NEAR(W(0, 0), std::sqrt(10.0), 1e-10);
  EXPECT_NEAR(W(1, 1), std::sqrt(1.0), 1e-10);
  EXPECT_NEAR(W(2, 2), std::sqrt(0.1), 1e-10);
  EXPECT_NEAR(W(3, 3), 0.0, 1e-10);  // Zero weight

  // Off-diagonal should be zero
  for (int i = 0; i < nv_; ++i) {
    for (int j = 0; j < nv_; ++j) {
      if (i != j) {
        EXPECT_NEAR(W(i, j), 0.0, 1e-10);
      }
    }
  }
}

// Test zero weight joints are effectively ignored
TEST_F(ConfigurationTaskTest, ZeroWeightJointsIgnored) {
  const Eigen::VectorXd& q = scene_->getCurrentJointPositions();

  // Create target with large offset on zero-weight joint
  Eigen::VectorXd target_q = q;
  target_q(0) += 100.0;  // Large offset on first joint

  // Zero weight on first joint
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);
  joint_weights(0) = 0.0;

  ConfigurationTask task(target_q, joint_weights);

  // Compute QP objective
  Eigen::SparseMatrix<double> H(nv_, nv_);
  Eigen::VectorXd c(nv_);
  task.computeQpObjective(*scene_, H, c);

  // First row/column of H should be effectively just damping
  // The weighted contribution from joint 0 error should be zero
  Eigen::MatrixXd H_dense = Eigen::MatrixXd(H);

  // c[0] should be zero since weight is zero (no gradient contribution)
  EXPECT_NEAR(c(0), 0.0, 1e-10);
}

// Test invalid target_q size
TEST_F(ConfigurationTaskTest, InvalidTargetSize) {
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(nq_ + 1);  // Wrong size
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);

  ConfigurationTask task(target_q, joint_weights);

  auto result = task.computeError(*scene_);

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("size") != std::string::npos);
}

// Test invalid joint_weights size
TEST_F(ConfigurationTaskTest, InvalidWeightsSize) {
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_ + 1);  // Wrong size

  ConfigurationTask task(target_q, joint_weights);

  auto result = task.computeJacobian(*scene_);

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("size") != std::string::npos);
}

// Test task gain parameter
TEST_F(ConfigurationTaskTest, TaskGainParameter) {
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);

  // Create tasks with different gains
  ConfigurationTaskOptions options_low{.task_gain = 0.1};
  ConfigurationTask task_low_gain(target_q, joint_weights, options_low);

  ConfigurationTaskOptions options_high{.task_gain = 0.9};
  ConfigurationTask task_high_gain(target_q, joint_weights, options_high);

  EXPECT_LT(task_low_gain.gain, task_high_gain.gain);

  // Both should compute without error
  Eigen::SparseMatrix<double> H(nv_, nv_);
  Eigen::VectorXd c(nv_);
  auto result = task_low_gain.computeQpObjective(*scene_, H, c);
  ASSERT_TRUE(result.has_value());
}

// Test that error and Jacobian signs are consistent:
// QP solution should move toward target, not away from it.
// This test verifies the sign convention is correct by checking that
// the analytical solution dq = -gain * J^{-1} * e moves toward target.
TEST_F(ConfigurationTaskTest, ErrorDirectionMatchesJacobian) {
  // Set current configuration to zero
  Eigen::VectorXd q_current = Eigen::VectorXd::Zero(nq_);
  scene_->setJointPositions(q_current);

  // Target is positive (we want to move in positive direction)
  Eigen::VectorXd target_q = Eigen::VectorXd::Constant(nq_, 0.5);
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);

  ConfigurationTaskOptions options{.task_gain = 1.0, .lm_damping = 0.0};
  ConfigurationTask task(target_q, joint_weights, options);

  // Compute error and Jacobian
  auto error_result = task.computeError(*scene_);
  ASSERT_TRUE(error_result.has_value());

  auto jacobian_result = task.computeJacobian(*scene_);
  ASSERT_TRUE(jacobian_result.has_value());

  // For the QP objective: min ||J*dq + gain*e||^2
  // The optimal solution is: dq = -gain * J^{-1} * e
  // With J = -I: dq = -gain * (-I)^{-1} * e = gain * e
  // If error points TO target (positive), dq should be positive
  Eigen::VectorXd dq_analytical =
      task.gain * (-task.jacobian_container).inverse() * task.error_container;

  // Verify dq points toward target (positive for all joints)
  for (int i = 0; i < nv_; ++i) {
    EXPECT_GT(dq_analytical(i), 0.0) << "Joint " << i << " should move toward positive target";
  }
}

// Test that error points in the correct direction (toward target)
TEST_F(ConfigurationTaskTest, ErrorPointsTowardTarget) {
  // Set current configuration
  Eigen::VectorXd q_current = Eigen::VectorXd::Zero(nq_);
  scene_->setJointPositions(q_current);

  // Target is above current (positive direction)
  Eigen::VectorXd target_q = Eigen::VectorXd::Constant(nq_, 0.3);
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(nv_);

  ConfigurationTask task(target_q, joint_weights);

  auto result = task.computeError(*scene_);
  ASSERT_TRUE(result.has_value());

  // Error should be positive (pointing toward positive target)
  for (int i = 0; i < nv_; ++i) {
    EXPECT_GT(task.error_container(i), 0.0) << "Error for joint " << i << " should point to target";
  }

  // Now test with target below current
  scene_->setJointPositions(Eigen::VectorXd::Constant(nq_, 0.5));
  target_q = Eigen::VectorXd::Zero(nq_);

  ConfigurationTask task2(target_q, joint_weights);
  auto result2 = task2.computeError(*scene_);
  ASSERT_TRUE(result2.has_value());

  // Error should be negative (pointing toward zero target from positive current)
  for (int i = 0; i < nv_; ++i) {
    EXPECT_LT(task2.error_container(i), 0.0)
        << "Error for joint " << i << " should point to target (negative)";
  }
}

}  // namespace roboplan
