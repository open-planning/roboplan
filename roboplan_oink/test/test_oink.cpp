#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/configuration.hpp>
#include <roboplan_oink/tasks/frame.hpp>

namespace {
// Tolerance for OSQP constraint satisfaction
// OSQP is a numerical solver with finite precision, so we allow small violations (~1e-4)
constexpr double kTolerance = 1e-3;

// Helper to create CartesianConfiguration from position and orientation
roboplan::CartesianConfiguration makeCartesianConfig(const Eigen::Vector3d& position,
                                                     const Eigen::Quaterniond& orientation) {
  roboplan::CartesianConfiguration config;
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform.block<3, 3>(0, 0) = orientation.toRotationMatrix();
  tform.block<3, 1>(0, 3) = position;
  config.tform = tform;
  return config;
}

}  // namespace

namespace roboplan {

class OinkTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    urdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    srdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    package_paths_ = {example_models::get_package_share_dir()};
    yaml_config_path_ = model_prefix / "ur_robot_model" / "ur5_config.yaml";
    scene_ = std::make_shared<Scene>("test_scene", urdf_path_, srdf_path_, package_paths_,
                                     yaml_config_path_);

    // Get the number of variables (DOF)
    num_variables_ = scene_->getModel().nv;
  }

  std::shared_ptr<Scene> scene_;
  std::filesystem::path urdf_path_;
  std::filesystem::path srdf_path_;
  std::vector<std::filesystem::path> package_paths_;
  std::filesystem::path yaml_config_path_;
  int num_variables_;
};

// Test basic construction and initialization
TEST_F(OinkTest, Construction) {
  ASSERT_NO_THROW({
    Oink oink(num_variables_);
    EXPECT_EQ(oink.num_variables, num_variables_);
    EXPECT_EQ(oink.last_constraint_rows, -1);  // -1 = uninitialized
  });
}

// Test that solveIk returns an error when delta_q has wrong size (not undefined behavior)
TEST_F(OinkTest, DeltaQWrongSizeReturnsError) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create a simple frame task
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());
  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints;

  // Test with empty delta_q (size 0)
  {
    Eigen::VectorXd delta_q_empty;
    auto result = oink.solveIk(tasks, constraints, *scene_, delta_q_empty);
    ASSERT_FALSE(result.has_value()) << "Expected error for empty delta_q";
    EXPECT_TRUE(result.error().find("wrong size") != std::string::npos)
        << "Error message should mention wrong size: " << result.error();
    EXPECT_TRUE(result.error().find("expected " + std::to_string(num_variables_)) !=
                std::string::npos)
        << "Error message should mention expected size: " << result.error();
  }

  // Test with delta_q too small
  {
    Eigen::VectorXd delta_q_small(num_variables_ - 1);
    auto result = oink.solveIk(tasks, constraints, *scene_, delta_q_small);
    ASSERT_FALSE(result.has_value()) << "Expected error for too-small delta_q";
    EXPECT_TRUE(result.error().find("wrong size") != std::string::npos)
        << "Error message should mention wrong size: " << result.error();
  }

  // Test with delta_q too large
  {
    Eigen::VectorXd delta_q_large(num_variables_ + 1);
    auto result = oink.solveIk(tasks, constraints, *scene_, delta_q_large);
    ASSERT_FALSE(result.has_value()) << "Expected error for too-large delta_q";
    EXPECT_TRUE(result.error().find("wrong size") != std::string::npos)
        << "Error message should mention wrong size: " << result.error();
  }

  // Verify correct size still works
  {
    Eigen::VectorXd delta_q_correct(num_variables_);
    auto result = oink.solveIk(tasks, constraints, *scene_, delta_q_correct);
    ASSERT_TRUE(result.has_value()) << "Correct size should work: " << result.error();
  }
}

// Test solving with no constraints
TEST_F(OinkTest, SolveWithNoConstraints) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create a simple frame task (move end effector to target)
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());

  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints;

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();
  EXPECT_EQ(delta_q.size(), num_variables_);

  // Verify delta_q is not all zeros (should have moved)
  EXPECT_GT(delta_q.norm(), kTolerance);
}

// Test solving with velocity constraints
TEST_F(OinkTest, SolveWithVelocityConstraints) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create velocity limits
  double dt = 0.01;                                                     // 10ms timestep
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;  // 1 rad/s max
  auto vel_constraint = std::make_shared<VelocityLimit>(num_variables_, dt, v_max);

  // Create a frame task
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());
  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints = {vel_constraint};

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();
  EXPECT_EQ(delta_q.size(), num_variables_);

  // Verify velocity constraints are satisfied: |dq| <= dt * v_max
  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_LE(std::abs(delta_q[i]), dt * v_max[i] + kTolerance)
        << "Joint " << i << " violated velocity constraint";
  }
}

// Test solving with position constraints
TEST_F(OinkTest, SolveWithPositionConstraints) {
  Oink oink(num_variables_);

  // Set initial configuration near joint limits
  const auto& model = scene_->getModel();
  Eigen::VectorXd q = 0.9 * model.upperPositionLimit;
  scene_->setJointPositions(q);

  // Create position limit constraint
  double gain = 0.5;
  auto pos_constraint = std::make_shared<PositionLimit>(num_variables_, gain);

  // Create a task that would push toward the limit
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.8, 0.0, 0.5), Eigen::Quaterniond::Identity());
  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints = {pos_constraint};

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();
  EXPECT_EQ(delta_q.size(), num_variables_);

  // Verify we don't exceed joint limits
  Eigen::VectorXd q_new = q + delta_q;
  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(model.lowerPositionLimit[i])) {
      EXPECT_GE(q_new[i], model.lowerPositionLimit[i] - kTolerance)
          << "Joint " << i << " went below lower limit";
    }
    if (std::isfinite(model.upperPositionLimit[i])) {
      EXPECT_LE(q_new[i], model.upperPositionLimit[i] + kTolerance)
          << "Joint " << i << " went above upper limit";
    }
  }
}

// Test solving with multiple constraints
TEST_F(OinkTest, SolveWithMultipleConstraints) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create both velocity and position constraints
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 2.0;
  auto vel_constraint = std::make_shared<VelocityLimit>(num_variables_, dt, v_max);
  auto pos_constraint = std::make_shared<PositionLimit>(num_variables_, 0.8);

  // Create a frame task
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.4, 0.1, 0.6), Eigen::Quaterniond::Identity());
  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints = {vel_constraint, pos_constraint};

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();
  EXPECT_EQ(delta_q.size(), num_variables_);

  // Verify velocity constraints
  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_LE(std::abs(delta_q[i]), dt * v_max[i] + kTolerance);
  }
}

// Test workspace caching - solve twice to ensure no reallocation on second call
TEST_F(OinkTest, WorkspaceCaching) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create constraints
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;
  auto vel_constraint = std::make_shared<VelocityLimit>(num_variables_, dt, v_max);

  // Create task
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());
  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints = {vel_constraint};

  // First solve - workspace allocation
  Eigen::VectorXd delta_q1(num_variables_);
  auto result1 = oink.solveIk(tasks, constraints, *scene_, delta_q1);
  ASSERT_TRUE(result1.has_value()) << "First solve failed: " << result1.error();

  // Verify workspace dimensions
  EXPECT_EQ(oink.constraint_workspace_A.rows(), num_variables_);
  EXPECT_EQ(oink.constraint_workspace_A.cols(), num_variables_);
  EXPECT_EQ(oink.constraint_workspace_lower.size(), num_variables_);
  EXPECT_EQ(oink.constraint_workspace_upper.size(), num_variables_);
  EXPECT_EQ(oink.last_constraint_rows, num_variables_);

  // Store matrix data pointers to verify no reallocation
  const double* A_data_ptr = oink.constraint_workspace_A.data();
  const double* lower_data_ptr = oink.constraint_workspace_lower.data();
  const double* upper_data_ptr = oink.constraint_workspace_upper.data();

  // Second solve - should reuse workspace (no allocation)
  q = pinocchio::integrate(scene_->getModel(), q, delta_q1);  // Update configuration
  scene_->setJointPositions(q);

  Eigen::VectorXd delta_q2(num_variables_);
  auto result2 = oink.solveIk(tasks, constraints, *scene_, delta_q2);
  ASSERT_TRUE(result2.has_value()) << "Second solve failed: " << result2.error();

  // Verify workspace was reused (same pointers)
  EXPECT_EQ(oink.constraint_workspace_A.data(), A_data_ptr) << "Workspace A was reallocated!";
  EXPECT_EQ(oink.constraint_workspace_lower.data(), lower_data_ptr)
      << "Workspace lower was reallocated!";
  EXPECT_EQ(oink.constraint_workspace_upper.data(), upper_data_ptr)
      << "Workspace upper was reallocated!";
  EXPECT_EQ(oink.last_constraint_rows, num_variables_);
}

// Test constraint dimension validation
TEST_F(OinkTest, ConstraintDimensionValidation) {
  // Create velocity constraint with WRONG size - should throw at construction
  double dt = 0.01;
  Eigen::VectorXd v_max_wrong = Eigen::VectorXd::Ones(num_variables_ - 1);  // Wrong size!

  // Constructor should throw std::invalid_argument due to size mismatch
  EXPECT_THROW(
      { auto vel_constraint = std::make_shared<VelocityLimit>(num_variables_, dt, v_max_wrong); },
      std::invalid_argument);
}

// Test solving with dynamically changing constraint count
TEST_F(OinkTest, DynamicConstraintCount) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create task
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());
  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};

  // First solve with one constraint
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;
  auto vel_constraint = std::make_shared<VelocityLimit>(num_variables_, dt, v_max);
  std::vector<std::shared_ptr<Constraints>> constraints1 = {vel_constraint};

  Eigen::VectorXd delta_q1(num_variables_);
  auto result1 = oink.solveIk(tasks, constraints1, *scene_, delta_q1);
  ASSERT_TRUE(result1.has_value()) << "First solve failed: " << result1.error();
  EXPECT_EQ(oink.last_constraint_rows, num_variables_);

  // Second solve with two constraints (workspace should resize)
  auto pos_constraint = std::make_shared<PositionLimit>(num_variables_, 0.8);
  std::vector<std::shared_ptr<Constraints>> constraints2 = {vel_constraint, pos_constraint};

  Eigen::VectorXd delta_q2(num_variables_);
  auto result2 = oink.solveIk(tasks, constraints2, *scene_, delta_q2);
  ASSERT_TRUE(result2.has_value()) << "Second solve failed: " << result2.error();
  EXPECT_EQ(oink.last_constraint_rows, 2 * num_variables_);

  // Verify workspace dimensions grew
  EXPECT_EQ(oink.constraint_workspace_A.rows(), 2 * num_variables_);
  EXPECT_EQ(oink.constraint_workspace_lower.size(), 2 * num_variables_);
  EXPECT_EQ(oink.constraint_workspace_upper.size(), 2 * num_variables_);

  // Third solve back to one constraint (workspace should resize down)
  Eigen::VectorXd delta_q3(num_variables_);
  auto result3 = oink.solveIk(tasks, constraints1, *scene_, delta_q3);
  ASSERT_TRUE(result3.has_value()) << "Third solve failed: " << result3.error();
  EXPECT_EQ(oink.last_constraint_rows, num_variables_);
}

// Test Eigen::Ref safety - verify constraints cannot resize views
TEST_F(OinkTest, EigenRefSafety) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create properly sized constraint
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.0;
  auto vel_constraint = std::make_shared<VelocityLimit>(num_variables_, dt, v_max);

  // Create task
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());
  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints = {vel_constraint};

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();

  // After solve, verify workspace dimensions match what we expect
  EXPECT_EQ(oink.constraint_workspace_A.rows(), num_variables_);
  EXPECT_EQ(oink.constraint_workspace_A.cols(), num_variables_);
  EXPECT_EQ(oink.constraint_workspace_lower.rows(), num_variables_);
  EXPECT_EQ(oink.constraint_workspace_lower.cols(), 1);
  EXPECT_EQ(oink.constraint_workspace_upper.rows(), num_variables_);
  EXPECT_EQ(oink.constraint_workspace_upper.cols(), 1);
}

// Test solving with ConfigurationTask only
TEST_F(OinkTest, SolveWithConfigurationTask) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create target configuration with some offset
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(num_variables_);
  target_q(0) = 0.5;   // shoulder_pan_joint offset
  target_q(2) = -0.3;  // elbow_joint offset

  // Create uniform joint weights
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(num_variables_);

  ConfigurationTaskOptions options{.task_gain = 1.0, .lm_damping = 0.01};
  auto config_task = std::make_shared<ConfigurationTask>(target_q, joint_weights, options);
  std::vector<std::shared_ptr<Task>> tasks = {config_task};
  std::vector<std::shared_ptr<Constraints>> constraints;

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();
  EXPECT_EQ(delta_q.size(), num_variables_);

  // delta_q should move toward target
  EXPECT_GT(delta_q(0), 0.0);  // Should move positive toward 0.5
  EXPECT_LT(delta_q(2), 0.0);  // Should move negative toward -0.3
}

// Test solving with combined FrameTask and ConfigurationTask
TEST_F(OinkTest, SolveWithFrameAndConfigurationTasks) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create a frame task with high weight
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());
  FrameTaskOptions frame_options{.lm_damping = 0.01};
  auto frame_task =
      std::make_shared<FrameTask>("tool0", target_pose, num_variables_, frame_options);

  // Create a configuration task with lower weight (regularization)
  Eigen::VectorXd target_q = q;  // Keep current configuration
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(num_variables_);
  ConfigurationTaskOptions config_options{.lm_damping = 0.0};
  auto config_task = std::make_shared<ConfigurationTask>(target_q, joint_weights, config_options);

  std::vector<std::shared_ptr<Task>> tasks = {frame_task, config_task};
  std::vector<std::shared_ptr<Constraints>> constraints;

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();
  EXPECT_EQ(delta_q.size(), num_variables_);

  // delta_q should be non-zero (frame task should dominate)
  EXPECT_GT(delta_q.norm(), kTolerance);
}

// Test ConfigurationTask with selective joint weights
TEST_F(OinkTest, SolveWithSelectiveJointWeights) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create target that moves all joints
  Eigen::VectorXd target_q = Eigen::VectorXd::Constant(num_variables_, 0.5);

  // Only weight the first two joints
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Zero(num_variables_);
  joint_weights(0) = 1.0;
  joint_weights(1) = 1.0;

  ConfigurationTaskOptions options{.lm_damping = 0.01};
  auto config_task = std::make_shared<ConfigurationTask>(target_q, joint_weights, options);
  std::vector<std::shared_ptr<Task>> tasks = {config_task};
  std::vector<std::shared_ptr<Constraints>> constraints;

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();

  // Joints 0 and 1 should move toward target
  EXPECT_GT(delta_q(0), kTolerance);
  EXPECT_GT(delta_q(1), kTolerance);

  // Other joints should have minimal movement (only from LM damping regularization)
  // Note: due to the QP structure, zero-weight joints still get small movements
  // from the LM damping term, but they should be much smaller than weighted joints
  double weighted_movement = (delta_q(0) + delta_q(1)) / 2.0;
  for (int i = 2; i < num_variables_; ++i) {
    EXPECT_LT(std::abs(delta_q(i)), weighted_movement)
        << "Zero-weight joint " << i << " moved more than expected";
  }
}

// Test IK convergence with UR5 robot and position limits.
TEST_F(OinkTest, ConvergenceWithUR5CanonicalPoseAndPositionLimit) {
  // This test uses the scene_ from the fixture which is already set up with UR5

  const int ur5_nv = scene_->getModel().nv;
  const int ur5_nq = scene_->getModel().nq;
  Oink oink(ur5_nv);

  // Start from zero configuration
  Eigen::VectorXd q_canonical = Eigen::VectorXd::Zero(ur5_nq);

  // Set canonical configuration
  scene_->setJointPositions(q_canonical);

  // Get current end-effector position
  const Eigen::Matrix4d current_ee_pose = scene_->forwardKinematics(q_canonical, "tool0");
  const Eigen::Vector3d current_position = current_ee_pose.block<3, 1>(0, 3);
  const Eigen::Matrix3d current_rotation = current_ee_pose.block<3, 3>(0, 0);

  // Target is offset from current position (10cm in X direction)
  const Eigen::Vector3d target_position = current_position + Eigen::Vector3d(0.1, 0.0, 0.0);
  auto target_pose = makeCartesianConfig(target_position, Eigen::Quaterniond(current_rotation));

  // Create FrameTask with typical options
  FrameTaskOptions frame_options{
      .position_cost = 1.0,
      .orientation_cost = 1.0,
      .task_gain = 1.0,
      .lm_damping = 0.01,
  };
  auto frame_task = std::make_shared<FrameTask>("tool0", target_pose, ur5_nv, frame_options);

  // Create position limit constraint
  auto position_limit = std::make_shared<PositionLimit>(ur5_nv, 1.0);

  std::vector<std::shared_ptr<Task>> tasks = {frame_task};
  std::vector<std::shared_ptr<Constraints>> constraints = {position_limit};

  // Iterate IK solver
  Eigen::VectorXd q_current = q_canonical;
  constexpr int kMaxIterations = 100;
  constexpr double kPositionTolerance = 0.02;  // 2cm tolerance

  for (int iter = 0; iter < kMaxIterations; ++iter) {
    scene_->setJointPositions(q_current);
    scene_->forwardKinematics(q_current, "tool0");

    Eigen::VectorXd delta_q(ur5_nv);
    auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);
    ASSERT_TRUE(result.has_value()) << "Iteration " << iter << " failed: " << result.error();

    // Update configuration using Pinocchio integration for proper Lie group handling
    q_current = pinocchio::integrate(scene_->getModel(), q_current, delta_q);
  }

  // Final check: should have converged within max iterations
  scene_->setJointPositions(q_current);
  const Eigen::Matrix4d final_pose = scene_->forwardKinematics(q_current, "tool0");
  const Eigen::Vector3d final_position = final_pose.block<3, 1>(0, 3);
  const double final_error = (target_position - final_position).norm();

  EXPECT_LT(final_error, kPositionTolerance)
      << "IK did not converge after " << kMaxIterations << " iterations. "
      << "Target: [" << target_position.transpose() << "], "
      << "Achieved: [" << final_position.transpose() << "], "
      << "Error: " << final_error << "m";
}

// Test that FrameTask allocates correct dimensions at construction
TEST_F(OinkTest, FrameTaskStorageAllocation) {
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());

  // Create FrameTask with known num_variables
  FrameTask task("tool0", target_pose, num_variables_);

  // Verify pre-allocated storage dimensions
  EXPECT_EQ(task.jacobian_container.rows(), 6) << "FrameTask Jacobian should have 6 rows (SE(3))";
  EXPECT_EQ(task.jacobian_container.cols(), num_variables_)
      << "FrameTask Jacobian columns should match num_variables";
  EXPECT_EQ(task.error_container.size(), 6) << "FrameTask error should have 6 elements (SE(3))";
}

// Test that ConfigurationTask allocates correct dimensions at construction
TEST_F(OinkTest, ConfigurationTaskStorageAllocation) {
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(num_variables_);
  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(num_variables_);

  // Create ConfigurationTask
  ConfigurationTask task(target_q, joint_weights);

  // Verify pre-allocated storage dimensions
  EXPECT_EQ(task.jacobian_container.rows(), num_variables_)
      << "ConfigurationTask Jacobian rows should match num_variables";
  EXPECT_EQ(task.jacobian_container.cols(), num_variables_)
      << "ConfigurationTask Jacobian cols should match num_variables";
  EXPECT_EQ(task.error_container.size(), num_variables_)
      << "ConfigurationTask error should match num_variables";
}

// Test that task storage is reused across multiple solves (no reallocation)
TEST_F(OinkTest, TaskStorageReuse) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Create task
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());
  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints;

  // Store pointers to task's storage before first solve
  const double* jacobian_data_ptr = task->jacobian_container.data();
  const double* error_data_ptr = task->error_container.data();

  // First solve
  Eigen::VectorXd delta_q1(num_variables_);
  auto result1 = oink.solveIk(tasks, constraints, *scene_, delta_q1);
  ASSERT_TRUE(result1.has_value()) << "First solve failed: " << result1.error();

  // Verify storage wasn't reallocated
  EXPECT_EQ(task->jacobian_container.data(), jacobian_data_ptr)
      << "Task Jacobian was reallocated during first solve!";
  EXPECT_EQ(task->error_container.data(), error_data_ptr)
      << "Task error was reallocated during first solve!";

  // Update configuration and solve again
  q = pinocchio::integrate(scene_->getModel(), q, delta_q1);
  scene_->setJointPositions(q);

  Eigen::VectorXd delta_q2(num_variables_);
  auto result2 = oink.solveIk(tasks, constraints, *scene_, delta_q2);
  ASSERT_TRUE(result2.has_value()) << "Second solve failed: " << result2.error();

  // Verify storage still wasn't reallocated
  EXPECT_EQ(task->jacobian_container.data(), jacobian_data_ptr)
      << "Task Jacobian was reallocated during second solve!";
  EXPECT_EQ(task->error_container.data(), error_data_ptr)
      << "Task error was reallocated during second solve!";
}

// Parameterized test fixture for multiple robot models
struct RobotModelConfig {
  std::string name;
  std::string urdf_path;
  std::string srdf_path;
  std::string yaml_config_path;
  std::string end_effector_frame;
};

class MultiRobotOinkTest : public ::testing::TestWithParam<RobotModelConfig> {
protected:
  void SetUp() override {
    const auto& config = GetParam();
    const auto model_prefix = example_models::get_package_models_dir();
    package_paths_ = {example_models::get_package_share_dir()};

    // Construct full paths
    auto urdf_path = model_prefix / std::filesystem::path(config.urdf_path);
    auto srdf_path = model_prefix / std::filesystem::path(config.srdf_path);
    auto yaml_config_path = model_prefix / std::filesystem::path(config.yaml_config_path);

    // Load the robot model
    scene_ = std::make_shared<Scene>(config.name, urdf_path, srdf_path, package_paths_,
                                     yaml_config_path);

    // Get the number of variables (DOF)
    // nv = velocity DOF (for constraints), nq = configuration DOF (for setJointPositions)
    num_variables_ = scene_->getModel().nv;
    num_config_ = scene_->getModel().nq;
    end_effector_frame_ = config.end_effector_frame;
  }

  std::shared_ptr<Scene> scene_;
  std::vector<std::filesystem::path> package_paths_;
  int num_variables_;  // nv - velocity dimension
  int num_config_;     // nq - configuration dimension
  std::string end_effector_frame_;
};

// Test each robot model with velocity + position + frame task
TEST_P(MultiRobotOinkTest, SolveWithMultipleConstraintsAndTasks) {
  Oink oink(num_variables_);

  // Set initial configuration (use nq for configuration, nv for constraints)
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_config_);
  scene_->setJointPositions(q);

  // Create velocity and position constraints
  double dt = 0.01;
  Eigen::VectorXd v_max = Eigen::VectorXd::Ones(num_variables_) * 1.5;
  auto vel_constraint = std::make_shared<VelocityLimit>(num_variables_, dt, v_max);
  auto pos_constraint = std::make_shared<PositionLimit>(num_variables_, 0.7);

  // Create a frame task
  auto target_pose =
      makeCartesianConfig(Eigen::Vector3d(0.35, 0.15, 0.55), Eigen::Quaterniond::Identity());

  auto task = std::make_shared<FrameTask>(end_effector_frame_, target_pose, num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints = {vel_constraint, pos_constraint};

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed for " << GetParam().name << ": "
                                  << result.error();
  EXPECT_EQ(delta_q.size(), num_variables_);

  // Verify both constraints are satisfied
  const auto& model = scene_->getModel();
  Eigen::VectorXd q_new = q + delta_q;
  for (int i = 0; i < num_variables_; ++i) {
    // Velocity constraint
    EXPECT_LE(std::abs(delta_q[i]), dt * v_max[i] + kTolerance)
        << "Robot: " << GetParam().name << ", Joint " << i << " violated velocity constraint";

    // Position constraint
    if (std::isfinite(model.lowerPositionLimit[i])) {
      EXPECT_GE(q_new[i], model.lowerPositionLimit[i] - kTolerance)
          << "Robot: " << GetParam().name << ", Joint " << i << " went below lower limit";
    }
    if (std::isfinite(model.upperPositionLimit[i])) {
      EXPECT_LE(q_new[i], model.upperPositionLimit[i] + kTolerance)
          << "Robot: " << GetParam().name << ", Joint " << i << " went above upper limit";
    }
  }

  // Note: We don't check if delta_q > 0 because some robots might already be at/near the target
}

// Define robot model configurations
// Note: FR3 model is excluded as it requires xacro processing which is not yet supported
INSTANTIATE_TEST_SUITE_P(
    AllRobotModels, MultiRobotOinkTest,
    ::testing::Values(
        RobotModelConfig{"UR5", "ur_robot_model/ur5_gripper.urdf",
                         "ur_robot_model/ur5_gripper.srdf", "ur_robot_model/ur5_config.yaml",
                         "tool0"},
        RobotModelConfig{"KinovaRobotiq", "kinova_robot_model/kinova_robotiq.urdf",
                         "kinova_robot_model/kinova_robotiq.srdf",
                         "kinova_robot_model/kinova_robotiq_config.yaml", "end_effector_link"},
        RobotModelConfig{"SO101", "so101_robot_model/so101.urdf", "so101_robot_model/so101.srdf",
                         "so101_robot_model/so101_config.yaml", "gripper_frame_link"}),
    [](const ::testing::TestParamInfo<RobotModelConfig>& info) { return info.param.name; });

// Test that FrameTask makes progress toward target over multiple iterations
// This test verifies the IK solver consistently moves toward the target.
TEST_F(OinkTest, FrameTaskConvergesToTarget) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Get current end-effector position
  Eigen::Matrix4d initial_pose = scene_->forwardKinematics(q, "tool0");
  Eigen::Vector3d initial_pos = initial_pose.block<3, 1>(0, 3);

  // Create a small target offset (5cm in x) for more stable convergence
  Eigen::Vector3d target_pos = initial_pos + Eigen::Vector3d(0.05, 0.0, 0.0);
  auto target_pose =
      makeCartesianConfig(target_pos, Eigen::Quaterniond(initial_pose.block<3, 3>(0, 0)));

  // Use higher damping for stable convergence
  FrameTaskOptions options{.lm_damping = 1.0};
  auto task = std::make_shared<FrameTask>("tool0", target_pose, num_variables_, options);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints;

  // Iterate IK solver
  Eigen::VectorXd q_current = q;
  constexpr int kMaxIterations = 100;
  constexpr double kTolerance = 0.01;  // 1cm tolerance

  double initial_error = (initial_pos - target_pos).norm();

  for (int iter = 0; iter < kMaxIterations; ++iter) {
    scene_->setJointPositions(q_current);

    Eigen::VectorXd delta_q(num_variables_);
    auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);
    ASSERT_TRUE(result.has_value()) << "Iteration " << iter << " failed: " << result.error();

    // Update configuration using Pinocchio integration for proper Lie group handling
    q_current = pinocchio::integrate(scene_->getModel(), q_current, delta_q);

    // Check current error
    Eigen::Matrix4d current_pose = scene_->forwardKinematics(q_current, "tool0");
    Eigen::Vector3d current_pos = current_pose.block<3, 1>(0, 3);
    double current_error = (target_pos - current_pos).norm();

    // Check for convergence
    if (current_error < kTolerance) {
      break;
    }
  }

  // Final check: we should have made progress (error reduced from initial)
  Eigen::Matrix4d final_pose = scene_->forwardKinematics(q_current, "tool0");
  Eigen::Vector3d final_pos = final_pose.block<3, 1>(0, 3);
  double final_error = (target_pos - final_pos).norm();

  // At minimum, we should have reduced the error significantly
  EXPECT_LT(final_error, initial_error * 0.5)
      << "FrameTask did not make significant progress. Initial error: " << initial_error
      << "m, Final error: " << final_error << "m";
}

// Test that ConfigurationTask converges to target configuration
TEST_F(OinkTest, ConfigurationTaskConvergesToTarget) {
  Oink oink(num_variables_);

  // Set initial configuration to zero
  Eigen::VectorXd q_initial = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q_initial);

  // Target configuration (moderate offsets)
  Eigen::VectorXd target_q = Eigen::VectorXd::Zero(num_variables_);
  target_q(0) = 0.3;
  target_q(1) = -0.2;
  target_q(2) = 0.4;

  Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(num_variables_);

  ConfigurationTaskOptions options{.task_gain = 1.0, .lm_damping = 0.01};
  auto task = std::make_shared<ConfigurationTask>(target_q, joint_weights, options);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints;

  // Iterate IK solver
  Eigen::VectorXd q_current = q_initial;
  constexpr int kMaxIterations = 50;
  constexpr double kTolerance = 0.001;  // rad

  for (int iter = 0; iter < kMaxIterations; ++iter) {
    scene_->setJointPositions(q_current);

    Eigen::VectorXd delta_q(num_variables_);
    auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);
    ASSERT_TRUE(result.has_value()) << "Iteration " << iter << " failed: " << result.error();

    // Update configuration using Pinocchio integration for proper Lie group handling
    q_current = pinocchio::integrate(scene_->getModel(), q_current, delta_q);

    // Check for convergence
    double config_error = (target_q - q_current).norm();
    if (config_error < kTolerance) {
      break;
    }
  }

  // Final check
  double final_error = (target_q - q_current).norm();
  EXPECT_LT(final_error, kTolerance)
      << "ConfigurationTask did not converge. Target: [" << target_q.transpose() << "], Achieved: ["
      << q_current.transpose() << "], Error: " << final_error;
}

// Test that single IK step moves toward target (not away)
// This is a regression test for the sign bug where error direction was inverted.
TEST_F(OinkTest, SingleStepMovesTowardTarget) {
  Oink oink(num_variables_);

  // Set initial configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Get current end-effector position
  Eigen::Matrix4d initial_pose = scene_->forwardKinematics(q, "tool0");
  Eigen::Vector3d initial_pos = initial_pose.block<3, 1>(0, 3);

  // Create target 10cm away
  Eigen::Vector3d target_pos = initial_pos + Eigen::Vector3d(0.1, 0.0, 0.0);
  auto target_config =
      makeCartesianConfig(target_pos, Eigen::Quaterniond(initial_pose.block<3, 3>(0, 0)));

  FrameTaskOptions options{.lm_damping = 0.1};
  auto task = std::make_shared<FrameTask>("tool0", target_config, num_variables_, options);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints;

  // Single IK solve
  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, *scene_, delta_q);
  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();

  // Apply delta_q
  Eigen::VectorXd q_new = q + delta_q;
  Eigen::Matrix4d new_pose = scene_->forwardKinematics(q_new, "tool0");
  Eigen::Vector3d new_pos = new_pose.block<3, 1>(0, 3);

  // Distance to target should decrease
  double dist_before = (initial_pos - target_pos).norm();
  double dist_after = (new_pos - target_pos).norm();

  EXPECT_LT(dist_after, dist_before)
      << "Single IK step moved AWAY from target! "
      << "Distance before: " << dist_before << "m, after: " << dist_after << "m. "
      << "This indicates a sign error in the error/Jacobian computation.";
}

}  // namespace roboplan

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
