#include <gtest/gtest.h>
#include <limits>
#include <memory>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/barriers/position_barrier.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/frame.hpp>

namespace {
constexpr double kTolerance = 1e-3;

roboplan::CartesianConfiguration makeCartesianConfig(const std::string& frame_name,
                                                     const Eigen::Vector3d& position,
                                                     const Eigen::Quaterniond& orientation) {
  roboplan::CartesianConfiguration config;
  config.tip_frame = frame_name;
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform.block<3, 3>(0, 0) = orientation.toRotationMatrix();
  tform.block<3, 1>(0, 3) = position;
  config.tform = tform;
  return config;
}
}  // namespace

namespace roboplan {

class PositionBarrierTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    urdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    srdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    package_paths_ = {example_models::get_package_share_dir()};
    yaml_config_path_ = model_prefix / "ur_robot_model" / "ur5_config.yaml";
    scene_ = std::make_shared<Scene>("test_scene", urdf_path_, srdf_path_, package_paths_,
                                     yaml_config_path_);

    num_variables_ = scene_->getModel().nv;
    dt_ = 0.01;  // 100 Hz control loop
  }

  std::shared_ptr<Scene> scene_;
  std::filesystem::path urdf_path_;
  std::filesystem::path srdf_path_;
  std::vector<std::filesystem::path> package_paths_;
  std::filesystem::path yaml_config_path_;
  int num_variables_;
  double dt_;
};

// Test basic construction with full box
TEST_F(PositionBarrierTest, ConstructionFullBox) {
  Eigen::Vector3d p_min(-1.0, -1.0, 0.0);
  Eigen::Vector3d p_max(1.0, 1.0, 2.0);

  auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_);

  // Full box creates 6 constraints (2 per axis: lower and upper bound)
  EXPECT_EQ(barrier->getNumBarriers(*scene_), 6);
  EXPECT_DOUBLE_EQ(barrier->gain, 1.0);
  EXPECT_DOUBLE_EQ(barrier->dt, dt_);
}

// Test selective axis constraint
TEST_F(PositionBarrierTest, SelectiveAxisConstraint) {
  // Only constrain Z axis (keep end-effector above table)
  Eigen::Vector3d p_min(-std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity(), 0.1);
  Eigen::Vector3d p_max(std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity());

  roboplan::ConstraintAxisSelection axes;
  axes.x = false;
  axes.y = false;
  axes.z = true;

  auto barrier =
      std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_, axes);

  // Only 1 constraint (lower bound on z; upper bound is +inf so not counted)
  EXPECT_EQ(barrier->getNumBarriers(*scene_), 1);
}

// Test barrier value computation
TEST_F(PositionBarrierTest, BarrierValueComputation) {
  // Set robot to known configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  Eigen::Vector3d current_pos =
      scene_->getData().oMf[scene_->getModel().getFrameId("tool0")].translation();

  // Create barrier where current position is safely inside the box
  Eigen::Vector3d p_min = current_pos.array() - 0.5;
  Eigen::Vector3d p_max = current_pos.array() + 0.5;

  auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_);

  auto result = barrier->computeBarrier(*scene_);
  ASSERT_TRUE(result.has_value()) << "computeBarrier failed: " << result.error();

  // All barrier values should be positive (safe)
  EXPECT_TRUE((barrier->barrier_values.array() >= 0.0).all());
  EXPECT_GT(barrier->barrier_values.minCoeff(), 0.0);
}

// Test that barrier limits motion toward boundary
TEST_F(PositionBarrierTest, BarrierLimitsMotion) {
  // Start from zero configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  // Get current EE position
  Eigen::Matrix4d current_pose = scene_->forwardKinematics(q, "tool0");
  Eigen::Vector3d current_pos = current_pose.block<3, 1>(0, 3);

  // Create a barrier that sets a z floor 10cm below current position
  // This ensures the EE starts safely inside the barrier
  double z_floor = current_pos[2] - 0.10;
  Eigen::Vector3d p_min(-2.0, -2.0, z_floor);
  Eigen::Vector3d p_max(2.0, 2.0, 2.0);

  // Use a high barrier gain to strongly discourage crossing the boundary
  auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                                   roboplan::ConstraintAxisSelection(), 50.0);

  // Verify we start inside the safe region
  auto compute_result = barrier->computeBarrier(*scene_);
  ASSERT_TRUE(compute_result.has_value());
  ASSERT_TRUE((barrier->barrier_values.array() >= 0.0).all()) << "Must start inside safe region";

  // Create a frame task that tries to move down (below the barrier)
  Eigen::Vector3d target_pos = current_pos;
  target_pos[2] = z_floor - 0.3;  // 30cm below floor, which would violate barrier
  auto target_config =
      makeCartesianConfig("tool0", target_pos, Eigen::Quaterniond(current_pose.block<3, 3>(0, 0)));

  // Use lower task gain so barrier has more influence
  FrameTaskOptions params{.task_gain = 0.5, .lm_damping = 0.1};
  auto frame_task = std::make_shared<FrameTask>(target_config, num_variables_, params);

  // Velocity limit to ensure smooth motion
  Eigen::VectorXd v_max = Eigen::VectorXd::Constant(num_variables_, 1.0);
  auto vel_limit = std::make_shared<VelocityLimit>(num_variables_, dt_, v_max);

  Oink oink(num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {frame_task};
  std::vector<std::shared_ptr<Constraints>> constraints = {vel_limit};
  std::vector<std::shared_ptr<Barrier>> barriers = {barrier};

  // Run IK loop
  Eigen::VectorXd q_current = q;
  constexpr int kMaxIterations = 100;

  for (int iter = 0; iter < kMaxIterations; ++iter) {
    scene_->setJointPositions(q_current);
    scene_->forwardKinematics(q_current, "tool0");

    Eigen::VectorXd delta_q(num_variables_);
    auto result = oink.solveIk(tasks, constraints, barriers, *scene_, delta_q);
    ASSERT_TRUE(result.has_value()) << "IK failed at iteration " << iter << ": " << result.error();

    // Integrate
    q_current = pinocchio::integrate(scene_->getModel(), q_current, delta_q);
  }

  // At the end, the EE should not have reached the target 30cm below the floor
  // The barrier should have significantly limited downward motion
  scene_->setJointPositions(q_current);
  Eigen::Vector3d final_pos =
      scene_->getData().oMf[scene_->getModel().getFrameId("tool0")].translation();

  // The barrier is a soft constraint in the QP formulation.
  // With high barrier gain, the EE should stay close to or above the floor.
  // We don't expect perfect hard constraint behavior, but significant limitation.
  EXPECT_GT(final_pos[2], z_floor - 0.05)
      << "EE should stay close to barrier floor, final z=" << final_pos[2] << ", floor=" << z_floor;

  // The EE should definitely not reach the target which is 30cm below
  EXPECT_GT(final_pos[2], target_pos[2] + 0.15)
      << "Barrier should prevent reaching target far below floor";
}

// Test barrier allows safe motion
TEST_F(PositionBarrierTest, BarrierAllowsSafeMotion) {
  // Start from zero configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  // Get current EE position
  Eigen::Matrix4d current_pose = scene_->forwardKinematics(q, "tool0");
  Eigen::Vector3d current_pos = current_pose.block<3, 1>(0, 3);

  // Create a large barrier box that should not restrict motion much
  // Use low safe_displacement_gain (0.1) since we're testing that the barrier allows motion
  Eigen::Vector3d p_min(-2.0, -2.0, -0.5);
  Eigen::Vector3d p_max(2.0, 2.0, 2.0);

  auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                                   roboplan::ConstraintAxisSelection(), 1.0, 0.1);

  // Create a frame task to move to a position inside the safe region
  Eigen::Vector3d target_pos = current_pos + Eigen::Vector3d(0.1, 0.0, 0.0);  // 10cm in x
  auto target_config =
      makeCartesianConfig("tool0", target_pos, Eigen::Quaterniond(current_pose.block<3, 3>(0, 0)));

  FrameTaskOptions params{.lm_damping = 0.1};
  auto frame_task = std::make_shared<FrameTask>(target_config, num_variables_, params);

  Oink oink(num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {frame_task};
  std::vector<std::shared_ptr<Constraints>> constraints;
  std::vector<std::shared_ptr<Barrier>> barriers = {barrier};

  // Run IK loop
  Eigen::VectorXd q_current = q;
  constexpr int kMaxIterations = 100;
  constexpr double kPositionTolerance = 0.02;  // 2cm

  for (int iter = 0; iter < kMaxIterations; ++iter) {
    scene_->setJointPositions(q_current);
    scene_->forwardKinematics(q_current, "tool0");

    Eigen::VectorXd delta_q(num_variables_);
    auto result = oink.solveIk(tasks, constraints, barriers, *scene_, delta_q);
    ASSERT_TRUE(result.has_value()) << "IK failed at iteration " << iter;

    q_current = pinocchio::integrate(scene_->getModel(), q_current, delta_q);

    // Check convergence
    Eigen::Matrix4d final_pose = scene_->forwardKinematics(q_current, "tool0");
    Eigen::Vector3d final_pos = final_pose.block<3, 1>(0, 3);
    if ((final_pos - target_pos).norm() < kPositionTolerance) {
      break;
    }
  }

  // Should reach target (barrier should not prevent safe motion)
  scene_->setJointPositions(q_current);
  Eigen::Matrix4d final_pose = scene_->forwardKinematics(q_current, "tool0");
  Eigen::Vector3d final_pos = final_pose.block<3, 1>(0, 3);

  EXPECT_LT((final_pos - target_pos).norm(), kPositionTolerance)
      << "Failed to reach target. Target: [" << target_pos.transpose() << "], Final: ["
      << final_pos.transpose() << "]";
}

// Test invalid frame name
TEST_F(PositionBarrierTest, InvalidFrameName) {
  Eigen::Vector3d p_min(-1.0, -1.0, 0.0);
  Eigen::Vector3d p_max(1.0, 1.0, 2.0);

  auto barrier =
      std::make_shared<PositionBarrier>("nonexistent_frame", p_min, p_max, num_variables_, dt_);

  auto result = barrier->computeBarrier(*scene_);
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("Frame not found") != std::string::npos);
}

// Test invalid gain
TEST_F(PositionBarrierTest, InvalidGain) {
  Eigen::Vector3d p_min(-1.0, -1.0, 0.0);
  Eigen::Vector3d p_max(1.0, 1.0, 2.0);

  EXPECT_THROW(
      {
        auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                                         roboplan::ConstraintAxisSelection(), 0.0);
      },
      std::invalid_argument);

  EXPECT_THROW(
      {
        auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                                         roboplan::ConstraintAxisSelection(), -1.0);
      },
      std::invalid_argument);
}

// Test invalid dt
TEST_F(PositionBarrierTest, InvalidDt) {
  Eigen::Vector3d p_min(-1.0, -1.0, 0.0);
  Eigen::Vector3d p_max(1.0, 1.0, 2.0);

  EXPECT_THROW(
      {
        auto barrier =
            std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, 0.0);
      },
      std::invalid_argument);

  EXPECT_THROW(
      {
        auto barrier =
            std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, -0.01);
      },
      std::invalid_argument);
}

// Test invalid bounds (p_min >= p_max)
TEST_F(PositionBarrierTest, InvalidBounds) {
  // Test p_min == p_max on X axis
  EXPECT_THROW(
      {
        Eigen::Vector3d p_min(0.5, -1.0, 0.0);
        Eigen::Vector3d p_max(0.5, 1.0, 2.0);  // x: 0.5 == 0.5
        auto barrier =
            std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_);
      },
      std::invalid_argument);

  // Test p_min > p_max on Y axis
  EXPECT_THROW(
      {
        Eigen::Vector3d p_min(-1.0, 1.0, 0.0);
        Eigen::Vector3d p_max(1.0, -1.0, 2.0);  // y: 1.0 > -1.0
        auto barrier =
            std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_);
      },
      std::invalid_argument);

  // Test that infinite bounds are allowed (no validation needed)
  EXPECT_NO_THROW({
    Eigen::Vector3d p_min(-std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity(), 0.0);
    Eigen::Vector3d p_max(std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity(), 2.0);
    auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_);
  });

  // Test that disabled axes don't trigger validation
  EXPECT_NO_THROW({
    Eigen::Vector3d p_min(1.0, -1.0, 0.0);  // x: 1.0 > -1.0 (invalid if enabled)
    Eigen::Vector3d p_max(-1.0, 1.0, 2.0);
    roboplan::ConstraintAxisSelection axes;
    axes.x = false;  // Disable X axis, so invalid bounds are ignored
    auto barrier =
        std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_, axes);
  });
}

// Test QP inequality computation
TEST_F(PositionBarrierTest, QpInequalityComputation) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  // Get EE position and create a box that safely contains it
  Eigen::Vector3d ee_pos =
      scene_->getData().oMf[scene_->getModel().getFrameId("tool0")].translation();
  Eigen::Vector3d p_min = ee_pos.array() - 0.5;
  Eigen::Vector3d p_max = ee_pos.array() + 0.5;

  auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                                   roboplan::ConstraintAxisSelection(), 5.0);

  int num_barriers = barrier->getNumBarriers(*scene_);
  Eigen::MatrixXd G(num_barriers, num_variables_);
  Eigen::VectorXd h(num_barriers);

  auto result = barrier->computeQpInequalities(*scene_, G, h);
  ASSERT_TRUE(result.has_value()) << "computeQpInequalities failed: " << result.error();

  // Verify dimensions
  EXPECT_EQ(G.rows(), num_barriers);
  EXPECT_EQ(G.cols(), num_variables_);
  EXPECT_EQ(h.size(), num_barriers);

  // h should be positive when safe (gain * barrier_values)
  // Since we centered the box on the EE, all h values should be positive
  EXPECT_TRUE((h.array() > 0.0).all())
      << "h values should be positive when safe: " << h.transpose();
}

// Test solving with empty barriers list
TEST_F(PositionBarrierTest, SolveWithEmptyBarriers) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  auto target_pose =
      makeCartesianConfig("tool0", Eigen::Vector3d(0.3, 0.2, 0.5), Eigen::Quaterniond::Identity());
  auto task = std::make_shared<FrameTask>(target_pose, num_variables_);

  Oink oink(num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {task};
  std::vector<std::shared_ptr<Constraints>> constraints;
  std::vector<std::shared_ptr<Barrier>> barriers;  // Empty

  Eigen::VectorXd delta_q(num_variables_);
  auto result = oink.solveIk(tasks, constraints, barriers, *scene_, delta_q);

  ASSERT_TRUE(result.has_value()) << "Solve failed: " << result.error();
  EXPECT_EQ(delta_q.size(), num_variables_);
}

// Test saturating class-K function (always used)
TEST_F(PositionBarrierTest, SaturatingClassKFunction) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  // Get EE position and create a box that safely contains it
  Eigen::Vector3d ee_pos =
      scene_->getData().oMf[scene_->getModel().getFrameId("tool0")].translation();
  Eigen::Vector3d p_min = ee_pos.array() - 0.5;
  Eigen::Vector3d p_max = ee_pos.array() + 0.5;

  // Create barrier (uses saturating class-K)
  auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                                   roboplan::ConstraintAxisSelection(), 5.0);

  int num_barriers = barrier->getNumBarriers(*scene_);
  Eigen::MatrixXd G(num_barriers, num_variables_);
  Eigen::VectorXd b(num_barriers);

  auto result = barrier->computeQpInequalities(*scene_, G, b);
  ASSERT_TRUE(result.has_value()) << "computeQpInequalities failed: " << result.error();

  // Verify b is positive when safe (h > 0)
  // For safe position, h_i > 0, so b should be positive
  EXPECT_TRUE((b.array() > 0.0).all()) << "b values should be positive when safe";

  // Verify saturating formula: b = gain * h / (1 + |h|)
  auto barrier_result = barrier->computeBarrier(*scene_);
  ASSERT_TRUE(barrier_result.has_value());

  for (int i = 0; i < num_barriers; ++i) {
    double h_i = barrier->barrier_values[i];
    double saturating_value = 5.0 * h_i / (1.0 + std::abs(h_i));
    EXPECT_NEAR(b[i], saturating_value, 1e-10)
        << "Saturating class-K should match theoretical value at index " << i;
  }
}

// Test safe displacement regularization
TEST_F(PositionBarrierTest, SafeDisplacementRegularization) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  Eigen::Vector3d ee_pos =
      scene_->getData().oMf[scene_->getModel().getFrameId("tool0")].translation();
  Eigen::Vector3d p_min = ee_pos.array() - 0.5;
  Eigen::Vector3d p_max = ee_pos.array() + 0.5;

  // Create barrier with custom safe displacement gain
  double safe_disp_gain = 3.0;
  auto barrier =
      std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                        roboplan::ConstraintAxisSelection(), 5.0, safe_disp_gain);

  EXPECT_DOUBLE_EQ(barrier->safe_displacement_gain, safe_disp_gain);

  // Compute QP objective contribution
  Eigen::MatrixXd H(num_variables_, num_variables_);
  Eigen::VectorXd c(num_variables_);

  auto result = barrier->computeQpObjective(*scene_, H, c);
  ASSERT_TRUE(result.has_value()) << "computeQpObjective failed: " << result.error();

  // H should be diagonal (identity scaled by weight)
  // c should be zero for default safe displacement (zero vector)
  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_GT(H(i, i), 0.0) << "Diagonal should be positive";
    for (int j = 0; j < num_variables_; ++j) {
      if (i != j) {
        EXPECT_NEAR(H(i, j), 0.0, 1e-10) << "Off-diagonal should be zero";
      }
    }
  }

  // Default safe displacement is zero, so c should be zero
  EXPECT_LT(c.norm(), 1e-10) << "c should be zero for default safe displacement";
}

// Test solver with barriers (saturating + regularization)
TEST_F(PositionBarrierTest, SolverWithBarrier) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  Eigen::Matrix4d current_pose = scene_->forwardKinematics(q, "tool0");
  Eigen::Vector3d current_pos = current_pose.block<3, 1>(0, 3);

  // Create floor barrier
  double z_floor = current_pos[2] - 0.10;
  Eigen::Vector3d p_min(-2.0, -2.0, z_floor);
  Eigen::Vector3d p_max(2.0, 2.0, 2.0);

  auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                                   roboplan::ConstraintAxisSelection(), 50.0);

  // Create task to move down (toward barrier)
  Eigen::Vector3d target_pos = current_pos;
  target_pos[2] = z_floor - 0.3;
  auto target_config =
      makeCartesianConfig("tool0", target_pos, Eigen::Quaterniond(current_pose.block<3, 3>(0, 0)));

  auto frame_task = std::make_shared<FrameTask>(
      target_config, num_variables_, FrameTaskOptions{.task_gain = 0.5, .lm_damping = 0.1});

  Oink oink(num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {frame_task};
  std::vector<std::shared_ptr<Constraints>> constraints;
  std::vector<std::shared_ptr<Barrier>> barriers = {barrier};

  Eigen::VectorXd q_current = q;
  constexpr int kMaxIterations = 50;

  for (int iter = 0; iter < kMaxIterations; ++iter) {
    scene_->setJointPositions(q_current);
    scene_->forwardKinematics(q_current, "tool0");

    Eigen::VectorXd delta_q(num_variables_);
    auto result = oink.solveIk(tasks, constraints, barriers, *scene_, delta_q);
    ASSERT_TRUE(result.has_value()) << "IK failed at iteration " << iter << ": " << result.error();

    q_current = pinocchio::integrate(scene_->getModel(), q_current, delta_q);
  }

  scene_->setJointPositions(q_current);
  Eigen::Vector3d final_pos =
      scene_->getData().oMf[scene_->getModel().getFrameId("tool0")].translation();

  // Barrier should limit motion
  EXPECT_GT(final_pos[2], z_floor - 0.05) << "Barrier should limit motion";
}

// ============================================================================
// HARD CONSTRAINT TESTS
// ============================================================================

// Test safety margin parameter
TEST_F(PositionBarrierTest, SafetyMarginParameter) {
  Eigen::Vector3d p_min(-1.0, -1.0, 0.0);
  Eigen::Vector3d p_max(1.0, 1.0, 2.0);

  double safety_margin = 0.05;
  auto barrier = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                                   roboplan::ConstraintAxisSelection(), 1.0, 1.0,
                                                   safety_margin);

  EXPECT_DOUBLE_EQ(barrier->safety_margin, safety_margin);
  EXPECT_DOUBLE_EQ(barrier->safe_displacement_gain, 1.0);
}

// Test invalid safety margin
TEST_F(PositionBarrierTest, InvalidSafetyMargin) {
  Eigen::Vector3d p_min(-1.0, -1.0, 0.0);
  Eigen::Vector3d p_max(1.0, 1.0, 2.0);

  EXPECT_THROW(
      {
        auto barrier =
            std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                              roboplan::ConstraintAxisSelection(), 1.0, 1.0, -0.1);
      },
      std::invalid_argument);
}

// Test safety margin makes constraint more conservative
TEST_F(PositionBarrierTest, SafetyMarginTightensConstraint) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  Eigen::Vector3d ee_pos =
      scene_->getData().oMf[scene_->getModel().getFrameId("tool0")].translation();
  Eigen::Vector3d p_min = ee_pos.array() - 0.5;
  Eigen::Vector3d p_max = ee_pos.array() + 0.5;

  // Create barriers with and without safety margin
  auto barrier_no_margin =
      std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                        roboplan::ConstraintAxisSelection(), 5.0, 1.0, 0.0);
  auto barrier_with_margin =
      std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                        roboplan::ConstraintAxisSelection(), 5.0, 1.0, 0.1);

  int num_barriers = barrier_no_margin->getNumBarriers(*scene_);
  Eigen::MatrixXd G_no(num_barriers, num_variables_);
  Eigen::VectorXd b_no(num_barriers);
  Eigen::MatrixXd G_with(num_barriers, num_variables_);
  Eigen::VectorXd b_with(num_barriers);

  auto result_no = barrier_no_margin->computeQpInequalities(*scene_, G_no, b_no);
  auto result_with = barrier_with_margin->computeQpInequalities(*scene_, G_with, b_with);
  ASSERT_TRUE(result_no.has_value());
  ASSERT_TRUE(result_with.has_value());

  // With safety margin, b values should be smaller (more conservative constraint)
  for (int i = 0; i < num_barriers; ++i) {
    EXPECT_LT(b_with[i], b_no[i])
        << "Safety margin should make constraint more conservative at index " << i;
  }
}

// Test backward compatibility: default parameters preserve old behavior
TEST_F(PositionBarrierTest, BackwardCompatibility) {
  Eigen::Vector3d p_min(-1.0, -1.0, 0.0);
  Eigen::Vector3d p_max(1.0, 1.0, 2.0);

  // Old-style construction (without safety_margin)
  auto barrier_old = std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_);

  // New-style construction with default safety_margin
  auto barrier_new =
      std::make_shared<PositionBarrier>("tool0", p_min, p_max, num_variables_, dt_,
                                        roboplan::ConstraintAxisSelection(), 1.0, 1.0, 0.0);

  EXPECT_DOUBLE_EQ(barrier_old->safety_margin, 0.0);
  EXPECT_DOUBLE_EQ(barrier_new->safety_margin, 0.0);
  EXPECT_DOUBLE_EQ(barrier_old->gain, barrier_new->gain);
  EXPECT_DOUBLE_EQ(barrier_old->dt, barrier_new->dt);
}

// ============================================================================
// LINEARIZATION ERROR TESTS (Large Jumps)
// ============================================================================

/**
 * Documents that barrier CAN be violated without enforceBarriers().
 *
 * When commanding a large target jump outside the barrier and starting near the boundary,
 * the linearized CBF constraint G·δq ≤ b can be satisfied while the actual barrier h(q + δq)
 * is violated. This happens because:
 *
 * 1. The linearization h(q + δq) ≈ h(q) + J_h·δq has O(||δq||²) error
 * 2. Starting near boundary (small h) with large δq maximizes this error
 * 3. Once h < 0 the barrier becomes ineffective
 *
 * This test verifies the violation occurs, documenting the need for enforceBarriers().
 */
TEST_F(PositionBarrierTest, BarrierCanBeViolatedWithoutEnforcement) {
  // Use a configuration that places the EE in a good test position
  Eigen::VectorXd q(num_variables_);
  q << 0.0, -1.5, 1.5, -1.5, -1.5, 0.0;

  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  Eigen::Matrix4d current_pose = scene_->forwardKinematics(q, "tool0");
  Eigen::Vector3d current_pos = current_pose.block<3, 1>(0, 3);
  Eigen::Quaterniond current_orientation(current_pose.block<3, 3>(0, 0));

  // Create small barrier box with robot starting near the +X boundary
  constexpr double barrier_box_size = 0.2;
  const double half_size = barrier_box_size / 2.0;

  Eigen::Vector3d barrier_center = current_pos;
  barrier_center[0] -= (half_size - 0.02);  // Start 2cm from +X boundary

  const Eigen::Vector3d p_min = barrier_center.array() - half_size;
  const Eigen::Vector3d p_max = barrier_center.array() + half_size;

  auto barrier = std::make_shared<PositionBarrier>(
      "tool0", p_min, p_max, num_variables_, dt_, roboplan::ConstraintAxisSelection(),
      /*gain=*/5.0, /*safe_displacement_gain=*/1.0, /*safety_margin=*/0.01);

  auto compute_result = barrier->computeBarrier(*scene_);
  ASSERT_TRUE(compute_result.has_value());
  ASSERT_TRUE((barrier->barrier_values.array() >= 0.0).all()) << "Must start inside safe region";

  // Command large jump outside barrier
  Eigen::Vector3d target_pos = current_pos;
  target_pos[0] += 0.5;  // 0.5m jump (barrier is only 0.02m away)

  auto target_config = makeCartesianConfig("tool0", target_pos, current_orientation);

  FrameTaskOptions task_params{.position_cost = 1.0,
                               .orientation_cost = 0.1,
                               .task_gain = 2.0,
                               .lm_damping = 0.01,
                               .max_position_error = std::numeric_limits<double>::infinity(),
                               .max_rotation_error = std::numeric_limits<double>::infinity()};

  auto frame_task = std::make_shared<FrameTask>(target_config, num_variables_, task_params);

  Eigen::VectorXd v_max = Eigen::VectorXd::Constant(num_variables_, 1.5);
  auto vel_limit = std::make_shared<VelocityLimit>(num_variables_, dt_, v_max);

  Oink oink(num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {frame_task};
  std::vector<std::shared_ptr<Constraints>> constraints = {vel_limit};
  std::vector<std::shared_ptr<Barrier>> barriers = {barrier};

  Eigen::VectorXd q_current = q;
  bool barrier_violated = false;
  double min_barrier_value_seen = barrier->barrier_values.minCoeff();

  for (int iter = 0; iter < 100; ++iter) {
    scene_->setJointPositions(q_current);
    scene_->forwardKinematics(q_current, "tool0");

    compute_result = barrier->computeBarrier(*scene_);
    ASSERT_TRUE(compute_result.has_value());

    const double min_barrier = barrier->barrier_values.minCoeff();
    min_barrier_value_seen = std::min(min_barrier_value_seen, min_barrier);

    if (!barrier_violated && min_barrier < -0.001) {
      barrier_violated = true;
    }

    Eigen::VectorXd delta_q(num_variables_);
    auto result = oink.solveIk(tasks, constraints, barriers, *scene_, delta_q, 1e-6);
    ASSERT_TRUE(result.has_value());

    q_current = pinocchio::integrate(scene_->getModel(), q_current, delta_q);
  }

  // This test documents that barrier CAN be violated without enforceBarriers()
  EXPECT_TRUE(barrier_violated) << "Without enforceBarriers(), the barrier should be violated. "
                                << "Min barrier: " << min_barrier_value_seen;
}

/**
 * Verifies that enforceBarriers() prevents barrier violations.
 *
 * Uses identical conditions to the previous test, but calls enforceBarriers() after each
 * solveIk() to validate the solution using forward kinematics. If a violation would occur,
 * delta_q is zeroed to prevent unsafe motion.
 */
TEST_F(PositionBarrierTest, EnforceBarriersPreventsViolation) {
  Eigen::VectorXd q(num_variables_);
  q << 0.0, -1.5, 1.5, -1.5, -1.5, 0.0;

  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  Eigen::Matrix4d current_pose = scene_->forwardKinematics(q, "tool0");
  Eigen::Vector3d current_pos = current_pose.block<3, 1>(0, 3);
  Eigen::Quaterniond current_orientation(current_pose.block<3, 3>(0, 0));

  constexpr double barrier_box_size = 0.2;
  const double half_size = barrier_box_size / 2.0;

  Eigen::Vector3d barrier_center = current_pos;
  barrier_center[0] -= (half_size - 0.02);

  const Eigen::Vector3d p_min = barrier_center.array() - half_size;
  const Eigen::Vector3d p_max = barrier_center.array() + half_size;

  auto barrier = std::make_shared<PositionBarrier>(
      "tool0", p_min, p_max, num_variables_, dt_, roboplan::ConstraintAxisSelection(),
      /*gain=*/5.0, /*safe_displacement_gain=*/1.0, /*safety_margin=*/0.01);

  auto compute_result = barrier->computeBarrier(*scene_);
  ASSERT_TRUE(compute_result.has_value());
  ASSERT_TRUE((barrier->barrier_values.array() >= 0.0).all()) << "Must start inside safe region";

  Eigen::Vector3d target_pos = current_pos;
  target_pos[0] += 0.5;

  auto target_config = makeCartesianConfig("tool0", target_pos, current_orientation);

  FrameTaskOptions task_params{.position_cost = 1.0,
                               .orientation_cost = 0.1,
                               .task_gain = 2.0,
                               .lm_damping = 0.01,
                               .max_position_error = std::numeric_limits<double>::infinity(),
                               .max_rotation_error = std::numeric_limits<double>::infinity()};

  auto frame_task = std::make_shared<FrameTask>(target_config, num_variables_, task_params);

  Eigen::VectorXd v_max = Eigen::VectorXd::Constant(num_variables_, 1.5);
  auto vel_limit = std::make_shared<VelocityLimit>(num_variables_, dt_, v_max);

  Oink oink(num_variables_);
  std::vector<std::shared_ptr<Task>> tasks = {frame_task};
  std::vector<std::shared_ptr<Constraints>> constraints = {vel_limit};
  std::vector<std::shared_ptr<Barrier>> barriers = {barrier};

  Eigen::VectorXd q_current = q;
  bool barrier_violated = false;
  double min_barrier_value_seen = barrier->barrier_values.minCoeff();

  for (int iter = 0; iter < 100; ++iter) {
    scene_->setJointPositions(q_current);
    scene_->forwardKinematics(q_current, "tool0");

    compute_result = barrier->computeBarrier(*scene_);
    ASSERT_TRUE(compute_result.has_value());

    const double min_barrier = barrier->barrier_values.minCoeff();
    min_barrier_value_seen = std::min(min_barrier_value_seen, min_barrier);

    if (min_barrier < -0.001) {
      barrier_violated = true;
    }

    Eigen::VectorXd delta_q(num_variables_);
    auto result = oink.solveIk(tasks, constraints, barriers, *scene_, delta_q, 1e-6);
    ASSERT_TRUE(result.has_value());

    // KEY: Call enforceBarriers() to validate solution using FK
    auto enforce_result = oink.enforceBarriers(barriers, *scene_, delta_q, 0.0);
    ASSERT_TRUE(enforce_result.has_value()) << "enforceBarriers failed: " << enforce_result.error();

    q_current = pinocchio::integrate(scene_->getModel(), q_current, delta_q);
  }

  // Final check
  scene_->setJointPositions(q_current);
  scene_->forwardKinematics(q_current, "tool0");
  compute_result = barrier->computeBarrier(*scene_);
  ASSERT_TRUE(compute_result.has_value());
  const double final_min_barrier = barrier->barrier_values.minCoeff();

  EXPECT_FALSE(barrier_violated) << "With enforceBarriers(), barrier should never be violated. "
                                 << "Min value: " << min_barrier_value_seen;

  EXPECT_GE(final_min_barrier, -kTolerance)
      << "Final barrier should be non-negative. Min: " << final_min_barrier;
}

}  // namespace roboplan

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
