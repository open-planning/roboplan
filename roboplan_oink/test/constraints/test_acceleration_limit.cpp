#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/constraints/acceleration_limit.hpp>
#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/frame.hpp>
#include <test_utils.hpp>

namespace roboplan {

class AccelerationLimitTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    urdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    srdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    package_paths_ = {example_models::get_package_share_dir()};
    yaml_config_path_ = model_prefix / "ur_robot_model" / "ur5_config.yaml";

    scene_ = std::make_shared<Scene>("test_scene", loadUrdfSceneDescription(urdf_path_, srdf_path_),
                                     package_paths_, yaml_config_path_);
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
  EXPECT_EQ(constraint.delta_q_prev.size(), num_variables_);
  EXPECT_TRUE(constraint.delta_q_prev.isApprox(Eigen::VectorXd::Zero(num_variables_)));
}

TEST_F(AccelerationLimitTest, GetNumConstraints) {
  AccelerationLimit constraint(*oink_, 0.01, Eigen::VectorXd::Ones(num_variables_) * 5.0);
  EXPECT_EQ(constraint.getNumConstraints(posed(*oink_, *scene_)), num_variables_);
}

TEST_F(AccelerationLimitTest, ConstraintMatrixIsIdentity) {
  AccelerationLimit constraint(*oink_, 0.01, Eigen::VectorXd::Ones(num_variables_) * 5.0);

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());

  EXPECT_TRUE(G.isApprox(Eigen::MatrixXd::Identity(num_variables_, num_variables_)));
}

// From rest (delta_q_prev = 0), the bound should be the symmetric acceleration box a_max*dt²
// (assuming the braking-distance term is looser, which holds at this mid-range configuration).
TEST_F(AccelerationLimitTest, BoundsFromRest) {
  double dt = 0.01;
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_) * 5.0;
  AccelerationLimit constraint(*oink_, dt, a_max);

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());

  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_NEAR(upper(i), a_max(i) * dt * dt, 1e-12);
    EXPECT_NEAR(lower(i), -a_max(i) * dt * dt, 1e-12);
  }
}

// The acceleration box is centered on the previous displacement: with delta_q_prev = d,
// the admissible displacement is [d - a_max*dt², d + a_max*dt²].
TEST_F(AccelerationLimitTest, BoundsCenteredOnPreviousDisplacement) {
  double dt = 0.01;
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_) * 5.0;
  AccelerationLimit constraint(*oink_, dt, a_max);

  // Previous velocity of 0.5 rad/s on every joint -> delta_q_prev = 0.5 * dt.
  Eigen::VectorXd v_prev = Eigen::VectorXd::Constant(num_variables_, 0.5);
  constraint.setLastVelocity(v_prev);
  EXPECT_TRUE(constraint.delta_q_prev.isApprox(v_prev * dt));

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());

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
  ASSERT_FALSE(constraint.delta_q_prev.isApprox(Eigen::VectorXd::Zero(num_variables_)));

  constraint.reset();
  EXPECT_TRUE(constraint.delta_q_prev.isApprox(Eigen::VectorXd::Zero(num_variables_)));
}

// An infinite acceleration limit leaves the joint unconstrained (bounds pass through as
// +/- infinity, which the QP solver treats as unbounded rows).
TEST_F(AccelerationLimitTest, InfiniteLimitIsUnconstrained) {
  Eigen::VectorXd a_max =
      Eigen::VectorXd::Constant(num_variables_, std::numeric_limits<double>::infinity());
  AccelerationLimit constraint(*oink_, 0.01, a_max);

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());

  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_TRUE(std::isinf(upper(i)) && upper(i) > 0.0);
    EXPECT_TRUE(std::isinf(lower(i)) && lower(i) < 0.0);
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
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());

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
  auto result = constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper);

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("size mismatch") != std::string::npos);
}

// ---------------------------------------------------------------------------------------
// Braking distance to the task target
// ---------------------------------------------------------------------------------------

// No target set (the default) means no target braking: the bounds are the plain Pink ones.
TEST_F(AccelerationLimitTest, NoTargetSetLeavesBoundsUnchanged) {
  const double dt = 0.01;
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_) * 5.0;
  AccelerationLimit constraint(*oink_, dt, a_max);
  ASSERT_FALSE(constraint.delta_q_target.has_value());

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());

  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_NEAR(upper(i), a_max(i) * dt * dt, 1e-12);
    EXPECT_NEAR(lower(i), -a_max(i) * dt * dt, 1e-12);
  }
}

// A nearby target bounds the step by dt*sqrt(2*a_max*|d|) on the side facing the target, and
// leaves the trailing side at the acceleration box edge. Starting from rest keeps the whole
// braking bound reachable, so it applies unclamped.
TEST_F(AccelerationLimitTest, TargetBrakingTightensSideFacingTarget) {
  const double dt = 0.01;
  const double a = 5.0;
  Eigen::VectorXd a_max = Eigen::VectorXd::Constant(num_variables_, a);
  AccelerationLimit constraint(*oink_, dt, a_max);

  // From rest the acceleration box is [-5e-4, 5e-4]. A target 0.1 mrad ahead gives a braking
  // bound of dt*sqrt(2*5*1e-4) = 3.16e-4, which is tighter than the box but still reachable.
  const double d = 1e-4;
  constraint.setTargetDisplacement(Eigen::VectorXd::Constant(num_variables_, d));

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());

  const double brake = dt * std::sqrt(2.0 * a * d);
  const double accel_box = a * dt * dt;
  ASSERT_LT(brake, accel_box) << "test setup: braking bound should be the tighter one";
  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_NEAR(upper(i), brake, 1e-12);
    // The trailing side is untouched: it stays the acceleration box's lower edge.
    EXPECT_NEAR(lower(i), -accel_box, 1e-12);
  }
}

// The braking bound may not demand a deceleration beyond a_max. A joint already travelling at
// 1 rad/s cannot drop to the braking bound in one step, so the bound is clamped to the slowest
// displacement the acceleration limit can reach this step.
TEST_F(AccelerationLimitTest, TargetBrakingClampedToAchievableDeceleration) {
  const double dt = 0.01;
  const double a = 5.0;
  Eigen::VectorXd a_max = Eigen::VectorXd::Constant(num_variables_, a);
  AccelerationLimit constraint(*oink_, dt, a_max);

  // delta_q_prev = 0.01, so the acceleration box is [0.0095, 0.0105].
  constraint.setLastVelocity(Eigen::VectorXd::Constant(num_variables_, 1.0));
  // Braking bound of dt*sqrt(2*5*1e-3) = 1e-3 sits far below the whole box.
  const double d = 1e-3;
  constraint.setTargetDisplacement(Eigen::VectorXd::Constant(num_variables_, d));

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());

  const double accel_lo = 1.0 * dt - a * dt * dt;
  const double accel_hi = 1.0 * dt + a * dt * dt;
  ASSERT_LT(dt * std::sqrt(2.0 * a * d), accel_lo) << "test setup: bound must be unreachable";
  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_LE(lower(i), upper(i)) << "clamping must keep the row feasible";
    // Braked as hard as a_max allows, i.e. down to the bottom of the acceleration box.
    EXPECT_NEAR(upper(i), accel_lo, 1e-12);
    EXPECT_LT(upper(i), accel_hi);
  }
}

// A target behind the joint bounds the lower side and leaves the upper side alone, so a joint
// that has just overshot is not asked for a deceleration the acceleration limit cannot deliver.
TEST_F(AccelerationLimitTest, TargetBrakingIsOneSidedAfterOvershoot) {
  const double dt = 0.01;
  const double a = 5.0;
  Eigen::VectorXd a_max = Eigen::VectorXd::Constant(num_variables_, a);
  AccelerationLimit constraint(*oink_, dt, a_max);

  // Still travelling forward fast, but the target is now behind us (overshoot).
  constraint.setLastVelocity(Eigen::VectorXd::Constant(num_variables_, 1.0));
  constraint.setTargetDisplacement(Eigen::VectorXd::Constant(num_variables_, -1e-6));

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);
  ASSERT_TRUE(constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());

  const double accel_hi = 1.0 * dt + a * dt * dt;
  for (int i = 0; i < num_variables_; ++i) {
    EXPECT_LE(lower(i), upper(i)) << "row must stay feasible right after an overshoot";
    EXPECT_NEAR(upper(i), accel_hi, 1e-12) << "leading side must stay free to decelerate";
  }
}

// The braking bound is self-consistent: it never blocks the final approach. Once the remaining
// distance is small (|d| <= 2*a*dt^2), the bound is looser than the step the task wants, so the
// joint can land exactly on the target instead of creeping.
TEST_F(AccelerationLimitTest, TargetBrakingDoesNotBlockFinalApproach) {
  const double dt = 0.01;
  const double a = 5.0;
  Eigen::VectorXd a_max = Eigen::VectorXd::Constant(num_variables_, a);
  AccelerationLimit constraint(*oink_, dt, a_max);

  const double d_threshold = 2.0 * a * dt * dt;
  for (const double d : {d_threshold * 0.5, d_threshold * 0.1, d_threshold * 0.01}) {
    constraint.reset();
    constraint.setTargetDisplacement(Eigen::VectorXd::Constant(num_variables_, d));

    Eigen::MatrixXd G(num_variables_, num_variables_);
    Eigen::VectorXd lower(num_variables_), upper(num_variables_);
    ASSERT_TRUE(
        constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());
    EXPECT_GE(upper(0), d) << "braking bound must admit the remaining step d=" << d;
  }
}

// clearTargetDisplacement() and reset() both turn the target bound back off.
TEST_F(AccelerationLimitTest, ClearingTargetDisablesTargetBraking) {
  const double dt = 0.01;
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_) * 5.0;
  const double accel_box = a_max(0) * dt * dt;

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);

  for (const bool use_reset : {false, true}) {
    AccelerationLimit constraint(*oink_, dt, a_max);
    constraint.setTargetDisplacement(Eigen::VectorXd::Constant(num_variables_, 1e-9));
    ASSERT_TRUE(constraint.delta_q_target.has_value());
    ASSERT_TRUE(
        constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());
    ASSERT_LT(upper(0), accel_box) << "target should be binding before it is cleared";

    if (use_reset) {
      constraint.reset();
    } else {
      constraint.clearTargetDisplacement();
    }
    EXPECT_FALSE(constraint.delta_q_target.has_value());

    ASSERT_TRUE(
        constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());
    for (int i = 0; i < num_variables_; ++i) {
      EXPECT_NEAR(upper(i), accel_box, 1e-12);
      EXPECT_NEAR(lower(i), -accel_box, 1e-12);
    }
  }
}

// A wrongly sized target is a caller bug, so it throws rather than being silently ignored.
TEST_F(AccelerationLimitTest, MismatchedTargetSizeThrows) {
  AccelerationLimit constraint(*oink_, 0.01, Eigen::VectorXd::Ones(num_variables_) * 5.0);
  EXPECT_THROW(
      { constraint.setTargetDisplacement(Eigen::VectorXd::Ones(num_variables_ - 1)); },
      std::invalid_argument);
  EXPECT_FALSE(constraint.delta_q_target.has_value());
}

// No combination of previous velocity, target and proximity to a position limit may produce an
// empty box: a braking term can never demand more deceleration than a_max delivers in one step.
TEST_F(AccelerationLimitTest, BoundsAreAlwaysFeasible) {
  const double dt = 0.01;
  Eigen::VectorXd a_max = Eigen::VectorXd::Ones(num_variables_) * 5.0;
  AccelerationLimit constraint(*oink_, dt, a_max);

  const auto limits = scene_->getPositionLimitVectors("", /*collapsed*/ true);
  ASSERT_TRUE(limits.has_value());
  const Eigen::VectorXd& q_max = limits->second;

  Eigen::MatrixXd G(num_variables_, num_variables_);
  Eigen::VectorXd lower(num_variables_), upper(num_variables_);

  // Sit hard against a position limit, which drives the position braking term to zero.
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  for (int i = 0; i < num_variables_; ++i) {
    if (std::isfinite(q_max(i))) {
      q(i) = q_max(i);
    }
  }
  scene_->setJointPositions(q);

  for (const double v : {-10.0, -3.0, -0.5, 0.0, 0.5, 3.0, 10.0}) {
    for (const double d : {-1.0, -1e-9, 0.0, 1e-9, 1.0}) {
      constraint.setLastVelocity(Eigen::VectorXd::Constant(num_variables_, v));
      constraint.setTargetDisplacement(Eigen::VectorXd::Constant(num_variables_, d));
      ASSERT_TRUE(
          constraint.computeQpConstraints(posed(*oink_, *scene_), G, lower, upper).has_value());
      for (int i = 0; i < num_variables_; ++i) {
        EXPECT_LE(lower(i), upper(i)) << "empty box at v=" << v << " d=" << d << " joint " << i;
      }
    }
  }
}

}  // namespace roboplan
