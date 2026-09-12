#include <cmath>
#include <numbers>

#include <gtest/gtest.h>
#include <roboplan/filters/se3_low_pass_filter.hpp>

namespace roboplan {

namespace {

pinocchio::SE3 makePose(const Eigen::Vector3d& translation, const Eigen::Quaterniond& quaternion) {
  return pinocchio::SE3(quaternion.normalized().toRotationMatrix(), translation);
}

}  // namespace

TEST(SE3LowPassFilterTest, ConstructorDefaults) {
  const SE3LowPassFilter filter;

  EXPECT_FALSE(filter.isInitialized());
}

TEST(SE3LowPassFilterTest, ResetInitializesFilter) {
  SE3LowPassFilter filter;
  const pinocchio::SE3 pose =
      makePose(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity());

  filter.reset(pose);

  EXPECT_TRUE(filter.isInitialized());
}

TEST(SE3LowPassFilterTest, FirstUpdateReturnsTargetPose) {
  SE3LowPassFilter filter;
  const Eigen::AngleAxisd rotation(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ());
  const pinocchio::SE3 target_pose =
      makePose(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond(rotation));

  const pinocchio::SE3 filtered_pose = filter.update(target_pose, 0.01);

  EXPECT_TRUE(filter.isInitialized());
  EXPECT_TRUE(filtered_pose.translation().isApprox(target_pose.translation()));
  EXPECT_TRUE(filtered_pose.rotation().isApprox(target_pose.rotation()));
}

TEST(SE3LowPassFilterTest, UpdateSmoothsPosition) {
  SE3LowPassFilter filter(0.1);
  const pinocchio::SE3 initial_pose =
      makePose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  const pinocchio::SE3 target_pose =
      makePose(Eigen::Vector3d::UnitX(), Eigen::Quaterniond::Identity());

  filter.reset(initial_pose);
  const pinocchio::SE3 filtered_pose = filter.update(target_pose, 0.1);

  const double expected_alpha = 1.0 - std::exp(-1.0);
  EXPECT_NEAR(filtered_pose.translation().x(), expected_alpha, 1e-12);
  EXPECT_NEAR(filtered_pose.translation().y(), 0.0, 1e-12);
  EXPECT_NEAR(filtered_pose.translation().z(), 0.0, 1e-12);
}

TEST(SE3LowPassFilterTest, UpdateSmoothsOrientation) {
  SE3LowPassFilter filter(0.1);
  const pinocchio::SE3 initial_pose =
      makePose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  const Eigen::AngleAxisd target_rotation(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ());
  const pinocchio::SE3 target_pose =
      makePose(Eigen::Vector3d::Zero(), Eigen::Quaterniond(target_rotation));

  filter.reset(initial_pose);
  const pinocchio::SE3 filtered_pose = filter.update(target_pose, 0.1);

  const double expected_alpha = 1.0 - std::exp(-1.0);
  const Eigen::AngleAxisd filtered_rotation(filtered_pose.rotation());
  EXPECT_NEAR(filtered_rotation.angle(), expected_alpha * std::numbers::pi / 2.0, 1e-12);
}

}  // namespace roboplan
