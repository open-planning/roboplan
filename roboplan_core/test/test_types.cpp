#include <gtest/gtest.h>

#include <roboplan/core/types.hpp>

namespace roboplan {

TEST(RoboPlanTypes, TestJointConfiguration) {
  JointConfiguration cfg;
  EXPECT_TRUE(cfg.joint_names.empty());
  EXPECT_TRUE(cfg.positions.size() == 0u);
  EXPECT_TRUE(cfg.velocities.size() == 0u);
  EXPECT_TRUE(cfg.accelerations.size() == 0u);
}

TEST(RoboPlanTypes, TestCartesianConfiguration) {
  CartesianConfiguration cfg;
  EXPECT_EQ(cfg.base_frame, "");
  EXPECT_EQ(cfg.tip_frame, "");
  EXPECT_TRUE(cfg.tform.isApprox(Eigen::Matrix4d::Identity()));
}

}  // namespace roboplan
