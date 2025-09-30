#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_utils.hpp>

// TODO: The next Pinocchio release should support mimics of continuous joints.
// https://github.com/stack-of-tasks/pinocchio/pull/2756/files
const std::string URDF = R"(
<robot name="robot">
  <link name="base_link"/>
  <link name="link1" />
  <link name="link2" />
  <link name="link3" />
  <joint name="continuous_joint" type="continuous">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="revolute_joint" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <joint name="mimic_joint" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    <mimic joint="revolute_joint" multiplier="1.0" offset="0.0"/>
  </joint>
</robot>
)";

const std::string SRDF = R"(
<robot name="robot">
  <group name="arm">
    <joint name="revolute_joint"/>
    <joint name="mimic_joint"/>
  </group>
  <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
  <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
  <disable_collisions link1="link2" link2="link3" reason="Adjacent"/>
</robot>
)";

namespace roboplan {

class RoboPlanJointTest : public ::testing::Test {
protected:
  void SetUp() override { scene_ = std::make_unique<Scene>("test_scene", URDF, SRDF); }

public:
  // No default constructor, so must be a pointer.
  std::unique_ptr<Scene> scene_;
};

TEST_F(RoboPlanJointTest, SceneProperties) {
  // Verify actuated joints and mimic joints are as expected
  std::vector<std::string> expected_joint_names = {"continuous_joint", "revolute_joint",
                                                   "mimic_joint"};
  ASSERT_EQ(scene_->getJointNames(), expected_joint_names);
  std::vector<std::string> expected_actuated_names = {"continuous_joint", "revolute_joint"};
  ASSERT_EQ(scene_->getActuatedJointNames(), expected_actuated_names);

  // Verify mimic joint info is as expected
  const auto mimic_joint_info = scene_->getJointInfo("mimic_joint").value();
  ASSERT_EQ(mimic_joint_info.mimic_info.value().mimicked_joint_name, "revolute_joint");
  EXPECT_FLOAT_EQ(mimic_joint_info.mimic_info.value().scaling, 1.0);
  EXPECT_FLOAT_EQ(mimic_joint_info.mimic_info.value().offset, 0.0);

  // Verify joint group info is as expected for both the default and sub group.
  const auto full_group_info = scene_->getJointGroupInfo("").value();
  ASSERT_EQ(full_group_info.joint_names, expected_joint_names);
  ASSERT_EQ(full_group_info.q_indices.size(), 4);
  ASSERT_EQ(full_group_info.v_indices.size(), 3);  // Continuous joint's velocity is theta_dot
  ASSERT_EQ(full_group_info.nq_collapsed, 3);
  ASSERT_TRUE(full_group_info.has_continuous_dofs);

  const auto arm_group_info = scene_->getJointGroupInfo("arm").value();
  expected_joint_names = {"revolute_joint", "mimic_joint"};
  ASSERT_EQ(arm_group_info.joint_names, expected_joint_names);
  ASSERT_EQ(arm_group_info.q_indices.size(), 2);
  ASSERT_EQ(arm_group_info.v_indices.size(), 2);
  ASSERT_EQ(arm_group_info.nq_collapsed, 2);
  ASSERT_FALSE(arm_group_info.has_continuous_dofs);
}

TEST_F(RoboPlanJointTest, RandomPositions) {
  // Generate a random pose for each continuous joint, which are represented as
  // [cos(theta), sin(theta)] in Pinocchio. Ensure mimic joint is matched.
  const auto random_positions = scene_->randomPositions();
  EXPECT_EQ(random_positions.size(), 4);
  EXPECT_FLOAT_EQ(random_positions[2], random_positions[3]);
}

TEST_F(RoboPlanJointTest, ExpandCollapse) {
  Eigen::VectorXd pos(4);
  pos << 0.7071067, -0.7071067, 0.5, 0.5;  // -45 degrees

  // Collapse the continuous joint
  Eigen::VectorXd expected_collapsed(3);
  expected_collapsed << -0.785398, 0.5, 0.5;
  const auto collapsed = collapseContinuousJointPositions(*scene_, "", pos);
  ASSERT_TRUE(collapsed.has_value());
  ASSERT_EQ(collapsed->size(), 3);
  EXPECT_FLOAT_EQ((*collapsed)(0), expected_collapsed(0));
  EXPECT_FLOAT_EQ((*collapsed)(1), expected_collapsed(1));
  EXPECT_FLOAT_EQ((*collapsed)(2), expected_collapsed(2));

  // Expand and ensure we get back the original pose
  const auto expanded = expandContinuousJointPositions(*scene_, "", *collapsed);
  ASSERT_TRUE(expanded.has_value());
  ASSERT_EQ(expanded->size(), 4);
  EXPECT_FLOAT_EQ((*expanded)(0), pos(0));
  EXPECT_FLOAT_EQ((*expanded)(1), pos(1));
  EXPECT_FLOAT_EQ((*expanded)(2), pos(2));
  EXPECT_FLOAT_EQ((*expanded)(3), pos(3));
}

}  // namespace roboplan
