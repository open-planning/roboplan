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
  // Verify actuated and full joint name lists
  ASSERT_EQ(scene_->getJointNames(),
            (std::vector<std::string>{"continuous_joint", "revolute_joint"}));
  const std::vector<std::string> expected_joint_names_with_mimics = {
      "continuous_joint", "revolute_joint", "mimic_joint"};
  ASSERT_EQ(scene_->getJointNamesWithMimics(), expected_joint_names_with_mimics);

  // Verify mimic joint info is as expected
  const auto mimic_joint_info = scene_->getJointInfo("mimic_joint").value();
  ASSERT_EQ(mimic_joint_info.mimic_info.value().mimicked_joint_name, "revolute_joint");
  EXPECT_FLOAT_EQ(mimic_joint_info.mimic_info.value().scaling, 1.0);
  EXPECT_FLOAT_EQ(mimic_joint_info.mimic_info.value().offset, 0.0);

  // Verify joint group info is as expected for both the default and sub group.
  const auto full_group_info = scene_->getJointGroupInfo("").value();
  ASSERT_EQ(full_group_info.joint_names, expected_joint_names_with_mimics);
  ASSERT_EQ(full_group_info.q_indices.size(), 3);  // Mimic joints have nq=0 in Pinocchio
  ASSERT_EQ(full_group_info.v_indices.size(), 2);
  ASSERT_EQ(full_group_info.nq_collapsed, 2);
  ASSERT_TRUE(full_group_info.has_continuous_dofs);

  const auto arm_group_info = scene_->getJointGroupInfo("arm").value();
  const std::vector<std::string> expected_arm_joint_names = {"revolute_joint", "mimic_joint"};
  ASSERT_EQ(arm_group_info.joint_names, expected_arm_joint_names);
  ASSERT_EQ(arm_group_info.q_indices.size(), 1);  // Only revolute_joint has a q slot
  ASSERT_EQ(arm_group_info.v_indices.size(), 1);
  ASSERT_EQ(arm_group_info.nq_collapsed, 1);
  ASSERT_FALSE(arm_group_info.has_continuous_dofs);
}

TEST_F(RoboPlanJointTest, CurrentJointPositionsWithMimics) {
  Eigen::VectorXd q(3);
  q << 1.0, 0.0, 0.5;
  scene_->setJointPositions(q);

  const auto& q_current = scene_->getCurrentJointPositions();
  ASSERT_EQ(q_current.size(), 3);
  EXPECT_FLOAT_EQ(q_current(0), 1.0);
  EXPECT_FLOAT_EQ(q_current(1), 0.0);
  EXPECT_FLOAT_EQ(q_current(2), 0.5);

  const auto positions_with_mimics = scene_->getCurrentJointPositionsWithMimics();
  ASSERT_EQ(positions_with_mimics.size(), 4);
  EXPECT_FLOAT_EQ(positions_with_mimics(0), 1.0);
  EXPECT_FLOAT_EQ(positions_with_mimics(1), 0.0);
  EXPECT_FLOAT_EQ(positions_with_mimics(2), 0.5);
  EXPECT_FLOAT_EQ(positions_with_mimics(3), 0.5);
}

TEST_F(RoboPlanJointTest, VerifyMimics) {
  // Pinocchio mimic joints share the mimicked q segment; FK moves link3 when revolute changes.
  Eigen::VectorXd q(3);
  q << 1.0, 0.0, 0.5;
  const auto T_revolute = scene_->forwardKinematics(q, "link3");
  q[2] = 1.0;
  const auto T_mimicked = scene_->forwardKinematics(q, "link3");
  EXPECT_FALSE(T_revolute.isApprox(T_mimicked));
}

TEST_F(RoboPlanJointTest, RandomPositions) {
  // Reduced q: continuous [cos, sin] + revolute; mimic has no separate entry.
  const auto random_positions = scene_->randomPositions();
  EXPECT_EQ(random_positions.size(), 3);
}

TEST_F(RoboPlanJointTest, ExpandCollapse) {
  Eigen::VectorXd pos(3);
  pos << 0.7071067, -0.7071067, 0.5;  // -45 degrees on continuous

  // Collapse the continuous joint
  Eigen::VectorXd expected_collapsed(2);
  expected_collapsed << -0.785398, 0.5;
  const auto maybe_collapsed = collapseContinuousJointPositions(*scene_, "", pos);
  ASSERT_TRUE(maybe_collapsed.has_value()) << maybe_collapsed.error();
  const auto& collapsed = maybe_collapsed.value();
  ASSERT_EQ(collapsed.size(), 2);
  EXPECT_FLOAT_EQ(collapsed(0), expected_collapsed(0));
  EXPECT_FLOAT_EQ(collapsed(1), expected_collapsed(1));

  // Expand and ensure we get back the original pose
  const auto maybe_expanded = expandContinuousJointPositions(*scene_, "", collapsed);
  ASSERT_TRUE(maybe_expanded.has_value()) << maybe_expanded.error();
  const auto& expanded = maybe_expanded.value();
  ASSERT_EQ(expanded.size(), 3);
  EXPECT_FLOAT_EQ(expanded(0), pos(0));
  EXPECT_FLOAT_EQ(expanded(1), pos(1));
  EXPECT_FLOAT_EQ(expanded(2), pos(2));
}

}  // namespace roboplan
