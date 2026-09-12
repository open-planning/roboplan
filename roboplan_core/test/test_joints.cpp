#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_utils.hpp>

// TODO: The next Pinocchio release should support mimics of continuous joints.
// https://github.com/stack-of-tasks/pinocchio/pull/2756/files
const std::string kUrdf = R"(
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

const std::string kSrdf = R"(
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
  void SetUp() override { scene = std::make_unique<Scene>("test_scene", kUrdf, kSrdf); }

public:
  // No default constructor, so must be a pointer.
  std::unique_ptr<Scene> scene;
};

TEST_F(RoboPlanJointTest, SceneProperties) {
  // Verify actuated and full joint name lists
  ASSERT_EQ(scene->getJointNames(),
            (std::vector<std::string>{"continuous_joint", "revolute_joint"}));
  const std::vector<std::string> expected_joint_names_with_mimics = {
      "continuous_joint", "revolute_joint", "mimic_joint"};
  ASSERT_EQ(scene->getJointNamesWithMimics(), expected_joint_names_with_mimics);

  // Verify mimic joint info is as expected
  const auto mimic_joint_info = scene->getJointInfo("mimic_joint").value();
  ASSERT_EQ(mimic_joint_info.mimic_info.value().mimicked_joint_name, "revolute_joint");
  EXPECT_FLOAT_EQ(mimic_joint_info.mimic_info.value().scaling, 1.0);
  EXPECT_FLOAT_EQ(mimic_joint_info.mimic_info.value().offset, 0.0);

  // Verify joint group info is as expected for both the default and sub group.
  const auto full_group_info = scene->getJointGroupInfo("").value();
  ASSERT_EQ(full_group_info.joint_names, expected_joint_names_with_mimics);
  ASSERT_EQ(full_group_info.q_indices.size(), 3);  // Mimic joints have nq=0 in Pinocchio
  ASSERT_EQ(full_group_info.v_indices.size(), 2);
  ASSERT_EQ(full_group_info.nq_collapsed, 2);
  ASSERT_TRUE(full_group_info.has_continuous_dofs);
  // The default group should contain every link in the model.
  EXPECT_THAT(full_group_info.link_names,
              ::testing::UnorderedElementsAre("base_link", "link1", "link2", "link3"));

  const auto arm_group_info = scene->getJointGroupInfo("arm").value();
  const std::vector<std::string> expected_arm_joint_names = {"revolute_joint", "mimic_joint"};
  ASSERT_EQ(arm_group_info.joint_names, expected_arm_joint_names);
  ASSERT_EQ(arm_group_info.q_indices.size(), 1);  // Only revolute_joint has a q slot
  ASSERT_EQ(arm_group_info.v_indices.size(), 1);
  ASSERT_EQ(arm_group_info.nq_collapsed, 1);
  ASSERT_FALSE(arm_group_info.has_continuous_dofs);
  // The arm group lists revolute_joint and mimic_joint, whose child links are link2 and link3.
  EXPECT_THAT(arm_group_info.link_names, ::testing::UnorderedElementsAre("link2", "link3"));
}

TEST_F(RoboPlanJointTest, JointGroupLinksFromChainAndExplicitLinks) {
  // A chain group should pull in every link along the chain, and an explicit <link> element
  // should be added on top of the links derived from the group's joints.
  const std::string srdf = R"(
<robot name="robot">
  <group name="chain_group">
    <chain base_link="base_link" tip_link="link3"/>
  </group>
  <group name="explicit_group">
    <joint name="revolute_joint"/>
    <link name="base_link"/>
  </group>
</robot>
)";
  Scene scene("chain_scene", kUrdf, srdf);

  // The chain from base_link to link3 covers link1, link2, and link3 (base_link is excluded).
  const auto chain_info = scene.getJointGroupInfo("chain_group").value();
  EXPECT_THAT(chain_info.link_names, ::testing::UnorderedElementsAre("link1", "link2", "link3"));

  // The explicit group derives link2 from revolute_joint and additionally includes base_link.
  const auto explicit_info = scene.getJointGroupInfo("explicit_group").value();
  EXPECT_THAT(explicit_info.link_names, ::testing::UnorderedElementsAre("base_link", "link2"));
}

TEST_F(RoboPlanJointTest, CurrentJointPositionsWithMimics) {
  Eigen::VectorXd q(3);
  q << 1.0, 0.0, 0.5;
  scene->setJointPositions(q);

  const auto& q_current = scene->getCurrentJointPositions();
  ASSERT_EQ(q_current.size(), 3);
  EXPECT_FLOAT_EQ(q_current(0), 1.0);
  EXPECT_FLOAT_EQ(q_current(1), 0.0);
  EXPECT_FLOAT_EQ(q_current(2), 0.5);

  const auto positions_with_mimics = scene->getCurrentJointPositionsWithMimics();
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
  const auto t_revolute = scene->forwardKinematics(q, "link3");
  q[2] = 1.0;
  const auto t_mimicked = scene->forwardKinematics(q, "link3");
  EXPECT_FALSE(t_revolute.isApprox(t_mimicked));
}

TEST_F(RoboPlanJointTest, RandomPositions) {
  // Reduced q: continuous [cos, sin] + revolute; mimic has no separate entry.
  const auto random_positions = scene->randomPositions();
  EXPECT_EQ(random_positions.size(), 3);
}

TEST_F(RoboPlanJointTest, ExpandCollapse) {
  Eigen::VectorXd pos(3);
  pos << 0.7071067, -0.7071067, 0.5;  // -45 degrees on continuous

  // Collapse the continuous joint
  Eigen::VectorXd expected_collapsed(2);
  expected_collapsed << -0.785398, 0.5;
  const auto maybe_collapsed = collapseContinuousJointPositions(*scene, "", pos);
  ASSERT_TRUE(maybe_collapsed.has_value()) << maybe_collapsed.error();
  const auto& collapsed = maybe_collapsed.value();
  ASSERT_EQ(collapsed.size(), 2);
  EXPECT_FLOAT_EQ(collapsed(0), expected_collapsed(0));
  EXPECT_FLOAT_EQ(collapsed(1), expected_collapsed(1));

  // Expand and ensure we get back the original pose
  const auto maybe_expanded = expandContinuousJointPositions(*scene, "", collapsed);
  ASSERT_TRUE(maybe_expanded.has_value()) << maybe_expanded.error();
  const auto& expanded = maybe_expanded.value();
  ASSERT_EQ(expanded.size(), 3);
  EXPECT_FLOAT_EQ(expanded(0), pos(0));
  EXPECT_FLOAT_EQ(expanded(1), pos(1));
  EXPECT_FLOAT_EQ(expanded(2), pos(2));
}

}  // namespace roboplan
