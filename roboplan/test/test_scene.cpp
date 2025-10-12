#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>

namespace {
constexpr double kTolerance = 1e-4;
}

namespace roboplan {

using ::testing::ContainerEq;
using ::testing::Not;

class RoboPlanSceneTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto share_prefix = example_models::get_install_prefix() / "share";
    const auto model_prefix = share_prefix / "roboplan_example_models" / "models";
    urdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    srdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    package_paths_ = {share_prefix};
    yaml_config_path_ = model_prefix / "ur_robot_model" / "ur5_config.yaml";
    scene_ = std::make_unique<Scene>("test_scene", urdf_path_, srdf_path_, package_paths_,
                                     yaml_config_path_);
  }

public:
  // No default constructor, so must be a pointer.
  std::unique_ptr<Scene> scene_;
  std::filesystem::path urdf_path_;
  std::filesystem::path srdf_path_;
  std::vector<std::filesystem::path> package_paths_;
  std::filesystem::path yaml_config_path_;
};

TEST_F(RoboPlanSceneTest, SceneProperties) {
  EXPECT_EQ(scene_->getName(), "test_scene");
  EXPECT_EQ(scene_->getModel().nq, 6u);
  EXPECT_THAT(scene_->getJointNames(),
              ContainerEq(std::vector<std::string>{"shoulder_pan_joint", "shoulder_lift_joint",
                                                   "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                                                   "wrist_3_joint"}));

  const auto maybe_joint_info = scene_->getJointInfo("shoulder_pan_joint");
  ASSERT_TRUE(maybe_joint_info.has_value()) << maybe_joint_info.error();
  const auto& joint_info = maybe_joint_info.value();
  EXPECT_EQ(joint_info.type, JointType::REVOLUTE);
  EXPECT_EQ(joint_info.num_position_dofs, 1u);
  EXPECT_EQ(joint_info.num_velocity_dofs, 1u);
  ASSERT_EQ(joint_info.limits.min_position.size(), 1u);
  EXPECT_NEAR(joint_info.limits.min_position[0], -M_PI, kTolerance);
  ASSERT_EQ(joint_info.limits.max_position.size(), 1u);
  EXPECT_NEAR(joint_info.limits.max_position[0], M_PI, kTolerance);
  ASSERT_EQ(joint_info.limits.max_velocity.size(), 1u);
  EXPECT_NEAR(joint_info.limits.max_velocity[0], 3.15, kTolerance);
  ASSERT_EQ(joint_info.limits.max_acceleration.size(), 1u);
  EXPECT_NEAR(joint_info.limits.max_acceleration[0], 2.0, kTolerance);
  ASSERT_EQ(joint_info.limits.max_jerk.size(), 1u);
  EXPECT_NEAR(joint_info.limits.max_jerk[0], 10.0, kTolerance);

  std::cout << *scene_;  // Test printing for good measure
}

TEST_F(RoboPlanSceneTest, RandomPositions) {
  // Test subsequent pseudorandom values.
  const auto orig_random_positions = scene_->randomPositions();
  const auto new_random_positions = scene_->randomPositions();
  EXPECT_EQ(orig_random_positions.size(), 6);
  EXPECT_THAT(orig_random_positions, Not(ContainerEq(new_random_positions)));

  // Test seeded values.
  scene_->setRngSeed(1234);
  const auto orig_seeded_positions = scene_->randomPositions();
  EXPECT_EQ(orig_seeded_positions.size(), 6);
  scene_->setRngSeed(1234);  // reset seed
  const auto new_seeded_positions = scene_->randomPositions();
  EXPECT_THAT(orig_seeded_positions, ContainerEq(new_seeded_positions));
}

TEST_F(RoboPlanSceneTest, CollisionCheck) {
  // Collision free
  Eigen::VectorXd q_free(6);
  q_free << 0.0, -1.57, 0.0, 0.0, 0.0, 0.0;
  EXPECT_FALSE(scene_->hasCollisions(q_free));

  // In collision
  Eigen::VectorXd q_coll(6);
  q_coll << 0.0, -1.57, 3.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(scene_->hasCollisions(q_coll));
}

TEST_F(RoboPlanSceneTest, CollisionCheckAlongPath) {
  // Collision free
  Eigen::VectorXd q_start_free(6);
  q_start_free << 0.0, -1.57, 0.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd q_end_free(6);
  q_end_free << 1.0, -1.57, 1.57, 0.0, 0.0, 0.0;
  EXPECT_FALSE(hasCollisionsAlongPath(*scene_, q_start_free, q_end_free, 0.05));

  Eigen::VectorXd q_end_coll(6);
  q_end_coll << 0.0, -1.57, 3.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(hasCollisionsAlongPath(*scene_, q_start_free, q_end_coll, 0.05));
}

TEST_F(RoboPlanSceneTest, GetFrameMapReturnsCorrectMapping) {
  const auto model = scene_->getModel();

  // Verify the frame IDs are correct
  for (const auto& frame : model.frames) {
    if (frame.name == "universe")
      continue;
    EXPECT_EQ(scene_->getFrameId(frame.name), model.getFrameId(frame.name));
  }
}

TEST_F(RoboPlanSceneTest, TestForwardKinematics) {
  // Collision free
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  const auto fk = scene_->forwardKinematics(q, "tool0");

  // Expected transformation matrix for zero configuration for a UR5e
  Eigen::Matrix4d expected;
  expected << -1.0, 0.0, 0.0, 0.81725, 0.0, 0.0, 1.0, 0.19145, 0.0, 1.0, 0.0, -0.005491, 0.0, 0.0,
      0.0, 1.0;

  EXPECT_TRUE(fk.isApprox(expected, 1e-6));
}

TEST_F(RoboPlanSceneTest, TestLoadXMLStrings) {
  // Load the sample XMLs from file as strings.
  auto urdf_xml = readFile(urdf_path_);
  auto srdf_xml = readFile(srdf_path_);

  // Just make sure it is the same as when loading from file (the validation is above)
  auto scene_xml =
      std::make_unique<Scene>("test_scene", urdf_xml, srdf_xml, package_paths_, yaml_config_path_);
  EXPECT_EQ(scene_xml->getModel().nq, scene_->getModel().nq);
  EXPECT_THAT(scene_xml->getJointNames(), scene_->getJointNames());

  const auto seeded_positions = scene_xml->randomPositions();
  EXPECT_EQ(seeded_positions.size(), 6);
}

TEST_F(RoboPlanSceneTest, TestCollisionGeometry) {
  // Nominally, this configuration is collision free.
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(scene_->hasCollisions(q));

  // Add some collision objects
  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);

  Eigen::Matrix4d box_tform = Eigen::Matrix4d::Identity();
  box_tform(2, 3) = 1.0;  // z position
  const auto add_box_result =
      scene_->addBoxGeometry("test_box", "universe", Box(1.0, 1.0, 0.5), box_tform, color);
  ASSERT_TRUE(add_box_result.has_value()) << add_box_result.error();

  Eigen::Matrix4d sphere_tform = Eigen::Matrix4d::Identity();
  sphere_tform(0, 3) = 3.0;  // x position
  const auto add_sphere_result =
      scene_->addSphereGeometry("test_sphere", "universe", Sphere(0.5), sphere_tform, color);
  ASSERT_TRUE(add_sphere_result.has_value()) << add_sphere_result.error();

  ASSERT_FALSE(scene_->hasCollisions(q));  // should still be collision free

  // Now move one of the collision objects to be in collision.
  sphere_tform(0, 3) = 0.0;
  const auto move_sphere_result =
      scene_->updateGeometryPlacement("test_sphere", "universe", sphere_tform);
  ASSERT_TRUE(move_sphere_result.has_value()) << move_sphere_result.error();
  ASSERT_TRUE(scene_->hasCollisions(q));

  // Remove the collision object and verify that the robot is no longer in collision.
  const auto remove_sphere_result = scene_->removeGeometry("test_sphere");
  ASSERT_TRUE(remove_sphere_result.has_value()) << remove_sphere_result.error();
  ASSERT_FALSE(scene_->hasCollisions(q));
}

}  // namespace roboplan
