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

OcTree createPointcloud() {
  // Create octree with resolution 0.1 which generates Box(1.0, 1.0, 0.5)
  const double resolution = 0.1;
  Eigen::Matrix<double, 500, 3> point_cloud;
  for (size_t x = 0; x < 10; ++x) {
    for (size_t y = 0; y < 10; ++y) {
      for (size_t z = 0; z < 5; ++z) {
        point_cloud(50 * x + 5 * y + z, 0) = x;
        point_cloud(50 * x + 5 * y + z, 1) = y;
        point_cloud(50 * x + 5 * y + z, 2) = z;
      }
    }
  }

  return OcTree(coal::makeOctree(point_cloud, resolution));
}

class RoboPlanSceneTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    urdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    srdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    package_paths_ = {example_models::get_package_share_dir()};
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

  // Compute FK from the wrist to the tool
  const auto fk_wrist = scene_->forwardKinematics(q, "tool0", "wrist_1_link");

  // Expected transformation matrix for zero configuration from the wrist to the tooltip
  expected << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.1753, 0.0, -1.0, 0.0, 0.09465, 0.0, 0.0, 0.0,
      1.0;
  EXPECT_TRUE(fk_wrist.isApprox(expected, 1e-6));
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

  Eigen::Matrix4d cylinder_tform = Eigen::Matrix4d::Identity();
  cylinder_tform(0, 3) = -3.0;  // x position
  const auto add_cylinder_result = scene_->addCylinderGeometry(
      "test_cylinder", "universe", Cylinder(0.25, 1.0), cylinder_tform, color);
  ASSERT_TRUE(add_cylinder_result.has_value()) << add_cylinder_result.error();

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

  // Move the cylinder so it is in collision with the robot.
  cylinder_tform(0, 3) = 0.0;
  const auto move_cylinder_result =
      scene_->updateGeometryPlacement("test_cylinder", "universe", cylinder_tform);
  ASSERT_TRUE(move_cylinder_result.has_value()) << move_cylinder_result.error();
  ASSERT_TRUE(scene_->hasCollisions(q));

  // Remove the cylinder and verify that the robot is no longer in collision.
  const auto remove_cylinder_result = scene_->removeGeometry("test_cylinder");
  ASSERT_TRUE(remove_cylinder_result.has_value()) << remove_cylinder_result.error();
  ASSERT_FALSE(scene_->hasCollisions(q));
}

TEST_F(RoboPlanSceneTest, TestCollisionGeometryRemoveReaddReparent) {
  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);
  const std::vector<std::string> box_names = {"test_box_1", "test_box_2", "test_box_3"};

  // Verifies that the scene's internal collision_geometry_map_ and the underlying pinocchio
  // collision model agree on the geometry's index, parent frame, and parent joint.
  auto verify_geometry = [&](const std::string& name, const std::string& expected_parent_frame) {
    // Index reported by the scene's internal collision_geometry_map_.
    const auto maybe_scene_ids = scene_->getCollisionGeometryIds(name);
    ASSERT_TRUE(maybe_scene_ids.has_value()) << maybe_scene_ids.error();
    ASSERT_EQ(maybe_scene_ids.value().size(), 1u);
    const auto scene_idx = maybe_scene_ids.value().front();

    // Index reported by the pinocchio collision model.
    const auto& collision_model = scene_->getCollisionModel();
    ASSERT_TRUE(collision_model.existGeometryName(name));
    const auto pinocchio_idx = collision_model.getGeometryId(name);
    EXPECT_EQ(scene_idx, pinocchio_idx);

    // The geometry object at that index should report the expected parent frame and joint.
    const auto& geom_obj = collision_model.geometryObjects.at(pinocchio_idx);
    EXPECT_EQ(geom_obj.name, name);

    const auto maybe_frame_id = scene_->getFrameId(expected_parent_frame);
    ASSERT_TRUE(maybe_frame_id.has_value()) << maybe_frame_id.error();
    const auto expected_frame_id = maybe_frame_id.value();
    const auto expected_joint_id = scene_->getModel().frames.at(expected_frame_id).parentJoint;
    EXPECT_EQ(geom_obj.parentFrame, expected_frame_id);
    EXPECT_EQ(geom_obj.parentJoint, expected_joint_id);
  };

  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();

  // Add 3 boxes parented to "universe".
  for (size_t i = 0; i < box_names.size(); ++i) {
    tform(2, 3) = 1.0 + 0.1 * static_cast<double>(i);
    const auto add_result =
        scene_->addBoxGeometry(box_names[i], "universe", Box(0.1, 0.1, 0.1), tform, color);
    ASSERT_TRUE(add_result.has_value()) << add_result.error();
  }
  for (const auto& name : box_names) {
    verify_geometry(name, "universe");
  }

  // Remove all boxes and re-add them. The internal index map must be re-shifted on each removal
  // to stay in sync with pinocchio, which renumbers indices after removals.
  for (const auto& name : box_names) {
    const auto remove_result = scene_->removeGeometry(name);
    ASSERT_TRUE(remove_result.has_value()) << remove_result.error();
  }
  for (size_t i = 0; i < box_names.size(); ++i) {
    tform(2, 3) = 1.0 + 0.1 * static_cast<double>(i);
    const auto add_result =
        scene_->addBoxGeometry(box_names[i], "universe", Box(0.1, 0.1, 0.1), tform, color);
    ASSERT_TRUE(add_result.has_value()) << add_result.error();
  }
  for (const auto& name : box_names) {
    verify_geometry(name, "universe");
  }

  // Reparent each box to "tool0".
  for (const auto& name : box_names) {
    const auto update_result = scene_->updateGeometryPlacement(name, "tool0", tform);
    ASSERT_TRUE(update_result.has_value()) << update_result.error();
  }
  for (const auto& name : box_names) {
    verify_geometry(name, "tool0");
  }
}

TEST_F(RoboPlanSceneTest, TestCollisionForMeshGeometry) {
  // Nominally, this configuration is collision free.
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(scene_->hasCollisions(q));

  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);

  // Resolve mesh paths from the example models package.
  const auto model_prefix = example_models::get_package_models_dir();
  const auto stl_path = model_prefix / "ur_robot_model" / "meshes" / "collision" / "wrist3.stl";
  const auto dae_path = model_prefix / "ur_robot_model" / "meshes" / "visual" / "wrist3.dae";

  // Place both meshes overlapping the robot base so they are in collision.
  Eigen::Matrix4d in_collision_tform = Eigen::Matrix4d::Identity();

  const auto add_stl_result = scene_->addMeshGeometry("test_stl_mesh", "universe", Mesh(stl_path),
                                                      in_collision_tform, color);
  ASSERT_TRUE(add_stl_result.has_value()) << add_stl_result.error();
  ASSERT_TRUE(scene_->hasCollisions(q));

  const auto add_dae_result = scene_->addMeshGeometry("test_dae_mesh", "universe", Mesh(dae_path),
                                                      in_collision_tform, color);
  ASSERT_TRUE(add_dae_result.has_value()) << add_dae_result.error();
  ASSERT_TRUE(scene_->hasCollisions(q));

  // Move both meshes away from the robot so they are no longer in collision.
  Eigen::Matrix4d collision_free_tform = Eigen::Matrix4d::Identity();
  collision_free_tform(0, 3) = 5.0;
  const auto move_stl_result =
      scene_->updateGeometryPlacement("test_stl_mesh", "universe", collision_free_tform);
  ASSERT_TRUE(move_stl_result.has_value()) << move_stl_result.error();

  collision_free_tform(0, 3) = -5.0;
  const auto move_dae_result =
      scene_->updateGeometryPlacement("test_dae_mesh", "universe", collision_free_tform);
  ASSERT_TRUE(move_dae_result.has_value()) << move_dae_result.error();

  ASSERT_FALSE(scene_->hasCollisions(q));
}

TEST_F(RoboPlanSceneTest, TestCollisionForOcTreeGeometry) {
  // Nominally, this configuration is collision free.
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(scene_->hasCollisions(q));

  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);

  auto octree_geometry = createPointcloud();

  Eigen::Matrix4d octree_tform = Eigen::Matrix4d::Identity();
  octree_tform(0, 3) = 1.0;  // z position

  const auto add_octree_result =
      scene_->addOcTreeGeometry("test_octree", "universe", octree_geometry, octree_tform, color);
  ASSERT_TRUE(add_octree_result.has_value()) << add_octree_result.error();

  ASSERT_FALSE(scene_->hasCollisions(q));  // should still be collision free

  // Now move one of the collision objects to be in collision.
  octree_tform(0, 3) = 0.0;
  const auto move_octree_result =
      scene_->updateGeometryPlacement("test_octree", "universe", octree_tform);

  ASSERT_TRUE(move_octree_result.has_value()) << move_octree_result.error();
  ASSERT_TRUE(scene_->hasCollisions(q));
}

TEST_F(RoboPlanSceneTest, TestSetCollisionsForOcTree) {
  // Nominally, this configuration is collision free.
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(scene_->hasCollisions(q));

  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);

  auto octree_geometry = createPointcloud();

  Eigen::Matrix4d octree_tform = Eigen::Matrix4d::Identity();
  octree_tform(0, 3) = 0.6;  // z position

  const auto add_octree_result =
      scene_->addOcTreeGeometry("test_octree", "universe", octree_geometry, octree_tform, color);
  ASSERT_TRUE(add_octree_result.has_value()) << add_octree_result.error();

  ASSERT_TRUE(scene_->hasCollisions(q));  // should be in collision

  // Disable the collision pair for the offending bodies.
  const auto remove_collision_result = scene_->setCollisions("forearm_link", "test_octree", false);
  ASSERT_TRUE(remove_collision_result.has_value()) << remove_collision_result.error();
  ASSERT_FALSE(scene_->hasCollisions(q));

  // Now re-add the collision pair, which should re-enable collision.
  const auto add_collision_result = scene_->setCollisions("test_octree", "forearm_link", true);
  ASSERT_TRUE(add_collision_result.has_value()) << add_collision_result.error();
  ASSERT_TRUE(scene_->hasCollisions(q));

  // Add an invalid collision pair and check for errors.
  const auto bad_set_collision_result =
      scene_->setCollisions("nonexistent_link", "test_octree", true);
  ASSERT_FALSE(bad_set_collision_result.has_value());
  ASSERT_EQ(bad_set_collision_result.error(),
            "Could not set collisions: Could not get collision geometry IDs: Frame name "
            "'nonexistent_link' not found in frame_map_.");
}

TEST_F(RoboPlanSceneTest, TestSetCollisions) {
  // Nominally, this configuration is collision free.
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(scene_->hasCollisions(q));

  // Add a collision object such that the configuration is in collision.
  Eigen::Matrix4d sphere_tform = Eigen::Matrix4d::Identity();
  sphere_tform(0, 3) = 0.6;  // x position, enough to collide with forearm_link.
  const auto add_sphere_result = scene_->addSphereGeometry(
      "test_sphere", "universe", Sphere(0.1), sphere_tform, Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));
  ASSERT_TRUE(scene_->hasCollisions(q));

  // Disable the collision pair for the offending bodies.
  const auto remove_collision_result = scene_->setCollisions("forearm_link", "test_sphere", false);
  ASSERT_TRUE(remove_collision_result.has_value()) << remove_collision_result.error();
  ASSERT_FALSE(scene_->hasCollisions(q));

  // Now re-add the collision pair, which should re-enable collision.
  const auto add_collision_result = scene_->setCollisions("test_sphere", "forearm_link", true);
  ASSERT_TRUE(add_collision_result.has_value()) << add_collision_result.error();
  ASSERT_TRUE(scene_->hasCollisions(q));

  // Add an invalid collision pair and check for errors.
  const auto bad_set_collision_result =
      scene_->setCollisions("nonexistent_link", "test_sphere", true);
  ASSERT_FALSE(bad_set_collision_result.has_value());
  ASSERT_EQ(bad_set_collision_result.error(),
            "Could not set collisions: Could not get collision geometry IDs: Frame name "
            "'nonexistent_link' not found in frame_map_.");
}

TEST_F(RoboPlanSceneTest, TestPositionLimitsVector) {
  Eigen::VectorXd expected_lower_limits(6);
  expected_lower_limits << -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159;
  Eigen::VectorXd expected_upper_limits(6);
  expected_upper_limits << 3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159;

  // Default group (all joints)
  auto maybe_position_limits = scene_->getPositionLimitVectors();
  ASSERT_TRUE(maybe_position_limits.has_value());
  const auto& [lower_limits, upper_limits] = maybe_position_limits.value();
  EXPECT_TRUE(lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(upper_limits.isApprox(expected_upper_limits, kTolerance));

  // Specific group (in this case, it's the same as all joints).
  maybe_position_limits = scene_->getPositionLimitVectors("arm");
  ASSERT_TRUE(maybe_position_limits.has_value());
  const auto& [group_lower_limits, group_upper_limits] = maybe_position_limits.value();
  EXPECT_TRUE(group_lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(group_upper_limits.isApprox(expected_upper_limits, kTolerance));
}

TEST_F(RoboPlanSceneTest, TestVelocityLimitsVector) {
  Eigen::VectorXd expected_lower_limits(6);
  expected_lower_limits << -3.15, -3.15, -3.15, -3.2, -3.2, -3.2;
  Eigen::VectorXd expected_upper_limits(6);
  expected_upper_limits << 3.15, 3.15, 3.15, 3.2, 3.2, 3.2;

  // Default group (all joints)
  auto maybe_velocity_limits = scene_->getVelocityLimitVectors();
  ASSERT_TRUE(maybe_velocity_limits.has_value());
  const auto& [lower_limits, upper_limits] = maybe_velocity_limits.value();
  EXPECT_TRUE(lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(upper_limits.isApprox(expected_upper_limits, kTolerance));

  // Specific group (in this case, it's the same as all joints).
  maybe_velocity_limits = scene_->getVelocityLimitVectors("arm");
  ASSERT_TRUE(maybe_velocity_limits.has_value());
  const auto& [group_lower_limits, group_upper_limits] = maybe_velocity_limits.value();
  EXPECT_TRUE(group_lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(group_upper_limits.isApprox(expected_upper_limits, kTolerance));
}

TEST_F(RoboPlanSceneTest, TestAccelerationLimitsVector) {
  Eigen::VectorXd expected_lower_limits(6);
  expected_lower_limits << -2.0, -2.0, -2.0, -2.0, -2.0, -2.0;
  Eigen::VectorXd expected_upper_limits(6);
  expected_upper_limits << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;

  // Default group (all joints)
  auto maybe_acceleration_limits = scene_->getAccelerationLimitVectors();
  ASSERT_TRUE(maybe_acceleration_limits.has_value());
  const auto& [lower_limits, upper_limits] = maybe_acceleration_limits.value();
  EXPECT_TRUE(lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(upper_limits.isApprox(expected_upper_limits, kTolerance));

  // Specific group (in this case, it's the same as all joints).
  maybe_acceleration_limits = scene_->getAccelerationLimitVectors("arm");
  ASSERT_TRUE(maybe_acceleration_limits.has_value());
  const auto& [group_lower_limits, group_upper_limits] = maybe_acceleration_limits.value();
  EXPECT_TRUE(group_lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(group_upper_limits.isApprox(expected_upper_limits, kTolerance));
}

TEST_F(RoboPlanSceneTest, TestJerkLimitsVector) {
  Eigen::VectorXd expected_lower_limits(6);
  expected_lower_limits << -10.0, -10.0, -10.0, -10.0, -10.0, -10.0;
  Eigen::VectorXd expected_upper_limits(6);
  expected_upper_limits << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;

  // Default group (all joints)
  auto maybe_jerk_limits = scene_->getJerkLimitVectors();
  ASSERT_TRUE(maybe_jerk_limits.has_value());
  const auto& [lower_limits, upper_limits] = maybe_jerk_limits.value();
  EXPECT_TRUE(lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(upper_limits.isApprox(expected_upper_limits, kTolerance));

  // Specific group (in this case, it's the same as all joints).
  maybe_jerk_limits = scene_->getJerkLimitVectors("arm");
  ASSERT_TRUE(maybe_jerk_limits.has_value());
  const auto& [group_lower_limits, group_upper_limits] = maybe_jerk_limits.value();
  EXPECT_TRUE(group_lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(group_upper_limits.isApprox(expected_upper_limits, kTolerance));
}

}  // namespace roboplan
