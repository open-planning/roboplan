#include <cmath>
#include <fstream>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <limits>
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
    urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    package_paths = {example_models::get_package_share_dir()};
    yaml_config_path = model_prefix / "ur_robot_model" / "ur5_config.yaml";
    scene = std::make_unique<Scene>("test_scene", urdf_path, srdf_path, package_paths,
                                    yaml_config_path);
  }

public:
  // No default constructor, so must be a pointer.
  std::unique_ptr<Scene> scene;
  std::filesystem::path urdf_path;
  std::filesystem::path srdf_path;
  std::vector<std::filesystem::path> package_paths;
  std::filesystem::path yaml_config_path;
};

TEST_F(RoboPlanSceneTest, SceneProperties) {
  EXPECT_EQ(scene->getName(), "test_scene");
  EXPECT_EQ(scene->getModel().nq, 6u);
  EXPECT_THAT(scene->getJointNames(),
              ContainerEq(std::vector<std::string>{"shoulder_pan_joint", "shoulder_lift_joint",
                                                   "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                                                   "wrist_3_joint"}));

  const auto maybe_joint_info = scene->getJointInfo("shoulder_pan_joint");
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
  EXPECT_NEAR(joint_info.limits.max_acceleration[0], 5.0, kTolerance);
  ASSERT_EQ(joint_info.limits.max_jerk.size(), 1u);
  EXPECT_NEAR(joint_info.limits.max_jerk[0], 10.0, kTolerance);

  std::cout << *scene;  // Test printing for good measure
}

TEST_F(RoboPlanSceneTest, RandomPositions) {
  // Test subsequent pseudorandom values.
  const auto orig_random_positions = scene->randomPositions();
  const auto new_random_positions = scene->randomPositions();
  EXPECT_EQ(orig_random_positions.size(), 6);
  EXPECT_THAT(orig_random_positions, Not(ContainerEq(new_random_positions)));

  // Test seeded values.
  scene->setRngSeed(1234);
  const auto orig_seeded_positions = scene->randomPositions();
  EXPECT_EQ(orig_seeded_positions.size(), 6);
  scene->setRngSeed(1234);  // reset seed
  const auto new_seeded_positions = scene->randomPositions();
  EXPECT_THAT(orig_seeded_positions, ContainerEq(new_seeded_positions));
}

TEST_F(RoboPlanSceneTest, CollisionCheck) {
  // Collision free
  Eigen::VectorXd q_free(6);
  q_free << 0.0, -1.57, 0.0, 0.0, 0.0, 0.0;
  EXPECT_FALSE(scene->hasCollisions(q_free));

  // In collision
  Eigen::VectorXd q_coll(6);
  q_coll << 0.0, -1.57, 3.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(scene->hasCollisions(q_coll));
}

TEST_F(RoboPlanSceneTest, CollisionCheckAlongPath) {
  // Collision free
  Eigen::VectorXd q_start_free(6);
  q_start_free << 0.0, -1.57, 0.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd q_end_free(6);
  q_end_free << 1.0, -1.57, 1.57, 0.0, 0.0, 0.0;
  EXPECT_FALSE(hasCollisionsAlongPath(*scene, q_start_free, q_end_free, 0.05));

  Eigen::VectorXd q_end_coll(6);
  q_end_coll << 0.0, -1.57, 3.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(hasCollisionsAlongPath(*scene, q_start_free, q_end_coll, 0.05));
}

TEST_F(RoboPlanSceneTest, GetFrameMapReturnsCorrectMapping) {
  const auto model = scene->getModel();

  // Verify the frame IDs are correct
  for (const auto& frame : model.frames) {
    if (frame.name == "universe")
      continue;
    EXPECT_EQ(scene->getFrameId(frame.name), model.getFrameId(frame.name));
  }
}

TEST_F(RoboPlanSceneTest, TestForwardKinematics) {
  // Collision free
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  const auto fk = scene->forwardKinematics(q, "tool0");

  // Expected transformation matrix for zero configuration for a UR5e
  Eigen::Matrix4d expected;
  expected << -1.0, 0.0, 0.0, 0.81725, 0.0, 0.0, 1.0, 0.19145, 0.0, 1.0, 0.0, -0.005491, 0.0, 0.0,
      0.0, 1.0;
  EXPECT_TRUE(fk.isApprox(expected, 1e-6));

  // Compute FK from the wrist to the tool
  const auto fk_wrist = scene->forwardKinematics(q, "tool0", "wrist_1_link");

  // Expected transformation matrix for zero configuration from the wrist to the tooltip
  expected << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.1753, 0.0, -1.0, 0.0, 0.09465, 0.0, 0.0, 0.0,
      1.0;
  EXPECT_TRUE(fk_wrist.isApprox(expected, 1e-6));
}

TEST_F(RoboPlanSceneTest, TestFrameJacobianSameBaseAndTipIsZero) {
  // When frame_id == base_frame_id the relative Jacobian must be zero: the EE is
  // stationary relative to itself regardless of which joints move.
  Eigen::VectorXd q(6);
  q << 0.5, -0.5, 1.0, 0.0, 0.3, 0.0;

  const auto maybe_frame_id = scene->getFrameId("tool0");
  ASSERT_TRUE(maybe_frame_id.has_value());
  const pinocchio::FrameIndex frame_id = maybe_frame_id.value();

  for (auto rf : {pinocchio::LOCAL, pinocchio::LOCAL_WORLD_ALIGNED, pinocchio::WORLD}) {
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, scene->getModel().nv);
    scene->computeRelativeFrameJacobian(q, frame_id, "tool0", rf, J);
    EXPECT_NEAR(J.norm(), 0.0, kTolerance)
        << "Relative Jacobian should be zero when base frame == tip frame";
  }
}

TEST_F(RoboPlanSceneTest, TestFrameJacobianBaseFrameNumerical) {
  // Property-based verification of the relative Jacobian for tool0 relative to
  // wrist_1_link on the UR5. Two mathematically certain properties hold regardless
  // of reference frame convention:
  //
  // 1. Joints UPSTREAM of the base frame (joints 0-3: shoulder_pan, shoulder_lift,
  //    elbow, wrist_1) move the entire sub-arm rigidly → no relative motion between
  //    wrist_1_link and tool0 → J_rel[:,0:4] = 0.
  //
  // 2. Joints DOWNSTREAM of the base frame (joints 4-5: wrist_2, wrist_3) do not
  //    affect wrist_1_link at all → J_rel[:,4:6] = J_ee_abs[:,4:6].
  Eigen::VectorXd q(6);
  q << 0.5, -0.5, 1.0, 0.2, 0.3, -0.1;
  const int nv = scene->getModel().nv;

  const auto maybe_ee_id = scene->getFrameId("tool0");
  const auto maybe_base_id = scene->getFrameId("wrist_1_link");
  ASSERT_TRUE(maybe_ee_id.has_value());
  ASSERT_TRUE(maybe_base_id.has_value());
  const pinocchio::FrameIndex ee_id = maybe_ee_id.value();

  // Relative Jacobian (tool0 relative to wrist_1_link)
  Eigen::MatrixXd J_rel = Eigen::MatrixXd::Zero(6, nv);
  scene->computeRelativeFrameJacobian(q, ee_id, "wrist_1_link", pinocchio::LOCAL_WORLD_ALIGNED,
                                      J_rel);

  // Absolute EE Jacobian (no base frame)
  Eigen::MatrixXd J_ee = Eigen::MatrixXd::Zero(6, nv);
  scene->computeFrameJacobian(q, ee_id, pinocchio::LOCAL_WORLD_ALIGNED, J_ee);

  // Property 1: upstream joints (0-3) → zero relative Jacobian columns.
  EXPECT_NEAR(J_rel.leftCols(4).norm(), 0.0, kTolerance)
      << "Joints upstream of the base frame should produce zero relative Jacobian.\n"
      << "J_rel.leftCols(4):\n"
      << J_rel.leftCols(4);

  // Property 2: downstream joints (4-5) → relative Jacobian == absolute EE Jacobian.
  EXPECT_TRUE(J_rel.rightCols(2).isApprox(J_ee.rightCols(2), kTolerance))
      << "Joints between base and EE should give J_rel == J_ee_abs.\n"
      << "J_rel.rightCols(2):\n"
      << J_rel.rightCols(2) << "\nJ_ee.rightCols(2):\n"
      << J_ee.rightCols(2);
}

TEST_F(RoboPlanSceneTest, TestLoadXMLStrings) {
  // Load the sample XMLs from file as strings.
  auto urdf_xml = readFile(urdf_path);
  auto srdf_xml = readFile(srdf_path);

  // Just make sure it is the same as when loading from file (the validation is above)
  auto scene_xml =
      std::make_unique<Scene>("test_scene", urdf_xml, srdf_xml, package_paths, yaml_config_path);
  EXPECT_EQ(scene_xml->getModel().nq, scene->getModel().nq);
  EXPECT_THAT(scene_xml->getJointNames(), scene->getJointNames());

  const auto seeded_positions = scene_xml->randomPositions();
  EXPECT_EQ(seeded_positions.size(), 6);
}

TEST_F(RoboPlanSceneTest, TestCollisionGeometry) {
  // Nominally, this configuration is collision free.
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(scene->hasCollisions(q));

  // Add some collision objects
  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);

  Eigen::Matrix4d box_tform = Eigen::Matrix4d::Identity();
  box_tform(2, 3) = 1.0;  // z position
  const auto add_box_result =
      scene->addBoxGeometry("test_box", "universe", Box(1.0, 1.0, 0.5), box_tform, color);
  ASSERT_TRUE(add_box_result.has_value()) << add_box_result.error();

  Eigen::Matrix4d sphere_tform = Eigen::Matrix4d::Identity();
  sphere_tform(0, 3) = 3.0;  // x position
  const auto add_sphere_result =
      scene->addSphereGeometry("test_sphere", "universe", Sphere(0.5), sphere_tform, color);
  ASSERT_TRUE(add_sphere_result.has_value()) << add_sphere_result.error();

  Eigen::Matrix4d cylinder_tform = Eigen::Matrix4d::Identity();
  cylinder_tform(0, 3) = -3.0;  // x position
  const auto add_cylinder_result = scene->addCylinderGeometry(
      "test_cylinder", "universe", Cylinder(0.25, 1.0), cylinder_tform, color);
  ASSERT_TRUE(add_cylinder_result.has_value()) << add_cylinder_result.error();

  ASSERT_FALSE(scene->hasCollisions(q));  // should still be collision free

  // Now move one of the collision objects to be in collision.
  sphere_tform(0, 3) = 0.0;
  const auto move_sphere_result =
      scene->updateGeometryPlacement("test_sphere", "universe", sphere_tform);
  ASSERT_TRUE(move_sphere_result.has_value()) << move_sphere_result.error();
  ASSERT_TRUE(scene->hasCollisions(q));

  // Remove the collision object and verify that the robot is no longer in collision.
  const auto remove_sphere_result = scene->removeGeometry("test_sphere");
  ASSERT_TRUE(remove_sphere_result.has_value()) << remove_sphere_result.error();
  ASSERT_FALSE(scene->hasCollisions(q));

  // Move the cylinder so it is in collision with the robot.
  cylinder_tform(0, 3) = 0.0;
  const auto move_cylinder_result =
      scene->updateGeometryPlacement("test_cylinder", "universe", cylinder_tform);
  ASSERT_TRUE(move_cylinder_result.has_value()) << move_cylinder_result.error();
  ASSERT_TRUE(scene->hasCollisions(q));

  // Remove the cylinder and verify that the robot is no longer in collision.
  const auto remove_cylinder_result = scene->removeGeometry("test_cylinder");
  ASSERT_TRUE(remove_cylinder_result.has_value()) << remove_cylinder_result.error();
  ASSERT_FALSE(scene->hasCollisions(q));
}

TEST_F(RoboPlanSceneTest, TestCollisionGeometryRemoveReaddReparent) {
  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);
  const std::vector<std::string> box_names = {"test_box_1", "test_box_2", "test_box_3"};

  // Verifies that the scene's internal collision_geometry_map_ and the underlying pinocchio
  // collision model agree on the geometry's index, parent frame, and parent joint.
  auto verify_geometry = [&](const std::string& name, const std::string& expected_parent_frame) {
    // Index reported by the scene's internal collision_geometry_map_.
    const auto maybe_scene_ids = scene->getCollisionGeometryIds(name);
    ASSERT_TRUE(maybe_scene_ids.has_value()) << maybe_scene_ids.error();
    ASSERT_EQ(maybe_scene_ids.value().size(), 1u);
    const auto scene_idx = maybe_scene_ids.value().front();

    // Index reported by the pinocchio collision model.
    const auto& collision_model = scene->getCollisionModel();
    ASSERT_TRUE(collision_model.existGeometryName(name));
    const auto pinocchio_idx = collision_model.getGeometryId(name);
    EXPECT_EQ(scene_idx, pinocchio_idx);

    // The geometry object at that index should report the expected parent frame and joint.
    const auto& geom_obj = collision_model.geometryObjects.at(pinocchio_idx);
    EXPECT_EQ(geom_obj.name, name);

    const auto maybe_frame_id = scene->getFrameId(expected_parent_frame);
    ASSERT_TRUE(maybe_frame_id.has_value()) << maybe_frame_id.error();
    const auto expected_frame_id = maybe_frame_id.value();
    const auto expected_joint_id = scene->getModel().frames.at(expected_frame_id).parentJoint;
    EXPECT_EQ(geom_obj.parentFrame, expected_frame_id);
    EXPECT_EQ(geom_obj.parentJoint, expected_joint_id);
  };

  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();

  // Add 3 boxes parented to "universe".
  for (size_t i = 0; i < box_names.size(); ++i) {
    tform(2, 3) = 1.0 + 0.1 * static_cast<double>(i);
    const auto add_result =
        scene->addBoxGeometry(box_names[i], "universe", Box(0.1, 0.1, 0.1), tform, color);
    ASSERT_TRUE(add_result.has_value()) << add_result.error();
  }
  for (const auto& name : box_names) {
    verify_geometry(name, "universe");
  }

  // Remove all boxes and re-add them. The internal index map must be re-shifted on each removal
  // to stay in sync with pinocchio, which renumbers indices after removals.
  for (const auto& name : box_names) {
    const auto remove_result = scene->removeGeometry(name);
    ASSERT_TRUE(remove_result.has_value()) << remove_result.error();
  }
  for (size_t i = 0; i < box_names.size(); ++i) {
    tform(2, 3) = 1.0 + 0.1 * static_cast<double>(i);
    const auto add_result =
        scene->addBoxGeometry(box_names[i], "universe", Box(0.1, 0.1, 0.1), tform, color);
    ASSERT_TRUE(add_result.has_value()) << add_result.error();
  }
  for (const auto& name : box_names) {
    verify_geometry(name, "universe");
  }

  // Reparent each box to "tool0".
  for (const auto& name : box_names) {
    const auto update_result = scene->updateGeometryPlacement(name, "tool0", tform);
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
  ASSERT_FALSE(scene->hasCollisions(q));

  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);

  // Resolve mesh paths from the example models package.
  const auto model_prefix = example_models::get_package_models_dir();
  const auto stl_path = model_prefix / "ur_robot_model" / "meshes" / "collision" / "wrist3.stl";
  const auto dae_path = model_prefix / "ur_robot_model" / "meshes" / "visual" / "wrist3.dae";

  // Place both meshes overlapping the robot base so they are in collision.
  Eigen::Matrix4d in_collision_tform = Eigen::Matrix4d::Identity();

  const auto add_stl_result = scene->addMeshGeometry("test_stl_mesh", "universe", Mesh(stl_path),
                                                     in_collision_tform, color);
  ASSERT_TRUE(add_stl_result.has_value()) << add_stl_result.error();
  ASSERT_TRUE(scene->hasCollisions(q));

  const auto add_dae_result = scene->addMeshGeometry("test_dae_mesh", "universe", Mesh(dae_path),
                                                     in_collision_tform, color);
  ASSERT_TRUE(add_dae_result.has_value()) << add_dae_result.error();
  ASSERT_TRUE(scene->hasCollisions(q));

  // Move both meshes away from the robot so they are no longer in collision.
  Eigen::Matrix4d collision_free_tform = Eigen::Matrix4d::Identity();
  collision_free_tform(0, 3) = 5.0;
  const auto move_stl_result =
      scene->updateGeometryPlacement("test_stl_mesh", "universe", collision_free_tform);
  ASSERT_TRUE(move_stl_result.has_value()) << move_stl_result.error();

  collision_free_tform(0, 3) = -5.0;
  const auto move_dae_result =
      scene->updateGeometryPlacement("test_dae_mesh", "universe", collision_free_tform);
  ASSERT_TRUE(move_dae_result.has_value()) << move_dae_result.error();

  ASSERT_FALSE(scene->hasCollisions(q));
}

TEST_F(RoboPlanSceneTest, TestCollisionForOcTreeGeometry) {
  // Nominally, this configuration is collision free.
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(scene->hasCollisions(q));

  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);

  auto octree_geometry = createPointcloud();

  Eigen::Matrix4d octree_tform = Eigen::Matrix4d::Identity();
  octree_tform(0, 3) = 1.0;  // z position

  const auto add_octree_result =
      scene->addOcTreeGeometry("test_octree", "universe", octree_geometry, octree_tform, color);
  ASSERT_TRUE(add_octree_result.has_value()) << add_octree_result.error();

  ASSERT_FALSE(scene->hasCollisions(q));  // should still be collision free

  // Now move one of the collision objects to be in collision.
  octree_tform(0, 3) = 0.0;
  const auto move_octree_result =
      scene->updateGeometryPlacement("test_octree", "universe", octree_tform);

  ASSERT_TRUE(move_octree_result.has_value()) << move_octree_result.error();
  ASSERT_TRUE(scene->hasCollisions(q));
}

TEST_F(RoboPlanSceneTest, TestSetCollisionsForOcTree) {
  // Nominally, this configuration is collision free.
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(scene->hasCollisions(q));

  const auto color = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);

  auto octree_geometry = createPointcloud();

  Eigen::Matrix4d octree_tform = Eigen::Matrix4d::Identity();
  octree_tform(0, 3) = 0.6;  // z position

  const auto add_octree_result =
      scene->addOcTreeGeometry("test_octree", "universe", octree_geometry, octree_tform, color);
  ASSERT_TRUE(add_octree_result.has_value()) << add_octree_result.error();

  ASSERT_TRUE(scene->hasCollisions(q));  // should be in collision

  // Disable the collision pair for the offending bodies.
  const auto remove_collision_result = scene->setCollisions("forearm_link", "test_octree", false);
  ASSERT_TRUE(remove_collision_result.has_value()) << remove_collision_result.error();
  ASSERT_FALSE(scene->hasCollisions(q));

  // Now re-add the collision pair, which should re-enable collision.
  const auto add_collision_result = scene->setCollisions("test_octree", "forearm_link", true);
  ASSERT_TRUE(add_collision_result.has_value()) << add_collision_result.error();
  ASSERT_TRUE(scene->hasCollisions(q));

  // Add an invalid collision pair and check for errors.
  const auto bad_set_collision_result =
      scene->setCollisions("nonexistent_link", "test_octree", true);
  ASSERT_FALSE(bad_set_collision_result.has_value());
  ASSERT_EQ(bad_set_collision_result.error(),
            "Could not set collisions: Could not get collision geometry IDs: Frame name "
            "'nonexistent_link' not found in frame_map_.");
}

TEST_F(RoboPlanSceneTest, TestSetCollisions) {
  // Nominally, this configuration is collision free.
  Eigen::VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(scene->hasCollisions(q));

  // Add a collision object such that the configuration is in collision.
  Eigen::Matrix4d sphere_tform = Eigen::Matrix4d::Identity();
  sphere_tform(0, 3) = 0.6;  // x position, enough to collide with forearm_link.
  const auto add_sphere_result = scene->addSphereGeometry(
      "test_sphere", "universe", Sphere(0.1), sphere_tform, Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));
  ASSERT_TRUE(scene->hasCollisions(q));

  // Disable the collision pair for the offending bodies.
  const auto remove_collision_result = scene->setCollisions("forearm_link", "test_sphere", false);
  ASSERT_TRUE(remove_collision_result.has_value()) << remove_collision_result.error();
  ASSERT_FALSE(scene->hasCollisions(q));

  // Now re-add the collision pair, which should re-enable collision.
  const auto add_collision_result = scene->setCollisions("test_sphere", "forearm_link", true);
  ASSERT_TRUE(add_collision_result.has_value()) << add_collision_result.error();
  ASSERT_TRUE(scene->hasCollisions(q));

  // Add an invalid collision pair and check for errors.
  const auto bad_set_collision_result =
      scene->setCollisions("nonexistent_link", "test_sphere", true);
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
  auto maybe_position_limits = scene->getPositionLimitVectors();
  ASSERT_TRUE(maybe_position_limits.has_value());
  const auto& [lower_limits, upper_limits] = maybe_position_limits.value();
  EXPECT_TRUE(lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(upper_limits.isApprox(expected_upper_limits, kTolerance));

  // Specific group (in this case, it's the same as all joints).
  maybe_position_limits = scene->getPositionLimitVectors("arm");
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
  auto maybe_velocity_limits = scene->getVelocityLimitVectors();
  ASSERT_TRUE(maybe_velocity_limits.has_value());
  const auto& [lower_limits, upper_limits] = maybe_velocity_limits.value();
  EXPECT_TRUE(lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(upper_limits.isApprox(expected_upper_limits, kTolerance));

  // Specific group (in this case, it's the same as all joints).
  maybe_velocity_limits = scene->getVelocityLimitVectors("arm");
  ASSERT_TRUE(maybe_velocity_limits.has_value());
  const auto& [group_lower_limits, group_upper_limits] = maybe_velocity_limits.value();
  EXPECT_TRUE(group_lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(group_upper_limits.isApprox(expected_upper_limits, kTolerance));
}

TEST_F(RoboPlanSceneTest, TestAccelerationLimitsVector) {
  Eigen::VectorXd expected_lower_limits(6);
  expected_lower_limits << -5.0, -5.0, -5.0, -5.0, -5.0, -5.0;
  Eigen::VectorXd expected_upper_limits(6);
  expected_upper_limits << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;

  // Default group (all joints)
  auto maybe_acceleration_limits = scene->getAccelerationLimitVectors();
  ASSERT_TRUE(maybe_acceleration_limits.has_value());
  const auto& [lower_limits, upper_limits] = maybe_acceleration_limits.value();
  EXPECT_TRUE(lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(upper_limits.isApprox(expected_upper_limits, kTolerance));

  // Specific group (in this case, it's the same as all joints).
  maybe_acceleration_limits = scene->getAccelerationLimitVectors("arm");
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
  auto maybe_jerk_limits = scene->getJerkLimitVectors();
  ASSERT_TRUE(maybe_jerk_limits.has_value());
  const auto& [lower_limits, upper_limits] = maybe_jerk_limits.value();
  EXPECT_TRUE(lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(upper_limits.isApprox(expected_upper_limits, kTolerance));

  // Specific group (in this case, it's the same as all joints).
  maybe_jerk_limits = scene->getJerkLimitVectors("arm");
  ASSERT_TRUE(maybe_jerk_limits.has_value());
  const auto& [group_lower_limits, group_upper_limits] = maybe_jerk_limits.value();
  EXPECT_TRUE(group_lower_limits.isApprox(expected_lower_limits, kTolerance));
  EXPECT_TRUE(group_upper_limits.isApprox(expected_upper_limits, kTolerance));
}

TEST_F(RoboPlanSceneTest, TestPositionLimitsOverrideFromYaml) {
  // Write a temporary YAML config that overrides the position limits of one joint.
  const auto tmp_config = std::filesystem::temp_directory_path() / "ur5_position_override.yaml";
  {
    std::ofstream out(tmp_config);
    out << "joint_limits:\n"
           "  shoulder_pan_joint:\n"
           "    min_position: [-1.0]\n"
           "    max_position: [1.0]\n";
  }

  Scene scene("override_scene", urdf_path, srdf_path, package_paths, tmp_config);

  const auto maybe_joint_info = scene.getJointInfo("shoulder_pan_joint");
  ASSERT_TRUE(maybe_joint_info.has_value()) << maybe_joint_info.error();
  const auto& limits = maybe_joint_info.value().limits;
  ASSERT_EQ(limits.min_position.size(), 1u);
  EXPECT_NEAR(limits.min_position[0], -1.0, kTolerance);
  ASSERT_EQ(limits.max_position.size(), 1u);
  EXPECT_NEAR(limits.max_position[0], 1.0, kTolerance);

  // A non-overridden joint should keep its URDF limits.
  const auto maybe_other = scene.getJointInfo("elbow_joint");
  ASSERT_TRUE(maybe_other.has_value()) << maybe_other.error();
  EXPECT_NEAR(maybe_other.value().limits.min_position[0], -M_PI, kTolerance);
  EXPECT_NEAR(maybe_other.value().limits.max_position[0], M_PI, kTolerance);

  std::filesystem::remove(tmp_config);
}

TEST_F(RoboPlanSceneTest, TestPositionLimitsOverrideInfinityFromYaml) {
  // Confirm that yaml-cpp parses the YAML infinity spellings (.inf / -.inf) for an unbounded
  // position limit, and that they are normalized to the same lowest()/max() sentinels that
  // JointInfo uses for an unbounded limit by default.
  const auto tmp_config = std::filesystem::temp_directory_path() / "ur5_position_inf.yaml";
  {
    std::ofstream out(tmp_config);
    out << "joint_limits:\n"
           "  shoulder_pan_joint:\n"
           "    min_position: [-.inf]\n"
           "    max_position: [.inf]\n";
  }

  Scene scene("inf_scene", urdf_path, srdf_path, package_paths, tmp_config);

  const auto maybe_joint_info = scene.getJointInfo("shoulder_pan_joint");
  ASSERT_TRUE(maybe_joint_info.has_value()) << maybe_joint_info.error();
  const auto& limits = maybe_joint_info.value().limits;
  ASSERT_EQ(limits.min_position.size(), 1u);
  EXPECT_TRUE(std::isfinite(limits.min_position[0]));
  EXPECT_EQ(limits.min_position[0], std::numeric_limits<double>::lowest());
  ASSERT_EQ(limits.max_position.size(), 1u);
  EXPECT_TRUE(std::isfinite(limits.max_position[0]));
  EXPECT_EQ(limits.max_position[0], std::numeric_limits<double>::max());

  std::filesystem::remove(tmp_config);
}

TEST_F(RoboPlanSceneTest, TestPositionLimitsOverrideWrongSizeThrows) {
  const auto tmp_config = std::filesystem::temp_directory_path() / "ur5_position_bad_size.yaml";
  {
    std::ofstream out(tmp_config);
    out << "joint_limits:\n"
           "  shoulder_pan_joint:\n"
           "    max_position: [1.0, 2.0]\n";  // joint nv is 1, so this is invalid.
  }

  EXPECT_THROW(Scene("bad_size_scene", urdf_path, srdf_path, package_paths, tmp_config),
               std::runtime_error);

  std::filesystem::remove(tmp_config);
}

}  // namespace roboplan
