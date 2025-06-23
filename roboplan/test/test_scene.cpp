#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>

namespace roboplan {

using ::testing::ContainerEq;
using ::testing::Not;

class RoboPlanSceneTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto share_prefix = roboplan_example_models::get_package_share_dir();
    const auto urdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const std::vector<std::filesystem::path> package_paths = {share_prefix};
    scene_ = std::make_unique<Scene>("test_scene", urdf_path, srdf_path, package_paths);
  }

public:
  // No default constructor, so must be a pointer.
  std::unique_ptr<Scene> scene_;
};

TEST_F(RoboPlanSceneTest, SceneProperties) {
  EXPECT_EQ(scene_->getName(), "test_scene");
  EXPECT_EQ(scene_->getModel().nq, 6u);
  EXPECT_THAT(scene_->getJointNames(),
              ContainerEq(std::vector<std::string>{"shoulder_pan_joint", "shoulder_lift_joint",
                                                   "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                                                   "wrist_3_joint"}));
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
  scene_->setRngSeed(1234); // reset seed
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
  EXPECT_FALSE(scene_->hasCollisionsAlongPath(q_start_free, q_end_free, 0.05));

  Eigen::VectorXd q_end_coll(6);
  q_end_coll << 0.0, -1.57, 3.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(scene_->hasCollisionsAlongPath(q_start_free, q_end_coll, 0.05));
}

} // namespace roboplan
