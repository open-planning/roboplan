#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_rrt/rrt.hpp>

namespace roboplan {

class RoboPlanRRTTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto share_prefix = roboplan_example_models::get_package_share_dir();
    const auto urdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const std::vector<std::filesystem::path> package_paths = {share_prefix};
    scene_ = std::make_unique<Scene>("test_scene", urdf_path, srdf_path, package_paths);
    rrt_ = std::make_unique<RRT>(*scene_, RRTOptions());
  }

public:
  // No default constructors, so must be pointers.
  std::unique_ptr<Scene> scene_;
  std::unique_ptr<RRT> rrt_;
};

TEST_F(RoboPlanRRTTest, Plan) {
  // TODO: Make method to generate valid collision-free positions.
  JointConfiguration q_start, q_goal;
  while (true) {
    q_start.positions = scene_->randomPositions();
    if (!scene_->hasCollisions(q_start.positions)) {
      break;
    }
  }
  while (true) {
    q_goal.positions = scene_->randomPositions();
    if (!scene_->hasCollisions(q_goal.positions)) {
      break;
    }
  }

  rrt_->plan(q_start, q_goal);
}

}  // namespace roboplan
