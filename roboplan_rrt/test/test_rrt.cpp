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
    scene_ = std::make_shared<Scene>("test_scene", urdf_path, srdf_path, package_paths);

    rrt_ = std::make_unique<RRT>(scene_, options_);
  }

public:
  // No default constructors, so must be pointers.
  std::shared_ptr<Scene> scene_;
  std::unique_ptr<RRT> rrt_;

  RRTOptions options_;
};

TEST_F(RoboPlanRRTTest, Plan) {
  rrt_->setRngSeed(1234);

  const auto maybe_q_start = scene_->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_start.has_value());
  const auto maybe_q_goal = scene_->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_goal.has_value());

  JointConfiguration start;
  start.positions = maybe_q_start.value();
  JointConfiguration goal;
  goal.positions = maybe_q_goal.value();

  const auto path = rrt_->plan(start, goal);
  ASSERT_TRUE(path.has_value());
  std::cout << path.value() << "\n";
}

}  // namespace roboplan
