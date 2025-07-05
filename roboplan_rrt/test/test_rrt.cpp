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
  }

public:
  // No default constructors, so must be pointers.
  std::shared_ptr<Scene> scene_;
  std::unique_ptr<RRT> rrt_;
};

TEST_F(RoboPlanRRTTest, Plan) {
  rrt_ = std::make_unique<RRT>(scene_);
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

TEST_F(RoboPlanRRTTest, InvalidPoses) {
  rrt_ = std::make_unique<RRT>(scene_);
  rrt_->setRngSeed(1234);

  const auto valid_pose = scene_->randomCollisionFreePositions().value();
  const Eigen::VectorXd invalid_pose{{-6, -6, -6, -6, -6, -6}};

  JointConfiguration start;
  start.positions = valid_pose;
  JointConfiguration goal;
  goal.positions = invalid_pose;

  // Planning will fail as the goal pose is unreachable due to joint limits.
  const auto path = rrt_->plan(start, goal);
  ASSERT_FALSE(path.has_value());
}

TEST_F(RoboPlanRRTTest, PlanningTimeout) {
  // Set planning timeout to be impossibly short.
  RRTOptions options;
  options.max_planning_time = 1E-6;
  options.max_connection_distance = 0.1;
  rrt_ = std::make_unique<RRT>(scene_, options);
  rrt_->setRngSeed(1234);

  const auto maybe_q_start = scene_->randomCollisionFreePositions();
  const auto maybe_q_goal = scene_->randomCollisionFreePositions();

  JointConfiguration start;
  start.positions = maybe_q_start.value();
  JointConfiguration goal;
  goal.positions = maybe_q_goal.value();

  // Planning will timeout.
  const auto path = rrt_->plan(start, goal);
  ASSERT_FALSE(path.has_value());
}

}  // namespace roboplan
