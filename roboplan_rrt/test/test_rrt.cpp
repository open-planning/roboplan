#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_rrt/rrt.hpp>

namespace roboplan {

/// @brief Override the class to provide access to protected functions for testing.
class RRTTest : public RRT {
public:
  using RRT::grow_tree;
  using RRT::initialize_tree;
  using RRT::kd_tree_;
  using RRT::nodes_;
  using RRT::options_;
  using RRT::RRT;
};

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
};

TEST_F(RoboPlanRRTTest, Plan) {
  auto rrt = std::make_unique<RRT>(scene_);
  rrt->setRngSeed(1234);

  const auto maybe_q_start = scene_->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_start.has_value());
  const auto maybe_q_goal = scene_->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_goal.has_value());

  JointConfiguration start;
  start.positions = maybe_q_start.value();
  JointConfiguration goal;
  goal.positions = maybe_q_goal.value();

  const auto path = rrt->plan(start, goal);
  ASSERT_TRUE(path.has_value());
  std::cout << path.value() << "\n";
}

TEST_F(RoboPlanRRTTest, PlanRRTConnect) {
  RRTOptions options;
  options.rrt_connect = true;
  auto rrt = std::make_unique<RRT>(scene_, options);
  rrt->setRngSeed(1234);

  const auto maybe_q_start = scene_->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_start.has_value());
  const auto maybe_q_goal = scene_->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_goal.has_value());

  JointConfiguration start;
  start.positions = maybe_q_start.value();
  JointConfiguration goal;
  goal.positions = maybe_q_goal.value();

  const auto path = rrt->plan(start, goal);
  ASSERT_TRUE(path.has_value());
  std::cout << path.value() << "\n";
}

TEST_F(RoboPlanRRTTest, InvalidPoses) {
  auto rrt = std::make_unique<RRT>(scene_);
  rrt->setRngSeed(1234);

  const auto valid_pose = scene_->randomCollisionFreePositions().value();
  const Eigen::VectorXd invalid_pose{{-6, -6, -6, -6, -6, -6}};

  JointConfiguration start;
  start.positions = valid_pose;
  JointConfiguration goal;
  goal.positions = invalid_pose;

  // Planning will fail as the goal pose is unreachable due to joint limits.
  const auto path = rrt->plan(start, goal);
  ASSERT_FALSE(path.has_value());
}

TEST_F(RoboPlanRRTTest, PlanningTimeout) {
  // Set planning timeout to be impossibly short.
  RRTOptions options;
  options.max_planning_time = 1E-6;
  options.max_connection_distance = 0.1;
  auto rrt = std::make_unique<RRT>(scene_, options);
  rrt->setRngSeed(1234);

  const auto maybe_q_start = scene_->randomCollisionFreePositions();
  const auto maybe_q_goal = scene_->randomCollisionFreePositions();

  JointConfiguration start;
  start.positions = maybe_q_start.value();
  JointConfiguration goal;
  goal.positions = maybe_q_goal.value();

  // Planning will timeout.
  const auto path = rrt->plan(start, goal);
  ASSERT_FALSE(path.has_value());
}

TEST_F(RoboPlanRRTTest, TestGrowTree) {
  RRTOptions options;
  options.rrt_connect = false;
  options.max_connection_distance = 0.1;
  auto rrt = std::make_unique<RRTTest>(scene_, options);

  const Eigen::VectorXd q_start{{0, 0, 0, 0, 0, 0}};
  const Eigen::VectorXd q_extend_expected{{0.1, 0, 0, 0, 0, 0}};
  const Eigen::VectorXd q_end{{0.5, 0, 0, 0, 0, 0}};

  // Initialize the search to the start pose.
  auto tree = rrt->kd_tree_;
  auto nodes = rrt->nodes_;
  rrt->initialize_tree(tree, nodes, q_start);

  // Extending WITHOUT RRT-Connect will add exactly one node at the expected pose
  // which is exactly options.max_connection_distance away.
  ASSERT_TRUE(rrt->grow_tree(tree, nodes, q_end));
  ASSERT_EQ(nodes.size(), 2);
  ASSERT_EQ(nodes.back().config, q_extend_expected);

  // Reset the search tree and enable RRT Connect.
  rrt->options_.rrt_connect = true;
  rrt->initialize_tree(tree, nodes, q_start);

  // Extending WITH RRT-Connect will add exactly 6 nodes and reach q_end.
  ASSERT_TRUE(rrt->grow_tree(tree, nodes, q_end));
  ASSERT_EQ(nodes.size(), 6);
  ASSERT_EQ(nodes.back().config, q_end);
}

}  // namespace roboplan
