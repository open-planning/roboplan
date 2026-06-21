#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <vector>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_rrt/rrt.hpp>

namespace roboplan {

class RoboPlanRRTTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const std::vector<std::filesystem::path> package_paths = {
        example_models::get_package_share_dir()};
    scene = std::make_shared<Scene>("test_scene", urdf_path, srdf_path, package_paths);
  }

public:
  // No default constructors, so must be pointers.
  std::shared_ptr<Scene> scene;
};

TEST_F(RoboPlanRRTTest, Plan) {
  RRTOptions options;
  options.group_name = "arm";
  auto rrt = std::make_unique<RRT>(scene, options);
  rrt->setRngSeed(1234);

  const auto maybe_q_start = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_start.has_value());
  const auto maybe_q_goal = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_goal.has_value());

  JointConfiguration start;
  start.positions = maybe_q_start.value();
  JointConfiguration goal;
  goal.positions = maybe_q_goal.value();

  const auto maybe_path = rrt->plan(start, goal);
  ASSERT_TRUE(maybe_path.has_value());

  // Ensure the path starts and ends at the correct configurations.
  const auto path = maybe_path.value();
  std::cout << path << "\n";
  ASSERT_EQ(path.positions[0], start.positions);
  ASSERT_EQ(path.positions.back(), goal.positions);
}

TEST_F(RoboPlanRRTTest, PlanRRTConnect) {
  RRTOptions options;
  options.group_name = "arm";
  options.rrt_connect = true;
  auto rrt = std::make_unique<RRT>(scene, options);
  rrt->setRngSeed(1234);

  const auto maybe_q_start = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_start.has_value());
  const auto maybe_q_goal = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_goal.has_value());

  JointConfiguration start;
  start.positions = maybe_q_start.value();
  JointConfiguration goal;
  goal.positions = maybe_q_goal.value();

  const auto maybe_path = rrt->plan(start, goal);
  ASSERT_TRUE(maybe_path.has_value());

  // Ensure the path starts and ends at the correct configurations.
  const auto path = maybe_path.value();
  std::cout << path << "\n";
  ASSERT_EQ(path.positions[0], start.positions);
  ASSERT_EQ(path.positions.back(), goal.positions);
}

TEST_F(RoboPlanRRTTest, PlanRRTStar) {
  // Plan the same problem with and without RRT*. RRT* keeps rewiring and optimizing until its
  // budget runs out, so its path must be equal or shorter than plain RRT.
  //
  // Seed the scene RNG so the start/goal pair is fixed: this seed yields a non-trivial problem
  // where plain RRT wanders noticeably, so rewiring produces a clearly shorter path (and the test
  // is reproducible instead of depending on a random problem each run).
  scene->setRngSeed(4);
  JointConfiguration start, goal;
  start.positions = scene->randomCollisionFreePositions().value();
  goal.positions = scene->randomCollisionFreePositions().value();

  const auto plan_with = [&](bool rrt_star) {
    RRTOptions options;
    options.group_name = "arm";
    options.rrt_star = rrt_star;
    // Disable fast_return so RRT* optimizes, and bound the search by a fixed node budget (not a
    // wall-clock time budget). A node budget makes both runs do exactly the same amount of work
    // for the same seed, so the comparison is deterministic and independent of machine load.
    options.fast_return = false;
    options.max_nodes = 150;
    auto rrt = std::make_unique<RRT>(scene, options);
    rrt->setRngSeed(1234);
    return rrt->plan(start, goal);
  };

  const auto maybe_star_path = plan_with(/*rrt_star*/ true);
  const auto maybe_rrt_path = plan_with(/*rrt_star*/ false);
  ASSERT_TRUE(maybe_star_path.has_value());
  ASSERT_TRUE(maybe_rrt_path.has_value());

  // Ensure the path starts and ends at the correct configurations.
  const auto path = maybe_star_path.value();
  std::cout << path << "\n";
  ASSERT_EQ(path.positions[0], start.positions);
  ASSERT_EQ(path.positions.back(), goal.positions);

  // RRT* must never produce a longer path than plain RRT.
  const auto star_length = computePathLength(*scene, "arm", maybe_star_path.value()).value();
  const auto rrt_length = computePathLength(*scene, "arm", maybe_rrt_path.value()).value();
  EXPECT_LE(star_length, rrt_length);
}

TEST_F(RoboPlanRRTTest, PlanRRTStarConnect) {
  // RRT* rewiring combined with the bidirectional RRT-Connect tree growth, contrasted against plain
  // RRT-Connect on the same (seeded) problem. As with single-tree RRT*, the rewired path must be
  // equal or shorter than its non-star counterpart.
  //
  // Seed the scene RNG so the start/goal pair is fixed and reproducible (see PlanRRTStar). Plain
  // RRT-Connect already produces fairly direct paths, so rewiring's benefit is smaller and needs a
  // slightly larger budget to show than for single-tree RRT*; this seed still gives a clear gain.
  scene->setRngSeed(4);
  JointConfiguration start, goal;
  start.positions = scene->randomCollisionFreePositions().value();
  goal.positions = scene->randomCollisionFreePositions().value();

  const auto plan_with = [&](bool rrt_star) {
    RRTOptions options;
    options.group_name = "arm";
    options.rrt_connect = true;
    options.rrt_star = rrt_star;
    // Bound by a fixed node budget rather than wall-clock time, so the comparison is deterministic
    // and independent of machine load (see PlanRRTStar for the rationale).
    options.fast_return = false;
    options.max_nodes = 150;
    auto rrt = std::make_unique<RRT>(scene, options);
    rrt->setRngSeed(1234);
    return rrt->plan(start, goal);
  };

  const auto maybe_star_path = plan_with(/*rrt_star*/ true);
  const auto maybe_connect_path = plan_with(/*rrt_star*/ false);
  ASSERT_TRUE(maybe_star_path.has_value());
  ASSERT_TRUE(maybe_connect_path.has_value());

  // Ensure the path starts and ends at the correct configurations.
  const auto path = maybe_star_path.value();
  std::cout << path << "\n";
  ASSERT_EQ(path.positions[0], start.positions);
  ASSERT_EQ(path.positions.back(), goal.positions);

  // RRT*-Connect must never produce a longer path than plain RRT-Connect.
  const auto star_length = computePathLength(*scene, "arm", maybe_star_path.value()).value();
  const auto connect_length = computePathLength(*scene, "arm", maybe_connect_path.value()).value();
  EXPECT_LE(star_length, connect_length);
}

TEST_F(RoboPlanRRTTest, FastReturnUsesFullBudget) {
  // fast_return is independent of the planner mode: with plain RRT, disabling it should keep
  // planning past the first solution until the node budget is exhausted.
  const auto plan_and_count = [this](bool fast_return) {
    RRTOptions options;
    options.group_name = "arm";
    options.max_connection_distance = 0.5;
    options.max_nodes = 200;
    options.fast_return = fast_return;
    options.max_planning_time = 1.0;
    auto rrt = std::make_unique<RRT>(scene, options);
    rrt->setRngSeed(1234);

    JointConfiguration start, goal;
    start.positions = scene->randomCollisionFreePositions().value();
    goal.positions = scene->randomCollisionFreePositions().value();

    const auto maybe_path = rrt->plan(start, goal);
    EXPECT_TRUE(maybe_path.has_value());
    const auto [start_nodes, goal_nodes] = rrt->getNodes();
    return start_nodes.size() + goal_nodes.size();
  };

  // Returning on the first path uses fewer nodes than running to the full budget.
  EXPECT_LT(plan_and_count(/*fast_return*/ true), plan_and_count(/*fast_return*/ false));
}

TEST_F(RoboPlanRRTTest, InvalidConfigurations) {
  RRTOptions options;
  options.group_name = "arm";
  auto rrt = std::make_unique<RRT>(scene, options);
  rrt->setRngSeed(1234);

  const auto valid_pose = scene->randomCollisionFreePositions().value();
  const Eigen::VectorXd invalid_pose{{-6, -6, -6, -6, -6, -6}};

  JointConfiguration start;
  start.positions = valid_pose;
  JointConfiguration goal;
  goal.positions = invalid_pose;

  // Planning will fail as the goal configuration is unreachable due to joint limits.
  const auto path = rrt->plan(start, goal);
  ASSERT_FALSE(path.has_value());
}

TEST_F(RoboPlanRRTTest, PlanningTimeout) {
  // Set planning timeout to be impossibly short.
  RRTOptions options;
  options.group_name = "arm";
  options.max_planning_time = 1E-6;
  options.max_connection_distance = 0.1;
  auto rrt = std::make_unique<RRT>(scene, options);
  rrt->setRngSeed(1234);

  const auto maybe_q_start = scene->randomCollisionFreePositions();
  const auto maybe_q_goal = scene->randomCollisionFreePositions();

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
  options.group_name = "arm";
  options.rrt_connect = false;
  options.max_connection_distance = 0.1;
  auto rrt = std::make_unique<RRT>(scene, options);

  const Eigen::VectorXd q_start{{0, 0, 0, 0, 0, 0}};
  const Eigen::VectorXd q_extend_expected{{0.1, 0, 0, 0, 0, 0}};
  const Eigen::VectorXd q_end{{0.5, 0, 0, 0, 0, 0}};

  const CollisionContext collision_context(*scene);

  // Initialize the search to the start configuration.
  KdTree tree;
  std::vector<Node> nodes;
  rrt->initializeTree(tree, nodes, q_start);

  // A single EXTEND step adds exactly one node at the expected configuration,
  // which is exactly options.max_connection_distance away.
  ASSERT_TRUE(rrt->growTree(tree, nodes, q_end, collision_context, /*greedy*/ false));
  ASSERT_EQ(nodes.size(), 2);
  ASSERT_EQ(nodes.back().config, q_extend_expected);

  // Reset the search tree and enable RRT-Connect.
  options.rrt_connect = true;
  auto rrt_connect = std::make_unique<RRT>(scene, options);
  rrt_connect->initializeTree(tree, nodes, q_start);

  // A greedy CONNECT step will add exactly 6 nodes and reach q_end.
  ASSERT_TRUE(rrt_connect->growTree(tree, nodes, q_end, collision_context, /*greedy*/ true));
  ASSERT_EQ(nodes.size(), 6);
  ASSERT_EQ(nodes.back().config, q_end);
}

TEST_F(RoboPlanRRTTest, TestJoinTrees) {
  RRTOptions options;
  options.group_name = "arm";
  options.rrt_connect = false;
  options.max_connection_distance = 0.1;
  auto rrt = std::make_unique<RRT>(scene, options);

  // Tree1 Nodes
  const Eigen::VectorXd q_start{{0, 0, 0, 0, 0, 0}};
  const Eigen::VectorXd q_start_nearest{{0.1, 0, 0, 0, 0, 0}};

  // Tree2 Nodes
  const Eigen::VectorXd q_goal_nearest{{0.2, 0, 0, 0, 0, 0}};
  const Eigen::VectorXd q_goal{{0.3, 0, 0, 0, 0, 0}};

  const std::vector<Eigen::VectorXd> expected_positions = {q_start, q_start_nearest, q_goal_nearest,
                                                           q_goal};

  const CollisionContext collision_context(*scene);

  // Initialize the search to the start configuration.
  KdTree start_tree, goal_tree;
  std::vector<Node> start_nodes, goal_nodes;
  rrt->initializeTree(start_tree, start_nodes, q_start);
  rrt->initializeTree(goal_tree, goal_nodes, q_goal);

  // The nodes should both be appended directly to the start and goal nodes.
  ASSERT_TRUE(
      rrt->growTree(start_tree, start_nodes, q_start_nearest, collision_context, /*greedy*/ false));
  ASSERT_EQ(start_nodes.size(), 2);
  ASSERT_EQ(start_nodes.back().config, q_start_nearest);

  ASSERT_TRUE(
      rrt->growTree(goal_tree, goal_nodes, q_goal_nearest, collision_context, /*greedy*/ false));
  ASSERT_EQ(goal_nodes.size(), 2);
  ASSERT_EQ(goal_nodes.back().config, q_goal_nearest);

  // Starting from the start_tree, the trees should be joinable.
  const auto maybe_path =
      rrt->joinTrees(start_nodes, goal_tree, goal_nodes, true, collision_context);
  ASSERT_TRUE(maybe_path.has_value());
  ASSERT_EQ(maybe_path.value().first.positions, expected_positions);

  // Starting from the goal_tree, the trees should be joinable.
  const auto maybe_path2 =
      rrt->joinTrees(goal_nodes, start_tree, start_nodes, false, collision_context);
  ASSERT_TRUE(maybe_path2.has_value());
  ASSERT_EQ(maybe_path2.value().first.positions, expected_positions);
}

}  // namespace roboplan
