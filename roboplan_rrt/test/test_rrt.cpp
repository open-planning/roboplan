#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <vector>

#include <pinocchio/math/rpy.hpp>

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

/// @brief Fixture for the pose constraint and constrained planning tests.
/// @details Constrains the UR5 tool frame to hold its z axis near vertical, leaving position and
/// the rotation about that axis free. The nominal orientation is a half turn about x, so a zero
/// displacement points the tool straight down.
class RoboPlanConstrainedRRTTest : public RoboPlanRRTTest {
protected:
  void SetUp() override {
    RoboPlanRRTTest::SetUp();

    Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
    tform.topLeftCorner<3, 3>() = pinocchio::rpy::rpyToMatrix(std::numbers::pi, 0.0, 0.0);

    const double tilt = 0.1;
    const double inf = std::numeric_limits<double>::infinity();
    TaskSpaceVector lower, upper;
    lower << -inf, -inf, -inf, -tilt, -tilt, -std::numbers::pi;
    upper << inf, inf, inf, tilt, tilt, std::numbers::pi;

    constraint = std::make_shared<PoseConstraint>(scene, "arm", "tool0", lower, upper, tform);
    constraints = {constraint};
    projector = std::make_unique<ConstraintProjector>(scene, "arm", constraints);
    q_indices = scene->getJointGroupInfo("arm").value().q_indices;

    // Projection is a local operation and does not converge from every configuration sampled
    // uniformly at random, since those can start arbitrarily far from the constraint. Pin the
    // scene RNG to a seed whose first two draws both project, so every test below starts from the
    // same reproducible pair rather than from whatever the draw happens to give.
    scene->setRngSeed(kSeed);
  }

public:
  /// @brief Scene RNG seed whose first two collision-free draws both project onto the constraint
  /// and are connectable under it.
  /// @details Connectability is the scarce part. This arm has six joints and the constraint pins
  /// two rotational degrees of freedom, so the set it leaves behind is thin and splits into
  /// components that a constrained path cannot cross -- reaching one from another would mean
  /// tipping the tool over. Most random pairs land in different components and are genuinely
  /// unreachable from one another, so the seed is pinned to a pair that is not.
  static constexpr unsigned int kSeed = 13;

public:
  std::shared_ptr<PoseConstraint> constraint;
  std::vector<std::shared_ptr<Constraint>> constraints;
  std::unique_ptr<ConstraintProjector> projector;
  Eigen::VectorXi q_indices;
};

TEST_F(RoboPlanConstrainedRRTTest, DisplacementMatchesForwardKinematics) {
  const auto maybe_q = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q.has_value());
  const auto& q = maybe_q.value();

  // The displacement is the tool pose expressed in the region frame, so recomposing it against the
  // region transform must reproduce plain forward kinematics.
  const auto displacement = constraint->computeDisplacement(q);
  const pinocchio::SE3 region(constraint->getTransform());
  const pinocchio::SE3 recomposed(pinocchio::rpy::rpyToMatrix(displacement.tail<3>()),
                                  displacement.head<3>());
  const Eigen::Matrix4d expected = scene->forwardKinematics(q, "tool0");
  ASSERT_TRUE((region * recomposed).toHomogeneousMatrix().isApprox(expected, 1.0e-9));
}

TEST_F(RoboPlanConstrainedRRTTest, UnboundedCoordinatesStayFree) {
  const auto maybe_q = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q.has_value());

  auto q = maybe_q.value();
  ASSERT_TRUE(projector->project(q));
  ASSERT_TRUE(projector->satisfies(q));

  // Position is unbounded here, so its residual must be exactly zero rather than merely small:
  // an unbounded coordinate contributes no rows to the projection at all.
  const auto error = constraint->computeError(q);
  ASSERT_EQ(error.head<3>().norm(), 0.0);

  // And the orientation must have been driven inside its bounds.
  const auto displacement = constraint->computeDisplacement(q);
  ASSERT_LE(std::abs(displacement(3)), 0.1 + constraint->getOrientationTolerance());
  ASSERT_LE(std::abs(displacement(4)), 0.1 + constraint->getOrientationTolerance());
}

TEST_F(RoboPlanConstrainedRRTTest, ProjectionLeavesConfigurationsInsideAlone) {
  const auto maybe_q = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q.has_value());

  auto q = maybe_q.value();
  ASSERT_TRUE(projector->project(q));

  // Projecting an already-satisfying configuration is a no-op: the residual is zero everywhere it
  // is inside the bounds, so there is nothing to correct and nothing to pull it toward.
  const auto q_before = q;
  ASSERT_TRUE(projector->project(q));
  ASSERT_TRUE(q.isApprox(q_before, 1.0e-12));
}

TEST_F(RoboPlanConstrainedRRTTest, PlanRejectsEndpointsOffTheConstraint) {
  // Draw before the planner exists: RRT::setRngSeed also reseeds the scene, which would discard
  // the fixture's pinned seed and with it the guarantee that this draw projects.
  const auto maybe_q_on = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_on.has_value());
  auto q_on = maybe_q_on.value();
  ASSERT_TRUE(projector->project(q_on));

  RRTOptions options;
  options.group_name = "arm";
  auto rrt = std::make_unique<RRT>(scene, options);
  rrt->setRngSeed(1234);

  // A configuration with the tool nearly horizontal is far outside a 0.1 rad tilt bound.
  Eigen::VectorXd q_off = q_on;
  q_off(4) += 1.5;
  ASSERT_FALSE(projector->satisfies(q_off));

  JointConfiguration on, off;
  on.positions = q_on(q_indices);
  off.positions = q_off(q_indices);

  ASSERT_FALSE(rrt->plan(off, on, constraints).has_value());
  ASSERT_FALSE(rrt->plan(on, off, constraints).has_value());
}

TEST_F(RoboPlanConstrainedRRTTest, PlannedPathStaysOnTheConstraint) {
  RRTOptions options;
  options.group_name = "arm";
  options.rrt_connect = true;
  options.max_connection_distance = 0.5;
  options.max_nodes = 20000;
  options.max_planning_time = 20.0;
  options.constraint_projection.path_step_size = 0.1;

  // Both endpoints must start on the constraint, so project them first. Draw them before the
  // planner exists: RRT::setRngSeed also reseeds the scene, which would discard the fixture's
  // pinned seed and with it the guarantee that these draws project.
  const auto maybe_q_start = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_start.has_value());
  auto q_start = maybe_q_start.value();
  ASSERT_TRUE(projector->project(q_start));

  const auto maybe_q_goal = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q_goal.has_value());
  auto q_goal = maybe_q_goal.value();
  ASSERT_TRUE(projector->project(q_goal));

  auto rrt = std::make_unique<RRT>(scene, options);
  rrt->setRngSeed(1234);

  JointConfiguration start, goal;
  start.positions = q_start(q_indices);
  goal.positions = q_goal(q_indices);

  const auto maybe_path = rrt->plan(start, goal, constraints);
  ASSERT_TRUE(maybe_path.has_value()) << maybe_path.error();
  const auto& path = maybe_path.value();
  ASSERT_GE(path.positions.size(), 2);

  // Every waypoint, and every edge between them, must satisfy the constraint at the resolution the
  // planner checked. This is the guarantee that separates constrained planning from planning that
  // merely starts and ends on the constraint.
  for (size_t idx = 0; idx + 1 < path.positions.size(); ++idx) {
    const auto q_a = scene->toFullJointPositions("arm", path.positions[idx]);
    const auto q_b = scene->toFullJointPositions("arm", path.positions[idx + 1]);
    ASSERT_TRUE(projector->satisfies(q_a)) << "waypoint " << idx << " left the constraint";
    ASSERT_TRUE(projector->satisfiesAlongPath(q_a, q_b, options.collision_check_step_size))
        << "edge " << idx << " left the constraint";
  }
  ASSERT_TRUE(projector->satisfies(scene->toFullJointPositions("arm", path.positions.back())));
}

/// @brief Builds a constraint whose bounds are pinned, so its residual is its displacement.
std::shared_ptr<PoseConstraint> makePinnedConstraint(const std::shared_ptr<Scene>& scene,
                                                     const std::string& frame_name,
                                                     const std::string& reference_frame,
                                                     const Eigen::Matrix4d& tform) {
  return std::make_shared<PoseConstraint>(scene, "arm", frame_name, TaskSpaceVector::Zero(),
                                          TaskSpaceVector::Zero(), tform, reference_frame);
}

TEST_F(RoboPlanConstrainedRRTTest, ReferenceFrameMakesTheConstraintRelative) {
  const auto maybe_q = scene->randomCollisionFreePositions();
  ASSERT_TRUE(maybe_q.has_value());
  const auto& q = maybe_q.value();

  const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  auto relative = makePinnedConstraint(scene, "tool0", "wrist_1_link", identity);

  // The displacement must be the tool pose expressed in the reference frame, not in the world.
  const auto displacement = relative->computeDisplacement(q);
  const pinocchio::SE3 expected(
      Eigen::Matrix4d(scene->forwardKinematics(q, "tool0", "wrist_1_link")));
  ASSERT_TRUE(displacement.head<3>().isApprox(expected.translation(), 1.0e-9));
  ASSERT_TRUE(
      pinocchio::rpy::rpyToMatrix(displacement.tail<3>()).isApprox(expected.rotation(), 1.0e-9));

  // Joints upstream of both frames move them rigidly together, so they cannot change a relative
  // displacement. The first three UR5 joints are upstream of wrist_1_link, so their Jacobian
  // columns must vanish -- the property that separates a relative constraint from an absolute one.
  const auto v_indices = scene->getJointGroupInfo("arm").value().v_indices;
  Eigen::VectorXd error(6);
  Eigen::MatrixXd jacobian(6, v_indices.size());
  relative->computeErrorAndJacobian(q, error, jacobian);
  ASSERT_TRUE(jacobian.leftCols<3>().isZero(1.0e-9)) << jacobian;
  ASSERT_FALSE(jacobian.rightCols<3>().isZero(1.0e-9)) << jacobian;
}

}  // namespace roboplan
