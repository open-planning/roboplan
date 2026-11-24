#include <algorithm>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <roboplan/core/geometry_wrappers.hpp>
#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_utils.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_ompl/ompl_planner.hpp>

namespace roboplan {

std::shared_ptr<Scene> loadScene(const std::string& model_dir, const std::string& model_name) {
  const auto model_prefix = example_models::get_package_models_dir() / model_dir;
  const std::vector<std::filesystem::path> package_paths = {
      example_models::get_package_share_dir()};
  const auto description =
      loadUrdfSceneDescription(model_prefix / (model_name + ".urdf"), package_paths);
  auto scene = std::make_shared<Scene>("test_scene", description);
  if (const auto imported = scene->importSrdf(loadTextFile(model_prefix / (model_name + ".srdf")));
      !imported) {
    throw std::runtime_error(imported.error());
  }
  scene->setRngSeed(1234);
  return scene;
}

class OMPLPlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    scene = loadScene("ur_robot_model", "ur5_gripper");
    // A pillar in the sweep between start and goal, so they are not directly connectable.
    Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
    tform.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.4, 0.4);
    ASSERT_TRUE(scene
                    ->addBoxGeometry("pillar", "universe", Box(0.1, 0.1, 0.8), tform,
                                     Eigen::Vector4d(1.0, 0.0, 0.0, 1.0))
                    .has_value());
    start.positions = Eigen::VectorXd{{0.0, -2.0, 2.0, -1.5, -1.5, 0.0}};
    goal.positions = Eigen::VectorXd{{3.0, -2.0, 2.0, -1.5, -1.5, 0.0}};
  }

  std::shared_ptr<Scene> scene;
  JointConfiguration start;
  JointConfiguration goal;
};

TEST_F(OMPLPlannerTest, StateSpaceRoundTrip) {
  auto space = std::make_shared<SceneStateSpace>(*scene, "arm");
  EXPECT_EQ(space->getDimension(), 6u);
  ompl::base::ScopedState<> s1(space);
  ompl::base::ScopedState<> s2(space);
  space->fromGroupPositions(start.positions, s1.get());
  space->fromGroupPositions(goal.positions, s2.get());
  EXPECT_TRUE(space->toGroupPositions(s1.get()).isApprox(start.positions));
  EXPECT_NEAR(space->distance(s1.get(), s2.get()),
              scene->configurationDistance(scene->toFullJointPositions("arm", start.positions),
                                           scene->toFullJointPositions("arm", goal.positions)),
              1e-9);
}

TEST(OMPLStateSpaceTest, ContinuousJointsUseSO2) {
  // The Kinova arm has four continuous joints, each stored as [cos, sin] in the configuration.
  const auto scene = loadScene("kinova_robot_model", "kinova_robotiq");
  auto space = std::make_shared<SceneStateSpace>(*scene, "manipulator");
  const auto q_indices = scene->getJointGroupInfo("manipulator")->q_indices;
  EXPECT_EQ(space->getDimension(), 7u);
  EXPECT_EQ(q_indices.size(), 11);

  const Eigen::VectorXd q = scene->randomCollisionFreePositions().value()(q_indices);
  ompl::base::ScopedState<> state(space);
  space->fromGroupPositions(q, state.get());
  EXPECT_TRUE(space->toGroupPositions(state.get()).isApprox(q));

  // Informed planners fall back to rejection sampling on this space and must still plan.
  JointConfiguration start;
  start.positions = q;
  JointConfiguration goal;
  goal.positions = scene->randomCollisionFreePositions().value()(q_indices);
  OMPLPlannerOptions options;
  options.group_name = "manipulator";
  options.planner_name = "AORRTC";
  options.max_planning_time = 0.5;
  OMPLPlanner planner(scene, options);
  const auto path = planner.plan(start, goal);
  ASSERT_TRUE(path.has_value()) << path.error();
  EXPECT_TRUE(path->positions.back().isApprox(goal.positions));
}

TEST(OMPLStateSpaceTest, PlanarJointsUseR2xSO2) {
  // The Reachback robot has a planar base joint, stored as [x, y, cos, sin] in the configuration.
  const auto scene = loadScene("reachback_robot_model", "reachback");
  auto space = std::make_shared<SceneStateSpace>(*scene, "arm_base");
  const auto q_indices = scene->getJointGroupInfo("arm_base")->q_indices;
  EXPECT_EQ(space->getDimension(), 9u);
  EXPECT_EQ(q_indices.size(), 10);
  // The URDF leaves the base translation unbounded, so it gets the Scene's default sampling range.
  const auto& bounds = space->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
  EXPECT_DOUBLE_EQ(bounds.low[0], -kDefaultPlanarJointTranslationLimit);
  EXPECT_DOUBLE_EQ(bounds.high[1], kDefaultPlanarJointTranslationLimit);

  const Eigen::VectorXd q = scene->randomCollisionFreePositions().value()(q_indices);
  ompl::base::ScopedState<> state(space);
  space->fromGroupPositions(q, state.get());
  EXPECT_TRUE(space->toGroupPositions(state.get()).isApprox(q));

  JointConfiguration start;
  start.positions = q;
  JointConfiguration goal;
  goal.positions = scene->randomCollisionFreePositions().value()(q_indices);
  OMPLPlannerOptions options;
  options.group_name = "arm_base";
  options.max_planning_time = 0.5;
  OMPLPlanner planner(scene, options);
  const auto path = planner.plan(start, goal);
  ASSERT_TRUE(path.has_value()) << path.error();
  EXPECT_TRUE(path->positions.front().isApprox(start.positions));
  EXPECT_TRUE(path->positions.back().isApprox(goal.positions));
}

TEST_F(OMPLPlannerTest, DirectOMPLUsage) {
  auto space = std::make_shared<SceneStateSpace>(*scene, "arm");
  auto context = std::make_shared<SceneContext>(*scene);
  const auto si = makeSpaceInformation(space, context, 0.05, true);
  ompl::base::ScopedState<> s1(space);
  ompl::base::ScopedState<> s2(space);
  space->fromGroupPositions(start.positions, s1.get());
  space->fromGroupPositions(goal.positions, s2.get());
  EXPECT_TRUE(si->isValid(s1.get()));
  EXPECT_TRUE(si->isValid(s2.get()));
  // A colliding configuration: fold the arm into itself.
  Eigen::VectorXd q_collide = Eigen::VectorXd::Zero(6);
  q_collide(1) = -3.0;
  q_collide(2) = 3.0;
  space->fromGroupPositions(q_collide, s2.get());
  EXPECT_FALSE(si->isValid(s2.get()));
  EXPECT_FALSE(si->checkMotion(s1.get(), s2.get()));
  // The planner factory hands out plain OMPL planners.
  EXPECT_NE(std::dynamic_pointer_cast<ompl::geometric::RRTConnect>(makePlanner("RRTConnect", si)),
            nullptr);
  EXPECT_EQ(makePlanner("NotAPlanner", si), nullptr);
}

TEST_F(OMPLPlannerTest, PlanRRTConnectAndAORRTC) {
  for (const std::string planner_name : {"RRTConnect", "AORRTC"}) {
    OMPLPlannerOptions options;
    options.group_name = "arm";
    options.planner_name = planner_name;
    options.max_planning_time = 1.0;
    options.simplify_path = true;
    OMPLPlanner planner(scene, options);

    const auto path = planner.plan(start, goal);
    ASSERT_TRUE(path.has_value()) << path.error();
    EXPECT_EQ(path->joint_names, scene->getJointGroupInfo("arm")->joint_names);
    EXPECT_GT(path->positions.size(), 2u);
    EXPECT_TRUE(path->positions.front().isApprox(start.positions));
    EXPECT_TRUE(path->positions.back().isApprox(goal.positions));
    EXPECT_GT(planner.getLastStats().planning_time, 0.0);
    EXPECT_GT(planner.getLastStats().num_state_checks, 0u);
    EXPECT_GT(planner.getLastStats().num_motion_checks, 0u);
    // AORRTC clears its trees between iterations, so only RRT-Connect reports a graph.
    EXPECT_EQ(planner.getEdges().empty(), planner_name == "AORRTC");
    if (planner_name == "RRTConnect") {
      EXPECT_FALSE(planner.getNodes().empty());
      for (const auto& [i, j] : planner.getEdges()) {
        EXPECT_LT(i, planner.getNodes().size());
        EXPECT_LT(j, planner.getNodes().size());
      }
      // Both trees are reported: tag 1 for the start tree and 2 for the goal tree.
      for (const int tag : {1, 2}) {
        EXPECT_TRUE(std::any_of(planner.getNodes().begin(), planner.getNodes().end(),
                                [tag](const auto& node) { return node.second == tag; }));
      }
    }
    for (size_t i = 1; i < path->positions.size(); ++i) {
      EXPECT_FALSE(
          hasCollisionsAlongPath(*scene, scene->toFullJointPositions("arm", path->positions[i - 1]),
                                 scene->toFullJointPositions("arm", path->positions[i]), 0.05));
    }
  }
}

TEST_F(OMPLPlannerTest, DirectConnection) {
  // Without the pillar the straight line is free, and is returned without running the planner.
  ASSERT_TRUE(scene->removeGeometry("pillar").has_value());
  OMPLPlannerOptions options;
  options.group_name = "arm";
  options.planner_name = "AORRTC";
  OMPLPlanner planner(scene, options);
  const auto path = planner.plan(start, goal);
  ASSERT_TRUE(path.has_value()) << path.error();
  EXPECT_EQ(path->positions.size(), 2u);
  EXPECT_TRUE(path->positions.front().isApprox(start.positions));
  EXPECT_TRUE(path->positions.back().isApprox(goal.positions));
  EXPECT_EQ(planner.getLastStats().num_motion_checks, 0u);
}

TEST_F(OMPLPlannerTest, PlannerParamsAndErrors) {
  OMPLPlannerOptions options;
  options.group_name = "arm";
  options.planner_params = {{"range", "0.5"}};
  OMPLPlanner planner(scene, options);
  ASSERT_TRUE(planner.plan(start, goal).has_value());

  options.planner_params = {{"not_a_param", "1"}};
  planner.setOptions(options);
  EXPECT_FALSE(planner.plan(start, goal).has_value());

  options.planner_params.clear();
  options.planner_name = "NotAPlanner";
  planner.setOptions(options);
  EXPECT_FALSE(planner.plan(start, goal).has_value());

  options.planner_name = "RRTConnect";
  planner.setOptions(options);
  goal.positions.setZero();
  goal.positions(1) = -3.0;
  goal.positions(2) = 3.0;
  const auto result = planner.plan(start, goal);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "Goal configuration is in collision, cannot plan!");

  options.group_name = "not_a_group";
  EXPECT_THROW(OMPLPlanner(scene, options), std::runtime_error);
}

}  // namespace roboplan
