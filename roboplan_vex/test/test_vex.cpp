#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_vex/vex.hpp>

namespace roboplan {
namespace {

// QP stopping tolerance is 1e-9 in position units; 1e-6 allows for accumulation over the
// trajectory and the 1/dt^2 scaling of accelerations.
constexpr double kTolerance = 1e-6;

// Samples per segment for the between-knot checks.
constexpr int kSubsamples = 10;

std::shared_ptr<Scene> loadScene(const std::string& model_dir, const std::string& name,
                                 const std::string& config) {
  const auto prefix = example_models::get_package_models_dir() / model_dir;
  const std::vector<std::filesystem::path> package_paths = {
      example_models::get_package_share_dir()};
  auto scene = std::make_shared<Scene>(
      "test_scene", loadUrdfSceneDescription(prefix / (name + ".urdf"), package_paths));
  scene->importJointLimitsFromConfig(loadJointLimitsConfig(prefix / config));
  if (const auto imported = scene->importSrdf(loadTextFile(prefix / (name + ".srdf"))); !imported) {
    throw std::runtime_error(imported.error());
  }
  return scene;
}

// Position at time t in [0, dt] of segment k under the zero-order-hold acceleration.
Eigen::VectorXd sampleSegment(const JointTrajectory& traj, size_t k, double t) {
  return traj.positions.at(k) + traj.velocities.at(k) * t + 0.5 * traj.accelerations.at(k) * t * t;
}

class VexTest : public ::testing::Test {
protected:
  void SetUp() override {
    // The YAML config adds acceleration limits (5 rad/s^2), which the URDF lacks.
    scene_ = loadScene("ur_robot_model", "ur5_gripper", "ur5_config.yaml");
    joint_names_ = scene_->getJointGroupInfo("arm")->joint_names;
    options_.group_name = "arm";
    options_.num_nodes = 30;
  }

  [[nodiscard]] JointConfiguration config(const Eigen::VectorXd& q) const {
    return JointConfiguration{.joint_names = joint_names_, .positions = q};
  }

  [[nodiscard]] JointPath path(const std::vector<Eigen::VectorXd>& positions) const {
    return JointPath{.joint_names = joint_names_, .positions = positions};
  }

  // Checks dynamics, rest-to-rest and the Scene's limits, sampling between knots.
  void expectFeasible(const JointTrajectory& traj, const Eigen::VectorXd& start,
                      const Eigen::VectorXd& goal) const {
    const auto [q_min, q_max] = *scene_->getPositionLimitVectors("arm");
    const Eigen::VectorXd v_max = scene_->getVelocityLimitVectors("arm")->second;
    const Eigen::VectorXd a_max = scene_->getAccelerationLimitVectors("arm")->second;
    const size_t num_nodes = traj.positions.size();
    ASSERT_EQ(num_nodes, options_.num_nodes);
    EXPECT_EQ(traj.positions.front(), start);
    EXPECT_EQ(traj.positions.back(), goal);
    EXPECT_TRUE(traj.velocities.front().isZero());
    EXPECT_TRUE(traj.velocities.back().isZero());
    for (size_t k = 0; k + 1 < num_nodes; ++k) {
      const double dt = traj.times.at(k + 1) - traj.times.at(k);
      const auto& a = traj.accelerations.at(k);
      EXPECT_LT((sampleSegment(traj, k, dt) - traj.positions.at(k + 1)).lpNorm<Eigen::Infinity>(),
                kTolerance);
      EXPECT_LT(
          (traj.velocities.at(k) + a * dt - traj.velocities.at(k + 1)).lpNorm<Eigen::Infinity>(),
          kTolerance);
      EXPECT_TRUE((a.cwiseAbs().array() <= a_max.array() + kTolerance).all());
      for (int s = 0; s <= kSubsamples; ++s) {
        const double t = dt * s / kSubsamples;
        const Eigen::VectorXd q = sampleSegment(traj, k, t);
        const Eigen::VectorXd v = traj.velocities.at(k) + a * t;
        EXPECT_TRUE((q.array() >= q_min.array() - kTolerance).all());
        EXPECT_TRUE((q.array() <= q_max.array() + kTolerance).all());
        EXPECT_TRUE((v.cwiseAbs().array() <= v_max.array() + kTolerance).all());
      }
    }
  }

  std::shared_ptr<Scene> scene_;
  std::vector<std::string> joint_names_;
  VexOptions options_;
};

TEST_F(VexTest, NoObstaclesGivesStraightLine) {
  Eigen::VectorXd start(6);
  start << 0.0, -1.5, 1.0, -1.0, -1.5, 0.0;
  Eigen::VectorXd goal(6);
  goal << 0.5, -1.0, 0.5, -0.5, -1.0, 0.5;
  // A seed with a detour: the optimum is still the straight joint-space line.
  Eigen::VectorXd detour = 0.5 * (start + goal);
  detour(2) += 0.4;

  Vex vex(scene_, options_);
  const auto result = vex.solve(config(start), config(goal), 4.0, path({start, detour, goal}));
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result->status, VexStatus::Converged);
  EXPECT_EQ(result->iterations, 1u);
  EXPECT_EQ(result->max_violation, 0.0);
  EXPECT_FALSE(result->continuous_collision_free);
  EXPECT_EQ(result->trajectory.joint_names, joint_names_);
  expectFeasible(result->trajectory, start, goal);

  // Every sample lies on the segment start -> goal, at a fraction in [0, 1].
  const Eigen::VectorXd direction = goal - start;
  const auto& traj = result->trajectory;
  for (size_t k = 0; k + 1 < traj.positions.size(); ++k) {
    const double dt = traj.times.at(k + 1) - traj.times.at(k);
    for (int s = 0; s <= kSubsamples; ++s) {
      const Eigen::VectorXd offset = sampleSegment(traj, k, dt * s / kSubsamples) - start;
      const double fraction = offset.dot(direction) / direction.squaredNorm();
      EXPECT_LT((offset - fraction * direction).lpNorm<Eigen::Infinity>(), kTolerance);
      EXPECT_GE(fraction, -kTolerance);
      EXPECT_LE(fraction, 1.0 + kTolerance);
    }
  }

  // Same input, same output.
  const auto again = vex.solve(config(start), config(goal), 4.0, path({start, detour, goal}));
  ASSERT_TRUE(again.has_value());
  EXPECT_EQ(again->trajectory.positions, traj.positions);
  EXPECT_EQ(again->cost, result->cost);
}

TEST_F(VexTest, LimitsRespected) {
  // Moving 2 rad in 1.5 s: the energy optimum would need more than the 5 rad/s^2 acceleration
  // limit, which is feasible (bang-bang reaches 5 * 1.5^2 / 4 = 2.8 rad) and therefore active.
  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  start(0) = -1.0;
  Eigen::VectorXd goal = start;
  goal(0) = 1.0;

  Vex vex(scene_, options_);
  const auto result = vex.solve(config(start), config(goal), 1.5, path({start, goal}));
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result->status, VexStatus::Converged);
  expectFeasible(result->trajectory, start, goal);

  double max_acceleration = 0.0;
  for (const auto& a : result->trajectory.accelerations) {
    max_acceleration = std::max(max_acceleration, std::abs(a(0)));
  }
  // The limit is active up to the QP tolerance.
  EXPECT_NEAR(max_acceleration, 5.0, kTolerance);
}

TEST_F(VexTest, InfeasibleDurationReturnsError) {
  // 4 rad in 1.5 s needs more than the 2.8 rad that the acceleration limit allows.
  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  start(0) = -2.0;
  Eigen::VectorXd goal = start;
  goal(0) = 2.0;

  Vex vex(scene_, options_);
  const auto result = vex.solve(config(start), config(goal), 1.5, path({start, goal}));
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("No trajectory"), std::string::npos) << result.error();
}

TEST_F(VexTest, InvalidInputReturnsError) {
  const Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd goal = start;
  goal(0) = 0.5;
  const auto seed = path({start, goal});

  Vex vex(scene_, options_);
  EXPECT_FALSE(vex.solve(config(start), config(goal), 0.0, seed).has_value());
  EXPECT_FALSE(vex.solve(config(start), config(goal), -1.0, seed).has_value());

  Eigen::VectorXd outside = start;
  outside(0) = 4.0;  // Beyond the +-pi limit.
  EXPECT_FALSE(vex.solve(config(outside), config(goal), 2.0, seed).has_value());
  EXPECT_FALSE(vex.solve(config(start), config(outside), 2.0, seed).has_value());
  EXPECT_FALSE(vex.solve(config(start), config(goal.head(5)), 2.0, seed).has_value());

  auto wrong_names = seed;
  wrong_names.joint_names.pop_back();
  EXPECT_FALSE(vex.solve(config(start), config(goal), 2.0, wrong_names).has_value());
  auto wrong_size = seed;
  wrong_size.positions.back() = goal.head(5);
  EXPECT_FALSE(vex.solve(config(start), config(goal), 2.0, wrong_size).has_value());
  EXPECT_FALSE(vex.solve(config(start), config(goal), 2.0, path({start})).has_value());

  auto options = options_;
  options.num_nodes = 2;
  EXPECT_FALSE(Vex(scene_, options).solve(config(start), config(goal), 2.0, seed).has_value());
  options = options_;
  options.group_name = "no_such_group";
  EXPECT_FALSE(Vex(scene_, options).solve(config(start), config(goal), 2.0, seed).has_value());
}

TEST(VexLieGroupTest, ContinuousJointReturnsError) {
  // The Kinova arm has continuous joints (nq = 2, nv = 1).
  const auto scene =
      loadScene("kinova_robot_model", "kinova_robotiq", "kinova_robotiq_config.yaml");
  const auto group = scene->getJointGroupInfo("manipulator");
  ASSERT_TRUE(group.has_value());
  const Eigen::VectorXd q = scene->getCurrentJointPositions()(group->q_indices);
  const JointConfiguration configuration{.joint_names = group->joint_names, .positions = q};

  Vex vex(scene, VexOptions{.group_name = "manipulator"});
  const auto result = vex.solve(configuration, configuration, 2.0,
                                JointPath{.joint_names = group->joint_names, .positions = {q, q}});
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("vector-space"), std::string::npos) << result.error();
}

}  // namespace
}  // namespace roboplan
