#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

#include <roboplan_aligator/types.hpp>

#include "test_util.hpp"

namespace roboplan {
namespace {

using testing::makeSo101Scene;
// SO-101 with the 5-DoF "arm" group (strict subset; FR3/dual_fr3 fail to load in core, so so101 is
// the loadable strict-subset fixture). See test_reduced_group_model.cpp for the rationale.

}  // namespace

// --- StageWindow: half-open validation --------------------------------------------------------

TEST(StageWindowTest, AllResolvesToEveryStage) {
  const StageWindow w = StageWindow::all();
  EXPECT_EQ(w.kind(), StageWindow::Kind::All);
  EXPECT_FALSE(w.isTerminal());
  EXPECT_EQ(w.resolveStages(5), (std::vector<int>{0, 1, 2, 3, 4}));
  EXPECT_EQ(w.resolveStages(1), (std::vector<int>{0}));
}

TEST(StageWindowTest, RangeIsHalfOpen) {
  const StageWindow w = StageWindow::range(1, 4);
  EXPECT_EQ(w.kind(), StageWindow::Kind::Range);
  EXPECT_FALSE(w.isTerminal());
  // [1, 4) excludes the upper bound: stages 1, 2, 3 — the documented half-open convention.
  EXPECT_EQ(w.resolveStages(5), (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(StageWindow::range(2, 3).resolveStages(5), (std::vector<int>{2}));  // single stage
  EXPECT_EQ(StageWindow::range(0, 5).resolveStages(5), (std::vector<int>{0, 1, 2, 3, 4}));  // full
}

TEST(StageWindowTest, TerminalAttachesToNoStage) {
  const StageWindow w = StageWindow::terminal();
  EXPECT_EQ(w.kind(), StageWindow::Kind::Terminal);
  EXPECT_TRUE(w.isTerminal());
  // Terminal targets the terminal node, so it resolves to an empty stage list.
  EXPECT_TRUE(w.resolveStages(5).empty());
}

TEST(StageWindowTest, InvalidRangeConstructionThrows) {
  // Negative begin / empty range (end == begin) / inverted range (end < begin).
  EXPECT_THROW(StageWindow::range(-1, 3), std::invalid_argument);
  EXPECT_THROW(StageWindow::range(2, 2), std::invalid_argument);
  EXPECT_THROW(StageWindow::range(4, 2), std::invalid_argument);
}

TEST(StageWindowTest, ResolveRejectsOutOfHorizonRange) {
  // end > horizon violates b <= N (begin at/after horizon is caught the same way).
  EXPECT_THROW(StageWindow::range(2, 6).resolveStages(5), std::invalid_argument);
  EXPECT_THROW(StageWindow::range(5, 7).resolveStages(5), std::invalid_argument);
}

TEST(StageWindowTest, ResolveRejectsNonPositiveHorizon) {
  EXPECT_THROW(StageWindow::all().resolveStages(0), std::invalid_argument);
  EXPECT_THROW(StageWindow::all().resolveStages(-3), std::invalid_argument);
  EXPECT_THROW(StageWindow::terminal().resolveStages(0), std::invalid_argument);
}

// --- TrajOptResult::toRoboplan round-trip -----------------------------------------------------

TEST(TrajOptResultTest, ToRoboplanExpandsReducedPositionsToFullLayout) {
  auto scene = makeSo101Scene();
  const std::string group_name = "arm";

  const auto group = scene->getJointGroupInfo(group_name);
  ASSERT_TRUE(group.has_value());
  const int nq_reduced = static_cast<int>(group->q_indices.size());

  // Hand-build a two-knot result trajectory in reduced-group layout.
  TrajOptResult result;
  Eigen::VectorXd q0 = Eigen::VectorXd::LinSpaced(nq_reduced, 0.1, 0.5);
  Eigen::VectorXd q1 = Eigen::VectorXd::LinSpaced(nq_reduced, -0.2, 0.3);
  result.trajectory.times = {0.0, 0.02};
  result.trajectory.positions = {q0, q1};
  result.trajectory.velocities = {Eigen::VectorXd::Ones(nq_reduced),
                                  -Eigen::VectorXd::Ones(nq_reduced)};

  const JointTrajectory jt = result.toRoboplan(*scene, group_name);

  // Labels and times pass through; positions/velocities expand to full-model size.
  EXPECT_EQ(jt.joint_names, scene->getJointNames());
  EXPECT_EQ(jt.times, result.trajectory.times);
  ASSERT_EQ(jt.positions.size(), result.trajectory.positions.size());
  ASSERT_EQ(jt.velocities.size(), result.trajectory.velocities.size());

  const Eigen::VectorXd& full_q0 = scene->getCurrentJointPositions();
  for (std::size_t k = 0; k < jt.positions.size(); ++k) {
    // Full-model configuration/velocity size.
    EXPECT_EQ(jt.positions[k].size(), full_q0.size());
    EXPECT_EQ(jt.velocities[k].size(), scene->getModel().nv);
    // Round-trip: the group's slice of the full vector equals the reduced input we put in.
    const Eigen::VectorXd& q_reduced = result.trajectory.positions[k];
    const Eigen::VectorXd& v_reduced = result.trajectory.velocities[k];
    for (int i = 0; i < group->q_indices.size(); ++i) {
      EXPECT_DOUBLE_EQ(jt.positions[k](group->q_indices(i)), q_reduced(i));
    }
    for (int i = 0; i < group->v_indices.size(); ++i) {
      EXPECT_DOUBLE_EQ(jt.velocities[k](group->v_indices(i)), v_reduced(i));
    }
    // Non-group velocity DoF are exactly zero (toFullJointVelocities).
    for (int idx = 0; idx < jt.velocities[k].size(); ++idx) {
      if ((group->v_indices.array() == idx).any()) {
        continue;
      }
      EXPECT_DOUBLE_EQ(jt.velocities[k](idx), 0.0);
    }
  }

  // Accelerations are intentionally empty: not a ProxDDP output.
  EXPECT_TRUE(jt.accelerations.empty());
}

TEST(TrajOptResultTest, ToRoboplanUnknownGroupThrows) {
  auto scene = makeSo101Scene();
  TrajOptResult result;
  result.trajectory.times = {0.0};
  result.trajectory.positions = {Eigen::VectorXd::Zero(5)};
  // toFullJointPositions throws for an unknown group.
  EXPECT_THROW(result.toRoboplan(*scene, "not_a_group"), std::exception);
}

}  // namespace roboplan
