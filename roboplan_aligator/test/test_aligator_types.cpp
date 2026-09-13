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
