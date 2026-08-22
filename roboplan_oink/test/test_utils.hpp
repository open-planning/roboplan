#pragma once

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_context.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Poses `oink`'s context at `scene`'s current joint positions and returns it.
/// @details Oink::solveIk() does this on entry. These tests drive tasks, constraints, and barriers
/// one method at a time, so they perform the same step a real solve would.
inline const SceneContext& posed(Oink& oink, const Scene& scene) {
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();
  oink.getContext().setJointPositions(q);
  oink.getContext().updateFramePlacements(q);
  return oink.getContext();
}

}  // namespace roboplan
