#pragma once

#include <Eigen/Dense>

namespace roboplan {

// A cost is a soft objective described in terms the user understands (per-DoF targets and
// weights). Each is attached with `TrajectoryOptimizer::addCost(spec, window, weight)`, which
// returns a CostHandle whose `setTarget` updates the target between solves.

/// @brief Penalize reduced-group velocity deviation from a target.
struct VelocityCost {
  /// @brief Per-DoF velocity weights (size nv). Nonnegative.
  Eigen::VectorXd weights;

  /// @brief Target velocity (size nv); empty means zero.
  Eigen::VectorXd v_target;
};

}  // namespace roboplan
