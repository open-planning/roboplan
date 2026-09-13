#pragma once

#include <Eigen/Dense>

namespace roboplan {

// A cost is a soft objective described in terms the user understands (per-DoF targets and
// weights). Each is attached with `TrajectoryOptimizer::addCost(spec, window, weight)`, which
// returns a CostHandle whose `setTarget` updates the target between solves.

/// @brief Penalize deviation of the reduced-group configuration from a target.
struct ConfigurationCost {
  /// @brief Target reduced-group configuration (size nq).
  Eigen::VectorXd q_target;

  /// @brief Per-DoF weights (size nv). Nonnegative.
  Eigen::VectorXd weights;
};

}  // namespace roboplan
