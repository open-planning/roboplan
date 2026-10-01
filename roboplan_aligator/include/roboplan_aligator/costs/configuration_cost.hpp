#pragma once

#include <Eigen/Dense>

namespace roboplan {

// A cost is a soft objective described in terms the user understands (per-DoF targets and
// weights). Attach with `TrajectoryOptimizer::addCost`/`addStageCost`/`addTerminalCost`.

/// @brief Penalize deviation of the reduced-group configuration from a target.
struct ConfigurationCost {
  /// @brief Target reduced-group configuration (size nq).
  Eigen::VectorXd q_target;

  /// @brief Per-DoF weights (size nv). Nonnegative.
  Eigen::VectorXd weights;
};

}  // namespace roboplan
