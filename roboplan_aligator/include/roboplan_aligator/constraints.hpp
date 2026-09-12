#pragma once

#include <string>

#include <Eigen/Dense>

namespace roboplan {

// A constraint is a hard bound described in terms the user understands (per-DoF bounds). Each is
// attached with `TrajectoryOptimizer::addConstraint(spec, window)` over a window of stages. Unlike
// costs, constraints are fixed at build time — rebuild via `resetProblem()` to change them.
//
// Bound vectors are in the optimizer's REDUCED-group layout (size nv for torques — the same
// layout as `setInitialState`). A user-supplied bound only ever tightens the model's own physical
// limit (never loosens it); leaving a bound empty uses the model default.

/// @brief Symmetric box limit on the control (joint torque) u, giving the box [-tau_max, +tau_max].
/// @details Defaults to the reduced model's effort limits when left empty. A DoF whose model effort
/// limit is non-finite or zero is treated as unbounded. Not allowed on the terminal window (the
/// terminal node has no control).
struct TorqueLimit {
  /// @brief Symmetric per-DoF torque bound (size nv). Empty ⇒ the model's effort limits.
  Eigen::VectorXd tau_max;
};

}  // namespace roboplan
