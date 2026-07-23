#pragma once

namespace roboplan {

/// @brief Solver settings for the Oink QP solver (ProxQP).
///
/// This is a solver-agnostic subset of the underlying ProxQP settings. See
/// https://simple-robotics.github.io/proxsuite/ for the semantics of the tolerances.
struct OinkSettings {
  /// Absolute stopping tolerance on the primal/dual residuals.
  double eps_abs = 1e-6;
  /// Relative stopping tolerance on the primal/dual residuals (0 disables it).
  double eps_rel = 0.0;
  /// Maximum number of solver iterations.
  int max_iter = 10000;
  /// Print solver internals to stdout.
  bool verbose = false;
  /// Warm start each solve with the previous solution (recommended for control loops).
  bool warm_start = true;
  /// When the QP is primal-infeasible (e.g., a violated barrier conflicting with velocity
  /// limits), solve the closest feasible problem in the least-squares sense instead of
  /// failing. This guarantees that solveIk() always returns a usable displacement.
  bool primal_infeasibility_solving = true;
};

}  // namespace roboplan
