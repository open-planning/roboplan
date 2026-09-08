#pragma once

#include <memory>
#include <string>

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

/// @brief Penalize reduced-group velocity deviation from a target.
struct VelocityCost {
  /// @brief Per-DoF velocity weights (size nv). Nonnegative.
  Eigen::VectorXd weights;

  /// @brief Target velocity (size nv); empty means zero.
  Eigen::VectorXd v_target;
};

/// @brief Mutable handle to an attached cost, for target updates between solves.
/// @details Returned by `TrajectoryOptimizer::addCost`. `setTarget` rewrites the target in place
/// (no rebuild, legal between solves). Move-only; dangles if the owning optimizer is destroyed or
/// `resetProblem()` is called.
class CostHandle {
public:
  /// @brief Constructs an empty handle (references nothing). Provided for default-construction
  /// only.
  CostHandle();
  ~CostHandle();

  CostHandle(CostHandle&&) noexcept;
  CostHandle& operator=(CostHandle&&) noexcept;
  CostHandle(const CostHandle&) = delete;
  CostHandle& operator=(const CostHandle&) = delete;

  /// @brief Sets a new target vector for a ConfigurationCost (q) or VelocityCost (v) handle.
  /// @throws std::invalid_argument if `target` has the wrong size for the cost.
  void setTarget(const Eigen::VectorXd& target);

private:
  friend class TrajectoryOptimizer;
  struct Impl;
  explicit CostHandle(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace roboplan
