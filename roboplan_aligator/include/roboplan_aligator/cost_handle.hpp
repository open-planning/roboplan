#pragma once

#include <memory>

#include <Eigen/Dense>

namespace roboplan {

/// @brief Mutable handle to an attached cost, for target updates between solves.
/// @details Returned by `TrajectoryOptimizer::addCost`. `setTarget` rewrites the target in place
/// (no rebuild, legal between solves). Move-only; dangles if the owning optimizer is destroyed or
/// `resetProblem()` is called. Shared infrastructure used by every cost type (ConfigurationCost,
/// VelocityCost, ...) — not a cost type itself.
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
