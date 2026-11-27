#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <tl/expected.hpp>

#include <roboplan_vex/vex.hpp>

#include "convex_problem.hpp"

// Internal SCP loop of roboplan_vex (not installed).
namespace roboplan::detail {

/// @brief One scalar row g(z) <= 0 of a nonconvex term.
struct NonconvexRow {
  double value = 0.0;

  /// @brief Sparse gradient dg/dz as (index into z, derivative) pairs.
  std::vector<std::pair<Eigen::Index, double>> jacobian;
};

/// @brief A nonconvex term, linearized at every iteration and penalized with ℓ1 slacks.
class NonconvexTerm {
public:
  virtual ~NonconvexTerm() = default;

  /// @brief Appends this term's rows at z, with their Jacobians, to `rows`.
  virtual tl::expected<void, std::string>
  evaluate(const VexLayout& layout, const Eigen::VectorXd& z, std::vector<NonconvexRow>& rows) = 0;
};

struct ScpResult {
  Eigen::VectorXd z;
  VexStatus status = VexStatus::QpFailure;
  size_t iterations = 0;
  double cost = 0.0;
  double max_violation = 0.0;
};

/// @brief Runs the ℓ1-penalty / trust-box SCP loop from a z that satisfies the hard rows.
/// @details Without nonconvex terms the first accepted step is optimal.
[[nodiscard]] tl::expected<ScpResult, std::string>
runScp(const ConvexProblem& problem, const std::vector<std::shared_ptr<NonconvexTerm>>& terms,
       Eigen::VectorXd z, const VexOptions& options);

}  // namespace roboplan::detail
