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

// Internal SCP loop of roboplan_vex (not installed, no public extension API; design L2/L4). The
// convex part of the problem is in convex_problem.hpp.
namespace roboplan::detail {

/// @brief One scalar row of a nonconvex term at a point z.
struct NonconvexRow {
  /// @brief The value g(z); the row requires g <= 0.
  double value = 0.0;

  /// @brief Sparse gradient dg/dz as (variable index into z, derivative) pairs. Only the local
  /// variables the row touches (e.g. q_k, v_k, a_k of one segment) appear.
  std::vector<std::pair<Eigen::Index, double>> jacobian;
};

/// @brief A nonconvex term, linearized at every SCP iteration and penalized with ℓ1 slacks.
class NonconvexTerm {
public:
  virtual ~NonconvexTerm() = default;

  /// @brief Appends this term's rows at z to `rows`.
  /// @param compute_jacobian If false, only the values are needed (merit evaluation).
  virtual tl::expected<void, std::string> evaluate(const VexLayout& layout,
                                                   const Eigen::VectorXd& z, bool compute_jacobian,
                                                   std::vector<NonconvexRow>& rows) = 0;
};

/// @brief Output of the SCP loop.
struct ScpResult {
  Eigen::VectorXd z;
  VexStatus status = VexStatus::QpFailure;
  size_t iterations = 0;
  double cost = 0.0;
  double max_violation = 0.0;
};

/// @brief Runs the ℓ1-penalty / trust-box SCP loop from a point z that satisfies the hard rows.
/// @details Without nonconvex terms the QP model is exact, so no trust box is used and the first
/// accepted step is optimal (status Converged).
tl::expected<ScpResult, std::string>
runScp(const ConvexProblem& problem, const std::vector<std::shared_ptr<NonconvexTerm>>& terms,
       Eigen::VectorXd z, const VexOptions& options);

}  // namespace roboplan::detail
