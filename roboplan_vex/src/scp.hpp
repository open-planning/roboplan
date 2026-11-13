#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <tl/expected.hpp>

#include <roboplan_vex/vex.hpp>

// Internal SCP engine of roboplan_vex (not installed, no public extension API; design L2/L4).
namespace roboplan::detail {

/// @brief Variable layout of the double-integrator model (physical units):
/// z = [q_0 .. q_{N-1}, v_0 .. v_{N-1}, a_0 .. a_{N-2}], each block of size n.
struct VexLayout {
  Eigen::Index n = 0;
  Eigen::Index num_nodes = 0;
  double dt = 0.0;

  Eigen::Index q(Eigen::Index k) const { return k * n; }
  Eigen::Index v(Eigen::Index k) const { return (num_nodes + k) * n; }
  Eigen::Index a(Eigen::Index k) const { return (2 * num_nodes + k) * n; }
  Eigen::Index size() const { return (3 * num_nodes - 1) * n; }
};

/// @brief One scalar row of a nonconvex term at a point z.
struct NonconvexRow {
  /// @brief The value g(z). Inequalities require g <= 0, equalities g == 0.
  double value = 0.0;

  /// @brief Whether the row is an equality.
  bool equality = false;

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

/// @brief Per-joint limits; entries that are non-finite or >= 1e20 in magnitude are unbounded.
struct JointBounds {
  Eigen::VectorXd q_min;
  Eigen::VectorXd q_max;
  Eigen::VectorXd v_max;
  Eigen::VectorXd a_max;
};

/// @brief The convex part of the problem, in physical units:
/// cost 1/2 z^T P z, hard rows A z = b and l <= C z <= u.
struct ConvexProblem {
  VexLayout layout;
  Eigen::SparseMatrix<double> P;
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b;
  Eigen::SparseMatrix<double> C;
  Eigen::VectorXd l;
  Eigen::VectorXd u;

  /// @brief Variable scaling z = scale .* y: the QP works in y, where q, v dt and a dt^2 all
  /// have position units. This conditions the QP and makes one trust box size fit all blocks.
  Eigen::VectorXd scale;

  double cost(const Eigen::VectorXd& z) const { return 0.5 * z.dot(P * z); }
};

/// @brief Builds the smoothness cost, dynamics, rest-to-rest boundary rows and limit rows.
ConvexProblem buildConvexProblem(const VexLayout& layout, const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal, const JointBounds& bounds,
                                 double velocity_weight, double acceleration_weight);

/// @brief Finds the trajectory closest to the seed positions (one per node) that satisfies all
/// hard rows of `problem`. Fails if the hard rows are infeasible.
tl::expected<Eigen::VectorXd, std::string>
projectOntoConstraints(const ConvexProblem& problem, const std::vector<Eigen::VectorXd>& q_seed);

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
