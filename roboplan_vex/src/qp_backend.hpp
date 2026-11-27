#pragma once

#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <tl/expected.hpp>

// Private interface to the sparse ProxQP backend. qp_backend.cpp is the only file that includes
// proxsuite.
namespace roboplan::detail {

/// @brief A sparse convex QP:
///   minimize    1/2 x^T H x + g^T x
///   subject to  A x = b,  l <= C x <= u.
/// @details Only the upper triangle of H is read. A and C may have zero rows. Bounds of magnitude
/// above 1e20 (including +-infinity) are unbounded.
struct SparseQp {
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd g;
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b;
  Eigen::SparseMatrix<double> C;
  Eigen::VectorXd l;
  Eigen::VectorXd u;
};

/// @brief Settings of the sparse QP solver.
struct SparseQpSettings {
  /// @brief Absolute stopping tolerance.
  double eps_abs = 1e-9;

  /// @brief Relative stopping tolerance.
  double eps_rel = 0.0;

  /// @brief Maximum number of ProxQP iterations.
  int max_iter = 10000;

  /// @brief Initial inequality proximal parameter mu_in of ProxQP's augmented Lagrangian.
  /// @details Much smaller than ProxQP's default on purpose, see vex.rst (Implementation Notes).
  double mu_in = 1e-3;

  /// @brief Threshold of ProxQP's primal infeasibility certificate (ProxQP default 1e-4).
  /// @details Lower it for QPs that are feasible by construction. A solve that then fails to
  /// converge ends at max_iter.
  double eps_primal_inf = 1e-4;
};

/// @brief Solves a sparse QP with ProxQP. Stateless: the solver is rebuilt on every call.
/// @param qp The problem; dimensions must be consistent.
/// @param settings The solver settings.
/// @param primal_guess Optional primal warm start (one entry per variable).
/// @return The primal solution, or a message describing why the solve failed.
[[nodiscard]] tl::expected<Eigen::VectorXd, std::string>
solveSparseQp(const SparseQp& qp, const SparseQpSettings& settings,
              std::optional<Eigen::Ref<const Eigen::VectorXd>> primal_guess = std::nullopt);

}  // namespace roboplan::detail
