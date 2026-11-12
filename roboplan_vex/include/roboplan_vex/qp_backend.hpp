#pragma once

#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <tl/expected.hpp>

// Private interface to the sparse ProxQP backend. qp_backend.cpp is the only translation unit
// that includes the (compile-time expensive) proxsuite headers; nothing else in roboplan_vex sees
// them, and proxsuite is not part of the package's public link interface.
namespace roboplan::detail {

/// @brief A sparse convex QP:
///   minimize    1/2 x^T H x + g^T x
///   subject to  A x = b,  l <= C x <= u.
/// @details Only the upper triangle of H is read. A / C may have zero rows. Bounds of magnitude
/// above 1e20 (including +-infinity) are treated as unbounded by ProxQP.
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
};

/// @brief Solves a sparse QP with ProxQP, re-initializing the solver from scratch.
/// @param qp The problem; dimensions must be consistent.
/// @param settings The solver settings.
/// @param primal_guess Optional primal warm start (size = number of variables).
/// @return The primal solution, or a message describing why the solve failed.
tl::expected<Eigen::VectorXd, std::string>
solveSparseQp(const SparseQp& qp, const SparseQpSettings& settings,
              std::optional<Eigen::Ref<const Eigen::VectorXd>> primal_guess = std::nullopt);

}  // namespace roboplan::detail
