#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <tl/expected.hpp>

// Internal convex part of the roboplan_vex problem (not installed; design L2/L4).
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

  /// @brief The matrices above in the scaled variables y (z = scale .* y): S P S, A S and C S.
  /// Computed once by buildConvexProblem.
  Eigen::SparseMatrix<double> P_scaled;
  Eigen::SparseMatrix<double> A_scaled;
  Eigen::SparseMatrix<double> C_scaled;

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

}  // namespace roboplan::detail
