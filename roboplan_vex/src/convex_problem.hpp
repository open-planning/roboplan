#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <tl/expected.hpp>

// Internal convex part of the roboplan_vex problem (not installed).
namespace roboplan::detail {

/// @brief Layout z = [q_0 .. q_{N-1}, v_0 .. v_{N-1}, a_0 .. a_{N-2}], each block of size n.
struct VexLayout {
  Eigen::Index n = 0;
  Eigen::Index num_nodes = 0;
  double dt = 0.0;

  Eigen::Index q(Eigen::Index k) const { return k * n; }
  Eigen::Index v(Eigen::Index k) const { return (num_nodes + k) * n; }
  Eigen::Index a(Eigen::Index k) const { return (2 * num_nodes + k) * n; }
  Eigen::Index size() const { return (3 * num_nodes - 1) * n; }
};

/// @brief Per-joint limits; non-finite entries and magnitudes >= 1e20 are unbounded.
struct JointBounds {
  Eigen::VectorXd q_min;
  Eigen::VectorXd q_max;
  Eigen::VectorXd v_max;
  Eigen::VectorXd a_max;
};

/// @brief Cost 1/2 z^T P z with hard rows A z = b and l <= C z <= u, in physical units.
struct ConvexProblem {
  VexLayout layout;
  Eigen::SparseMatrix<double> P;
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b;
  Eigen::SparseMatrix<double> C;
  Eigen::VectorXd l;
  Eigen::VectorXd u;

  /// @brief The same limits per variable (infinite where unbounded).
  Eigen::VectorXd z_min;
  Eigen::VectorXd z_max;

  /// @brief Scaling z = scale .* y; q, v dt and a dt^2 all have position units in y.
  Eigen::VectorXd scale;

  /// @brief S P S, A S and C S for the scaled variables, computed once.
  Eigen::SparseMatrix<double> P_scaled;
  Eigen::SparseMatrix<double> A_scaled;
  Eigen::SparseMatrix<double> C_scaled;

  double cost(const Eigen::VectorXd& z) const { return 0.5 * z.dot(P * z); }
};

/// @brief Builds the smoothness cost, dynamics, rest-to-rest boundary rows and limit rows.
[[nodiscard]] ConvexProblem buildConvexProblem(const VexLayout& layout,
                                               const Eigen::VectorXd& start,
                                               const Eigen::VectorXd& goal,
                                               const JointBounds& bounds, double velocity_weight,
                                               double acceleration_weight);

/// @brief Finds the z closest to the seed positions (one per node) that satisfies the hard rows.
/// @return An error if the hard rows are infeasible.
[[nodiscard]] tl::expected<Eigen::VectorXd, std::string>
projectOntoConstraints(const ConvexProblem& problem, const std::vector<Eigen::VectorXd>& q_seed);

}  // namespace roboplan::detail
