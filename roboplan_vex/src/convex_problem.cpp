#include "convex_problem.hpp"

#include <cmath>
#include <limits>

#include <roboplan_vex/qp_backend.hpp>

namespace roboplan::detail {

namespace {

using Triplets = std::vector<Eigen::Triplet<double>>;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Keeps the projection QP Hessian nonsingular on blocks the equality rows already determine.
constexpr double kProjectionRegularization = 1e-6;

bool isBounded(double value) { return std::isfinite(value) && std::abs(value) < 1e20; }

Eigen::SparseMatrix<double> fromTriplets(Eigen::Index rows, Eigen::Index cols,
                                         const Triplets& triplets) {
  Eigen::SparseMatrix<double> matrix(rows, cols);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  return matrix;
}

}  // namespace

ConvexProblem buildConvexProblem(const VexLayout& layout, const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal, const JointBounds& bounds,
                                 double velocity_weight, double acceleration_weight) {
  const Eigen::Index n = layout.n;
  const Eigen::Index num_nodes = layout.num_nodes;
  const Eigen::Index nz = layout.size();
  const double dt = layout.dt;

  ConvexProblem problem;
  problem.layout = layout;

  // Exact time integrals: v is piecewise linear, so per segment and joint
  //   int ||v||^2 dt = dt / 3 * (v_k^2 + v_k v_{k+1} + v_{k+1}^2),
  // and a is piecewise constant, so int ||a||^2 dt = dt * a_k^2.
  Triplets p_triplets;
  const double c_v = velocity_weight * dt / 3.0;
  for (Eigen::Index k = 0; k + 1 < num_nodes; ++k) {
    for (Eigen::Index j = 0; j < n; ++j) {
      const Eigen::Index v0 = layout.v(k) + j;
      const Eigen::Index v1 = layout.v(k + 1) + j;
      p_triplets.emplace_back(v0, v0, 2.0 * c_v);
      p_triplets.emplace_back(v1, v1, 2.0 * c_v);
      p_triplets.emplace_back(v0, v1, c_v);
      p_triplets.emplace_back(v1, v0, c_v);
      p_triplets.emplace_back(layout.a(k) + j, layout.a(k) + j, 2.0 * acceleration_weight * dt);
    }
  }
  problem.P = fromTriplets(nz, nz, p_triplets);

  // Dynamics and rest-to-rest boundary conditions.
  Triplets a_triplets;
  std::vector<double> b_values;
  Eigen::Index row = 0;
  for (Eigen::Index k = 0; k + 1 < num_nodes; ++k) {
    for (Eigen::Index j = 0; j < n; ++j) {
      // q_{k+1} - q_k - dt v_k - dt^2 / 2 a_k = 0
      a_triplets.emplace_back(row, layout.q(k + 1) + j, 1.0);
      a_triplets.emplace_back(row, layout.q(k) + j, -1.0);
      a_triplets.emplace_back(row, layout.v(k) + j, -dt);
      a_triplets.emplace_back(row, layout.a(k) + j, -0.5 * dt * dt);
      b_values.push_back(0.0);
      ++row;
      // v_{k+1} - v_k - dt a_k = 0
      a_triplets.emplace_back(row, layout.v(k + 1) + j, 1.0);
      a_triplets.emplace_back(row, layout.v(k) + j, -1.0);
      a_triplets.emplace_back(row, layout.a(k) + j, -dt);
      b_values.push_back(0.0);
      ++row;
    }
  }
  const auto fix = [&](Eigen::Index index, double value) {
    a_triplets.emplace_back(row++, index, 1.0);
    b_values.push_back(value);
  };
  for (Eigen::Index j = 0; j < n; ++j) {
    fix(layout.q(0) + j, start(j));
    fix(layout.q(num_nodes - 1) + j, goal(j));
    fix(layout.v(0) + j, 0.0);
    fix(layout.v(num_nodes - 1) + j, 0.0);
  }
  problem.A = fromTriplets(row, nz, a_triplets);
  problem.b = Eigen::Map<const Eigen::VectorXd>(b_values.data(),
                                                static_cast<Eigen::Index>(b_values.size()));

  // Joint limits at the nodes. Velocities are piecewise linear, so node bounds hold on the whole
  // segment; accelerations are constant per segment.
  Triplets c_triplets;
  std::vector<double> lower;
  std::vector<double> upper;
  const auto bound = [&](Eigen::Index index, double lo, double hi) {
    if (!isBounded(lo) && !isBounded(hi)) {
      return;
    }
    c_triplets.emplace_back(static_cast<Eigen::Index>(lower.size()), index, 1.0);
    lower.push_back(isBounded(lo) ? lo : -kInfinity);
    upper.push_back(isBounded(hi) ? hi : kInfinity);
  };
  for (Eigen::Index k = 0; k < num_nodes; ++k) {
    for (Eigen::Index j = 0; j < n; ++j) {
      bound(layout.q(k) + j, bounds.q_min(j), bounds.q_max(j));
      bound(layout.v(k) + j, -bounds.v_max(j), bounds.v_max(j));
      if (k + 1 < num_nodes) {
        bound(layout.a(k) + j, -bounds.a_max(j), bounds.a_max(j));
      }
    }
  }
  const auto num_limit_rows = static_cast<Eigen::Index>(lower.size());
  problem.C = fromTriplets(num_limit_rows, nz, c_triplets);
  problem.l = Eigen::Map<const Eigen::VectorXd>(lower.data(), num_limit_rows);
  problem.u = Eigen::Map<const Eigen::VectorXd>(upper.data(), num_limit_rows);

  problem.scale = Eigen::VectorXd::Ones(nz);
  problem.scale.segment(layout.v(0), num_nodes * n).setConstant(1.0 / dt);
  problem.scale.segment(layout.a(0), (num_nodes - 1) * n).setConstant(1.0 / (dt * dt));

  const auto scale = problem.scale.asDiagonal();
  problem.P_scaled = scale * problem.P * scale;
  problem.A_scaled = problem.A * scale;
  problem.C_scaled = problem.C * scale;
  return problem;
}

tl::expected<Eigen::VectorXd, std::string>
projectOntoConstraints(const ConvexProblem& problem, const std::vector<Eigen::VectorXd>& q_seed) {
  const auto& layout = problem.layout;
  const Eigen::Index nz = layout.size();
  const Eigen::Index nq = layout.num_nodes * layout.n;

  // min sum_k ||q_k - q_seed_k||^2 (+ tiny regularization) in the scaled variables, where y_q = q.
  SparseQp qp;
  Eigen::VectorXd diagonal = Eigen::VectorXd::Constant(nz, kProjectionRegularization);
  diagonal.head(nq).setOnes();
  qp.H = Eigen::SparseMatrix<double>(diagonal.asDiagonal());
  qp.g = Eigen::VectorXd::Zero(nz);
  for (Eigen::Index k = 0; k < layout.num_nodes; ++k) {
    qp.g.segment(layout.q(k), layout.n) = -q_seed.at(static_cast<size_t>(k));
  }
  qp.A = problem.A_scaled;
  qp.b = problem.b;
  qp.C = problem.C_scaled;
  qp.l = problem.l;
  qp.u = problem.u;

  const auto y = solveSparseQp(qp, SparseQpSettings{});
  if (!y) {
    return tl::make_unexpected(y.error());
  }
  return Eigen::VectorXd(problem.scale.cwiseProduct(*y));
}

}  // namespace roboplan::detail
