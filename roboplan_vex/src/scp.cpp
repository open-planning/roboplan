#include "scp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <roboplan_vex/qp_backend.hpp>

namespace roboplan::detail {

namespace {

using Triplets = std::vector<Eigen::Triplet<double>>;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// The SCP subproblem is feasible by construction (dy = 0 with s = max(g, 0)), so ProxQP's primal
// infeasibility certificate can only be a false positive there; it was observed on ~1 in 20
// subproblems at the default threshold (1e-4) and never at this one.
constexpr double kSubproblemPrimalInfeasibility = 1e-12;

// Weight of the velocity / acceleration regularization in the projection QP; only there so the
// QP Hessian is not singular on blocks that the equality rows already determine.
constexpr double kProjectionRegularization = 1e-6;

// Rows whose gradient norm is at or below this are constant under the step and stay out of the
// QP. Measured on the collision rows (ur5 + gripper): pairs the group cannot move have gradient
// norms below 1e-15, the smallest influenceable ones are many orders of magnitude above 1e-9.
constexpr double kMinRowGradientNorm = 1e-9;

bool isBounded(double value) { return std::isfinite(value) && std::abs(value) < 1e20; }

// Whether the step can change the row's value. A row with a (numerically) zero gradient, e.g. a
// collision pair between two bodies the group does not move, adds the same constant to the QP
// model and to the merit; it is therefore left out of the QP (no row, no slack) but still counted
// in the merit and the violation (design §6), so a violated one is still reported.
bool isInfluenceable(const NonconvexRow& row) {
  double squared_norm = 0.0;
  for (const auto& [index, derivative] : row.jacobian) {
    squared_norm += derivative * derivative;
  }
  return squared_norm > kMinRowGradientNorm * kMinRowGradientNorm;
}

Eigen::SparseMatrix<double> fromTriplets(Eigen::Index rows, Eigen::Index cols,
                                         const Triplets& triplets) {
  Eigen::SparseMatrix<double> matrix(rows, cols);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  return matrix;
}

void appendTriplets(const Eigen::SparseMatrix<double>& matrix, Eigen::Index row_offset,
                    Triplets& triplets) {
  for (Eigen::Index col = 0; col < matrix.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, col); it; ++it) {
      triplets.emplace_back(row_offset + it.row(), it.col(), it.value());
    }
  }
}

// Violation of one row: hinge for inequalities, absolute value for equalities.
double violation(const NonconvexRow& row, double value) {
  return row.equality ? std::abs(value) : std::max(0.0, value);
}

// Evaluates all terms at z, values only.
tl::expected<void, std::string>
evaluateTerms(const std::vector<std::shared_ptr<NonconvexTerm>>& terms, const VexLayout& layout,
              const Eigen::VectorXd& z, bool compute_jacobian, std::vector<NonconvexRow>& rows) {
  rows.clear();
  for (const auto& term : terms) {
    if (auto evaluated = term->evaluate(layout, z, compute_jacobian, rows); !evaluated) {
      return evaluated;
    }
  }
  return {};
}

double sumViolation(const std::vector<NonconvexRow>& rows) {
  double sum = 0.0;
  for (const auto& row : rows) {
    sum += violation(row, row.value);
  }
  return sum;
}

double maxViolation(const std::vector<NonconvexRow>& rows) {
  double max = 0.0;
  for (const auto& row : rows) {
    max = std::max(max, violation(row, row.value));
  }
  return max;
}

// The convex QP data mapped to the scaled variables y (z = scale .* y).
struct ScaledProblem {
  Eigen::SparseMatrix<double> hessian;
  Eigen::SparseMatrix<double> equality;
  Eigen::SparseMatrix<double> inequality;
};

ScaledProblem scaleProblem(const ConvexProblem& problem) {
  const auto scale = problem.scale.asDiagonal();
  ScaledProblem scaled;
  scaled.hessian = scale * problem.P * scale;
  scaled.equality = problem.A * scale;
  scaled.inequality = problem.C * scale;
  return scaled;
}

// Builds the SCP subproblem in increments dy (and one slack per QP row) at z:
//   min 1/2 dy^T H dy + (S P z)^T dy + penalty * sum(s)
// passed to the solver divided by `penalty` (same minimizer). With the slack cost at 1 the dual
// variables stay O(1) as the penalty grows; unscaled, ProxQP needed hundreds of iterations at
// penalty >= 1e2 and reported PROXQP_PRIMAL_INFEASIBLE at 1e5..1e6, although dy = 0 with
// s = max(g, 0) is always feasible.
//   s.t. A S dy = b - A z,  l - C z <= C S dy <= u - C z,  -box <= dy <= box,
//        J S dy - s <= -g (all rows),  J S dy + s >= -g (equality rows),  s >= 0.
SparseQp assembleSubproblem(const ConvexProblem& problem, const ScaledProblem& scaled,
                            const Eigen::VectorXd& z, const std::vector<const NonconvexRow*>& rows,
                            double penalty, double box) {
  const Eigen::Index nz = problem.layout.size();
  const auto ns = static_cast<Eigen::Index>(rows.size());
  const Eigen::Index nx = nz + ns;

  SparseQp qp;
  Triplets h_triplets;
  appendTriplets(scaled.hessian, 0, h_triplets);
  qp.H = fromTriplets(nx, nx, h_triplets) / penalty;
  qp.g = Eigen::VectorXd::Ones(nx);
  qp.g.head(nz) = problem.scale.cwiseProduct(problem.P * z) / penalty;

  Triplets a_triplets;
  appendTriplets(scaled.equality, 0, a_triplets);
  qp.A = fromTriplets(scaled.equality.rows(), nx, a_triplets);
  qp.b = problem.b - problem.A * z;

  const bool use_box = std::isfinite(box);
  const Eigen::Index num_equality_rows =
      std::count_if(rows.begin(), rows.end(), [](const auto* row) { return row->equality; });
  const Eigen::Index num_rows =
      scaled.inequality.rows() + (use_box ? nz : 0) + ns + num_equality_rows + ns;
  Triplets c_triplets;
  qp.l.resize(num_rows);
  qp.u.resize(num_rows);

  // Hard limit rows.
  appendTriplets(scaled.inequality, 0, c_triplets);
  const Eigen::VectorXd cz = problem.C * z;
  Eigen::Index row = scaled.inequality.rows();
  qp.l.head(row) = problem.l - cz;
  qp.u.head(row) = problem.u - cz;

  // Trust box on all trajectory variables.
  if (use_box) {
    for (Eigen::Index i = 0; i < nz; ++i, ++row) {
      c_triplets.emplace_back(row, i, 1.0);
      qp.l(row) = -box;
      qp.u(row) = box;
    }
  }

  // Linearized nonconvex rows with their ℓ1 slacks.
  for (Eigen::Index i = 0; i < ns; ++i) {
    const auto& nonconvex_row = *rows.at(static_cast<size_t>(i));
    const int num_sides = nonconvex_row.equality ? 2 : 1;
    for (int side = 0; side < num_sides; ++side, ++row) {
      for (const auto& [index, derivative] : nonconvex_row.jacobian) {
        c_triplets.emplace_back(row, index, derivative * problem.scale(index));
      }
      c_triplets.emplace_back(row, nz + i, side == 0 ? -1.0 : 1.0);
      qp.l(row) = side == 0 ? -kInfinity : -nonconvex_row.value;
      qp.u(row) = side == 0 ? -nonconvex_row.value : kInfinity;
    }
  }

  // Slacks are nonnegative.
  for (Eigen::Index i = 0; i < ns; ++i, ++row) {
    c_triplets.emplace_back(row, nz + i, 1.0);
    qp.l(row) = 0.0;
    qp.u(row) = kInfinity;
  }

  qp.C = fromTriplets(num_rows, nx, c_triplets);
  return qp;
}

// Penalty part of the QP model at z + dz, from the linearization at z.
double linearizedViolation(const std::vector<NonconvexRow>& rows, const Eigen::VectorXd& dz) {
  double sum = 0.0;
  for (const auto& row : rows) {
    double value = row.value;
    for (const auto& [index, derivative] : row.jacobian) {
      value += derivative * dz(index);
    }
    sum += violation(row, value);
  }
  return sum;
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
  return problem;
}

tl::expected<Eigen::VectorXd, std::string>
projectOntoConstraints(const ConvexProblem& problem, const std::vector<Eigen::VectorXd>& q_seed) {
  const auto& layout = problem.layout;
  const Eigen::Index nz = layout.size();
  const Eigen::Index nq = layout.num_nodes * layout.n;
  const ScaledProblem scaled = scaleProblem(problem);

  // min sum_k ||q_k - q_seed_k||^2 (+ tiny regularization) in the scaled variables, where y_q = q.
  SparseQp qp;
  Eigen::VectorXd diagonal = Eigen::VectorXd::Constant(nz, kProjectionRegularization);
  diagonal.head(nq).setOnes();
  qp.H = Eigen::SparseMatrix<double>(diagonal.asDiagonal());
  qp.g = Eigen::VectorXd::Zero(nz);
  for (Eigen::Index k = 0; k < layout.num_nodes; ++k) {
    qp.g.segment(layout.q(k), layout.n) = -q_seed.at(static_cast<size_t>(k));
  }
  qp.A = scaled.equality;
  qp.b = problem.b;
  qp.C = scaled.inequality;
  qp.l = problem.l;
  qp.u = problem.u;

  const auto y = solveSparseQp(qp, SparseQpSettings{});
  if (!y) {
    return tl::make_unexpected(y.error());
  }
  return Eigen::VectorXd(problem.scale.cwiseProduct(*y));
}

tl::expected<ScpResult, std::string>
runScp(const ConvexProblem& problem, const std::vector<std::shared_ptr<NonconvexTerm>>& terms,
       Eigen::VectorXd z, const VexOptions& options) {
  const auto start_time = std::chrono::steady_clock::now();
  const auto& layout = problem.layout;
  const Eigen::Index nz = layout.size();
  const ScaledProblem scaled = scaleProblem(problem);

  // Without nonconvex terms the QP model is exact: no trust box is needed.
  const bool has_terms = !terms.empty();
  double penalty = options.initial_penalty;
  double box = has_terms ? options.initial_trust_box : kInfinity;

  // `linearized` always holds all rows (values and Jacobians) at the current z: candidates are
  // evaluated with Jacobians, so an accepted candidate's rows are swapped in without evaluating
  // the terms again (the signed distances dominate the cost; the Jacobians are cheap).
  std::vector<NonconvexRow> linearized;
  std::vector<NonconvexRow> candidate_rows;
  std::vector<const NonconvexRow*> qp_rows;
  ScpResult result;
  if (auto evaluated = evaluateTerms(terms, layout, z, true, linearized); !evaluated) {
    return tl::make_unexpected(evaluated.error());
  }

  const auto finish = [&](VexStatus status) -> tl::expected<ScpResult, std::string> {
    result.z = z;
    result.status = status;
    result.cost = problem.cost(z);
    result.max_violation = maxViolation(linearized);
    return result;
  };
  const auto timed_out = [&]() {
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_time;
    return elapsed.count() > options.max_time;
  };

  for (size_t penalty_round = 0;; ++penalty_round) {
    bool converged = false;
    while (!converged) {
      // The merit counts all rows; only the rows the step can influence enter the QP.
      const double merit = problem.cost(z) + penalty * sumViolation(linearized);
      qp_rows.clear();
      for (const auto& row : linearized) {
        if (isInfluenceable(row)) {
          qp_rows.push_back(&row);
        }
      }

      // Trust region loop: shrink the box until a step passes the ratio test.
      while (true) {
        if (result.iterations >= options.max_iterations) {
          return finish(VexStatus::MaxIterations);
        }
        if (timed_out()) {
          return finish(VexStatus::Timeout);
        }
        ++result.iterations;

        // Primal warm start at dy = 0 with the smallest feasible slacks.
        Eigen::VectorXd guess =
            Eigen::VectorXd::Zero(nz + static_cast<Eigen::Index>(qp_rows.size()));
        for (size_t i = 0; i < qp_rows.size(); ++i) {
          guess(nz + static_cast<Eigen::Index>(i)) =
              violation(*qp_rows.at(i), qp_rows.at(i)->value);
        }
        const SparseQp qp = assembleSubproblem(problem, scaled, z, qp_rows, penalty, box);
        const auto solution = solveSparseQp(
            qp, SparseQpSettings{.eps_primal_inf = kSubproblemPrimalInfeasibility}, guess);
        if (!solution) {
          box *= options.trust_shrink;
          if (!has_terms || box < options.tolerance) {
            return finish(VexStatus::QpFailure);
          }
          continue;
        }

        const Eigen::VectorXd dz = problem.scale.cwiseProduct(solution->head(nz));
        const Eigen::VectorXd z_candidate = z + dz;
        const double candidate_cost = problem.cost(z_candidate);
        const double approx_improve =
            merit - (candidate_cost + penalty * linearizedViolation(linearized, dz));
        if (approx_improve < options.tolerance) {
          converged = true;
          break;
        }

        if (auto evaluated = evaluateTerms(terms, layout, z_candidate, true, candidate_rows);
            !evaluated) {
          return tl::make_unexpected(evaluated.error());
        }
        const double exact_improve =
            merit - (candidate_cost + penalty * sumViolation(candidate_rows));
        if (exact_improve / approx_improve >= options.improve_ratio) {
          z = z_candidate;
          linearized.swap(candidate_rows);
          box *= options.trust_expand;
          // An exact model's accepted step is the optimum.
          converged = !has_terms;
          break;
        }
        box *= options.trust_shrink;
        if (box < options.tolerance) {
          converged = true;
          break;
        }
      }
    }

    // Converged for this penalty: done if feasible, else increase the penalty.
    if (maxViolation(linearized) <= options.tolerance) {
      return finish(VexStatus::Converged);
    }
    if (penalty_round >= options.max_penalty_increases) {
      return finish(VexStatus::ConvergedInfeasible);
    }
    penalty *= options.penalty_increase;
    box = std::max(box, 1.5 * options.tolerance / options.trust_shrink);
  }
}

}  // namespace roboplan::detail
