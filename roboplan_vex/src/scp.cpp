#include "scp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include <roboplan_vex/qp_backend.hpp>

namespace roboplan::detail {

// SCP loop (ScpLoop::run), starting from a z that satisfies the hard rows:
//
//   penalty round                       (penalty *= penalty_increase, <= max_penalty_increases)
//     repeat until converged:           <- ScpLoop::trustRegionStep
//       merit(z) = cost(z) + penalty * sum_i violation(g_i(z))
//       QP at z: minimize the linearized merit s.t. hard rows and |dz| <= box
//         QP failed             -> box *= trust_shrink, retry
//         predicted gain < tol  -> converged
//       evaluate g at z + dz
//         actual / predicted >= improve_ratio -> accept, z += dz, box *= trust_expand
//         else                                -> box *= trust_shrink, retry (box < tol: converged)
//     max_violation <= tol -> Converged; else next round, or ConvergedInfeasible after the last.
//
// MaxIterations and Timeout end the loop from any step.

namespace {

using Triplets = std::vector<Eigen::Triplet<double>>;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// The SCP subproblem is feasible by construction (dy = 0 with s = max(g, 0)), so ProxQP's primal
// infeasibility certificate can only be a false positive there; it was observed on ~1 in 20
// subproblems at the default threshold (1e-4) and never at this one.
constexpr double kSubproblemPrimalInfeasibility = 1e-12;

// Rows whose gradient norm is at or below this are constant under the step and stay out of the
// QP. Measured on the collision rows (ur5 + gripper): pairs the group cannot move have gradient
// norms below 1e-15, the smallest influenceable ones are many orders of magnitude above 1e-9.
constexpr double kMinRowGradientNorm = 1e-9;

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

// Violation of a row with value g: the hinge max(0, g).
double violation(double value) { return std::max(0.0, value); }

// Evaluates all terms at z; the Jacobians only if requested.
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
    sum += violation(row.value);
  }
  return sum;
}

double maxViolation(const std::vector<NonconvexRow>& rows) {
  double max = 0.0;
  for (const auto& row : rows) {
    max = std::max(max, violation(row.value));
  }
  return max;
}

// Builds the SCP subproblem in increments dy (and one slack per QP row) at z:
//   min 1/2 dy^T H dy + (S P z)^T dy + penalty * sum(s)
// passed to the solver divided by `penalty` (same minimizer). With the slack cost at 1 the dual
// variables stay O(1) as the penalty grows; unscaled, ProxQP needed hundreds of iterations at
// penalty >= 1e2 and reported PROXQP_PRIMAL_INFEASIBLE at 1e5..1e6, although dy = 0 with
// s = max(g, 0) is always feasible.
//   s.t. A S dy = b - A z,  l - C z <= C S dy <= u - C z,  -box <= dy <= box,
//        J S dy - s <= -g (all rows),  s >= 0.
SparseQp assembleSubproblem(const ConvexProblem& problem, const Eigen::VectorXd& z,
                            const std::vector<const NonconvexRow*>& rows, double penalty,
                            double box) {
  const Eigen::Index nz = problem.layout.size();
  const auto ns = static_cast<Eigen::Index>(rows.size());
  const Eigen::Index nx = nz + ns;

  SparseQp qp;
  Triplets h_triplets;
  appendTriplets(problem.P_scaled, 0, h_triplets);
  qp.H = fromTriplets(nx, nx, h_triplets) / penalty;
  qp.g = Eigen::VectorXd::Ones(nx);
  qp.g.head(nz) = problem.scale.cwiseProduct(problem.P * z) / penalty;

  Triplets a_triplets;
  appendTriplets(problem.A_scaled, 0, a_triplets);
  qp.A = fromTriplets(problem.A_scaled.rows(), nx, a_triplets);
  qp.b = problem.b - problem.A * z;

  const bool use_box = std::isfinite(box);
  const Eigen::Index num_rows = problem.C_scaled.rows() + (use_box ? nz : 0) + ns + ns;
  Triplets c_triplets;
  qp.l.resize(num_rows);
  qp.u.resize(num_rows);

  // Hard limit rows.
  appendTriplets(problem.C_scaled, 0, c_triplets);
  const Eigen::VectorXd cz = problem.C * z;
  Eigen::Index row = problem.C_scaled.rows();
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
  for (Eigen::Index i = 0; i < ns; ++i, ++row) {
    const auto& nonconvex_row = *rows.at(static_cast<size_t>(i));
    for (const auto& [index, derivative] : nonconvex_row.jacobian) {
      c_triplets.emplace_back(row, index, derivative * problem.scale(index));
    }
    c_triplets.emplace_back(row, nz + i, -1.0);
    qp.l(row) = -kInfinity;
    qp.u(row) = -nonconvex_row.value;
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
    sum += violation(value);
  }
  return sum;
}

// Outcome of one trust-region step attempt (see ScpLoop::trustRegionStep).
enum class StepOutcome {
  /// z moved to the candidate.
  Accepted,
  /// No step improves the merit by more than the tolerance, or the box collapsed.
  Converged,
  MaxIterations,
  Timeout,
  QpFailure,
};

// The l1-penalty / trust-box loop. State shared by the steps lives in the members; the constructor
// takes the point z that satisfies the hard rows.
class ScpLoop {
public:
  ScpLoop(const ConvexProblem& problem, const std::vector<std::shared_ptr<NonconvexTerm>>& terms,
          Eigen::VectorXd z, const VexOptions& options)
      : problem_{problem}, terms_{terms}, options_{options}, z_{std::move(z)},
        // Without nonconvex terms the QP model is exact: no trust box is needed.
        has_terms_{!terms.empty()}, penalty_{options.initial_penalty},
        box_{has_terms_ ? options.initial_trust_box : kInfinity},
        start_time_{std::chrono::steady_clock::now()} {}

  tl::expected<ScpResult, std::string> run() {
    // `linearized_` always holds all rows (values and Jacobians) at the current z.
    if (auto evaluated = evaluateTerms(terms_, problem_.layout, z_, true, linearized_);
        !evaluated) {
      return tl::make_unexpected(evaluated.error());
    }

    for (size_t penalty_round = 0;; ++penalty_round) {
      // Minimize the merit for the current penalty.
      for (bool converged = false; !converged;) {
        const auto outcome = trustRegionStep();
        if (!outcome) {
          return tl::make_unexpected(outcome.error());
        }
        switch (*outcome) {
        case StepOutcome::Accepted:
          // An exact model's accepted step is the optimum.
          converged = !has_terms_;
          break;
        case StepOutcome::Converged:
          converged = true;
          break;
        case StepOutcome::MaxIterations:
          return finish(VexStatus::MaxIterations);
        case StepOutcome::Timeout:
          return finish(VexStatus::Timeout);
        case StepOutcome::QpFailure:
          return finish(VexStatus::QpFailure);
        }
      }

      // Converged for this penalty: done if feasible, else increase the penalty.
      if (maxViolation(linearized_) <= options_.tolerance) {
        return finish(VexStatus::Converged);
      }
      if (penalty_round >= options_.max_penalty_increases) {
        return finish(VexStatus::ConvergedInfeasible);
      }
      penalty_ *= options_.penalty_increase;
      box_ = std::max(box_, 1.5 * options_.tolerance / options_.trust_shrink);
    }
  }

private:
  // Solves subproblems at z, shrinking the box after every rejected step, until a step passes the
  // ratio test or a stopping condition holds. Every attempt counts as one iteration.
  tl::expected<StepOutcome, std::string> trustRegionStep() {
    const Eigen::Index nz = problem_.layout.size();

    // The merit counts all rows; only the rows the step can influence enter the QP. The pointers
    // refer to `linearized_` and are valid until it is swapped on acceptance, which ends the step.
    const double merit = problem_.cost(z_) + penalty_ * sumViolation(linearized_);
    std::vector<const NonconvexRow*> qp_rows;
    for (const auto& row : linearized_) {
      if (isInfluenceable(row)) {
        qp_rows.push_back(&row);
      }
    }

    while (true) {
      if (result_.iterations >= options_.max_iterations) {
        return StepOutcome::MaxIterations;
      }
      if (timedOut()) {
        return StepOutcome::Timeout;
      }
      ++result_.iterations;

      // Primal warm start at dy = 0 with the smallest feasible slacks.
      Eigen::VectorXd guess = Eigen::VectorXd::Zero(nz + static_cast<Eigen::Index>(qp_rows.size()));
      for (size_t i = 0; i < qp_rows.size(); ++i) {
        guess(nz + static_cast<Eigen::Index>(i)) = violation(qp_rows.at(i)->value);
      }
      const SparseQp qp = assembleSubproblem(problem_, z_, qp_rows, penalty_, box_);
      const auto solution = solveSparseQp(
          qp, SparseQpSettings{.eps_primal_inf = kSubproblemPrimalInfeasibility}, guess);
      if (!solution) {
        box_ *= options_.trust_shrink;
        if (!has_terms_ || box_ < options_.tolerance) {
          return StepOutcome::QpFailure;
        }
        continue;
      }

      const Eigen::VectorXd dz = problem_.scale.cwiseProduct(solution->head(nz));
      const Eigen::VectorXd z_candidate = z_ + dz;
      const double candidate_cost = problem_.cost(z_candidate);
      const double approx_improve =
          merit - (candidate_cost + penalty_ * linearizedViolation(linearized_, dz));
      if (approx_improve < options_.tolerance) {
        return StepOutcome::Converged;
      }

      // Candidates are evaluated with Jacobians, so an accepted candidate's rows are swapped in
      // without evaluating the terms again (the signed distances dominate the cost).
      if (auto evaluated =
              evaluateTerms(terms_, problem_.layout, z_candidate, true, candidate_rows_);
          !evaluated) {
        return tl::make_unexpected(evaluated.error());
      }
      const double exact_improve =
          merit - (candidate_cost + penalty_ * sumViolation(candidate_rows_));
      if (exact_improve / approx_improve >= options_.improve_ratio) {
        z_ = z_candidate;
        linearized_.swap(candidate_rows_);
        box_ *= options_.trust_expand;
        return StepOutcome::Accepted;
      }
      box_ *= options_.trust_shrink;
      if (box_ < options_.tolerance) {
        return StepOutcome::Converged;
      }
    }
  }

  bool timedOut() const {
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_time_;
    return elapsed.count() > options_.max_time;
  }

  tl::expected<ScpResult, std::string> finish(VexStatus status) {
    result_.z = z_;
    result_.status = status;
    result_.cost = problem_.cost(z_);
    result_.max_violation = maxViolation(linearized_);
    return result_;
  }

  const ConvexProblem& problem_;
  const std::vector<std::shared_ptr<NonconvexTerm>>& terms_;
  const VexOptions& options_;
  Eigen::VectorXd z_;
  const bool has_terms_;
  double penalty_;
  double box_;
  const std::chrono::steady_clock::time_point start_time_;
  std::vector<NonconvexRow> linearized_;
  std::vector<NonconvexRow> candidate_rows_;
  ScpResult result_;
};

}  // namespace

tl::expected<ScpResult, std::string>
runScp(const ConvexProblem& problem, const std::vector<std::shared_ptr<NonconvexTerm>>& terms,
       Eigen::VectorXd z, const VexOptions& options) {
  return ScpLoop(problem, terms, std::move(z), options).run();
}

}  // namespace roboplan::detail
