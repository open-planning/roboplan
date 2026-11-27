#include "scp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "qp_backend.hpp"

namespace roboplan::detail {

// SCP loop: stages ③ to ⑥ of docs/source/concepts/trajectory_optimization/vex.rst.

namespace {

using Triplets = std::vector<Eigen::Triplet<double>>;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// After a penalty increase the trust box is at least this factor above the size at which a
// rejected step ends the round (tolerance / trust_shrink), so one rejection cannot end it.
constexpr double kBoxResetFactor = 1.5;

// The subproblems are feasible by construction (dy = 0, s = max(g, 0)); ProxQP's default
// primal infeasibility threshold (1e-4) still reports infeasibility on ~1 in 20 of them.
constexpr double kSubproblemPrimalInfeasibility = 1e-12;

// Rows with a gradient norm at or below this are constant under the step (measured: below 1e-15
// for pairs the group cannot move, far above 1e-9 for the others).
constexpr double kMinRowGradientNorm = 1e-9;

// Whether the step can change the row. Constant rows stay out of the QP but still count in the
// merit and the violation.
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

tl::expected<void, std::string>
evaluateTerms(const std::vector<std::shared_ptr<NonconvexTerm>>& terms, const VexLayout& layout,
              const Eigen::VectorXd& z, std::vector<NonconvexRow>& rows) {
  rows.clear();
  for (const auto& term : terms) {
    if (auto evaluated = term->evaluate(layout, z, rows); !evaluated) {
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

// Builds the QP for the scaled step dy, one slack per row (vex.rst, ④). The objective is divided
// by `penalty` so the duals stay O(1); undivided, ProxQP stalls at large penalties.
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

  // Joint limits and trust box bound the same variables, so they share one row per variable.
  const Eigen::VectorXd step_lower =
      (problem.z_min - z).cwiseQuotient(problem.scale).cwiseMax(-box);
  const Eigen::VectorXd step_upper = (problem.z_max - z).cwiseQuotient(problem.scale).cwiseMin(box);
  const Eigen::Index num_bound_rows =
      ((step_lower.array() > -kInfinity) || (step_upper.array() < kInfinity)).count();
  const Eigen::Index num_rows = num_bound_rows + ns + ns;
  Triplets c_triplets;
  qp.l.resize(num_rows);
  qp.u.resize(num_rows);

  Eigen::Index row = 0;
  for (Eigen::Index i = 0; i < nz; ++i) {
    if (step_lower(i) > -kInfinity || step_upper(i) < kInfinity) {
      c_triplets.emplace_back(row, i, 1.0);
      qp.l(row) = step_lower(i);
      qp.u(row) = step_upper(i);
      ++row;
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

// Trust-region loop with penalty rounds (vex.rst, ⑤ and ⑥).
class ScpLoop {
public:
  ScpLoop(const ConvexProblem& problem, const std::vector<std::shared_ptr<NonconvexTerm>>& terms,
          Eigen::VectorXd z, const VexOptions& options)
      : problem_{problem}, terms_{terms}, options_{options}, z_{std::move(z)},
        // No nonconvex terms: the QP model is exact, so no trust box.
        has_terms_{!terms.empty()}, penalty_{options.initial_penalty},
        box_{has_terms_ ? options.initial_trust_box : kInfinity},
        start_time_{std::chrono::steady_clock::now()} {}

  tl::expected<ScpResult, std::string> run() {
    // `linearized_` always holds all rows (values and Jacobians) at the current z.
    if (auto evaluated = evaluateTerms(terms_, problem_.layout, z_, linearized_); !evaluated) {
      return tl::make_unexpected(evaluated.error());
    }

    for (size_t penalty_round = 0;; ++penalty_round) {
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

      if (maxViolation(linearized_) <= options_.tolerance) {
        return finish(VexStatus::Converged);
      }
      if (penalty_round >= options_.max_penalty_increases) {
        return finish(VexStatus::ConvergedInfeasible);
      }
      penalty_ *= options_.penalty_increase;
      box_ = std::max(box_, kBoxResetFactor * options_.tolerance / options_.trust_shrink);
    }
  }

private:
  // Solves QPs at z, shrinking the box on every rejection, until a step is accepted or the loop
  // stops. Every solve counts as one iteration.
  tl::expected<StepOutcome, std::string> trustRegionStep() {
    const Eigen::Index nz = problem_.layout.size();

    // Only rows the step can change enter the QP. The pointers refer to `linearized_` and are
    // valid until it is swapped on acceptance.
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

      // Candidates carry Jacobians, so an accepted candidate's rows replace `linearized_` as is.
      if (auto evaluated = evaluateTerms(terms_, problem_.layout, z_candidate, candidate_rows_);
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

  [[nodiscard]] bool timedOut() const {
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
  bool has_terms_;
  double penalty_;
  double box_;
  std::chrono::steady_clock::time_point start_time_;
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
