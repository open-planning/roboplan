#include <roboplan_oink/qp_backend.hpp>

#include <proxsuite/proxqp/dense/dense.hpp>

namespace roboplan::detail {

void QpDeleter::operator()(proxsuite::proxqp::dense::QP<double>* solver) const { delete solver; }

tl::expected<void, std::string>
solveQp(QpSolverPtr& solver, const OinkSettings& settings, bool init_required, int num_variables,
        int total_rows, const Eigen::MatrixXd& H, const Eigen::VectorXd& c,
        const Eigen::MatrixXd& A, const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
        Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q) {
  using proxsuite::nullopt;
  using proxsuite::proxqp::InitialGuessStatus;
  using proxsuite::proxqp::QPSolverOutput;

  if (init_required) {
    // ProxQP fixes the problem dimensions at construction, so the solver is rebuilt
    // whenever the number of constraint rows changes.
    solver.reset(new proxsuite::proxqp::dense::QP<double>(num_variables, /*n_eq=*/0,
                                                          /*n_in=*/total_rows));
    solver->settings.eps_abs = settings.eps_abs;
    solver->settings.eps_rel = settings.eps_rel;
    solver->settings.max_iter = settings.max_iter;
    solver->settings.verbose = settings.verbose;
    solver->settings.primal_infeasibility_solving = settings.primal_infeasibility_solving;
    solver->settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;

    if (total_rows > 0) {
      solver->init(H, c, nullopt, nullopt, A, lower, upper);
    } else {
      solver->init(H, c, nullopt, nullopt, nullopt, nullopt, nullopt);
    }
  } else {
    if (total_rows > 0) {
      solver->update(H, c, nullopt, nullopt, A, lower, upper);
    } else {
      solver->update(H, c, nullopt, nullopt, nullopt, nullopt, nullopt);
    }
  }

  solver->solve();

  // PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE is returned when the QP was primal-infeasible and
  // settings.primal_infeasibility_solving is enabled: the solution minimizes the constraint
  // violation in the least-squares sense and is the safest displacement available.
  const QPSolverOutput status = solver->results.info.status;
  if (status != QPSolverOutput::PROXQP_SOLVED &&
      status != QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE) {
    // The solve did not converge, so results.x is not a trustworthy displacement. Reset the
    // initial guess so a subsequent solveIk() does not warm start from this bad solution (the
    // equivalent of OsqpEigen::Solver::clearSolverVariables()).
    solver->settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
    switch (status) {
    case QPSolverOutput::PROXQP_MAX_ITER_REACHED:
      return tl::make_unexpected("QP solver reached the maximum number of iterations");
    case QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
      return tl::make_unexpected(
          "QP is primal infeasible (constraints and barriers cannot be satisfied "
          "simultaneously). Enable OinkSettings::primal_infeasibility_solving to compute the "
          "closest feasible solution instead.");
    case QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
      return tl::make_unexpected("QP is dual infeasible (objective unbounded below)");
    default:
      return tl::make_unexpected("QP solver failed to find a solution");
    }
  }

  // Warm start subsequent solves with the previous solution. This must only be enabled
  // after a successful solve has populated the results and factorization; enabling it before
  // the first solve (or after a failed one) makes ProxQP reuse a factorization/solution that
  // was never validly computed.
  if (settings.warm_start) {
    solver->settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
  }

  // Extract the solution and copy into delta_q
  delta_q.noalias() = solver->results.x;
  return {};
}

}  // namespace roboplan::detail
