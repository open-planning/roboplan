#include <roboplan_vex/qp_backend.hpp>

#include <proxsuite/proxqp/sparse/sparse.hpp>

namespace roboplan::detail {

tl::expected<Eigen::VectorXd, std::string>
solveSparseQp(const SparseQp& qp, const SparseQpSettings& settings,
              std::optional<Eigen::Ref<const Eigen::VectorXd>> primal_guess) {
  using proxsuite::nullopt;
  using proxsuite::proxqp::QPSolverOutput;
  using Solver = proxsuite::proxqp::sparse::QP<double, int>;

  const auto n = qp.H.rows();
  const auto n_eq = qp.A.rows();
  const auto n_in = qp.C.rows();
  if (qp.H.cols() != n || qp.g.size() != n || (n_eq > 0 && qp.A.cols() != n) ||
      qp.b.size() != n_eq || (n_in > 0 && qp.C.cols() != n) || qp.l.size() != n_in ||
      qp.u.size() != n_in) {
    return tl::make_unexpected("Inconsistent QP dimensions.");
  }
  if (primal_guess && primal_guess->size() != n) {
    return tl::make_unexpected("QP primal warm start has the wrong size.");
  }

  // The number of rows changes between SCP iterations, so the solver is rebuilt for every
  // solve. H must always be passed (sparse init dereferences it even when it is nullopt).
  Solver solver(n, n_eq, n_in);
  solver.settings.eps_abs = settings.eps_abs;
  solver.settings.eps_rel = settings.eps_rel;
  solver.settings.max_iter = settings.max_iter;
  solver.settings.eps_primal_inf = settings.eps_primal_inf;
  solver.settings.verbose = false;
  using OptMat = proxsuite::optional<proxsuite::proxqp::sparse::SparseMat<double, int>>;
  using OptVec = proxsuite::optional<proxsuite::proxqp::sparse::VecRef<double>>;
  solver.init(qp.H, qp.g, n_eq > 0 ? OptMat(qp.A) : nullopt, n_eq > 0 ? OptVec(qp.b) : nullopt,
              n_in > 0 ? OptMat(qp.C) : nullopt, n_in > 0 ? OptVec(qp.l) : nullopt,
              n_in > 0 ? OptVec(qp.u) : nullopt, /*compute_preconditioner_=*/true,
              /*rho=*/nullopt, /*mu_eq=*/nullopt, /*mu_in=*/settings.mu_in);

  if (primal_guess) {
    solver.solve(*primal_guess, nullopt, nullopt);
  } else {
    solver.solve();
  }

  switch (solver.results.info.status) {
  case QPSolverOutput::PROXQP_SOLVED:
    return solver.results.x;
  case QPSolverOutput::PROXQP_MAX_ITER_REACHED:
    return tl::make_unexpected("QP solver reached the maximum number of iterations.");
  case QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
    return tl::make_unexpected("QP is primal infeasible.");
  case QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
    return tl::make_unexpected("QP is dual infeasible (objective unbounded below).");
  default:
    return tl::make_unexpected("QP solver failed to find a solution.");
  }
}

}  // namespace roboplan::detail
