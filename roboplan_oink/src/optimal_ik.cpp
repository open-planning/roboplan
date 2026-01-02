#include <OsqpEigen/OsqpEigen.h>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

namespace {
// Small regularization value added to Hessian diagonal for numerical stability
constexpr double kHessianRegularization = 1e-12;
}  // namespace

tl::expected<void, std::string>
Task::computeQpObjective(const Scene& scene, Eigen::SparseMatrix<double>& H, Eigen::VectorXd& c) {
  // Compute Jacobian and error into internal containers
  auto jacobian_result = computeJacobian(scene);
  if (!jacobian_result.has_value()) {
    return tl::make_unexpected("Failed to compute Jacobian: " + jacobian_result.error());
  }

  auto error_result = computeError(scene);
  if (!error_result.has_value()) {
    return tl::make_unexpected("Failed to compute error: " + error_result.error());
  }

  // Apply weights
  jacobian_container.applyOnTheLeft(weight);
  error_container *= -gain;
  error_container.applyOnTheLeft(weight);

  // Compute Levenberg-Marquardt damping based on weighted error
  const double mu = lm_damping * error_container.squaredNorm();

  // Compute H = J^T * J + mu * I using pre-allocated H_dense
  H_dense.noalias() = jacobian_container.transpose() * jacobian_container;
  H_dense.diagonal().array() += mu;
  H = H_dense.sparseView();

  // Compute c = - J^T * e_w
  c.noalias() = -jacobian_container.transpose() * error_container;
  return {};
}

Oink::Oink(int num_variables)
    : num_variables(num_variables), task_c(Eigen::VectorXd::Zero(num_variables)),
      task_H(num_variables, num_variables), H(num_variables, num_variables),
      c(Eigen::VectorXd::Zero(num_variables)) {
  settings.setWarmStart(true);
  settings.setVerbosity(false);
}

Oink::Oink(int num_variables, const OsqpEigen::Settings& custom_settings)
    : settings(custom_settings), num_variables(num_variables),
      task_c(Eigen::VectorXd::Zero(num_variables)), task_H(num_variables, num_variables),
      H(num_variables, num_variables), c(Eigen::VectorXd::Zero(num_variables)) {}

tl::expected<void, std::string>
Oink::solveIk(const std::vector<std::shared_ptr<Task>>& tasks,
              const std::vector<std::shared_ptr<Constraints>>& constraints, const Scene& scene,
              Eigen::VectorXd& delta_q) {
  // Reset Hessian and Gradient
  H.setIdentity();
  H.diagonal().array() *= kHessianRegularization;
  c.setZero();

  // Calculate accumulated Hessian and Gradient
  // Each task computes its QP objective using internal pre-allocated storage
  for (const auto& task : tasks) {
    auto objective_result = task->computeQpObjective(scene, task_H, task_c);
    if (!objective_result.has_value()) {
      return tl::make_unexpected(objective_result.error());
    }

    H += task_H;
    c += task_c;
  }
  H.makeCompressed();

  // Query total constraint dimensions and cache sizes to avoid redundant calls
  constraint_sizes.reserve(constraints.size());
  int total_constraint_rows = 0;
  for (const auto& constraint : constraints) {
    int num_rows = constraint->getNumConstraints(scene);
    constraint_sizes.push_back(num_rows);
    total_constraint_rows += num_rows;
  }

  const bool init_required =
      !solver.isInitialized() || (total_constraint_rows != last_constraint_rows);

  // Resize constraint workspace only if dimensions changed (zero allocations in steady state)
  // Note: Workspace is allocated even when total_constraint_rows == 0 for consistency,
  // but OSQP constraint matrices are only set when constraints exist (for unconstrained QP)
  if (init_required) {
    constraint_workspace_A.resize(total_constraint_rows, num_variables);
    constraint_workspace_lower.resize(total_constraint_rows);
    constraint_workspace_upper.resize(total_constraint_rows);

    // Resize sparse workspace to match
    A_sparse.resize(total_constraint_rows, num_variables);

    last_constraint_rows = total_constraint_rows;
  }

  // Fill constraint matrices block by block using Eigen::Ref (zero-copy views)
  int row_offset = 0;
  for (size_t i = 0; i < constraints.size(); ++i) {
    const int num_rows = constraint_sizes.at(i);

    // Safety check: ensure we don't exceed workspace bounds
    if (row_offset + num_rows > total_constraint_rows) {
      return tl::make_unexpected("Internal error: constraint row offset " +
                                 std::to_string(row_offset + num_rows) + " exceeds total rows " +
                                 std::to_string(total_constraint_rows));
    }

    // Create Eigen::Ref views into the workspace (zero-copy, no allocation)
    Eigen::Ref<Eigen::MatrixXd> constraint_A_view =
        constraint_workspace_A.middleRows(row_offset, num_rows);
    Eigen::Ref<Eigen::VectorXd> constraint_lower_view =
        constraint_workspace_lower.segment(row_offset, num_rows);
    Eigen::Ref<Eigen::VectorXd> constraint_upper_view =
        constraint_workspace_upper.segment(row_offset, num_rows);

    // Compute constraints directly into workspace views
    auto constraint_result = constraints.at(i)->computeQpConstraints(
        scene, constraint_A_view, constraint_lower_view, constraint_upper_view);
    if (!constraint_result.has_value()) {
      return tl::make_unexpected("Failed to compute constraints: " + constraint_result.error());
    }

    // Validation: Ensure constraint didn't resize the views (safety check)
    if (constraint_A_view.rows() != num_rows || constraint_A_view.cols() != num_variables) {
      return tl::make_unexpected("Constraint implementation error: resized output matrices. "
                                 "Expected (" +
                                 std::to_string(num_rows) + " x " + std::to_string(num_variables) +
                                 "), got (" + std::to_string(constraint_A_view.rows()) + " x " +
                                 std::to_string(constraint_A_view.cols()) + ")");
    }

    row_offset += num_rows;
  }
  // Clear constraint_sizes for the next iteration
  constraint_sizes.clear();

  // Convert constraint matrix to sparse format for OSQP
  A_sparse = constraint_workspace_A.sparseView();

  if (init_required) {
    // Clear previous solver state and data if it exists
    if (solver.isInitialized()) {
      solver.clearSolver();
    }
    // Clear previous data matrices to allow re-setting them
    solver.data()->clearHessianMatrix();
    solver.data()->clearLinearConstraintsMatrix();

    // Apply solver settings by copying from the stored settings
    const OSQPSettings* stored_settings = settings.getSettings();
    solver.settings()->setWarmStart(stored_settings->warm_starting);
    solver.settings()->setVerbosity(stored_settings->verbose);
    solver.settings()->setAlpha(stored_settings->alpha);
    solver.settings()->setAbsoluteTolerance(stored_settings->eps_abs);
    solver.settings()->setRelativeTolerance(stored_settings->eps_rel);
    solver.settings()->setPrimalInfeasibilityTolerance(stored_settings->eps_prim_inf);
    solver.settings()->setDualInfeasibilityTolerance(stored_settings->eps_dual_inf);
    solver.settings()->setMaxIteration(stored_settings->max_iter);
    solver.settings()->setRho(stored_settings->rho);
    solver.settings()->setPolish(stored_settings->polishing);
    solver.settings()->setAdaptiveRho(stored_settings->adaptive_rho);
    solver.settings()->setTimeLimit(stored_settings->time_limit);

    // Initialize solver with new dimensions
    solver.data()->setNumberOfVariables(num_variables);
    solver.data()->setNumberOfConstraints(total_constraint_rows);
    if (total_constraint_rows > 0) {
      solver.data()->setLinearConstraintsMatrix(A_sparse);
      solver.data()->setLowerBound(constraint_workspace_lower);
      solver.data()->setUpperBound(constraint_workspace_upper);
    }
    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(c);
    if (!solver.initSolver()) {
      return tl::make_unexpected("Failed to initialize solver");
    }
  } else {
    if (!solver.updateHessianMatrix(H)) {
      return tl::make_unexpected("Failed to update Hessian matrix");
    }

    if (!solver.updateGradient(c)) {
      return tl::make_unexpected("Failed to update gradient vector");
    }

    if (total_constraint_rows > 0) {
      if (!solver.updateLinearConstraintsMatrix(A_sparse)) {
        return tl::make_unexpected("Failed to update linear constraints matrix");
      }
      if (!solver.updateBounds(constraint_workspace_lower, constraint_workspace_upper)) {
        return tl::make_unexpected("Failed to update constraint bounds");
      }
    }
  }

  // Solve the QP problem
  auto result = solver.solveProblem();
  if (result != OsqpEigen::ErrorExitFlag::NoError) {
    return tl::make_unexpected("QP solver failed to find a solution");
  }

  // Extract the solution and update delta_q
  delta_q = solver.getSolution();

  return {};
}

}  // namespace roboplan
