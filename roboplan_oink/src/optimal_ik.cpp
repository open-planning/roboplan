#include <limits>

#include <OsqpEigen/OsqpEigen.h>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace {
// Minimum squared norm threshold to avoid division by zero in barrier regularization
constexpr double kMinNormSq = 1e-12;
}  // namespace

namespace roboplan {

// Barrier base class implementation
Barrier::Barrier(double gain_, double dt_, double safe_displacement_gain_, double safety_margin_)
    : gain(gain_), dt(dt_), safe_displacement_gain(safe_displacement_gain_),
      safety_margin(safety_margin_) {
  if (gain <= 0.0) {
    throw std::invalid_argument("Barrier gain must be positive");
  }
  if (dt <= 0.0) {
    throw std::invalid_argument("Barrier dt must be positive");
  }
  if (safe_displacement_gain < 0.0) {
    throw std::invalid_argument("Barrier safe_displacement_gain must be non-negative");
  }
  if (safety_margin < 0.0) {
    throw std::invalid_argument("Barrier safety_margin must be non-negative");
  }
}

void Barrier::initializeStorage(int num_barriers, int num_vars) {
  num_variables = num_vars;
  barrier_values = Eigen::VectorXd::Zero(num_barriers);
  jacobian_container = Eigen::MatrixXd::Zero(num_barriers, num_vars);
}

Eigen::VectorXd Barrier::computeSafeDisplacement(const Scene& /*scene*/) const {
  // Default: zero displacement (stay in place is always safe)
  return Eigen::VectorXd::Zero(num_variables);
}

tl::expected<double, std::string>
Barrier::evaluateAtConfiguration(const pinocchio::Model& /*model*/, pinocchio::Data& /*data*/,
                                 const Eigen::VectorXd& /*q*/) const {
  // Default: return infinity to indicate not supported by this barrier type
  return OSQP_INFTY;
}

tl::expected<void, std::string> Barrier::computeQpInequalities(const Scene& scene,
                                                               Eigen::Ref<Eigen::MatrixXd> G,
                                                               Eigen::Ref<Eigen::VectorXd> b) {
  // Compute barrier values and Jacobian
  auto barrier_result = computeBarrier(scene);
  if (!barrier_result) {
    return barrier_result;
  }

  auto jacobian_result = computeJacobian(scene);
  if (!jacobian_result) {
    return jacobian_result;
  }

  // G = -J_h / dt
  G = -jacobian_container / dt;

  // Saturating class-K function with safety margin: α(h - m) = γ·(h - m) / (1 + |h - m|)
  // The safety margin shifts the "zero crossing" point, making the constraint more conservative.
  // This helps account for linearization errors in discrete-time CBF formulation.
  // Saturating function provides bounded recovery force, preventing over-reaction when far from
  // boundary and giving more consistent behavior across configurations.
  for (int i = 0; i < barrier_values.size(); ++i) {
    const double h_shifted = barrier_values[i] - safety_margin;
    b[i] = gain * h_shifted / (1.0 + std::abs(h_shifted));
  }

  return {};
}

tl::expected<void, std::string> Barrier::computeQpObjective(const Scene& scene,
                                                            Eigen::Ref<Eigen::MatrixXd> H,
                                                            Eigen::Ref<Eigen::VectorXd> c) {
  // Ensure barrier and Jacobian are computed (may already be computed by computeQpInequalities)
  auto barrier_result = computeBarrier(scene);
  if (!barrier_result) {
    return barrier_result;
  }

  auto jacobian_result = computeJacobian(scene);
  if (!jacobian_result) {
    return jacobian_result;
  }

  // Compute squared Frobenius norm of Jacobian: ‖J_h‖²
  const double jacobian_norm_sq = jacobian_container.squaredNorm();

  // Avoid division by zero - if Jacobian is near zero, no regularization needed
  if (jacobian_norm_sq < kMinNormSq) {
    H.setZero();
    c.setZero();
    return {};
  }

  // Compute safe displacement
  const Eigen::VectorXd dq_safe = computeSafeDisplacement(scene);

  // Regularization weight: r / ‖J_h‖²
  // The 1/‖J_h‖² normalizes based on barrier sensitivity
  const double weight = safe_displacement_gain / jacobian_norm_sq;

  // QP objective contribution: (r / (2·‖J_h‖²)) · ‖δq - δq_safe‖²
  // Expanding: (r / (2·‖J_h‖²)) · (δq^T δq - 2 δq^T δq_safe + δq_safe^T δq_safe)
  //          = (r / (2·‖J_h‖²)) · δq^T I δq - (r / ‖J_h‖²) · δq_safe^T δq + const
  //
  // For QP formulation (min 1/2 x^T H x + c^T x):
  // H_contribution = (r / ‖J_h‖²) · I = weight · I
  // c_contribution = -(r / ‖J_h‖²) · δq_safe = -weight · δq_safe

  H.setIdentity();
  H *= weight;
  c = -weight * dq_safe;

  return {};
}

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

Oink::Oink(const Scene& scene, const std::string& group_name,
           const OsqpEigen::Settings& custom_settings)
    : settings(custom_settings) {
  const auto maybe_group_info = scene.getJointGroupInfo(group_name);
  if (!maybe_group_info) {
    throw std::runtime_error("Oink: joint group '" + group_name +
                             "' not found: " + maybe_group_info.error());
  }
  q_indices = maybe_group_info->q_indices;
  v_indices = maybe_group_info->v_indices;
  num_variables = static_cast<int>(v_indices.size());

  task_c = Eigen::VectorXd::Zero(num_variables);
  task_H = Eigen::SparseMatrix<double>(num_variables, num_variables);
  H = Eigen::SparseMatrix<double>(num_variables, num_variables);
  c = Eigen::VectorXd::Zero(num_variables);
}

Oink::Oink(const Scene& scene, const std::string& group_name) {
  const auto maybe_group_info = scene.getJointGroupInfo(group_name);
  if (!maybe_group_info) {
    throw std::runtime_error("Oink: joint group '" + group_name +
                             "' not found: " + maybe_group_info.error());
  }
  q_indices = maybe_group_info->q_indices;
  v_indices = maybe_group_info->v_indices;
  num_variables = static_cast<int>(v_indices.size());

  task_c = Eigen::VectorXd::Zero(num_variables);
  task_H = Eigen::SparseMatrix<double>(num_variables, num_variables);
  H = Eigen::SparseMatrix<double>(num_variables, num_variables);
  c = Eigen::VectorXd::Zero(num_variables);

  settings.setWarmStart(true);
  settings.setVerbosity(false);
}

tl::expected<void, std::string>
Oink::solveIk(const Scene& scene, const std::vector<std::shared_ptr<Task>>& tasks,
              const std::vector<std::shared_ptr<Constraints>>& constraints,
              const std::vector<std::shared_ptr<Barrier>>& barriers,
              Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
              double regularization) {
  // Validate delta_q size before proceeding
  if (delta_q.size() != num_variables) {
    return tl::make_unexpected("delta_q has wrong size: expected " + std::to_string(num_variables) +
                               ", got " + std::to_string(delta_q.size()) +
                               ". delta_q must be pre-allocated to num_variables.");
  }

  // Reset Hessian and Gradient
  H.setIdentity();
  H.diagonal().array() *= regularization;
  c.setZero();

  // Calculate accumulated Hessian and Gradient from tasks
  for (const auto& task : tasks) {
    auto objective_result = task->computeQpObjective(scene, task_H, task_c);
    if (!objective_result.has_value()) {
      return tl::make_unexpected(objective_result.error());
    }

    H += task_H;
    c += task_c;
  }

  // Add barrier regularization contributions (safe displacement)
  // Resize workspace if needed
  if (barrier_H_contribution.rows() != num_variables) {
    barrier_H_contribution.resize(num_variables, num_variables);
    barrier_c_contribution.resize(num_variables);
  }

  for (const auto& barrier : barriers) {
    auto obj_result =
        barrier->computeQpObjective(scene, barrier_H_contribution, barrier_c_contribution);
    if (obj_result.has_value()) {
      // Add dense contribution to sparse H (convert to sparse for efficient addition)
      H += barrier_H_contribution.sparseView();
      c += barrier_c_contribution;
    }
  }

  H.makeCompressed();

  // Query total constraint dimensions and cache sizes
  constraint_sizes.reserve(constraints.size());
  int total_constraint_rows = 0;
  for (const auto& constraint : constraints) {
    int num_rows = constraint->getNumConstraints(scene);
    constraint_sizes.push_back(num_rows);
    total_constraint_rows += num_rows;
  }

  // Query total barrier dimensions and cache sizes
  barrier_sizes.reserve(barriers.size());
  int total_barrier_rows = 0;
  for (const auto& barrier : barriers) {
    int num_rows = barrier->getNumBarriers(scene);
    barrier_sizes.push_back(num_rows);
    total_barrier_rows += num_rows;
  }

  // Total inequality constraints = constraints (box) + barriers (one-sided)
  // For barriers: -inf <= G*dq <= h (only upper bounded)
  const int total_rows = total_constraint_rows + total_barrier_rows;
  const bool init_required =
      !solver.isInitialized() ||
      (total_constraint_rows != last_constraint_rows || total_barrier_rows != last_barrier_rows);

  // Resize constraint workspace if dimensions changed
  if (init_required) {
    constraint_workspace_A.resize(total_rows, num_variables);
    constraint_workspace_lower.resize(total_rows);
    constraint_workspace_upper.resize(total_rows);
    A_sparse.resize(total_rows, num_variables);
    last_constraint_rows = total_constraint_rows;
    last_barrier_rows = total_barrier_rows;
  }

  // Fill constraint matrices block by block
  int row_offset = 0;
  for (size_t i = 0; i < constraints.size(); ++i) {
    const int num_rows = constraint_sizes.at(i);

    if (row_offset + num_rows > total_rows) {
      return tl::make_unexpected("Internal error: constraint row offset exceeds total rows");
    }

    Eigen::Ref<Eigen::MatrixXd> constraint_A_view =
        constraint_workspace_A.middleRows(row_offset, num_rows);
    Eigen::Ref<Eigen::VectorXd> constraint_lower_view =
        constraint_workspace_lower.segment(row_offset, num_rows);
    Eigen::Ref<Eigen::VectorXd> constraint_upper_view =
        constraint_workspace_upper.segment(row_offset, num_rows);

    auto constraint_result = constraints.at(i)->computeQpConstraints(
        scene, constraint_A_view, constraint_lower_view, constraint_upper_view);
    if (!constraint_result.has_value()) {
      return tl::make_unexpected("Failed to compute constraints: " + constraint_result.error());
    }

    row_offset += num_rows;
  }

  // Fill barrier constraints (one-sided: -inf <= G*dq <= h)
  for (size_t i = 0; i < barriers.size(); ++i) {
    const int num_rows = barrier_sizes.at(i);

    if (row_offset + num_rows > total_rows) {
      return tl::make_unexpected("Internal error: barrier row offset exceeds total rows");
    }

    Eigen::Ref<Eigen::MatrixXd> barrier_G_view =
        constraint_workspace_A.middleRows(row_offset, num_rows);
    Eigen::Ref<Eigen::VectorXd> barrier_h_view =
        constraint_workspace_upper.segment(row_offset, num_rows);

    auto barrier_result =
        barriers.at(i)->computeQpInequalities(scene, barrier_G_view, barrier_h_view);
    if (!barrier_result.has_value()) {
      return tl::make_unexpected("Failed to compute barriers: " + barrier_result.error());
    }

    // Barrier constraints are one-sided: -inf <= G*dq <= h
    constraint_workspace_lower.segment(row_offset, num_rows).setConstant(-OsqpEigen::INFTY);

    row_offset += num_rows;
  }

  // Clear sizes for next iteration
  constraint_sizes.clear();
  barrier_sizes.clear();

  // Convert constraint matrix to sparse format
  A_sparse = constraint_workspace_A.sparseView();

  if (init_required) {
    if (solver.isInitialized()) {
      solver.clearSolver();
    }
    solver.data()->clearHessianMatrix();
    solver.data()->clearLinearConstraintsMatrix();

    const OSQPSettings* stored_settings = settings.getSettings();
#ifdef OSQP_EIGEN_OSQP_IS_V1
    solver.settings()->setWarmStart(stored_settings->warm_starting);
    solver.settings()->setPolish(stored_settings->polishing);
#else
    solver.settings()->setWarmStart(stored_settings->warm_start);
    solver.settings()->setPolish(stored_settings->polish);
#endif
    solver.settings()->setVerbosity(stored_settings->verbose);
    solver.settings()->setAlpha(stored_settings->alpha);
    solver.settings()->setAbsoluteTolerance(stored_settings->eps_abs);
    solver.settings()->setRelativeTolerance(stored_settings->eps_rel);
    solver.settings()->setPrimalInfeasibilityTolerance(stored_settings->eps_prim_inf);
    solver.settings()->setDualInfeasibilityTolerance(stored_settings->eps_dual_inf);
    solver.settings()->setMaxIteration(stored_settings->max_iter);
    solver.settings()->setRho(stored_settings->rho);
    solver.settings()->setAdaptiveRho(stored_settings->adaptive_rho);
    solver.settings()->setTimeLimit(stored_settings->time_limit);

    solver.data()->setNumberOfVariables(num_variables);
    solver.data()->setNumberOfConstraints(total_rows);
    if (total_rows > 0) {
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

    if (total_rows > 0) {
      if (!solver.updateLinearConstraintsMatrix(A_sparse)) {
        return tl::make_unexpected("Failed to update linear constraints matrix");
      }
      if (!solver.updateBounds(constraint_workspace_lower, constraint_workspace_upper)) {
        return tl::make_unexpected("Failed to update constraint bounds");
      }
    }
  }

  auto result = solver.solveProblem();
  if (result != OsqpEigen::ErrorExitFlag::NoError) {
    return tl::make_unexpected("QP solver failed to find a solution");
  }

  // Extract the solution and copy into delta_q
  delta_q.noalias() = solver.getSolution();

  return {};
}

// Overload: tasks only
tl::expected<void, std::string>
Oink::solveIk(const Scene& scene, const std::vector<std::shared_ptr<Task>>& tasks,
              Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
              double regularization) {
  return solveIk(scene, tasks, {}, {}, delta_q, regularization);
}

// Overload: tasks + constraints
tl::expected<void, std::string>
Oink::solveIk(const Scene& scene, const std::vector<std::shared_ptr<Task>>& tasks,
              const std::vector<std::shared_ptr<Constraints>>& constraints,
              Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
              double regularization) {
  return solveIk(scene, tasks, constraints, {}, delta_q, regularization);
}

// Overload: tasks + barriers
tl::expected<void, std::string>
Oink::solveIk(const Scene& scene, const std::vector<std::shared_ptr<Task>>& tasks,
              const std::vector<std::shared_ptr<Barrier>>& barriers,
              Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
              double regularization) {
  return solveIk(scene, tasks, {}, barriers, delta_q, regularization);
}

tl::expected<void, std::string>
Oink::enforceBarriers(const Scene& scene, const std::vector<std::shared_ptr<Barrier>>& barriers,
                      Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
                      double tolerance) {
  if (barriers.empty()) {
    return {};
  }

  const auto& model = scene.getModel();
  const Eigen::VectorXd q = scene.getCurrentJointPositions();

  // Compute candidate configuration by integrating delta_q
  const Eigen::VectorXd q_candidate = pinocchio::integrate(model, q, delta_q);

  // Create a temporary Data object for FK evaluation to avoid modifying scene state
  pinocchio::Data temp_data(model);

  // Evaluate all barriers at the candidate configuration
  double min_h = OSQP_INFTY;
  for (const auto& barrier : barriers) {
    auto h_result = barrier->evaluateAtConfiguration(model, temp_data, q_candidate);
    if (!h_result.has_value()) {
      // Propagate evaluation error
      return tl::make_unexpected(h_result.error());
    }
    // Only consider finite barrier values (infinity means not supported)
    if (std::isfinite(h_result.value())) {
      min_h = std::min(min_h, h_result.value());
    }
  }

  // If any barrier is violated, stop completely
  if (min_h < -tolerance) {
    delta_q.setZero();
  }

  return {};
}

}  // namespace roboplan
