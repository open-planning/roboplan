#include <algorithm>
#include <limits>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/qp_backend.hpp>

namespace {
// Minimum squared norm threshold to avoid division by zero in barrier regularization
constexpr double kMinNormSq = 1e-12;
}  // namespace

namespace roboplan {

// Barrier base class implementation
Barrier::Barrier(double gain, double dt, double safe_displacement_gain, double safety_margin)
    : gain(gain), dt(dt), safe_displacement_gain(safe_displacement_gain),
      safety_margin(safety_margin) {
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
  return kInfinity;
}

void Barrier::formatQpInequalities(Eigen::Ref<Eigen::MatrixXd> G,
                                   Eigen::Ref<Eigen::VectorXd> b) const {
  // G = -J_h / dt
  G = -jacobian_container / dt;

  // Saturating class-K function with safety margin: α(h - m) = γ·(h - m) / (1 + |h - m|)
  for (int i = 0; i < barrier_values.size(); ++i) {
    const double h_shifted = barrier_values[i] - safety_margin;
    b[i] = gain * h_shifted / (1.0 + std::abs(h_shifted));
  }
}

void Barrier::formatQpObjective(const Scene& scene, Eigen::Ref<Eigen::MatrixXd> H,
                                Eigen::Ref<Eigen::VectorXd> c) const {
  // Compute squared Frobenius norm of Jacobian: ‖J_h‖²
  const double jacobian_norm_sq = jacobian_container.squaredNorm();

  // Avoid division by zero - if Jacobian is near zero, no regularization needed
  if (jacobian_norm_sq < kMinNormSq) {
    H.setZero();
    c.setZero();
    return;
  }

  // Compute safe displacement
  const Eigen::VectorXd dq_safe = computeSafeDisplacement(scene);

  // Regularization weight: r / ‖J_h‖²
  // The 1/‖J_h‖² normalizes based on barrier sensitivity
  const double weight = safe_displacement_gain / jacobian_norm_sq;

  // QP objective contribution: (r / (2·‖J_h‖²)) · ‖δq - δq_safe‖²
  // H_contribution = (r / ‖J_h‖²) · I = weight · I
  // c_contribution = -(r / ‖J_h‖²) · δq_safe = -weight · δq_safe
  H.setIdentity();
  H *= weight;
  c = -weight * dq_safe;
}

tl::expected<void, std::string> Barrier::computeQpInequalities(const Scene& scene,
                                                               Eigen::Ref<Eigen::MatrixXd> G,
                                                               Eigen::Ref<Eigen::VectorXd> b) {
  auto barrier_result = computeBarrier(scene);
  if (!barrier_result) {
    return barrier_result;
  }
  auto jacobian_result = computeJacobian(scene);
  if (!jacobian_result) {
    return jacobian_result;
  }
  formatQpInequalities(G, b);
  return {};
}

tl::expected<void, std::string> Barrier::computeQpObjective(const Scene& scene,
                                                            Eigen::Ref<Eigen::MatrixXd> H,
                                                            Eigen::Ref<Eigen::VectorXd> c) {
  auto barrier_result = computeBarrier(scene);
  if (!barrier_result) {
    return barrier_result;
  }
  auto jacobian_result = computeJacobian(scene);
  if (!jacobian_result) {
    return jacobian_result;
  }
  formatQpObjective(scene, H, c);
  return {};
}

tl::expected<void, std::string> Task::computeQpObjective(const Scene& scene, Eigen::MatrixXd& H,
                                                         Eigen::VectorXd& c) {
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

  // Compute H = J^T * J + mu * I
  H.noalias() = jacobian_container.transpose() * jacobian_container;
  H.diagonal().array() += mu;

  // Compute c = - J^T * e_w
  c.noalias() = -jacobian_container.transpose() * error_container;
  return {};
}

Oink::Oink(const Scene& scene, const std::string& group_name, const OinkSettings& custom_settings)
    : Oink(scene, group_name) {
  settings = custom_settings;
}

Oink::Oink(const Scene& scene, const std::string& group_name)
    : enforce_barriers_data(scene.getModel()) {
  const auto maybe_group_info = scene.getJointGroupInfo(group_name);
  if (!maybe_group_info) {
    throw std::runtime_error("Oink: joint group '" + group_name +
                             "' not found: " + maybe_group_info.error());
  }
  q_indices = maybe_group_info->q_indices;
  v_indices = maybe_group_info->v_indices;
  num_variables = static_cast<int>(v_indices.size());

  H = Eigen::MatrixXd::Zero(num_variables, num_variables);
  c = Eigen::VectorXd::Zero(num_variables);

  collision_context_ = std::make_unique<CollisionContext>(scene);
}

Oink::~Oink() = default;

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

  // Build a flat, priority-sorted view into `tasks` so we can walk levels in one pass.
  // The buffer is a pre-allocated member of Oink, so steady-state calls hit no heap.
  sorted_tasks.clear();
  for (const auto& task : tasks) {
    if (task)
      sorted_tasks.push_back(task.get());
  }
  std::stable_sort(sorted_tasks.begin(), sorted_tasks.end(),
                   [](Task* a, Task* b) { return a->priority < b->priority; });

  // Reset Hessian and Gradient.
  H.setZero();
  H.diagonal().setConstant(regularization);
  c.setZero();

  // Cumulative nullspace projector and Jacobian stack.
  // This is only built up if there are 2+ priority levels — otherwise N stays at identity.
  nullspace_projector.setIdentity(num_variables, num_variables);
  jacobian_stack.resize(0, num_variables);

  // Walk tasks in priority order. Tasks are projected through the current nullspace_projector
  // (encoding all strictly-higher priorities); whenever we cross into a new priority level,
  // rebuild the projector from everything stacked so far so the new level acts only in the
  // nullspace of everything above it. Tasks are sorted ascending by priority (1 = highest),
  // so the back holds the lowest priority level — its tasks don't need to be appended to
  // `jacobian_stack` since no further levels will project against them.
  const int lowest_priority = sorted_tasks.empty() ? 0 : sorted_tasks.back()->priority;
  const Task* prev_task = nullptr;
  for (Task* task : sorted_tasks) {
    if (prev_task && task->priority != prev_task->priority) {
      rebuildNullspaceProjector(regularization);
    }
    auto result = addTaskContribution(scene, task);
    if (!result.has_value()) {
      return tl::make_unexpected(result.error());
    }
    // Stacked Jacobians are only needed for levels above the lowest priority.
    if (task->priority < lowest_priority) {
      const int n = static_cast<int>(task->jacobian_container.rows());
      const int prev = static_cast<int>(jacobian_stack.rows());
      jacobian_stack.conservativeResize(prev + n, num_variables);
      jacobian_stack.middleRows(prev, n) = task->jacobian_container;
    }
    prev_task = task;
  }

  // Compute barrier values and Jacobians once, then add objective contributions.
  if (barrier_H_contribution.rows() != num_variables) {
    barrier_H_contribution.resize(num_variables, num_variables);
    barrier_c_contribution.resize(num_variables);
  }

  for (const auto& barrier : barriers) {
    auto barrier_result = barrier->computeBarrier(scene);
    if (!barrier_result.has_value()) {
      return tl::make_unexpected("Failed to compute barrier: " + barrier_result.error());
    }
    auto jacobian_result = barrier->computeJacobian(scene);
    if (!jacobian_result.has_value()) {
      return tl::make_unexpected("Failed to compute barrier Jacobian: " + jacobian_result.error());
    }
    barrier->formatQpObjective(scene, barrier_H_contribution, barrier_c_contribution);
    H += barrier_H_contribution;
    c += barrier_c_contribution;
  }

  // Build inequality rows for constraints and barriers, in the original dq space.
  // No transformation is needed because dq is the decision variable directly.
  // Additionally, cache sizes.
  constraint_sizes.reserve(constraints.size());
  int total_constraint_rows = 0;
  for (const auto& constraint : constraints) {
    int num_rows = constraint->getNumConstraints(scene);
    constraint_sizes.push_back(num_rows);
    total_constraint_rows += num_rows;
  }
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

  const bool init_required = !solver || (total_constraint_rows != last_constraint_rows ||
                                         total_barrier_rows != last_barrier_rows);

  // Resize constraint workspace if dimensions changed
  if (init_required) {
    constraint_workspace_A.resize(total_rows, num_variables);
    constraint_workspace_lower.resize(total_rows);
    constraint_workspace_upper.resize(total_rows);
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

  // Fill barrier constraints (one-sided: -inf <= G*dq <= h).
  // Barrier values and Jacobians were already computed above.
  for (size_t i = 0; i < barriers.size(); ++i) {
    const int num_rows = barrier_sizes.at(i);

    if (row_offset + num_rows > total_rows) {
      return tl::make_unexpected("Internal error: barrier row offset exceeds total rows");
    }

    Eigen::Ref<Eigen::MatrixXd> barrier_G_view =
        constraint_workspace_A.middleRows(row_offset, num_rows);
    Eigen::Ref<Eigen::VectorXd> barrier_h_view =
        constraint_workspace_upper.segment(row_offset, num_rows);

    barriers.at(i)->formatQpInequalities(barrier_G_view, barrier_h_view);

    constraint_workspace_lower.segment(row_offset, num_rows).setConstant(-kInfinity);

    row_offset += num_rows;
  }

  // Clear sizes for next iteration
  constraint_sizes.clear();
  barrier_sizes.clear();

  return detail::solveQp(solver, settings, init_required, num_variables, total_rows, H, c,
                         constraint_workspace_A, constraint_workspace_lower,
                         constraint_workspace_upper, delta_q);
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
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();

  // Compute candidate configuration by integrating delta_q. The copy into a plain VectorXd
  // makes the call match pinocchio's pre-instantiated integrate() signature, so this TU does
  // not instantiate the joint-visitor templates itself.
  const Eigen::VectorXd dq(delta_q);
  const Eigen::VectorXd q_candidate = pinocchio::integrate(model, q, dq);

  // Evaluate all barriers at the candidate configuration. enforce_barriers_data is a
  // pre-allocated pinocchio::Data scoped to this method, so we don't mutate scene state.
  for (const auto& barrier : barriers) {
    auto h_candidate_result =
        barrier->evaluateAtConfiguration(model, enforce_barriers_data, q_candidate);
    if (!h_candidate_result.has_value()) {
      return tl::make_unexpected(h_candidate_result.error());
    }

    // Safe (or unsupported, i.e., infinite) barriers do not restrict the step.
    const double h_candidate = h_candidate_result.value();
    if (!std::isfinite(h_candidate) || h_candidate >= -tolerance) {
      continue;
    }

    // Violated at the candidate: allow the step only if it improves this barrier's value
    // relative to the current configuration (recovery), otherwise veto its joints.
    auto h_current_result = barrier->evaluateAtConfiguration(model, enforce_barriers_data, q);
    if (!h_current_result.has_value()) {
      return tl::make_unexpected(h_current_result.error());
    }
    if (h_candidate > h_current_result.value()) {
      continue;
    }

    // Recompute the barrier Jacobian at the current configuration. Zero only the joints with a
    // nonzero column, mapping the group velocity indices to the full-model indices of delta_q.
    auto jacobian_result = barrier->computeJacobian(scene);
    if (!jacobian_result.has_value()) {
      return tl::make_unexpected(jacobian_result.error());
    }
    const Eigen::MatrixXd& barrier_jacobian = barrier->jacobian_container;
    for (int j = 0; j < num_variables; ++j) {
      if (barrier_jacobian.col(j).cwiseAbs().maxCoeff() > 0.0) {
        delta_q(v_indices(j)) = 0.0;
      }
    }
  }

  return {};
}

tl::expected<void, std::string> Oink::addTaskContribution(const Scene& scene, Task* task) {
  auto jacobian_result = task->computeJacobian(scene);
  if (!jacobian_result.has_value()) {
    return tl::make_unexpected("Failed to compute Jacobian: " + jacobian_result.error());
  }
  auto error_result = task->computeError(scene);
  if (!error_result.has_value()) {
    return tl::make_unexpected("Failed to compute error: " + error_result.error());
  }

  // min ||W J (N z) + W alpha e||^2 with delta_q = N z (z lives in the priority's nullspace).
  // We absorb the parameterization into a projected effective Jacobian and keep the
  // optimization variable as dq, so the same QP can be reused regardless of priority count.
  projected_weighted_jacobian.noalias() =
      task->weight * task->jacobian_container * nullspace_projector;
  weighted_error.noalias() = task->weight * (task->gain * task->error_container);

  const double mu = task->lm_damping * weighted_error.squaredNorm();

  task->H_dense.noalias() = projected_weighted_jacobian.transpose() * projected_weighted_jacobian;
  task->H_dense.diagonal().array() += mu;
  H += task->H_dense;
  c.noalias() += projected_weighted_jacobian.transpose() * weighted_error;

  return {};
}

void Oink::rebuildNullspaceProjector(double lambda_sq) {
  // Damped pseudoinverse using `lambda_sq` (caller passes the QP's Tikhonov regularization).
  // At well-conditioned configurations (sigma >> sqrt(lambda_sq)) this is numerically the
  // standard nullspace projector; near singularities the damping preserves SPD-ness of
  // (J J^T + lambda_sq I).
  Eigen::MatrixXd jjt_damped = jacobian_stack * jacobian_stack.transpose();
  jjt_damped.diagonal().array() += lambda_sq;
  const Eigen::MatrixXd jjt_inv_j = jjt_damped.llt().solve(jacobian_stack);
  nullspace_projector.setIdentity(num_variables, num_variables);
  nullspace_projector.noalias() -= jacobian_stack.transpose() * jjt_inv_j;
}

}  // namespace roboplan
