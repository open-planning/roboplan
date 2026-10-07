#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include <Eigen/Eigenvalues>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/qp_backend.hpp>

namespace {
// Minimum squared norm threshold to avoid division by zero in barrier regularization
constexpr double kMinNormSq = 1e-12;
}  // namespace

namespace roboplan {

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

Eigen::VectorXd Barrier::computeSafeDisplacement(const SceneContext& /*context*/) const {
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
  G = -jacobian_container / dt;

  // Saturating class-K function with safety margin: α(h - m) = γ·(h - m) / (1 + |h - m|)
  for (int i = 0; i < barrier_values.size(); ++i) {
    const double h_shifted = barrier_values[i] - safety_margin;
    b[i] = gain * h_shifted / (1.0 + std::abs(h_shifted));
  }
}

void Barrier::formatQpObjective(const SceneContext& context, Eigen::Ref<Eigen::MatrixXd> H,
                                Eigen::Ref<Eigen::VectorXd> c) const {
  const double jacobian_norm_sq = jacobian_container.squaredNorm();

  // Avoid division by zero: with a near-zero Jacobian, no regularization is needed.
  if (jacobian_norm_sq < kMinNormSq) {
    H.setZero();
    c.setZero();
    return;
  }

  const Eigen::VectorXd dq_safe = computeSafeDisplacement(context);

  // Regularization: The 1/‖J_h‖² factor normalizes by barrier sensitivity.
  const double weight = safe_displacement_gain / jacobian_norm_sq;

  // QP objective contribution: (r / (2·‖J_h‖²)) · ‖δq - δq_safe‖²
  // H_contribution = (r / ‖J_h‖²) · I = weight · I
  // c_contribution = -(r / ‖J_h‖²) · δq_safe = -weight · δq_safe
  H.setIdentity();
  H *= weight;
  c = -weight * dq_safe;
}

tl::expected<void, std::string> Barrier::computeQpInequalities(const SceneContext& context,
                                                               Eigen::Ref<Eigen::MatrixXd> G,
                                                               Eigen::Ref<Eigen::VectorXd> b) {
  auto barrier_result = computeBarrier(context);
  if (!barrier_result) {
    return barrier_result;
  }
  auto jacobian_result = computeJacobian(context);
  if (!jacobian_result) {
    return jacobian_result;
  }
  formatQpInequalities(G, b);
  return {};
}

tl::expected<void, std::string> Barrier::computeQpObjective(const SceneContext& context,
                                                            Eigen::Ref<Eigen::MatrixXd> H,
                                                            Eigen::Ref<Eigen::VectorXd> c) {
  auto barrier_result = computeBarrier(context);
  if (!barrier_result) {
    return barrier_result;
  }
  auto jacobian_result = computeJacobian(context);
  if (!jacobian_result) {
    return jacobian_result;
  }
  formatQpObjective(context, H, c);
  return {};
}

void Task::setLastDisplacement(const Eigen::VectorXd& delta_q_prev_in) {
  if (delta_q_prev_in.size() != delta_q_prev.size()) {
    throw std::invalid_argument("Task: delta_q_prev size (" +
                                std::to_string(delta_q_prev_in.size()) + ") does not match " +
                                std::to_string(delta_q_prev.size()));
  }
  delta_q_prev = delta_q_prev_in;
}

tl::expected<void, std::string> Task::computeQpObjective(const SceneContext& context,
                                                         Eigen::MatrixXd& H, Eigen::VectorXd& c) {
  auto jacobian_result = computeJacobian(context);
  if (!jacobian_result.has_value()) {
    return tl::make_unexpected("Failed to compute Jacobian: " + jacobian_result.error());
  }

  auto error_result = computeError(context);
  if (!error_result.has_value()) {
    return tl::make_unexpected("Failed to compute error: " + error_result.error());
  }
  // Carry (1 - β) of the previous task-space velocity, so the QP solves
  //     min ‖J (Δq - (1 - β) Δq_prev) + α e‖²  ⇒  Δq = (1 - β) Δq_prev + α J⁺ e,
  // folded into the error as e - ((1 - β) / α) · J · Δq_prev.
  const double beta = std::min(1.0, 2.0 * std::sqrt(gain));
  if (beta < 1.0) {
    error_container.noalias() -= ((1.0 - beta) / gain) * (jacobian_container * delta_q_prev);
  }

  // Apply weights: J_w = W*J, e_w = -α*W*e
  jacobian_container.applyOnTheLeft(weight);
  error_container *= -gain;
  error_container.applyOnTheLeft(weight);

  // Levenberg-Marquardt damping, scaled by the weighted error
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

  context_ = std::make_unique<SceneContext>(scene);
}

Oink::~Oink() = default;

void Oink::refreshContext(const Scene& scene) { context_ = std::make_unique<SceneContext>(scene); }

tl::expected<void, std::string>
Oink::solveIk(const Eigen::VectorXd& q, const std::vector<std::shared_ptr<Task>>& tasks,
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

  // Pose this solver's own context at the requested configuration. Everything below reads `q`
  // and writes its kinematics scratch through this context, so the configuration being solved
  // at never passes through state another thread can write.
  context_->setJointPositions(q);
  const SceneContext& context = *context_;

  // Barriers are evaluated value-first, so computeBarrier() reads frame placements that no
  // Jacobian call has populated yet, so refresh them up front. Tasks do not need this; each
  // computes its own Jacobian (which runs forward kinematics) before its error term reads oMf,
  // so the barrier-free path does not pay for the pass.
  if (!barriers.empty()) {
    context.updateFramePlacements(q);
  }

  // Objective terms shared by every priority level: Tikhonov regularization and barriers.
  H_base.setZero(num_variables, num_variables);
  H_base.diagonal().setConstant(regularization);
  c_base.setZero(num_variables);

  // Compute barrier values and Jacobians once, then add objective contributions.
  if (barrier_H_contribution.rows() != num_variables) {
    barrier_H_contribution.resize(num_variables, num_variables);
    barrier_c_contribution.resize(num_variables);
  }

  for (const auto& barrier : barriers) {
    auto barrier_result = barrier->computeBarrier(context);
    if (!barrier_result.has_value()) {
      return tl::make_unexpected("Failed to compute barrier: " + barrier_result.error());
    }
    auto jacobian_result = barrier->computeJacobian(context);
    if (!jacobian_result.has_value()) {
      return tl::make_unexpected("Failed to compute barrier Jacobian: " + jacobian_result.error());
    }
    barrier->formatQpObjective(context, barrier_H_contribution, barrier_c_contribution);
    H_base += barrier_H_contribution;
    c_base += barrier_c_contribution;
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

  // Cache the row count of each constraint and barrier. Rows are in dq space, which is the
  // decision variable itself.
  constraint_sizes.reserve(constraints.size());
  int total_constraint_rows = 0;
  for (const auto& constraint : constraints) {
    int num_rows = constraint->getNumConstraints(context);
    constraint_sizes.push_back(num_rows);
    total_constraint_rows += num_rows;
  }
  barrier_sizes.reserve(barriers.size());
  int total_barrier_rows = 0;
  for (const auto& barrier : barriers) {
    int num_rows = barrier->getNumBarriers(context);
    barrier_sizes.push_back(num_rows);
    total_barrier_rows += num_rows;
  }
  last_constraint_rows = total_constraint_rows;
  last_barrier_rows = total_barrier_rows;

  // Total inequality rows = constraints (box) + barriers (one-sided: -inf <= G*dq <= h)
  const int total_rows = total_constraint_rows + total_barrier_rows;
  if (constraint_workspace_A.rows() != total_rows) {
    constraint_workspace_A.resize(total_rows, num_variables);
    constraint_workspace_lower.resize(total_rows);
    constraint_workspace_upper.resize(total_rows);
  }

  // Fill constraint matrices block by block
  int row_offset = 0;
  for (size_t i = 0; i < constraints.size(); ++i) {
    const int num_rows = constraint_sizes.at(i);

    Eigen::Ref<Eigen::MatrixXd> constraint_A_view =
        constraint_workspace_A.middleRows(row_offset, num_rows);
    Eigen::Ref<Eigen::VectorXd> constraint_lower_view =
        constraint_workspace_lower.segment(row_offset, num_rows);
    Eigen::Ref<Eigen::VectorXd> constraint_upper_view =
        constraint_workspace_upper.segment(row_offset, num_rows);

    auto constraint_result = constraints.at(i)->computeQpConstraints(
        context, constraint_A_view, constraint_lower_view, constraint_upper_view);
    if (!constraint_result.has_value()) {
      return tl::make_unexpected("Failed to compute constraints: " + constraint_result.error());
    }

    row_offset += num_rows;
  }

  // Fill barrier constraints (one-sided: -inf <= G*dq <= h).
  // Barrier values and Jacobians were already computed above.
  for (size_t i = 0; i < barriers.size(); ++i) {
    const int num_rows = barrier_sizes.at(i);

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

  // Solve one QP per priority level (1 = highest), as a cascade. Level k searches only the
  // nullspace Z of every higher-priority task's weighted Jacobian, δq = δq_{k-1} + Z·z,
  // so the task velocities W·J·δq that the higher levels achieved are preserved exactly.
  // Every level keeps the constraints and barriers, so an inequality restricts a lower level
  // only where it is active.
  nullspace_basis.setIdentity(num_variables, num_variables);
  nullspace_penalty_H.setZero(num_variables, num_variables);
  nullspace_penalty_c.setZero(num_variables);
  size_t level_begin = 0;
  for (size_t level = 0;; ++level) {
    size_t level_end = level_begin;
    while (level_end < sorted_tasks.size() &&
           sorted_tasks[level_end]->priority == sorted_tasks[level_begin]->priority) {
      ++level_end;
    }

    H = H_base;
    c = c_base;
    for (size_t i = level_begin; i < level_end; ++i) {
      auto result = addTaskContribution(context, sorted_tasks[i], H, c);
      if (!result.has_value()) {
        return tl::make_unexpected(result.error());
      }
    }

    // ProxQP fixes the problem dimensions at construction, so each level keeps its own solver
    // and rebuilds it only when that level's dimensions change.
    const int num_free = static_cast<int>(nullspace_basis.cols());
    if (solvers.size() <= level) {
      solvers.resize(level + 1);
      solver_dims.resize(level + 1, {-1, -1});
    }
    const bool init_required =
        !solvers[level] || solver_dims[level] != std::pair{num_free, total_rows};
    solver_dims[level] = {num_free, total_rows};

    tl::expected<void, std::string> result;
    if (level == 0) {
      result = detail::solveQp(solvers[level], settings, init_required, num_variables, total_rows,
                               H, c, constraint_workspace_A, constraint_workspace_lower,
                               constraint_workspace_upper, delta_q);
    } else {
      // Substitute δq = δq_{k-1} + Z·z into this level's objective and constraints.
      previous_delta_q = delta_q;
      H += nullspace_penalty_H;
      c += nullspace_penalty_c;
      level_H.noalias() = nullspace_basis.transpose() * H * nullspace_basis;
      level_c.noalias() = nullspace_basis.transpose() * (H * previous_delta_q + c);
      level_A.noalias() = constraint_workspace_A * nullspace_basis;
      level_offset.noalias() = constraint_workspace_A * previous_delta_q;
      level_lower = constraint_workspace_lower - level_offset;
      level_upper = constraint_workspace_upper - level_offset;
      level_z.resize(num_free);
      // Z is only unique up to rotation, so it can differ between solves and the previous z is no
      // warm start. Map this level's previous solution through the current Z instead.
      const bool has_guess = level < level_solutions.size();
      if (has_guess) {
        level_guess.noalias() =
            nullspace_basis.transpose() * (level_solutions[level] - previous_delta_q);
      }
      OinkSettings level_settings = settings;
      level_settings.warm_start = false;
      result = detail::solveQp(solvers[level], level_settings, init_required, num_free, total_rows,
                               level_H, level_c, level_A, level_lower, level_upper, level_z,
                               has_guess && settings.warm_start
                                   ? std::optional<Eigen::Ref<const Eigen::VectorXd>>(level_guess)
                                   : std::nullopt);
      if (result.has_value()) {
        delta_q = previous_delta_q + nullspace_basis * level_z;
      }
    }
    if (!result.has_value()) {
      return result;
    }
    if (level_solutions.size() <= level) {
      level_solutions.resize(level + 1);
    }
    level_solutions[level] = delta_q;
    if (level_end == sorted_tasks.size()) {
      return result;
    }

    // Shrink the basis to the nullspace of this level's weighted task Jacobians.
    int level_rows = 0;
    for (size_t i = level_begin; i < level_end; ++i) {
      level_rows += static_cast<int>(sorted_tasks[i]->jacobian_container.rows());
    }
    level_jacobian.resize(level_rows, num_variables);
    for (size_t i = level_begin, row = 0; i < level_end; ++i) {
      const Task* task = sorted_tasks[i];
      const auto n = task->jacobian_container.rows();
      level_jacobian.middleRows(row, n).noalias() = task->weight * task->jacobian_container;
      row += n;
    }
    // Split the directions of Z by the singular values of J·Z (eigenvalues of its Gram matrix,
    // ascending): those above the level's singularity threshold are its range and leave Z (exact
    // lexicographic ordering); the rest stay for the lower levels, which pay
    // ‖J (Δq - Δq*)‖²/threshold² for using them, so a direction opens up gradually as it becomes
    // singular instead of switching at the threshold. Without a threshold, only the numerically
    // zero singular values are left free.
    double threshold = 0.0;
    for (size_t i = level_begin; i < level_end; ++i) {
      threshold = std::max(threshold, sorted_tasks[i]->singularity_threshold);
    }
    level_A.noalias() = level_jacobian * nullspace_basis;
    level_H.noalias() = level_A.transpose() * level_A;
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(level_H);
    const double cutoff = threshold > 0.0 ? threshold * threshold
                                          : eig.eigenvalues().maxCoeff() * num_free *
                                                std::numeric_limits<double>::epsilon();
    const int free = static_cast<int>((eig.eigenvalues().array() <= cutoff).count());
    if (free == 0) {
      return result;  // No freedom left for the lower levels.
    }
    if (threshold > 0.0) {
      level_H.noalias() = level_jacobian.transpose() * level_jacobian / (threshold * threshold);
      nullspace_penalty_H += level_H;
      nullspace_penalty_c.noalias() -= level_H * delta_q;
    }
    nullspace_basis = (nullspace_basis * eig.eigenvectors().leftCols(free)).eval();
    level_begin = level_end;
  }
}

// Overload: tasks, constraints, and barriers, solved at the scene's current joint positions.
// This is the single read of Scene::getCurrentJointPositions() on the Oink path: it happens once,
// on entry, and the value is immediately copied into this solver's context.
tl::expected<void, std::string>
Oink::solveIk(const Scene& scene, const std::vector<std::shared_ptr<Task>>& tasks,
              const std::vector<std::shared_ptr<Constraints>>& constraints,
              const std::vector<std::shared_ptr<Barrier>>& barriers,
              Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
              double regularization) {
  return solveIk(scene.getCurrentJointPositions(), tasks, constraints, barriers, delta_q,
                 regularization);
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
  return enforceBarriers(scene.getCurrentJointPositions(), barriers, delta_q, tolerance);
}

tl::expected<void, std::string> Oink::enforceBarriers(
    const Eigen::VectorXd& q, const std::vector<std::shared_ptr<Barrier>>& barriers,
    Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q, double tolerance) {
  if (barriers.empty()) {
    return {};
  }

  // Barrier Jacobians below are recomputed at `q`, so pose the context before touching them.
  context_->setJointPositions(q);
  context_->updateFramePlacements(q);
  const SceneContext& context = *context_;
  const auto& model = context.getModel();

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
    auto jacobian_result = barrier->computeJacobian(context);
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

tl::expected<void, std::string> Oink::addTaskContribution(const SceneContext& context, Task* task,
                                                          Eigen::MatrixXd& H_out,
                                                          Eigen::VectorXd& c_out) {
  auto jacobian_result = task->computeJacobian(context);
  if (!jacobian_result.has_value()) {
    return tl::make_unexpected("Failed to compute Jacobian: " + jacobian_result.error());
  }
  auto error_result = task->computeError(context);
  if (!error_result.has_value()) {
    return tl::make_unexpected("Failed to compute error: " + error_result.error());
  }
  // Carry (1 - β) of the previous task-space velocity, so the QP solves
  //     min ‖J (Δq - (1 - β) Δq_prev) + α e‖²  ⇒  Δq = (1 - β) Δq_prev + α J⁺ e,
  // folded into the error as e - ((1 - β) / α) · J · Δq_prev.
  const double beta = std::min(1.0, 2.0 * std::sqrt(task->gain));
  if (beta < 1.0) {
    task->error_container.noalias() -=
        ((1.0 - beta) / task->gain) * (task->jacobian_container * task->delta_q_prev);
  }

  weighted_jacobian.noalias() = task->weight * task->jacobian_container;
  weighted_error.noalias() = task->weight * (task->gain * task->error_container);

  const double mu = task->lm_damping * weighted_error.squaredNorm();

  task->H_dense.noalias() = weighted_jacobian.transpose() * weighted_jacobian;
  task->H_dense.diagonal().array() += mu;

  // Selective singularity damping: floor the Hessian's eigenvalues at threshold², which damps
  // only the near-singular directions of W·J and leaves the well-conditioned ones untouched.
  if (task->singularity_threshold > 0.0) {
    const double threshold_sq = task->singularity_threshold * task->singularity_threshold;
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(task->H_dense);
    if (eig.eigenvalues().minCoeff() < threshold_sq) {
      task->H_dense.noalias() = eig.eigenvectors() *
                                eig.eigenvalues().cwiseMax(threshold_sq).asDiagonal() *
                                eig.eigenvectors().transpose();
    }
  }

  H_out += task->H_dense;
  c_out.noalias() += weighted_jacobian.transpose() * weighted_error;

  return {};
}

}  // namespace roboplan
