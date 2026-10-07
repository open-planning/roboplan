#include <algorithm>
#include <chrono>
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
// Relative error improvement below which an iterative solve counts as having stagnated.
constexpr double kStagnationImprovement = 0.99;
// Consecutive stagnant iterations after which an iterative solve attempt stops.
constexpr int kMaxStagnantIters = 5;
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
  joint_names = maybe_group_info->joint_names;
  q_indices = maybe_group_info->q_indices;
  v_indices = maybe_group_info->v_indices;
  num_variables = static_cast<int>(v_indices.size());

  H = Eigen::MatrixXd::Zero(num_variables, num_variables);
  c = Eigen::VectorXd::Zero(num_variables);
  // Keep num_variables columns even with no constraint rows, so the cascade can multiply by it.
  constraint_workspace_A.resize(0, num_variables);

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
  // Insertion sort: stable and allocates no buffer.
  for (size_t i = 1; i < sorted_tasks.size(); ++i) {
    for (size_t j = i; j > 0 && sorted_tasks[j - 1]->priority > sorted_tasks[j]->priority; --j) {
      std::swap(sorted_tasks[j - 1], sorted_tasks[j]);
    }
  }

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
  nullspace_penalty_H.setZero(num_variables, num_variables);
  nullspace_penalty_c.setZero(num_variables);
  size_t level_begin = 0;
  for (size_t k = 0;; ++k) {
    size_t level_end = level_begin;
    while (level_end < sorted_tasks.size() &&
           sorted_tasks[level_end]->priority == sorted_tasks[level_begin]->priority) {
      ++level_end;
    }
    if (levels.size() < k + 2) {
      levels.resize(k + 2);  // This level, and a home for the next level's basis.
    }
    Level& level = levels[k];
    const Eigen::MatrixXd& Z = level.basis;  // Unused at level 0, where Z would be the identity.

    H = H_base;
    c = c_base;
    for (size_t i = level_begin; i < level_end; ++i) {
      auto result = addTaskContribution(context, sorted_tasks[i], H, c);
      if (!result.has_value()) {
        return tl::make_unexpected(result.error());
      }
    }

    const int num_free = k == 0 ? num_variables : static_cast<int>(Z.cols());
    const bool init_required = !level.solver || level.dims != std::pair{num_free, total_rows};
    level.dims = {num_free, total_rows};

    tl::expected<void, std::string> result;
    if (k == 0) {
      result = detail::solveQp(level.solver, settings, init_required, num_variables, total_rows, H,
                               c, constraint_workspace_A, constraint_workspace_lower,
                               constraint_workspace_upper, delta_q);
    } else {
      // Substitute δq = δq_{k-1} + Z·z into this level's objective and constraints.
      previous_delta_q = delta_q;
      H += nullspace_penalty_H;
      c += nullspace_penalty_c;
      c.noalias() += H * previous_delta_q;
      level.HZ.noalias() = H * Z;
      level.H.noalias() = Z.transpose() * level.HZ;
      level.c.noalias() = Z.transpose() * c;
      level.A.noalias() = constraint_workspace_A * Z;
      level.offset.noalias() = constraint_workspace_A * previous_delta_q;
      level.lower = constraint_workspace_lower - level.offset;
      level.upper = constraint_workspace_upper - level.offset;
      level.z.resize(num_free);
      // Z is only unique up to rotation, so it can differ between solves and the previous z is no
      // warm start. Map this level's previous solution through the current Z instead.
      const bool has_guess = level.solution.size() > 0;
      if (has_guess) {
        level.guess.noalias() = Z.transpose() * level.solution;
        level.guess.noalias() -= Z.transpose() * previous_delta_q;
      }
      OinkSettings level_settings = settings;
      level_settings.warm_start = false;
      result = detail::solveQp(level.solver, level_settings, init_required, num_free, total_rows,
                               level.H, level.c, level.A, level.lower, level.upper, level.z,
                               has_guess && settings.warm_start
                                   ? std::optional<Eigen::Ref<const Eigen::VectorXd>>(level.guess)
                                   : std::nullopt);
      if (result.has_value()) {
        previous_delta_q.noalias() += Z * level.z;
        delta_q = previous_delta_q;
      }
    }
    if (!result.has_value()) {
      return result;
    }
    level.solution = delta_q;
    if (level_end == sorted_tasks.size()) {
      return result;
    }

    // Shrink the basis to the nullspace of this level's weighted task Jacobians.
    int level_rows = 0;
    for (size_t i = level_begin; i < level_end; ++i) {
      level_rows += static_cast<int>(sorted_tasks[i]->jacobian_container.rows());
    }
    level.jacobian.resize(level_rows, num_variables);
    for (size_t i = level_begin, row = 0; i < level_end; ++i) {
      const auto n = sorted_tasks[i]->weighted_jacobian.rows();
      level.jacobian.middleRows(row, n) = sorted_tasks[i]->weighted_jacobian;
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
    if (k == 0) {
      level.gram.noalias() = level.jacobian.transpose() * level.jacobian;
    } else {
      level.JZ.noalias() = level.jacobian * Z;
      level.gram.noalias() = level.JZ.transpose() * level.JZ;
    }
    level.eig.compute(level.gram);
    const double cutoff = threshold > 0.0 ? threshold * threshold
                                          : level.eig.eigenvalues().maxCoeff() * num_free *
                                                std::numeric_limits<double>::epsilon();
    const int free = static_cast<int>((level.eig.eigenvalues().array() <= cutoff).count());
    if (free == 0) {
      return result;
    }
    if (threshold > 0.0) {
      H.noalias() = level.jacobian.transpose() * level.jacobian / (threshold * threshold);
      nullspace_penalty_H += H;
      nullspace_penalty_c.noalias() -= H * delta_q;
    }
    Eigen::MatrixXd& next_basis = levels[k + 1].basis;
    if (k == 0) {
      next_basis = level.eig.eigenvectors().leftCols(free);
    } else {
      next_basis.noalias() = Z * level.eig.eigenvectors().leftCols(free);
    }
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

  task->weighted_jacobian.noalias() = task->weight * task->jacobian_container;
  task->weighted_error.noalias() = task->weight * (task->gain * task->error_container);

  const double mu = task->lm_damping * task->weighted_error.squaredNorm();

  task->H_dense.noalias() = task->weighted_jacobian.transpose() * task->weighted_jacobian;
  task->H_dense.diagonal().array() += mu;

  // Selective singularity damping: floor the Hessian's eigenvalues at threshold², which damps
  // only the near-singular directions of W·J and leaves the well-conditioned ones untouched.
  if (task->singularity_threshold > 0.0) {
    const double threshold_sq = task->singularity_threshold * task->singularity_threshold;
    task_eig.compute(task->H_dense);
    if (task_eig.eigenvalues().minCoeff() < threshold_sq) {
      task_VD.noalias() =
          task_eig.eigenvectors() * task_eig.eigenvalues().cwiseMax(threshold_sq).asDiagonal();
      task->H_dense.noalias() = task_VD * task_eig.eigenvectors().transpose();
    }
  }

  H_out += task->H_dense;
  c_out.noalias() += task->weighted_jacobian.transpose() * task->weighted_error;

  return {};
}

tl::expected<Eigen::VectorXd, std::string> Oink::solveIterativeIk(
    const Eigen::VectorXd& q_start, const std::vector<std::shared_ptr<Task>>& goal_tasks,
    const std::vector<std::shared_ptr<Task>>& extra_tasks,
    const std::vector<std::shared_ptr<Constraints>>& constraints,
    const std::vector<std::shared_ptr<Barrier>>& barriers, const IterativeSolveOptions& options) {
  const auto start_time = std::chrono::steady_clock::now();
  const std::chrono::duration<double> timeout(options.max_time);
  const Scene& scene = context_->getScene();

  std::vector<std::shared_ptr<Task>> tasks(goal_tasks.begin(), goal_tasks.end());
  tasks.insert(tasks.end(), extra_tasks.begin(), extra_tasks.end());

  // Worst error at `q` over the goal tasks, constraints, and barriers, each relative to its own
  // tolerance, so a value of at most 1 means `q` is a solution.
  const auto normalizedError = [&](const Eigen::VectorXd& q) -> tl::expected<double, std::string> {
    context_->setJointPositions(q);
    context_->updateFramePlacements(q);
    double error = 0.0;
    for (const auto& task : goal_tasks) {
      if (const auto result = task->computeError(*context_); !result) {
        return tl::make_unexpected(result.error());
      }
      error = std::max(error,
                       (task->weight * task->error_container).norm() / options.max_task_error_norm);
    }
    for (const auto& constraint : constraints) {
      error = std::max(error, constraint->computeViolation(*context_).norm() /
                                  options.max_constraint_violation_norm);
    }
    for (const auto& barrier : barriers) {
      const auto h = barrier->evaluateAtConfiguration(context_->getModel(), context_->getData(), q);
      if (!h) {
        return tl::make_unexpected(h.error());
      }
      if (*h < 0.0) {
        error = kInfinity;
      }
    }
    if (options.check_collisions && context_->hasCollisions(q)) {
      error = kInfinity;
    }
    return error;
  };

  Eigen::VectorXd q = q_start;
  Eigen::VectorXd delta_q(num_variables);
  Eigen::VectorXd delta_q_full = Eigen::VectorXd::Zero(scene.getModel().nv);
  for (size_t attempt = 0; attempt <= options.max_restarts; ++attempt) {
    if (attempt > 0) {
      context_->randomizeJointPositions(joint_names, q);
    }
    double best_error = kInfinity;
    double error = kInfinity;
    int stagnant = 0;
    for (size_t iter = 0; iter < options.max_iters; ++iter) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        return tl::make_unexpected("Iterative IK solve timed out");
      }
      delta_q.setZero();
      if (const auto result =
              solveIk(q, tasks, constraints, barriers, delta_q, options.regularization);
          !result) {
        return tl::make_unexpected(result.error());
      }
      delta_q_full(v_indices) = delta_q;
      q = scene.clampToValidConfiguration(scene.integrate(q, delta_q_full));

      const auto result = normalizedError(q);
      if (!result) {
        return tl::make_unexpected(result.error());
      }
      error = *result;
      if (error <= 1.0 && options.fast_return) {
        return q;
      }
      // Give up on this attempt once the error plateaus. Without fast_return a healthy solve
      // plateaus at the solver's own precision and is accepted below if it is within tolerance.
      if (error < best_error * kStagnationImprovement) {
        best_error = error;
        stagnant = 0;
      } else if (++stagnant >= kMaxStagnantIters) {
        break;
      }
    }
    if (error <= 1.0) {
      return q;
    }
  }
  return tl::make_unexpected("Iterative IK solve failed within " +
                             std::to_string(options.max_restarts + 1) + " attempts");
}

}  // namespace roboplan
