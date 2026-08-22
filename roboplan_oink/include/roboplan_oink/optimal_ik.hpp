#pragma once

#include <limits>
#include <memory>
#include <string>

#include <tl/expected.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_context.hpp>
#include <roboplan/core/types.hpp>

#include <roboplan_oink/detail/qp_solver_fwd.hpp>
#include <roboplan_oink/oink_settings.hpp>

namespace roboplan {

/// @brief Infinity value used for unbounded QP constraint bounds.
constexpr double kInfinity = std::numeric_limits<double>::infinity();

/// @brief Abstract base class for IK tasks.
///
/// Each task owns pre-allocated storage for Jacobian, error, and H_dense matrices.
/// Subclasses must:
/// 1. Call initializeStorage() in their constructor with correct dimensions
/// 2. Implement computeJacobian() to fill jacobian_container
/// 3. Implement computeError() to fill error_container
struct Task {
  Task(int task_priority, Eigen::MatrixXd weight_matrix, double task_gain = 1.0,
       double lm_damp = 0.0)
      : gain(task_gain), weight(weight_matrix), lm_damping(lm_damp), priority(task_priority) {
    if (priority < 1) {
      throw std::invalid_argument("Task priority must be >= 1");
    }
  }
  virtual ~Task() = default;

  /// @brief Initialize pre-allocated storage with correct dimensions.
  /// @param task_rows Number of rows for the task (e.g., 6 for SE(3), nv for configuration)
  /// @param num_vars Number of optimization variables (model.nv)
  void initializeStorage(int task_rows, int num_vars) {
    num_variables = num_vars;
    jacobian_container = Eigen::MatrixXd::Zero(task_rows, num_vars);
    error_container = Eigen::VectorXd::Zero(task_rows);
    H_dense = Eigen::MatrixXd::Zero(num_vars, num_vars);
  }

  /// @brief Compute the task Jacobian and store in jacobian_container.
  /// @param context The context supplying the configuration and the kinematics scratch to write.
  /// @return void on success, error message on failure.
  virtual tl::expected<void, std::string> computeJacobian(const SceneContext& context) = 0;

  /// @brief Compute the task error and store in error_container.
  /// @param context The context supplying the configuration and the frame placements to read.
  /// @return void on success, error message on failure.
  virtual tl::expected<void, std::string> computeError(const SceneContext& context) = 0;

  /// @brief Compute QP objective matrices (H, c) for this task.
  ///
  /// Computes the contribution of this task to the quadratic program objective:
  ///     minimize  ½ ‖J Δq + α e‖²_W
  ///
  /// This is equivalent to:
  ///     minimize  ½ Δq^T H Δq + c^T Δq
  ///
  /// Where:
  /// - J: Task Jacobian matrix
  /// - Δq: Configuration displacement
  /// - α: Task gain for low-pass filtering
  /// - e: Task error vector
  /// - W: Weight matrix for cost normalization
  ///
  /// The method returns:
  /// - H = J_w^T J_w + μ I  (num_variables x num_variables Hessian matrix)
  /// - c = -J_w^T e_w       (num_variables x 1 linear term)
  ///
  /// Where J_w = W*J, e_w = -α*W*e, and μ is the Levenberg-Marquardt damping.
  /// @param context The context supplying the configuration and the kinematics scratch.
  /// @param H Output Hessian matrix
  /// @param c Output linear cost term
  /// @return void on success, error message on failure.
  tl::expected<void, std::string> computeQpObjective(const SceneContext& context,
                                                     Eigen::MatrixXd& H, Eigen::VectorXd& c);

  const double gain = 1.0;        // Task gain for low-pass filtering
  const Eigen::MatrixXd weight;   // Weight matrix for cost normalization
  const double lm_damping = 0.0;  // Levenberg-Marquardt damping
  const int priority = 1;         // Priority level (1 = highest; lower priorities are projected
                                  // into the nullspace of higher ones)
  int num_variables = 0;          // Number of optimization variables

  /// @brief Pre-allocated Jacobian container (task_rows × num_variables).
  Eigen::MatrixXd jacobian_container;

  /// @brief Pre-allocated error container (task_rows).
  Eigen::VectorXd error_container;

  /// @brief Pre-allocated dense Hessian matrix (num_variables × num_variables).
  Eigen::MatrixXd H_dense;
};

struct Constraints {
  virtual ~Constraints() = default;

  /// @brief Get the number of constraint rows this constraint will produce
  /// @param context The context (unused; the row count is fixed at construction).
  /// @return Number of constraint rows
  virtual int getNumConstraints(const SceneContext& context) const = 0;

  /// @brief Compute QP constraint matrices using pre-allocated workspace views
  ///
  /// The constraint_matrix, lower_bounds, and upper_bounds parameters are Eigen::Ref views
  /// into pre-allocated workspace memory. The views are already sized to match
  /// getNumConstraints() rows, so implementations should fill the entire view.
  ///
  /// @param context The context supplying the configuration and the kinematics scratch.
  /// @param constraint_matrix Output constraint matrix G (pre-sized view: num_constraints ×
  /// num_variables)
  /// @param lower_bounds Output lower bounds vector (pre-sized view: num_constraints)
  /// @param upper_bounds Output upper bounds vector (pre-sized view: num_constraints)
  /// @return void on success, error message on failure
  virtual tl::expected<void, std::string>
  computeQpConstraints(const SceneContext& context, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
                       Eigen::Ref<Eigen::VectorXd> lower_bounds,
                       Eigen::Ref<Eigen::VectorXd> upper_bounds) const = 0;
};

/// @brief Abstract base class for Control Barrier Functions
///
/// Barriers enforce safety constraints derived from the CBF condition:
///
///   Standard CBF:     ḣ(q) + α(h(q)) ≥ 0
///   Discrete time:    J_h · δq/dt + α(h(q)) ≥ 0
///   Rearranging:      -J_h · δq ≤ dt · α(h(q))
///   QP form:          G · δq ≤ b  where G = -J_h/dt, b = α(h(q))
///
/// Uses a saturating class-K function: α(h) = γ·h / (1 + |h|)
/// This provides bounded recovery force, preventing over-reaction when far from
/// the boundary while giving smooth, proportional behavior near constraints.
///
/// Safe displacement regularization adds a QP objective term:
///   (safe_displacement_gain / (2·‖J_h‖²)) · ‖δq - δq_safe‖²
///
/// This encourages the robot to move toward a known safe configuration when near
/// constraint boundaries. The weighting by 1/‖J_h‖² normalizes the contribution
/// based on how sensitive the barrier is to joint motion.
///
/// The safety_margin parameter provides a conservative buffer for hard constraints.
/// When safety_margin > 0, the CBF constraint is tightened by this amount, meaning
/// the barrier will begin to resist motion earlier (when h = safety_margin rather
/// than h = 0). This compensates for linearization errors in the discrete-time
/// formulation.
struct Barrier {
  /// @brief Constructor with barrier parameters
  /// @param gain Barrier gain (gamma), controls aggressiveness
  /// @param dt Timestep for discrete-time formulation (must match control loop period)
  /// @param safe_displacement_gain Gain for safe displacement regularization
  /// @param safety_margin Conservative margin for hard constraint guarantee (default 0.0)
  explicit Barrier(double gain, double dt, double safe_displacement_gain = 1.0,
                   double safety_margin = 0.0);

  /// @brief Initialize pre-allocated storage
  /// @param num_barriers Number of barrier constraints this barrier produces
  /// @param num_vars Number of optimization variables (model.nv)
  void initializeStorage(int num_barriers, int num_vars);

  /// @brief Get the number of barrier constraints this barrier produces
  /// @param context The context (unused; the row count is fixed at construction).
  /// @return Number of barrier constraint rows
  virtual int getNumBarriers(const SceneContext& context) const = 0;

  /// @brief Compute the barrier function values h(q)
  /// @param context The context supplying the configuration and the collision scratch to write.
  /// @note Barrier values h(q) >= 0 indicate safety; h(q) < 0 indicates violation
  /// @return void on success, error message on failure
  virtual tl::expected<void, std::string> computeBarrier(const SceneContext& context) = 0;

  /// @brief Compute the barrier Jacobian J_h = dh/dq
  /// @param context The context supplying the configuration and the kinematics scratch to write.
  /// @return void on success, error message on failure
  virtual tl::expected<void, std::string> computeJacobian(const SceneContext& context) = 0;

  /// @brief Compute safe displacement for regularization
  ///
  /// Subclasses can override to provide a non-zero safe displacement that
  /// the robot will be encouraged to move toward when near constraint boundaries.
  ///
  /// @param context The context supplying the configuration.
  /// @return Safe displacement vector (num_variables), default is zero
  virtual Eigen::VectorXd computeSafeDisplacement(const SceneContext& context) const;

  /// @brief Format the QP inequality constraints from already-computed barrier values/Jacobian.
  ///
  /// Produces: G_b * delta_q <= b_b
  /// Where:
  ///   G_b = -J_h / dt
  ///   b_b = γ·(h - m) / (1 + |h - m|)  (saturating class-K function)
  ///
  /// @pre computeBarrier() and computeJacobian() must have been called first.
  /// @param G Output constraint matrix (pre-sized view: num_barriers x num_variables)
  /// @param b Output constraint upper bound vector (pre-sized view: num_barriers)
  void formatQpInequalities(Eigen::Ref<Eigen::MatrixXd> G, Eigen::Ref<Eigen::VectorXd> b) const;

  /// @brief Format the QP objective contribution from already-computed barrier Jacobian.
  ///
  /// Computes: (safe_displacement_gain / (2·‖J_h‖²)) · ‖δq - δq_safe‖²
  ///
  /// @pre computeBarrier() and computeJacobian() must have been called first.
  /// @param context The context (passed to computeSafeDisplacement).
  /// @param H Output Hessian matrix contribution (num_variables x num_variables)
  /// @param c Output gradient vector contribution (num_variables)
  void formatQpObjective(const SceneContext& context, Eigen::Ref<Eigen::MatrixXd> H,
                         Eigen::Ref<Eigen::VectorXd> c) const;

  /// @brief Convenience: compute barrier + Jacobian, then format QP inequalities.
  /// Equivalent to calling computeBarrier(), computeJacobian(), formatQpInequalities().
  tl::expected<void, std::string> computeQpInequalities(const SceneContext& context,
                                                        Eigen::Ref<Eigen::MatrixXd> G,
                                                        Eigen::Ref<Eigen::VectorXd> b);

  /// @brief Convenience: compute barrier + Jacobian, then format QP objective.
  /// Equivalent to calling computeBarrier(), computeJacobian(), formatQpObjective().
  tl::expected<void, std::string> computeQpObjective(const SceneContext& context,
                                                     Eigen::Ref<Eigen::MatrixXd> H,
                                                     Eigen::Ref<Eigen::VectorXd> c);

  /// @brief Evaluate the minimum barrier value at a candidate configuration using FK.
  ///
  /// This method allows post-solve validation by computing the actual barrier value
  /// at a candidate configuration q, independent of the linearized constraint used
  /// in the QP. Used by Oink::enforceBarriers() to detect linearization errors.
  ///
  /// @param model Pinocchio model
  /// @param data Pinocchio data (will be modified by FK computation)
  /// @param q Candidate joint configuration to evaluate
  /// @return Expected containing minimum barrier value across all barrier constraints,
  ///         or infinity if this barrier type does not support configuration evaluation.
  ///         Returns error message if evaluation fails (e.g., frame not found).
  virtual tl::expected<double, std::string> evaluateAtConfiguration(const pinocchio::Model& model,
                                                                    pinocchio::Data& data,
                                                                    const Eigen::VectorXd& q) const;

  virtual ~Barrier() = default;

  const double gain;                    ///< Barrier gain (gamma)
  const double dt;                      ///< Timestep
  const double safe_displacement_gain;  ///< Gain for safe displacement regularization
  const double safety_margin;           ///< Conservative margin for hard constraints
  int num_variables = 0;

  /// Pre-allocated containers
  Eigen::VectorXd barrier_values;      ///< h(q) values (num_barriers)
  Eigen::MatrixXd jacobian_container;  ///< J_h matrix (num_barriers x num_variables)
};

/// @brief Oink - Optimal Inverse Kinematics solver
///
/// @par Thread safety
/// An Oink is single-threaded by construction: it owns the QP solver plus a large amount of
/// pre-allocated scratch that every solveIk() call writes. To solve on several threads, give each
/// thread its own Oink (and its own tasks, constraints, and barriers). They may share one Scene:
/// each Oink owns a SceneContext, and all task / barrier evaluation runs against that, so no two
/// solvers touch the same kinematics or collision scratch.
struct Oink {
  /// @brief Constructs an Oink solver for a named joint group.
  ///
  /// Resolves the group to its velocity indices and sizes all internal matrices
  /// to the group's velocity DOF count, which can be much smaller than model.nv
  /// when planning for a subset of joints.
  ///
  /// @param scene The scene used to resolve the group at construction time.
  /// @param group_name Joint group name. Pass an empty string for the full robot.
  /// @throws std::runtime_error if group_name is not found in the scene.
  Oink(const Scene& scene, const std::string& group_name);

  /// @brief Constructs an Oink solver for a named joint group with custom solver settings.
  ///
  /// @param scene The scene used to resolve the group at construction time.
  /// @param group_name Joint group name. Pass an empty string for the full robot.
  /// @param custom_settings Custom QP solver settings.
  /// @throws std::runtime_error if group_name is not found in the scene.
  Oink(const Scene& scene, const std::string& group_name, const OinkSettings& custom_settings);

  /// @brief Constructs an Oink solver for the full robot (all joints).
  ///
  /// Equivalent to Oink(scene, "").
  explicit Oink(const Scene& scene) : Oink(scene, "") {}

  /// @brief Constructs an Oink solver for the full robot with custom solver settings.
  ///
  /// Equivalent to Oink(scene, "", custom_settings).
  Oink(const Scene& scene, const OinkSettings& custom_settings)
      : Oink(scene, "", custom_settings) {}

  ~Oink();

  /// @brief Solve inverse kinematics for tasks only
  ///
  /// Solves a QP optimization problem to compute the joint velocity that minimizes
  /// weighted task errors.
  ///
  /// @param scene The scene; the solve runs at its current joint positions
  /// @param tasks Vector of weighted tasks to optimize for
  /// @param delta_q Pre-allocated output buffer for configuration displacement
  /// @param regularization Tikhonov regularization weight (default: 1e-12)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  solveIk(const Scene& scene, const std::vector<std::shared_ptr<Task>>& tasks,
          Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
          double regularization = 1e-12);

  /// @brief Solve inverse kinematics for tasks with constraints.
  ///
  /// Solves a QP optimization problem to compute the joint velocity that minimizes
  /// weighted task errors while satisfying all constraints.
  ///
  /// @param scene The scene; the solve runs at its current joint positions
  /// @param tasks Vector of weighted tasks to optimize for
  /// @param constraints Vector of constraints to satisfy
  /// @param delta_q Pre-allocated output buffer for configuration displacement
  /// @param regularization Tikhonov regularization weight (default: 1e-12)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  solveIk(const Scene& scene, const std::vector<std::shared_ptr<Task>>& tasks,
          const std::vector<std::shared_ptr<Constraints>>& constraints,
          Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
          double regularization = 1e-12);

  /// @brief Solve inverse kinematics for tasks with barriers.
  ///
  /// Solves a QP optimization problem to compute the joint velocity that minimizes
  /// weighted task errors while satisfying all barrier functions.
  ///
  /// @param scene The scene; the solve runs at its current joint positions
  /// @param tasks Vector of weighted tasks to optimize for
  /// @param barriers Vector of barrier functions for safety constraints
  /// @param delta_q Pre-allocated output buffer for configuration displacement
  /// @param regularization Tikhonov regularization weight (default: 1e-12)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  solveIk(const Scene& scene, const std::vector<std::shared_ptr<Task>>& tasks,
          const std::vector<std::shared_ptr<Barrier>>& barriers,
          Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
          double regularization = 1e-12);

  /// @brief Solve inverse kinematics for tasks with constraints and barriers.
  ///
  /// Solves a QP optimization problem to compute the joint velocity that minimizes
  /// weighted task errors while satisfying all constraints and barrier functions.
  /// The result is written directly into the provided delta_q buffer.
  ///
  /// @param scene The scene; the solve runs at its current joint positions
  /// @param tasks Vector of weighted tasks to optimize for
  /// @param constraints Vector of constraints to satisfy
  /// @param barriers Vector of barrier functions for safety constraints
  /// @param delta_q Pre-allocated output buffer for configuration displacement.
  ///                Must be sized to num_variables (velocity space dimension).
  ///                Using Eigen::Ref allows zero-copy access from Python numpy arrays.
  /// @param regularization Tikhonov regularization weight added to the Hessian diagonal.
  ///                This provides numerical stability by ensuring the Hessian is
  ///                strictly positive definite. Higher values increase regularization
  ///                but may reduce task tracking accuracy. Default is 1e-12.
  /// @return void on success, error message on failure
  ///
  /// @note The delta_q parameter must be pre-allocated to the correct size before calling.
  ///       Eigen::Ref cannot be resized, so passing an empty or incorrectly sized vector
  ///       will result in a failure.
  tl::expected<void, std::string>
  solveIk(const Scene& scene, const std::vector<std::shared_ptr<Task>>& tasks,
          const std::vector<std::shared_ptr<Constraints>>& constraints,
          const std::vector<std::shared_ptr<Barrier>>& barriers,
          Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
          double regularization = 1e-12);

  /// @brief Solve inverse kinematics at an explicitly supplied configuration.
  ///
  /// This is the primary entry point; the overloads above are this one, called with the scene's
  /// current joint positions. Prefer this whenever more than one solver is running: passing `q`
  /// directly means the configuration never travels through the shared Scene, so two threads
  /// cannot overwrite each other's notion of "current".
  ///
  /// `q` is copied into this solver's own SceneContext, which every task, constraint, and barrier
  /// reads and whose Pinocchio data they write.
  ///
  /// @param q The configuration to solve at (size model.nq)
  /// @param tasks Vector of weighted tasks to optimize for
  /// @param constraints Vector of constraints to satisfy
  /// @param barriers Vector of barrier functions for safety constraints
  /// @param delta_q Pre-allocated output buffer for configuration displacement
  /// @param regularization Tikhonov regularization weight (default: 1e-12)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  solveIk(const Eigen::VectorXd& q, const std::vector<std::shared_ptr<Task>>& tasks,
          const std::vector<std::shared_ptr<Constraints>>& constraints,
          const std::vector<std::shared_ptr<Barrier>>& barriers,
          Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
          double regularization = 1e-12);

  /// @brief Validate delta_q against barriers using forward kinematics.
  ///
  /// This method provides a post-solve safety check by evaluating the actual barrier
  /// values at the candidate configuration (q + delta_q). It is a backup safety mechanism
  /// for cases where the linearized CBF constraint in the QP has significant error (e.g.,
  /// large jumps, near-boundary configurations). The QP constraint uses a first-order
  /// approximation h(q + δq) ≈ h(q) + J_h · δq, which can have error O(||δq||²) for large
  /// displacements.
  ///
  /// Enforcement is per-barrier rather than global. For each barrier that would be violated
  /// at the candidate configuration, only the joints that affect that barrier (its nonzero
  /// Jacobian columns) are zeroed, so an unrelated kinematic chain, e.g, the other arm in a
  /// dual-arm setup, is not frozen just because one frame left its bound.
  /// A step that is still violated but strictly reduces the violation is allowed/
  /// For example, a frame that started outside its bound can recover instead of deadlocking.
  ///
  /// @param scene The scene; the solve runs at its current joint positions
  /// @param barriers Vector of barrier functions to check
  /// @param delta_q Configuration displacement to validate. Modified in place: the joints of
  ///                each violated, non-recovering barrier are set to zero.
  /// @param tolerance Tolerance for barrier violation detection. A barrier is considered
  ///                  violated if h(q + delta_q) < -tolerance. Default is 0.0.
  /// @return void on success, error message if barrier evaluation fails
  ///
  /// @note Only barriers that implement evaluateAtConfiguration() are checked.
  ///       Barriers returning infinity are assumed safe.
  tl::expected<void, std::string>
  enforceBarriers(const Scene& scene, const std::vector<std::shared_ptr<Barrier>>& barriers,
                  Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
                  double tolerance = 0.0);

  /// @brief Validate delta_q against barriers at an explicitly supplied configuration.
  /// @details As with solveIk, this is the primary entry point and the overload above forwards to
  /// it with the scene's current joint positions.
  tl::expected<void, std::string>
  enforceBarriers(const Eigen::VectorXd& q, const std::vector<std::shared_ptr<Barrier>>& barriers,
                  Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
                  double tolerance = 0.0);

  /// @brief The solver's private scratch (Data + GeometryData + broadphase + configuration).
  /// @details Tasks, constraints, and barriers evaluate against this context, so a single snapshot
  /// of the scene's collision geometry is reused across the whole solve and no two solvers share
  /// kinematics scratch. It is snapshotted from the scene at construction; if the scene's collision
  /// geometry changes, call refreshContext() (the context does not auto-sync, and will report the
  /// mismatch rather than answer against geometry it was not sized for).
  const SceneContext& getContext() const { return *context_; }

  /// @brief Re-snapshots the solver's context from `scene`, picking up its current collision
  /// geometry.
  /// @details Call this after adding or removing geometry, or changing collision pairs, instead of
  /// rebuilding the solver. Tasks, constraints, and barriers attached to this solver resolve the
  /// context through the solver on each use, so they follow the new one; any that additionally
  /// size buffers from the collision geometry re-derive them on their next evaluation.
  /// @note The previous context is destroyed, so nothing may hold a reference to it across this
  /// call. Not safe to call while another thread is solving on this Oink.
  /// @param scene The scene to snapshot. Normally the same scene the solver was built from.
  void refreshContext(const Scene& scene);

  /// @brief Mutable access to the solver's context, for posing it at a configuration.
  /// @details solveIk() does this itself. Use this only to drive the pieces of a solve by hand
  /// (evaluating a single task or barrier at a chosen configuration, as the tests do).
  SceneContext& getContext() { return *context_; }

private:
  /// @brief Compute `task`'s Jacobian and error, and add its contribution to the QP Hessian
  /// and gradient (projecting through the current `nullspace_projector` for hierarchical
  /// priorities).
  /// @param context The context supplying the configuration and the scratch.
  /// @param task The task to add to the QP objective.
  /// @return void if successful, else an error message describing the failure.
  tl::expected<void, std::string> addTaskContribution(const SceneContext& context, Task* task);

  /// @brief Rebuild `nullspace_projector` from the current `jacobian_stack` via a damped
  /// pseudoinverse, so subsequent priority levels are projected into the nullspace of
  /// everything stacked so far.
  /// @param lambda_sq Damping factor for the pseudoinverse.
  void rebuildNullspaceProjector(double lambda_sq);

public:
  // QP solver (ProxQP dense backend). Reconstructed whenever the constraint row count
  // changes, since ProxQP fixes the problem dimensions at construction.
  detail::QpSolverPtr solver;
  OinkSettings settings;

  // Problem dimensions
  int num_variables;

  /// @brief Position indices of the joint group (used to scatter group q into model.nq space).
  Eigen::VectorXi q_indices;

  /// @brief Velocity indices of the joint group (used to scatter delta_q back into model.nv space).
  Eigen::VectorXi v_indices;

  // Pre-allocated accumulated QP matrices
  Eigen::MatrixXd H;
  Eigen::VectorXd c;

  // Pre-allocated constraint matrices
  Eigen::MatrixXd constraint_workspace_A;
  Eigen::VectorXd constraint_workspace_lower;
  Eigen::VectorXd constraint_workspace_upper;
  std::vector<int> constraint_sizes;
  int last_constraint_rows = -1;  // -1 indicates uninitialized

  // Pre-allocated barrier workspace sizes
  std::vector<int> barrier_sizes;
  int last_barrier_rows = 0;

  // Pre-allocated barrier regularization workspace
  Eigen::MatrixXd barrier_H_contribution;
  Eigen::VectorXd barrier_c_contribution;

  // Cumulative unweighted Jacobian stack used by the hierarchical-priority projector.
  // Rows from each priority level are appended after that level's task contributions are
  // accumulated, then a damped pseudoinverse builds the nullspace projector N used to
  // project the NEXT priority level's Jacobian into the higher levels' nullspace.
  Eigen::MatrixXd jacobian_stack;
  Eigen::MatrixXd nullspace_projector;

  // Per-task scratch: W·J·N and W·(α·e). Resized per task (dims depend on task rows); steady-state
  // calls reuse the existing allocation when sizes match across iterations and across solveIk
  // calls.
  Eigen::MatrixXd projected_weighted_jacobian;
  Eigen::VectorXd weighted_error;

  // Pre-allocated, priority-sorted view into the tasks passed to solveIk. Reusing this buffer
  // avoids heap traffic on the hot path; capacity persists across calls.
  std::vector<Task*> sorted_tasks;

  // Pinocchio Data buffer used by enforceBarriers() for FK at the candidate configuration.
  // Allocated once at construction so the per-solve barrier-feasibility check does not
  // create a fresh Data (which is sized for the entire model) on every call.
  pinocchio::Data enforce_barriers_data;

  // Shared collision context, snapshotted from the construction scene.
  std::unique_ptr<SceneContext> context_;
};
}  // namespace roboplan
