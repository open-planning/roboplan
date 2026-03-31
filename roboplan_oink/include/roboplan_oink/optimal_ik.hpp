#pragma once

#include <memory>
#include <string>

#include "OsqpEigen/OsqpEigen.h"
#include <tl/expected.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

/// @brief Abstract base class for IK tasks.
///
/// Each task owns pre-allocated storage for Jacobian, error, and H_dense matrices.
/// Subclasses must:
/// 1. Call initializeStorage() in their constructor with correct dimensions
/// 2. Implement computeJacobian() to fill jacobian_container
/// 3. Implement computeError() to fill error_container
struct Task {
  Task(Eigen::MatrixXd weight_matrix, double task_gain = 1.0, double lm_damp = 0.0)
      : gain(task_gain), weight(weight_matrix), lm_damping(lm_damp) {}
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
  /// @param scene The scene containing robot model and state.
  /// @return void on success, error message on failure.
  virtual tl::expected<void, std::string> computeJacobian(const Scene& scene) = 0;

  /// @brief Compute the task error and store in error_container.
  /// @param scene The scene containing robot model and state.
  /// @return void on success, error message on failure.
  virtual tl::expected<void, std::string> computeError(const Scene& scene) = 0;

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
  /// - H = J_w^T J_w + μ I  (num_variables x num_variables Hessian matrix, sparse)
  /// - c = -J_w^T e_w       (num_variables x 1 linear term)
  ///
  /// Where J_w = W*J, e_w = -α*W*e, and μ is the Levenberg-Marquardt damping.
  /// @param scene The scene containing robot model and state.
  /// @param H Output Hessian matrix (sparse)
  /// @param c Output linear cost term
  /// @return void on success, error message on failure.
  tl::expected<void, std::string>
  computeQpObjective(const Scene& scene, Eigen::SparseMatrix<double>& H, Eigen::VectorXd& c);

  const double gain = 1.0;        // Task gain for low-pass filtering
  const Eigen::MatrixXd weight;   // Weight matrix for cost normalization
  const double lm_damping = 0.0;  // Levenberg-Marquardt damping
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
  /// @param scene The scene containing robot state and model
  /// @return Number of constraint rows
  virtual int getNumConstraints(const Scene& scene) const = 0;

  /// @brief Compute QP constraint matrices using pre-allocated workspace views
  ///
  /// The constraint_matrix, lower_bounds, and upper_bounds parameters are Eigen::Ref views
  /// into pre-allocated workspace memory. The views are already sized to match
  /// getNumConstraints() rows, so implementations should fill the entire view.
  ///
  /// @param scene The scene containing robot state and model
  /// @param constraint_matrix Output constraint matrix G (pre-sized view: num_constraints ×
  /// num_variables)
  /// @param lower_bounds Output lower bounds vector (pre-sized view: num_constraints)
  /// @param upper_bounds Output upper bounds vector (pre-sized view: num_constraints)
  /// @return void on success, error message on failure
  virtual tl::expected<void, std::string>
  computeQpConstraints(const Scene& scene, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
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
  /// @param scene The scene containing robot state and model
  /// @return Number of barrier constraint rows
  virtual int getNumBarriers(const Scene& scene) const = 0;

  /// @brief Compute the barrier function values h(q)
  /// @param scene The scene containing robot state and model
  /// @note Barrier values h(q) >= 0 indicate safety; h(q) < 0 indicates violation
  /// @return void on success, error message on failure
  virtual tl::expected<void, std::string> computeBarrier(const Scene& scene) = 0;

  /// @brief Compute the barrier Jacobian J_h = dh/dq
  /// @param scene The scene containing robot state and model
  /// @return void on success, error message on failure
  virtual tl::expected<void, std::string> computeJacobian(const Scene& scene) = 0;

  /// @brief Compute safe displacement for regularization
  ///
  /// Subclasses can override to provide a non-zero safe displacement that
  /// the robot will be encouraged to move toward when near constraint boundaries.
  ///
  /// @param scene The scene containing robot state and model
  /// @return Safe displacement vector (num_variables), default is zero
  virtual Eigen::VectorXd computeSafeDisplacement(const Scene& scene) const;

  /// @brief Compute QP inequality constraints for this barrier
  ///
  /// Computes: G_b * delta_q <= b_b
  /// Where:
  ///   G_b = -J_h / dt
  ///   b_b = gain * h(q)  (linear class-K function)
  ///
  /// @param scene The scene containing robot state and model
  /// @param G Output constraint matrix (pre-sized view: num_barriers x num_variables)
  /// @param b Output constraint upper bound vector (pre-sized view: num_barriers)
  /// @return void on success, error message on failure
  tl::expected<void, std::string> computeQpInequalities(const Scene& scene,
                                                        Eigen::Ref<Eigen::MatrixXd> G,
                                                        Eigen::Ref<Eigen::VectorXd> b);

  /// @brief Compute QP objective contribution for safe displacement regularization
  ///
  /// Computes: (safe_displacement_gain / (2·‖J_h‖²)) · ‖δq - δq_safe‖²
  ///
  /// This encourages the robot to move toward a safe configuration when near
  /// constraint boundaries. The weighting by 1/‖J_h‖² normalizes the contribution
  /// based on how sensitive the barrier is to joint motion.
  ///
  /// @param scene The scene containing robot state and model
  /// @param H Output Hessian matrix contribution (num_variables x num_variables)
  /// @param c Output gradient vector contribution (num_variables)
  /// @return void on success, error message on failure
  tl::expected<void, std::string> computeQpObjective(const Scene& scene,
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
struct Oink {
  /// @brief Constructor that initializes matrices and solver with given dimensions
  ///
  /// @param num_variables Number of optimization variables (typically number of actuatable DOFs)
  explicit Oink(int num_variables);

  /// @brief Constructor with custom settings
  ///
  /// @param num_variables Number of optimization variables (typically number of DOFs)
  /// @param custom_settings Custom OSQP solver settings
  Oink(int num_variables, const OsqpEigen::Settings& custom_settings);

  /// @brief Solve inverse kinematics for tasks only
  ///
  /// Solves a QP optimization problem to compute the joint velocity that minimizes
  /// weighted task errors.
  ///
  /// @param tasks Vector of weighted tasks to optimize for
  /// @param scene Scene containing robot model and state
  /// @param delta_q Pre-allocated output buffer for configuration displacement
  /// @param regularization Tikhonov regularization weight (default: 1e-12)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  solveIk(const std::vector<std::shared_ptr<Task>>& tasks, const Scene& scene,
          Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
          double regularization = 1e-12);

  /// @brief Solve inverse kinematics for tasks with constraints.
  ///
  /// Solves a QP optimization problem to compute the joint velocity that minimizes
  /// weighted task errors while satisfying all constraints.
  ///
  /// @param tasks Vector of weighted tasks to optimize for
  /// @param constraints Vector of constraints to satisfy
  /// @param scene Scene containing robot model and state
  /// @param delta_q Pre-allocated output buffer for configuration displacement
  /// @param regularization Tikhonov regularization weight (default: 1e-12)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  solveIk(const std::vector<std::shared_ptr<Task>>& tasks,
          const std::vector<std::shared_ptr<Constraints>>& constraints, const Scene& scene,
          Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
          double regularization = 1e-12);

  /// @brief Solve inverse kinematics for tasks with barriers.
  ///
  /// Solves a QP optimization problem to compute the joint velocity that minimizes
  /// weighted task errors while satisfying all barrier functions.
  ///
  /// @param tasks Vector of weighted tasks to optimize for
  /// @param barriers Vector of barrier functions for safety constraints
  /// @param scene Scene containing robot model and state
  /// @param delta_q Pre-allocated output buffer for configuration displacement
  /// @param regularization Tikhonov regularization weight (default: 1e-12)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  solveIk(const std::vector<std::shared_ptr<Task>>& tasks,
          const std::vector<std::shared_ptr<Barrier>>& barriers, const Scene& scene,
          Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
          double regularization = 1e-12);

  /// @brief Solve inverse kinematics for tasks with constraints and barriers
  ///
  /// Solves a QP optimization problem to compute the joint velocity that minimizes
  /// weighted task errors while satisfying all constraints and barrier functions.
  /// The result is written directly into the provided delta_q buffer.
  ///
  /// @param tasks Vector of weighted tasks to optimize for
  /// @param constraints Vector of constraints to satisfy
  /// @param barriers Vector of barrier functions for safety constraints
  /// @param scene Scene containing robot model and state
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
  solveIk(const std::vector<std::shared_ptr<Task>>& tasks,
          const std::vector<std::shared_ptr<Constraints>>& constraints,
          const std::vector<std::shared_ptr<Barrier>>& barriers, const Scene& scene,
          Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
          double regularization = 1e-12);

  /// @brief Validate delta_q against barriers using forward kinematics.
  ///
  /// This method provides a post-solve safety check by evaluating the actual barrier
  /// values at the candidate configuration (q + delta_q). If any barrier would be
  /// violated, delta_q is set to zero to prevent unsafe motion.
  ///
  /// This is a backup safety mechanism for cases where the linearized CBF constraint
  /// in the QP has significant error (e.g., large jumps, near-boundary configurations).
  /// The QP constraint uses a first-order approximation h(q + δq) ≈ h(q) + J_h · δq,
  /// which can have significant error O(||δq||²) for large displacements.
  ///
  /// @param barriers Vector of barrier functions to check
  /// @param scene Scene containing robot model and state (current configuration q)
  /// @param delta_q Configuration displacement to validate. Modified in place: set to
  ///                zero if barrier violation is detected.
  /// @param tolerance Tolerance for barrier violation detection. A barrier is considered
  ///                  violated if h(q + delta_q) < -tolerance. Default is 0.0.
  /// @return void on success, error message if barrier evaluation fails
  ///
  /// @note Only barriers that implement evaluateAtConfiguration() are checked.
  ///       Barriers returning infinity are assumed safe.
  tl::expected<void, std::string>
  enforceBarriers(const std::vector<std::shared_ptr<Barrier>>& barriers, Scene& scene,
                  Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q,
                  double tolerance = 0.0);

  // QP solver
  OsqpEigen::Solver solver;
  OsqpEigen::Settings settings;

  // Problem dimensions
  int num_variables;

  // Pre-allocated QP contribution matrices (reused for each task)
  Eigen::VectorXd task_c;
  Eigen::SparseMatrix<double> task_H;

  // Pre-allocated accumulated QP matrices
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd c;

  // Pre-allocated constraint matrices
  Eigen::MatrixXd constraint_workspace_A;
  Eigen::VectorXd constraint_workspace_lower;
  Eigen::VectorXd constraint_workspace_upper;
  Eigen::SparseMatrix<double> A_sparse;
  std::vector<int> constraint_sizes;
  int last_constraint_rows = -1;  // -1 indicates uninitialized

  // Pre-allocated barrier workspace matrices
  Eigen::MatrixXd barrier_workspace_G;
  Eigen::VectorXd barrier_workspace_h;
  std::vector<int> barrier_sizes;
  int last_barrier_rows = 0;

  // Pre-allocated barrier regularization workspace
  Eigen::MatrixXd barrier_H_contribution;
  Eigen::VectorXd barrier_c_contribution;
};
}  // namespace roboplan
