#pragma once

#include <optional>

#include <Eigen/Dense>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Acceleration limit constraint for inverse kinematics.
///
/// Bounds the change in joint velocity between successive differential-IK steps so the
/// executed motion does not "jump" in velocity (i.e. unbounded acceleration). Inspired by
/// pink.limits.AccelerationLimit.
///
/// The limit combines up to three inequalities, all expressed as box bounds on the
/// configuration displacement Δq (the QP variable). With dt the control timestep, a_max the
/// per-joint acceleration limit, and Δq_prev the displacement applied on the previous step:
///
///  1. Finite-difference acceleration bound:
///         -a_max <= ((Δq/dt) - (Δq_prev/dt)) / dt <= a_max
///     i.e.  Δq_prev - a_max*dt² <= Δq <= Δq_prev + a_max*dt²
///
///  2. "Braking distance" to the joint position limits, so the velocity can always be
///     brought to zero before hitting a limit (see [Flacco2015], [DelPrete2018]):
///         -dt*sqrt(2*a_max*(q - q_min)) <= Δq <= dt*sqrt(2*a_max*(q_max - q))
///
///  3. Optionally, the same braking-distance law applied to the *task target* rather than to
///     a position limit, active only while a target displacement has been supplied via
///     setTargetDisplacement():
///         |Δq| <= dt*sqrt(2*a_max*|Δq_target|)     (applied on the side facing the target)
///
/// The tightest of these is taken per joint, yielding box bounds l <= G*Δq <= u with
/// G = identity (one constraint row per group velocity DOF).
struct AccelerationLimit : public Constraints {
  /// @brief Constructor with dimension validation.
  /// @param oink The Oink solver this constraint will be used with (provides num_variables and
  ///        v_indices for selecting the correct joints from the full model).
  /// @param dt Control timestep used to convert acceleration to displacement (seconds).
  /// @param a_max Maximum acceleration vector for each group joint (rad/s² or m/s²).
  ///        Size must equal oink.num_variables. Entries must be non-negative; use infinity to
  ///        leave a joint unconstrained.
  /// @throws std::invalid_argument if dt <= 0, a_max size mismatches, or any entry is negative.
  AccelerationLimit(const Oink& oink, double dt, const Eigen::VectorXd& a_max);

  /// @brief Record the velocity applied on the previous IK step.
  ///
  /// The acceleration bound is centered on the previous velocity, so this must be called once
  /// per control step (before solving) with the velocity that was just integrated. The constraint
  /// timestep `dt` is reused to form the previous displacement: Δq_prev = v_prev * dt.
  ///
  /// @param v_prev Latest integrated group velocity (size oink.num_variables).
  /// @throws std::invalid_argument if v_prev size mismatches.
  void setLastVelocity(const Eigen::VectorXd& v_prev);

  /// @brief Enable the braking-distance bound toward the task target for the next solve.
  ///
  /// Pass the remaining joint displacement to the target: the step that would zero the task
  /// errors outright. The natural source is a task-only solve, which is the QP objective's
  /// unconstrained minimizer:
  ///
  ///     oink.solveIk(scene, tasks, delta_q_target);          // no constraints, no barriers
  ///     accel_limit.setTargetDisplacement(delta_q_target);
  ///     oink.solveIk(scene, tasks, constraints, barriers, delta_q);
  ///
  /// Call it once per control step, before solving. While no target is set, the bound is
  /// simply absent. Use clearTargetDisplacement() or reset() to turn it back off.
  ///
  /// @param delta_q_target Remaining displacement to the task target (size oink.num_variables).
  /// @throws std::invalid_argument if delta_q_target size mismatches.
  void setTargetDisplacement(const Eigen::VectorXd& delta_q_target);

  /// @brief Drop the target displacement, disabling the target braking bound.
  void clearTargetDisplacement();

  /// @brief Reset the previous-step displacement to zero (e.g., when the robot is at rest),
  /// and clear the target displacement, if any.
  void reset();

  /// @brief Get the number of constraint rows (num_variables).
  int getNumConstraints(const SceneContext& context) const override;

  /// @brief Compute QP constraint matrices for acceleration limits.
  /// @param context The context supplying the configuration and the kinematics scratch.
  /// @param constraint_matrix Output constraint matrix G (num_variables × num_variables).
  /// @param lower_bounds Output lower bounds vector (num_variables).
  /// @param upper_bounds Output upper bounds vector (num_variables).
  /// @return void on success, error message on failure.
  tl::expected<void, std::string>
  computeQpConstraints(const SceneContext& context, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
                       Eigen::Ref<Eigen::VectorXd> lower_bounds,
                       Eigen::Ref<Eigen::VectorXd> upper_bounds) const override;

  double dt;                     /// Control timestep (seconds).
  Eigen::VectorXd a_max;         /// Maximum acceleration per group joint.
  Eigen::VectorXd delta_q_prev;  /// Displacement applied on the previous step.

  /// Remaining displacement to the task target. If not set, target braking is disabled.
  std::optional<Eigen::VectorXd> delta_q_target;

  int num_variables;                    /// Number of group velocity DOFs.
  Eigen::VectorXi v_indices;            /// Velocity indices of the joint group.
  mutable Eigen::VectorXd q_max;        /// Pre-allocated maximum joint position limits.
  mutable Eigen::VectorXd q_min;        /// Pre-allocated minimum joint position limits.
  mutable Eigen::VectorXd delta_q_max;  /// Pre-allocated workspace for distance to upper limit.
  mutable Eigen::VectorXd delta_q_min;  /// Pre-allocated workspace for distance to lower limit.
};

}  // namespace roboplan
