#pragma once

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Task dimension for LookAtTask (3 alignment + 1 distance).
constexpr int kLookAtDimension = 4;

/// @brief Optional parameters for LookAtTask configuration.
struct LookAtTaskOptions {
  /// @brief Cost weight for the look-at alignment error (default: 1.0).
  double orientation_cost = 1.0;

  /// @brief Cost weight for the standoff distance error (default: 1.0).
  double distance_cost = 1.0;

  /// @brief Proportional gain for error feedback (default: 1.0).
  double task_gain = 1.0;

  /// @brief Levenberg-Marquardt damping for regularization (default: 0.0).
  double lm_damping = 0.0;

  /// @brief Maximum distance error magnitude in meters (default: unlimited).
  /// Limits the distance error to prevent large jumps when the target point moves
  /// far away in a single step. Recommended: 0.1-0.2m for systems with barriers.
  double max_distance_error = std::numeric_limits<double>::infinity();

  /// @brief Look axis expressed in the controlled frame's local coordinates
  /// (default: +Z). Normalized at task construction; must be nonzero.
  Eigen::Vector3d look_axis = Eigen::Vector3d::UnitZ();

  /// @brief Priority level (default: 1). Tasks at higher priority numbers are projected
  /// into the nullspace of all lower priority numbers. Must be >= 1.
  int priority = 1;
};

/// @brief Task for pointing a frame's look axis at a target point while holding a
/// standoff distance.
///
/// This task drives a chosen axis of the controlled frame (the "look axis", e.g. the
/// optical axis of a wrist camera or the tool Z axis) to face a target point in the
/// world frame, while simultaneously regulating the distance between the frame origin
/// and the target point to a desired standoff value.
///
/// The 4D task error stacks:
/// - Alignment (3 rows): e_align = a × u, where a is the look axis in world
///   coordinates and u is the unit vector from the frame origin to the target point.
///   This vanishes when the frame faces the target, and leaves rotation about the
///   look axis unconstrained (rank 2).
/// - Distance (1 row): e_dist = ||p_target - p_frame|| - target_distance.
///
/// Both the position of the frame on the standoff sphere and the roll about the look
/// axis remain free, so redundant degrees of freedom can be used by lower-priority
/// tasks (e.g. a ConfigurationTask).
///
/// @note The alignment error has an unstable equilibrium when the look axis points
/// exactly away from the target (a = -u); in practice other tasks or regularization
/// perturb the solver off this configuration.
///
/// The task owns pre-allocated storage for its 4×nv Jacobian and 4D error vector,
/// allocated at construction time to avoid runtime allocations during IK solving.
struct LookAtTask : public Task {
  /// @brief Constructs a LookAtTask for facing a target point.
  ///
  /// The Oink solver provides the velocity indices (for Jacobian column selection).
  /// The scene is used at construction time to resolve the frame ID and allocate
  /// the full Jacobian buffer.
  ///
  /// @param oink The Oink solver instance this task will be used with.
  /// @param scene The scene used to resolve the frame ID and allocate storage.
  /// @param frame_name The name of the frame to point (e.g., end-effector link name).
  /// @param target_point The point to look at, in world coordinates.
  /// @param target_distance The desired standoff distance from the target point, in meters.
  /// @param options Optional task options (default: all options set to defaults).
  /// @throws std::runtime_error if the frame name is not found in the scene.
  /// @throws std::invalid_argument if target_distance is negative or the look axis is zero.
  LookAtTask(const Oink& oink, const Scene& scene, const std::string& frame_name,
             const Eigen::Vector3d& target_point, double target_distance,
             const LookAtTaskOptions& options = {});

  /// @brief Computes the alignment and distance errors.
  ///
  /// With a = R_frame * look_axis, d = p_target - p_frame, r = ||d||, u = d / r:
  ///     error = [a × u; r - target_distance]
  ///
  /// If the frame origin coincides with the target point (r ≈ 0), the look
  /// direction is undefined and the whole error is set to zero, deactivating the
  /// task until the two separate.
  ///
  /// Results are stored in error_container.
  ///
  /// @param scene The scene containing the robot model and current state.
  /// @return Void if successful, else an error message string.
  tl::expected<void, std::string> computeError(const Scene& scene) override;

  /// @brief Computes the task Jacobian for the look-at task.
  ///
  /// The Jacobian J(q) ∈ ℝ^(4 × n_v) is the derivative of the task error with
  /// respect to the configuration. With the frame Jacobian [J_v; J_ω] expressed in
  /// LOCAL_WORLD_ALIGNED coordinates and S(·) the skew-symmetric operator:
  ///
  ///     J_align = S(u)·S(a)·J_ω − (S(a)·(I − u·uᵀ) / r)·J_v
  ///     J_dist  = −uᵀ·J_v
  ///
  /// Results are stored in jacobian_container.
  ///
  /// @param scene The scene containing the robot model and current state.
  /// @return Void if successful, else an error message string.
  tl::expected<void, std::string> computeJacobian(const Scene& scene) override;

  /// @brief Creates a diagonal weight matrix from scalar cost weights.
  ///
  /// The weight matrix W ∈ ℝ^(4 × 4) is constructed as:
  ///     W = diag(√orientation_cost * I_3, √distance_cost)
  ///
  /// @param orientation_cost Cost weight for the alignment error (first 3 dimensions).
  /// @param distance_cost Cost weight for the distance error (last dimension).
  /// @return A 4×4 diagonal weight matrix.
  static Eigen::MatrixXd createWeightMatrix(double orientation_cost, double distance_cost);

  /// @brief Sets the target point to look at, for runtime retargeting.
  /// @param point The target point in world coordinates.
  void setTargetPoint(const Eigen::Vector3d& point) { target_point = point; }

  /// @brief Sets the desired standoff distance, for runtime retargeting.
  /// @param distance The desired distance in meters (must be non-negative).
  /// @throws std::invalid_argument if distance is negative.
  void setTargetDistance(double distance);

  /// @brief Name of the frame to point (e.g., end-effector link name).
  std::string frame_name;

  /// @brief Index of the frame in the scene's Pinocchio model.
  pinocchio::Index frame_id;

  /// @brief Velocity vector indices for the joint group (used to select Jacobian columns).
  Eigen::VectorXi v_indices;

  /// @brief The point to look at, in world coordinates.
  Eigen::Vector3d target_point;

  /// @brief Desired standoff distance from the target point, in meters.
  double target_distance;

  /// @brief Unit look axis in the controlled frame's local coordinates.
  Eigen::Vector3d look_axis;

  /// @brief Maximum distance error magnitude (meters). Infinite means no limit.
  double max_distance_error;

  // Pre-allocated full Jacobian (6 x model.nv) for column selection (mutable for const methods)
  mutable Eigen::MatrixXd full_jacobian;

  // Pre-allocated task Jacobian in full velocity space (4 x model.nv), before column selection
  mutable Eigen::MatrixXd task_jacobian_full;
};

}  // namespace roboplan
