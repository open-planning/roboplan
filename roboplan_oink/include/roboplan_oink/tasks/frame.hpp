#pragma once

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief SE(3) spatial dimension (3 position + 3 orientation).
constexpr int kSpatialDimension = 6;

/// @brief Optional parameters for FrameTask configuration.
struct FrameTaskOptions {
  /// @brief Cost weight for position error (default: 1.0).
  double position_cost = 1.0;

  /// @brief Cost weight for orientation error (default: 1.0).
  double orientation_cost = 1.0;

  /// @brief Task gain for low-pass filtering (default: 1.0).
  double task_gain = 1.0;

  /// @brief Levenberg-Marquardt damping for regularization (default: 0.0).
  double lm_damping = 0.0;

  /// @brief Softly caps the position error fed to each step at this norm (meters; default:
  /// unlimited). A large step can swing a frame through a barrier that its linearization said was
  /// safe; a VelocityLimit bounds the step directly, so this only matters without one.
  /// Recommended then: 0.1-0.2 m.
  double position_saturation = std::numeric_limits<double>::infinity();

  /// @brief Softly caps the orientation error fed to each step at this norm (radians; default:
  /// unlimited). See position_saturation. Recommended without a VelocityLimit: 0.5-1.0 rad.
  double orientation_saturation = std::numeric_limits<double>::infinity();

  /// @brief Position error norm (meters) within which Oink::solveIterativeIk() treats the task
  /// as reached. Ignored when position_cost is zero.
  double position_tolerance = 0.001;

  /// @brief Orientation error norm (radians) within which Oink::solveIterativeIk() treats the
  /// task as reached. Ignored when orientation_cost is zero.
  double orientation_tolerance = 0.001;

  /// @brief Priority level (default: 1). Tasks at higher priority numbers are projected
  /// into the nullspace of all lower priority numbers. Must be >= 1.
  int priority = 1;
};

/// @brief Task for tracking a target Cartesian pose with a specified frame.
///
/// Computes the SE(3) error between a target pose and the current frame pose, for full 6-DOF
/// (position + orientation) tracking.
///
/// The task owns pre-allocated storage for its 6×nv Jacobian and 6D error vector,
/// allocated at construction time to avoid runtime allocations during IK solving.
struct FrameTask : public Task {
  /// @brief Constructs a FrameTask for tracking a target pose.
  /// @param oink The Oink solver this task will be used with (provides the velocity indices for
  ///        Jacobian column selection).
  /// @param scene The scene used to resolve the frame ID and allocate storage.
  /// @param target_pose The target Cartesian configuration to reach.
  /// @param options Optional task options (default: all options set to defaults).
  /// @throws std::runtime_error if the frame or its base frame is not found in the scene.
  FrameTask(const Oink& oink, const Scene& scene, const CartesianConfiguration& target_pose,
            const FrameTaskOptions& options = {});

  /// @brief Computes the 6D pose error between the target and the current frame pose.
  ///
  /// The error is expressed in world-aligned coordinates, split into a position and a
  /// rotation part:
  ///     e_pos = p_target - p_frame
  ///     e_rot = R_frame * log_3(R_frame^T * R_target)
  ///
  /// Each part is then softly saturated to `position_saturation` / `orientation_saturation`
  /// (when finite) using e_max * tanh(||e|| / e_max) * e / ||e||, which bounds the step
  /// requested from the QP and keeps the CBF linearization valid.
  ///
  /// Results are stored in error_container.
  ///
  /// @param context The context supplying the configuration and the frame placements to read.
  /// @return Void if successful, else an error message string.
  tl::expected<void, std::string> computeError(const SceneContext& context) override;

  double normalizedError() const override;

  /// @brief Computes the task Jacobian for the frame tracking task.
  ///
  /// The task Jacobian J(q) ∈ ℝ^(6 × n_v) is the negated frame Jacobian of the tracked
  /// frame, expressed in LOCAL_WORLD_ALIGNED coordinates so that it matches the error
  /// convention of computeError():
  ///
  ///     J(q) = -J_frame(q)
  ///
  /// When a base frame is set, the relative Jacobian of the frame with respect to that base
  /// is used instead, so that the base frame's own motion through the joints is accounted for.
  /// The negation ensures the QP formulation (min ||J Δq + α e||²) moves toward the target.
  ///
  /// Results are stored in jacobian_container.
  ///
  /// @param context The context supplying the configuration and the kinematics scratch to write.
  /// @return Void if successful, else an error message string.
  tl::expected<void, std::string> computeJacobian(const SceneContext& context) override;

  /// @brief Creates a diagonal weight matrix from scalar cost weights.
  ///
  /// The weight matrix W ∈ ℝ^(6 × 6) is constructed as:
  ///     W = diag(√position_cost * I_3, √orientation_cost * I_3)
  ///
  /// @param position_cost Cost weight for position error (first 3 dimensions).
  /// @param orientation_cost Cost weight for orientation error (last 3 dimensions).
  /// @return A 6×6 diagonal weight matrix.
  static Eigen::MatrixXd createWeightMatrix(double position_cost, double orientation_cost);

  /// @brief Sets the target transform for this frame task.
  /// @param tform The target transform.
  void setTargetFrameTransform(const Eigen::Matrix4d& tform) { target_pose.tform = tform; }

  /// @brief Name of the frame to track (e.g., end-effector link name).
  std::string frame_name;

  /// @brief Index of the frame in the scene's Pinocchio model.
  pinocchio::Index frame_id;

  /// @brief Optional index of the base frame (from CartesianConfiguration::base_frame).
  /// When set, computeError converts the target to world frame using the base frame's current
  /// FK pose, and computeJacobian returns the relative Jacobian of the EE w.r.t. the base.
  std::optional<pinocchio::FrameIndex> base_frame_id;

  /// @brief Velocity vector indices for the joint group (used to select Jacobian columns).
  Eigen::VectorXi v_indices;

  /// @brief Target Cartesian configuration to reach.
  CartesianConfiguration target_pose;

  /// @brief Per-step error caps (meters, radians). Infinite means no cap.
  double position_saturation;
  double orientation_saturation;

  /// @brief Convergence tolerances for Oink::solveIterativeIk().
  double position_tolerance;
  double orientation_tolerance;

  // Pre-allocated full Jacobian (6 x model.nv) for column selection (mutable for const methods)
  mutable Eigen::MatrixXd full_jacobian;

  // Pre-allocated logarithmic Jacobian (mutable for use in const computeJacobian)
  mutable Eigen::Matrix<double, 6, 6> Jlog = Eigen::Matrix<double, 6, 6>::Identity();
};

}  // namespace roboplan
