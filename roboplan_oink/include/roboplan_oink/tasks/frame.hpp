#pragma once

#include <memory>
#include <optional>
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

  /// @brief Proportional gain for error feedback (default: 1.0).
  double task_gain = 1.0;

  /// @brief Levenberg-Marquardt damping for regularization (default: 0.0).
  double lm_damping = 0.0;
};

/// @brief Task for tracking a target Cartesian pose with a specified frame.
///
/// This task computes the SE(3) error between a target pose and the current
/// frame pose, enabling full 6-DOF (position + orientation) tracking.
///
/// The task owns pre-allocated storage for its 6×nv Jacobian and 6D error vector,
/// allocated at construction time to avoid runtime allocations during IK solving.
struct FrameTask : public Task {
  /// @brief Constructs a FrameTask for tracking a target pose.
  /// @param target_pose The target Cartesian configuration to reach.
  /// @param num_vars Number of robot DOFs (velocity dimension, model.nv).
  /// @param options Optional task options (default: all options set to defaults).
  FrameTask(const CartesianConfiguration& target_pose, int num_vars,
            const FrameTaskOptions& options = {});

  /// @brief Computes the SE(3) error between target and current frame pose.
  ///
  /// The error is computed as the logarithm of the relative transform:
  ///     error = log_6(T_frame_to_world^{-1} * T_target_to_world)
  ///
  /// Results are stored in error_container.
  ///
  /// @param scene The scene containing the robot model and current state.
  /// @return Void if successful, else an error message string.
  tl::expected<void, std::string> computeError(const Scene& scene) override;

  /// @brief Computes the task Jacobian for the frame tracking task.
  ///
  /// The task Jacobian J(q) ∈ ℝ^(6 × n_v) is the derivative of the task
  /// error e(q) ∈ ℝ^6 with respect to the configuration q. The formula is:
  ///
  ///     J(q) = -Jlog_6(T_frame_to_target) * J_frame(q)
  ///
  /// Where:
  /// - T_frame_to_target: Transform from current frame to target
  /// - J_frame(q): Frame Jacobian (expressed in frame coordinates)
  /// - Jlog_6: Pinocchio's logarithmic Jacobian
  ///
  /// Results are stored in jacobian_container.
  ///
  /// @param scene The scene containing the robot model and current state.
  /// @return Void if successful, else an error message string.
  tl::expected<void, std::string> computeJacobian(const Scene& scene) override;

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
  std::optional<pinocchio::Index> frame_id;

  /// @brief Target Cartesian configuration to reach.
  CartesianConfiguration target_pose;

  /// @brief Pre-allocated logarithmic Jacobian (mutable for use in const computeJacobian)
  mutable Eigen::Matrix<double, 6, 6> Jlog = Eigen::Matrix<double, 6, 6>::Identity();
};

}  // namespace roboplan
