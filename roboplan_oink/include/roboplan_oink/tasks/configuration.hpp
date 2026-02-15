#pragma once

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief ConfigurationTask configuration.
struct ConfigurationTaskOptions {
  /// @brief Proportional gain for error feedback (default: 1.0).
  double task_gain = 1.0;

  /// @brief Levenberg-Marquardt damping for regularization (default: 0.0).
  double lm_damping = 0.0;
};

/// @brief Task for tracking a target joint configuration.
///
/// This task computes the error between a target configuration and the current
/// configuration in the tangent space, enabling joint-space
/// tracking with per-joint weights.
///
/// The task owns pre-allocated storage for its nv×nv Jacobian and nv error vector,
/// allocated at construction time to avoid runtime allocations during IK solving.
///
struct ConfigurationTask : public Task {
  /// @brief Target joint configuration to reach.
  Eigen::VectorXd target_q;

  /// @brief Per-joint weights for cost function.
  Eigen::VectorXd joint_weights;

  /// @brief Constructs a ConfigurationTask for tracking a target configuration.
  ///
  /// Pre-allocates storage for the nv×nv Jacobian and nv error vector based on
  /// joint_weights.size() (which equals model.nv).
  ///
  /// @param target_q The target joint configuration.
  /// @param joint_weights Per-joint weights. All weights must be non-negative.
  /// @param options Optional task options.
  /// @throws std::invalid_argument if target_q and joint_weights have different sizes
  ///         or if any joint weight is negative.
  ConfigurationTask(const Eigen::VectorXd& target_q, const Eigen::VectorXd& joint_weights,
                    const ConfigurationTaskOptions& options = {});

  /// @brief Computes the configuration space error.
  ///
  ///     error = pin.difference(model, q_current, q_target)
  ///
  /// @param scene The scene containing the robot model and current state.
  /// @return Void if successful, else an error message string.
  tl::expected<void, std::string> computeError(const Scene& scene) override;

  /// @brief Computes the task Jacobian for the configuration task.
  ///
  /// The task Jacobian J(q) ∈ ℝ^(nv × nv) is the negative identity matrix (-I).
  /// The negative sign ensures that the QP formulation (minimize ||J*dq + α*e||²)
  /// produces movement toward the target configuration.
  ///
  /// Results are stored in jacobian_container.
  ///
  /// @param scene The scene containing the robot model and current state.
  /// @return Void if successful, else an error message string.
  tl::expected<void, std::string> computeJacobian(const Scene& scene) override;

  /// @brief Creates a diagonal weight matrix from per-joint weights.
  ///
  /// The weight matrix W ∈ ℝ^(nv × nv) is constructed as:
  ///     W = diag(√joint_weights[i])
  ///
  /// @param joint_weights Per-joint weight vector.
  /// @return A nv×nv diagonal weight matrix.
  static Eigen::MatrixXd createWeightMatrix(const Eigen::VectorXd& joint_weights);
};

}  // namespace roboplan
