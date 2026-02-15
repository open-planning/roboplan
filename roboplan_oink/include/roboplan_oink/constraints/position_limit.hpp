#pragma once

#include <Eigen/Dense>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Position limit constraint for inverse kinematics
///
/// Implements joint position constraints by restricting velocities to prevent exceeding joint
/// limits. The constraint is formulated as: l <= G*dq <= u where G is an identity matrix, and the
/// bounds are computed based on the distance to limits.
struct PositionLimit : public Constraints {
  /// @brief Constructor with pre-allocation for optimal performance
  /// @param num_variables Number of variables (model.nv) for pre-allocating workspace
  /// @param gain Scaling factor for how aggressively to steer away from limits (0 < gain <= 1)
  explicit PositionLimit(int num_variables, double gain = 1.0);

  /// @brief Get the number of constraint rows (number_variables)
  /// @param scene The scene containing robot state and model
  /// @return Number of constraint rows
  int getNumConstraints(const Scene& scene) const override;

  /// @brief Compute QP constraint matrices for position limits
  /// @param scene The scene containing robot state and model
  /// @param constraint_matrix Output constraint matrix G (number_variables × number_variables)
  /// @param lower_bounds Output lower bounds vector (number_variables)
  /// @param upper_bounds Output upper bounds vector (number_variables)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  computeQpConstraints(const Scene& scene, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
                       Eigen::Ref<Eigen::VectorXd> lower_bounds,
                       Eigen::Ref<Eigen::VectorXd> upper_bounds) const override;

  double config_limit_gain;             /// Gain parameter for steering away from limits
  int num_variables;                    /// Number of variables (cached from model.nv)
  mutable Eigen::VectorXd delta_q_max;  /// Pre-allocated workspace for max joint deltas
  mutable Eigen::VectorXd delta_q_min;  /// Pre-allocated workspace for min joint deltas
};

}  // namespace roboplan
