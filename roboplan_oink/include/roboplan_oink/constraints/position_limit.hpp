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
  /// @brief Constructor; pre-allocates the workspace.
  /// @param oink The Oink solver this constraint will be used with (provides num_variables and
  ///        v_indices for selecting the correct joint limits from the full model).
  /// @param gain Scaling factor for how aggressively to steer away from limits (0 < gain <= 1)
  /// @param joint_lower_limits Lower limits for each group joint, in group joint order. If not
  ///                           specified, the limits from the scene model joint limits are used.
  ///                           Size must equal oink.num_variables if provided.
  /// @param joint_upper_limits Upper limits for each group joint, in group joint order. If not
  ///                           specified, the limits from the scene model joint limits are used.
  ///                           Size must equal oink.num_variables if provided.
  /// @param max_pullback The maximum amount a joint position that's outside the bounds can be
  ///                     pulled back by the QP problem each iteration. 0 means that a joint
  ///                     that starts outside the range is not pulled back inside. The effective
  ///                     pullback is `gain * max_pullback`; too high a value coupled with a
  ///                     VelocityLimit can result in an infeasible QP problem if
  ///                     `gain * max_pullback > v_max * dt`.
  /// @throw std::invalid_argument if the size of joint_lower_limits or joint_upper_limits is not
  ///        equal to oink.num_variables or 0.
  /// @throw std::invalid_argument if `joint_lower_limits[i] > joint_upper_limits[i]` for some i.
  /// @throw std::invalid_argument if max_pullback < 0.0
  explicit PositionLimit(const Oink& oink, double gain = 1.0,
                         const Eigen::VectorXd& joint_lower_limits = Eigen::VectorXd(),
                         const Eigen::VectorXd& joint_upper_limits = Eigen::VectorXd(),
                         double max_pullback = 0.0);

  /// @brief Get the number of constraint rows (num_variables)
  /// @param context The context (unused; the row count is fixed at construction).
  /// @return Number of constraint rows
  int getNumConstraints(const SceneContext& context) const override;

  /// @brief Compute QP constraint matrices for position limits
  /// @param context The context supplying the configuration and the kinematics scratch.
  /// @param constraint_matrix Output constraint matrix G (num_variables × num_variables)
  /// @param lower_bounds Output lower bounds vector (num_variables)
  /// @param upper_bounds Output upper bounds vector (num_variables)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  computeQpConstraints(const SceneContext& context, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
                       Eigen::Ref<Eigen::VectorXd> lower_bounds,
                       Eigen::Ref<Eigen::VectorXd> upper_bounds) const override;

  /// @brief Returns the per-row excess of the position relative to the active joint limits.
  Eigen::VectorXd computeViolation(const SceneContext& context) const override;

  double config_limit_gain;             ///< Gain parameter for steering away from limits
  int num_variables;                    ///< Number of group velocity DOFs
  double max_pullback;                  ///< The maximum pullback for joints outside limits.
  Eigen::VectorXi v_indices;            ///< Velocity indices of the joint group
  mutable Eigen::VectorXd q_max;        ///< Pre-allocated maximum joint position limits in
                                        ///< group joint order.
  mutable Eigen::VectorXd q_min;        ///< Pre-allocated minimum joint position limits in
                                        ///< group joint order.
  mutable Eigen::VectorXd delta_q_max;  ///< Pre-allocated workspace for max joint deltas
  mutable Eigen::VectorXd delta_q_min;  ///< Pre-allocated workspace for min joint deltas

private:
  tl::expected<void, std::string> initializeQMinAndQMax(const SceneContext& context) const;
};

}  // namespace roboplan
