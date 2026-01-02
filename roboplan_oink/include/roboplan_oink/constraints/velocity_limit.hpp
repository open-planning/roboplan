#pragma once

#include <Eigen/Dense>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Velocity limit constraint for inverse kinematics
///
/// Implements joint velocity constraints to ensure velocities stay within robot limits.
/// The constraint is formulated as: l <= G*dq <= u
/// where G is an identity matrix, l = -dt*v_max, and u = dt*v_max.
struct VelocityLimit : public Constraints {
  /// @brief Constructor with dimension validation
  /// @param num_variables Number of variables (model.nv) - must match v_max size
  /// @param dt Time step for the velocity integration (seconds)
  /// @param v_max Maximum velocity vector for each joint (rad/s or m/s)
  VelocityLimit(int num_variables, double dt, const Eigen::VectorXd& v_max);

  /// @brief Get the number of constraint rows (number_variables)
  /// @param scene The scene containing robot state and model
  /// @return Number of constraint rows
  int getNumConstraints(const Scene& scene) const override;

  /// @brief Compute QP constraint matrices for velocity limits
  /// @param scene The scene containing robot state and model
  /// @param constraint_matrix Output constraint matrix G (number_variables × number_variables)
  /// @param lower_bounds Output lower bounds vector (number_variables)
  /// @param upper_bounds Output upper bounds vector (number_variables)
  /// @return void on success, error message on failure
  tl::expected<void, std::string>
  computeQpConstraints(const Scene& scene, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
                       Eigen::Ref<Eigen::VectorXd> lower_bounds,
                       Eigen::Ref<Eigen::VectorXd> upper_bounds) const override;

  double dt;
  Eigen::VectorXd v_max;
  int num_variables;  /// Number of variables (cached from model.nv)
};

}  // namespace roboplan
