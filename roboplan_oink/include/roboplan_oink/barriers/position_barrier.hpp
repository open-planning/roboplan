#pragma once

#include <Eigen/Dense>
#include <string>

#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Axis selection for position barrier constraints.
struct ConstraintAxisSelection {
  /// @brief Constrain X axis.
  bool x = true;
  /// @brief Constrain Y axis.
  bool y = true;
  /// @brief Constrain Z axis.
  bool z = true;
};

/// @brief Position barrier constraint for end-effector box constraint
///
/// Constrains a frame's position to remain within an axis-aligned bounding box:
///     p_min <= p(q) <= p_max
///
/// This creates up to 6 barrier constraints (2 per enabled axis).
///
/// The barrier functions are:
///     h_lower_i = p_i(q) - p_min_i  (for min bounds)
///     h_upper_i = p_max_i - p_i(q)  (for max bounds)
///
/// Uses a saturating class-K function α(h) = γ·h/(1+|h|) for smooth behavior.
///
/// Safe displacement regularization encourages moving toward the center of the safe region.
struct PositionBarrier : public Barrier {
  /// @brief Constructs a position barrier for box constraint.
  /// @param frame_name Name of the frame to constrain.
  /// @param p_min Minimum position bounds [x, y, z] in world frame (use -inf for no constraint).
  /// @param p_max Maximum position bounds [x, y, z] in world frame (use +inf for no constraint).
  /// @param num_variables Number of optimization variables (model.nv).
  /// @param dt Timestep matching your control loop period (required; must match actual control
  /// loop).
  /// @param axis_selection Which axes to constrain (default: all three axes).
  /// @param gain Barrier gain (gamma), controls convergence to safe set. Default 1.0
  /// @param safe_displacement_gain Gain for safe displacement regularization. Default 1.0
  /// @param safety_margin Conservative margin for hard constraint guarantee. Default 0.0
  /// @note The dt parameter significantly affects barrier behavior - ensure it matches
  ///       your actual control/integration timestep.
  PositionBarrier(const std::string& frame_name, const Eigen::Vector3d& p_min,
                  const Eigen::Vector3d& p_max, int num_variables, double dt,
                  const ConstraintAxisSelection& axis_selection = ConstraintAxisSelection(),
                  double gain = 1.0, double safe_displacement_gain = 1.0,
                  double safety_margin = 0.0);

  /// @brief Get the number of active barrier constraints.
  /// @param scene The scene containing robot model and state.
  /// @return Number of active barriers (up to 6: 2 per enabled axis).
  int getNumBarriers(const Scene& scene) const override;

  /// @brief Compute barrier function values h(q) for all active constraints.
  ///
  /// Evaluates the barrier functions:
  ///   - h_lower_i = p_i(q) - p_min_i  (for min bounds on enabled axes)
  ///   - h_upper_i = p_max_i - p_i(q)  (for max bounds on enabled axes)
  ///
  /// Also computes right-hand side bounds using the saturating class-K function:
  ///   rhs_i = dt * gamma * h_i / (1 + |h_i|) - safety_margin
  ///
  /// Results are stored in the inherited `barrier_values` and `barrier_rhs` vectors.
  ///
  /// @param scene The scene containing robot model and current state.
  /// @return Expected void on success, or error message if frame is not found.
  tl::expected<void, std::string> computeBarrier(const Scene& scene) override;

  /// @brief Compute barrier constraint Jacobian matrix.
  ///
  /// Computes the Jacobian -J_h used in the QP constraint: -J_h * delta_q <= rhs
  /// Each barrier uses one row of the frame's position Jacobian (first 3 rows only).
  /// The sign is negated for upper bounds to convert p_max - p(q) >= 0 into the standard form.
  ///
  /// Results are stored in the inherited `barrier_jacobian` matrix (num_barriers x nv).
  ///
  /// @param scene The scene containing robot model and current state.
  /// @return Expected void on success, or error message if frame is not found.
  tl::expected<void, std::string> computeJacobian(const Scene& scene) override;

  /// @brief Evaluate minimum barrier value at a candidate configuration.
  ///
  /// Computes forward kinematics for the candidate configuration and returns
  /// the minimum barrier value across all position constraints (x, y, z min/max).
  ///
  /// @param model Pinocchio model
  /// @param data Pinocchio data (will be modified by FK computation)
  /// @param q Candidate joint configuration to evaluate
  /// @return Expected containing minimum barrier value (negative if any constraint is violated),
  ///         or error message if frame is not found
  tl::expected<double, std::string>
  evaluateAtConfiguration(const pinocchio::Model& model, pinocchio::Data& data,
                          const Eigen::VectorXd& q) const override;

  /// @brief Get current frame position in world coordinates.
  /// @param scene The scene containing robot state.
  /// @return Frame position in world coordinates.
  Eigen::Vector3d getFramePosition(const Scene& scene) const;

  /// @brief Name of the frame to constrain.
  const std::string frame_name;

  /// @brief Axis selection for constraints (x, y, z).
  const ConstraintAxisSelection axis_selection;

  /// @brief Minimum position bounds in world frame for each axis.
  const Eigen::Vector3d p_min;

  /// @brief Maximum position bounds in world frame for each axis.
  const Eigen::Vector3d p_max;

private:
  /// @brief Cached frame index for fast lookups.
  mutable pinocchio::FrameIndex frame_id = 0;

  /// @brief Whether frame_id has been cached.
  mutable bool frame_id_cached = false;

  /// @brief Pre-allocated workspace for frame Jacobian (6 x nv).
  mutable Eigen::MatrixXd frame_jacobian;
};

}  // namespace roboplan
