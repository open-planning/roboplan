#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

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
  /// @brief Constructor for full box constraint (all 3 axes)
  /// @param frame_name Name of the frame to constrain
  /// @param p_min Minimum position bounds [x, y, z] in world frame (use -inf for no constraint)
  /// @param p_max Maximum position bounds [x, y, z] in world frame (use +inf for no constraint)
  /// @param num_variables Number of optimization variables (model.nv)
  /// @param gain Barrier gain (gamma), controls convergence to safe set. Default 1.0
  /// @param dt Timestep matching your control loop period. Default 0.01 (100 Hz)
  /// @param safe_displacement_gain Gain for safe displacement regularization. Default 1.0
  /// @param safety_margin Conservative margin for hard constraint guarantee. Default 0.0
  /// @note The dt parameter significantly affects barrier behavior - ensure it matches
  ///       your actual control/integration timestep
  PositionBarrier(const std::string& frame_name, const Eigen::Vector3d& p_min,
                  const Eigen::Vector3d& p_max, int num_variables, double gain = 1.0,
                  double dt = 0.01, double safe_displacement_gain = 1.0,
                  double safety_margin = 0.0);

  /// @brief Constructor for selective axis constraint
  /// @param frame_name Name of the frame to constrain
  /// @param indices Vector of axis indices to constrain (0=x, 1=y, 2=z)
  /// @param p_min Minimum bounds for selected axes in world frame (size must match indices)
  /// @param p_max Maximum bounds for selected axes in world frame (size must match indices)
  /// @param num_variables Number of optimization variables (model.nv)
  /// @param gain Barrier gain (gamma), controls convergence to safe set. Default 1.0
  /// @param dt Timestep matching your control loop period. Default 0.01 (100 Hz)
  /// @param safe_displacement_gain Gain for safe displacement regularization. Default 1.0
  /// @param safety_margin Conservative margin for hard constraint guarantee. Default 0.0
  PositionBarrier(const std::string& frame_name, const std::vector<int>& indices,
                  const Eigen::VectorXd& p_min, const Eigen::VectorXd& p_max, int num_variables,
                  double gain = 1.0, double dt = 0.01, double safe_displacement_gain = 1.0,
                  double safety_margin = 0.0);

  int getNumBarriers(const Scene& scene) const override;

  tl::expected<void, std::string> computeBarrier(const Scene& scene) override;

  tl::expected<void, std::string> computeJacobian(const Scene& scene) override;

  /// @brief Get current frame position in world coordinates
  /// @param scene The scene containing robot state
  /// @return Frame position
  Eigen::Vector3d getFramePosition(const Scene& scene) const;

  const std::string frame_name;
  const std::vector<int> indices;  ///< Axes to constrain (0, 1, 2 for x, y, z)
  const Eigen::VectorXd p_min;     ///< Min bounds for each constrained axis
  const Eigen::VectorXd p_max;     ///< Max bounds for each constrained axis

private:
  mutable pinocchio::FrameIndex frame_id = 0;
  mutable bool frame_id_cached = false;

  /// Pre-allocated workspace for frame Jacobian (6 x nv)
  mutable Eigen::MatrixXd frame_jacobian;
};

}  // namespace roboplan
