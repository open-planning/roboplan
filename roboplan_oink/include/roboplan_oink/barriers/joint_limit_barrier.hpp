#pragma once

#include <Eigen/Dense>
#include <string>

#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Joint position limit barrier (CBF) for a joint group.
///
/// Creates two barrier constraints (lower/upper) per 1-DOF joint with finite position limits
/// in the group:
///     h_lower_i = q_i - (q_min_i + m_i)
///     h_upper_i = (q_max_i - m_i) - q_i
/// where m_i is a per-joint safety margin expressed as a fraction of the joint's position
/// span. Multi-DOF joints (e.g. continuous joints) and joints with non-finite limits are
/// skipped.
///
/// Unlike the hard PositionLimit constraint, which permits consuming the entire remaining
/// distance to a bound in a single step (so joints can legally park exactly on their
/// limits), the CBF condition shapes the approach so joints decelerate smoothly toward a
/// standoff of m_i from each bound. The safe-displacement regularization additionally
/// pushes joints back toward mid-range once they are inside the margin zone. Intended to
/// compose with PositionLimit, which remains the hard backstop at the true bounds.
struct JointLimitBarrier : public Barrier {
  /// @brief Constructor.
  /// @param oink The Oink solver this barrier will be used with (provides num_variables).
  /// @param scene The scene used to resolve the joint group and its limits.
  /// @param group_name The joint group whose position limits to guard.
  /// @param dt Timestep matching the control loop period.
  /// @param margin_fraction Safety margin per joint as a fraction of its position span
  ///        (e.g. 0.05 keeps each joint 5% of its range away from its bounds). Clamped per
  ///        joint so the margins can never invert the bounds.
  /// @param gain Barrier gain (gamma), controls convergence to the safe set.
  /// @param safe_displacement_gain Gain for the safe displacement regularization.
  /// @throws std::runtime_error if the joint group is not found in the scene.
  JointLimitBarrier(const Oink& oink, const Scene& scene, const std::string& group_name,
                    double dt, double margin_fraction = 0.05, double gain = 1.0,
                    double safe_displacement_gain = 1.0);

  /// @brief Get the number of barrier constraints (2 per participating DOF).
  int getNumBarriers(const Scene& scene) const override;

  /// @brief Compute the barrier values from the scene's current joint positions.
  tl::expected<void, std::string> computeBarrier(const Scene& scene) override;

  /// @brief No-op; the barrier Jacobian rows are constant selector rows set at construction.
  tl::expected<void, std::string> computeJacobian(const Scene& scene) override;

  /// @brief Push joints inside their margin zone back toward mid-range.
  /// @details Zero for joints outside the margin zone, so the regularization does not bias
  /// the solution in the interior.
  Eigen::VectorXd computeSafeDisplacement(const Scene& scene) const override;

  /// @brief Full-configuration position index of each participating DOF.
  Eigen::VectorXi q_index;

  /// @brief Group velocity slot (column in the barrier Jacobian) of each participating DOF.
  Eigen::VectorXi v_slot;

  /// @brief Margin-adjusted lower and upper bounds per participating DOF.
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;

  /// @brief Per-joint safety margin, in the joint's position units.
  Eigen::VectorXd margin;

  /// @brief Number of optimization variables (group velocity DOFs).
  int num_variables = 0;
};

}  // namespace roboplan
