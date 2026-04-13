#include <roboplan_oink/constraints/position_limit.hpp>

#include <OsqpEigen/OsqpEigen.h>
#include <roboplan/core/scene_utils.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

PositionLimit::PositionLimit(const Oink& oink, double gain)
    : config_limit_gain(gain), num_variables(oink.num_variables), v_indices(oink.v_indices),
      delta_q_max(oink.num_variables), delta_q_min(oink.num_variables) {}

int PositionLimit::getNumConstraints(const Scene& /*scene*/) const { return num_variables; }

tl::expected<void, std::string> PositionLimit::computeQpConstraints(
    const Scene& scene, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
    Eigen::Ref<Eigen::VectorXd> lower_bounds, Eigen::Ref<Eigen::VectorXd> upper_bounds) const {
  const auto& q = scene.getCurrentJointPositions();

  auto maybe_q_collapsed = collapseContinuousJointPositions(scene, "", q);
  if (!maybe_q_collapsed) {
    return tl::make_unexpected("PositionLimit: " + maybe_q_collapsed.error());
  }
  const auto& q_collapsed = maybe_q_collapsed.value();

  // Get joint limits from the model (only do this once).
  if (q_min.size() == 0u) {
    const auto maybe_position_limits = scene.getPositionLimitVectors();
    if (!maybe_position_limits) {
      return tl::make_unexpected("PositionLimit: " + maybe_position_limits.error());
    }
    q_min = maybe_position_limits->first;
    q_max = maybe_position_limits->second;
  }

  // Validate pre-allocated workspace dimensions
  if (constraint_matrix.rows() != num_variables || constraint_matrix.cols() != num_variables) {
    return tl::make_unexpected("PositionLimit: constraint_matrix size mismatch. Expected (" +
                               std::to_string(num_variables) + " x " +
                               std::to_string(num_variables) + "), got (" +
                               std::to_string(constraint_matrix.rows()) + " x " +
                               std::to_string(constraint_matrix.cols()) + ")");
  }
  if (lower_bounds.size() != num_variables) {
    return tl::make_unexpected("PositionLimit: lower_bounds size mismatch. Expected " +
                               std::to_string(num_variables) + ", got " +
                               std::to_string(lower_bounds.size()));
  }
  if (upper_bounds.size() != num_variables) {
    return tl::make_unexpected("PositionLimit: upper_bounds size mismatch. Expected " +
                               std::to_string(num_variables) + ", got " +
                               std::to_string(upper_bounds.size()));
  }

  // Assuming single DOF joints (revolute/prismatic), nq == nv
  // Compute distances to limits and scale by gain, then write to bounds
  // Use v_indices to select the correct joints from the full model
  for (int i = 0; i < num_variables; ++i) {
    const int vi = v_indices(i);
    // Compute distance to upper limit
    if (std::isfinite(q_max(vi))) {
      delta_q_max(i) = q_max(vi) - q_collapsed(vi);
    } else {
      delta_q_max(i) = std::numeric_limits<double>::infinity();
    }

    // Compute distance to lower limit
    if (std::isfinite(q_min(vi))) {
      delta_q_min(i) = q_collapsed(vi) - q_min(vi);
    } else {
      delta_q_min(i) = std::numeric_limits<double>::infinity();
    }
  }

  // Scale by gain parameter in-place
  delta_q_max *= config_limit_gain;
  delta_q_min *= config_limit_gain;

  // Fill constraint matrix: identity matrix (write directly into workspace)
  constraint_matrix.setIdentity();

  // For box constraints l <= G*dq <= u where G = I
  // Clamp infinite bounds to OSQP's INFTY constant
  for (int i = 0; i < num_variables; ++i) {
    double lower = -delta_q_min(i);
    double upper = delta_q_max(i);

    // Clamp to OSQP's valid range
    if (!std::isfinite(lower) || lower < -OsqpEigen::INFTY) {
      lower = -OsqpEigen::INFTY;
    }
    if (!std::isfinite(upper) || upper > OsqpEigen::INFTY) {
      upper = OsqpEigen::INFTY;
    }

    lower_bounds(i) = lower;
    upper_bounds(i) = upper;
  }

  return {};
}

}  // namespace roboplan
