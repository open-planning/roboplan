#include <roboplan_oink/constraints/position_limit.hpp>

#include <OsqpEigen/OsqpEigen.h>

namespace roboplan {

PositionLimit::PositionLimit(int num_vars, double gain)
    : config_limit_gain(gain), num_variables(num_vars), delta_q_max(num_vars),
      delta_q_min(num_vars) {}

int PositionLimit::getNumConstraints(const Scene& /*scene*/) const { return num_variables; }

tl::expected<void, std::string> PositionLimit::computeQpConstraints(
    const Scene& scene, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
    Eigen::Ref<Eigen::VectorXd> lower_bounds, Eigen::Ref<Eigen::VectorXd> upper_bounds) const {
  const auto& model = scene.getModel();
  const auto& q = scene.getCurrentJointPositions();

  // Validate that model dimensions match constructor
  if (model.nv != num_variables) {
    return tl::make_unexpected("PositionLimit: model.nv (" + std::to_string(model.nv) +
                               ") does not match num_variables (" + std::to_string(num_variables) +
                               ") from constructor");
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

  // Get joint limits from the model
  const Eigen::VectorXd& q_min = model.lowerPositionLimit;
  const Eigen::VectorXd& q_max = model.upperPositionLimit;

  // Assuming single DOF joints (revolute/prismatic), nq == nv
  // Compute distances to limits and scale by gain, then write to bounds
  for (int i = 0; i < num_variables; ++i) {
    // Compute distance to upper limit
    if (std::isfinite(q_max(i))) {
      delta_q_max(i) = q_max(i) - q(i);
    } else {
      delta_q_max(i) = std::numeric_limits<double>::infinity();
    }

    // Compute distance to lower limit
    if (std::isfinite(q_min(i))) {
      delta_q_min(i) = q(i) - q_min(i);
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
