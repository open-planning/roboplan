#include <roboplan_oink/constraints/position_limit.hpp>

#include <OsqpEigen/OsqpEigen.h>
#include <roboplan/core/scene_utils.hpp>

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

  auto maybe_q_collapsed = collapseContinuousJointPositions(scene, "", q);
  if (!maybe_q_collapsed) {
    return tl::make_unexpected("Failed to compute position constraint: " +
                               maybe_q_collapsed.error());
  }
  const auto& q_collapsed = maybe_q_collapsed.value();

  // Get joint limits from the model (only do this once)
  if (q_min.size() == 0u) {
    q_min.resize(num_variables);
    q_max.resize(num_variables);
    const auto joint_names = scene.getJointNames();
    for (int idx = 0; idx < num_variables; ++idx) {
      const auto& joint_name = joint_names.at(idx);
      const auto maybe_joint_info = scene.getJointInfo(joint_name);
      if (!maybe_joint_info) {
        return tl::make_unexpected("Failed to get joint limits for position constraint: " +
                                   maybe_joint_info.error());
      }
      const auto& joint_info = maybe_joint_info.value();

      switch (joint_info.type) {
      case JointType::FLOATING:
      case JointType::PLANAR:
        return tl::make_unexpected("Multi-DOF joints not yet supported by position constraints.");
      case JointType::CONTINUOUS:
        q_min(idx) = -std::numeric_limits<double>::infinity();
        q_max(idx) = std::numeric_limits<double>::infinity();
        break;
      default:
        if (joint_info.limits.min_position.size() == 0) {
          q_min(idx) = -std::numeric_limits<double>::infinity();
        } else {
          q_min(idx) = joint_info.limits.min_position(0);
        }
        if (joint_info.limits.max_position.size() == 0) {
          q_max(idx) = std::numeric_limits<double>::infinity();
        } else {
          q_max(idx) = joint_info.limits.max_position(0);
        }
      }
    }
  }

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

  // Assuming single DOF joints (revolute/prismatic), nq == nv
  // Compute distances to limits and scale by gain, then write to bounds
  for (int i = 0; i < num_variables; ++i) {
    // Compute distance to upper limit
    if (std::isfinite(q_max(i))) {
      delta_q_max(i) = q_max(i) - q_collapsed(i);
    } else {
      delta_q_max(i) = std::numeric_limits<double>::infinity();
    }

    // Compute distance to lower limit
    if (std::isfinite(q_min(i))) {
      delta_q_min(i) = q_collapsed(i) - q_min(i);
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
