#include <algorithm>
#include <roboplan_oink/constraints/position_limit.hpp>

#include <roboplan/core/scene_utils.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

PositionLimit::PositionLimit(const Oink& oink, double gain,
                             const Eigen::VectorXd& joint_lower_limits,
                             const Eigen::VectorXd& joint_upper_limits, double max_pullback)
    : config_limit_gain(gain), num_variables(oink.num_variables), max_pullback(max_pullback),
      v_indices(oink.v_indices), delta_q_max(oink.num_variables), delta_q_min(oink.num_variables) {
  if (max_pullback < 0.0) {
    throw std::invalid_argument("PositionLimit: max_pullback=" + std::to_string(max_pullback) +
                                " < 0");
  }

  if (joint_lower_limits.size() != 0 && joint_lower_limits.size() != num_variables) {
    throw std::invalid_argument(
        "PositionLimit: joint_lower_limits size (" + std::to_string(joint_lower_limits.size()) +
        ") does not match oink.num_variables (" + std::to_string(num_variables) + ")");
  }

  if (joint_upper_limits.size() != 0 && joint_upper_limits.size() != num_variables) {
    throw std::invalid_argument(
        "PositionLimit: joint_upper_limits size (" + std::to_string(joint_upper_limits.size()) +
        ") does not match oink.num_variables (" + std::to_string(num_variables) + ")");
  }

  if (joint_lower_limits.size() > 0 && joint_upper_limits.size() > 0) {
    for (Eigen::Index i = 0; i < joint_lower_limits.size(); i++) {
      if (joint_lower_limits[i] > joint_upper_limits[i]) {
        throw std::invalid_argument("PositionLimit: joint_lower_limits[" + std::to_string(i) +
                                    "]=" + std::to_string(joint_lower_limits[i]) +
                                    " > joint_upper_limits[" + std::to_string(i) +
                                    "]=" + std::to_string(joint_upper_limits[i]));
      }
    }
  }

  if (joint_lower_limits.size() > 0) {
    q_min = joint_lower_limits;
  }

  if (joint_upper_limits.size() > 0) {
    q_max = joint_upper_limits;
  }
}

int PositionLimit::getNumConstraints(const SceneContext& /*context*/) const {
  return num_variables;
}

tl::expected<void, std::string> PositionLimit::computeQpConstraints(
    const SceneContext& context, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
    Eigen::Ref<Eigen::VectorXd> lower_bounds, Eigen::Ref<Eigen::VectorXd> upper_bounds) const {
  const auto& q = context.getJointPositions();

  auto maybe_q_collapsed = collapseContinuousJointPositions(context.getScene(), "", q);
  if (!maybe_q_collapsed) {
    return tl::make_unexpected("PositionLimit: " + maybe_q_collapsed.error());
  }
  const auto& q_collapsed = maybe_q_collapsed.value();

  const auto& res = initializeQMinAndQMax(context);
  if (!res) {
    return res;
  }

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

  // Assumes single-DOF joints (nq == nv once continuous joints are collapsed); v_indices selects
  // the group's joints from the full model.
  for (int i = 0; i < num_variables; ++i) {
    const int vi = v_indices(i);
    // Compute distance to upper limit
    if (std::isfinite(q_max(i))) {
      delta_q_max(i) = std::max(q_max(i) - q_collapsed(vi), -max_pullback);
    } else {
      delta_q_max(i) = std::numeric_limits<double>::infinity();
    }

    // Compute distance to lower limit
    if (std::isfinite(q_min(i))) {
      delta_q_min(i) = std::max(q_collapsed(vi) - q_min(i), -max_pullback);
    } else {
      delta_q_min(i) = std::numeric_limits<double>::infinity();
    }
  }

  delta_q_max *= config_limit_gain;
  delta_q_min *= config_limit_gain;

  constraint_matrix.setIdentity();

  // Unlimited joints (e.g. continuous joints) keep infinite bounds; the QP solver treats those
  // rows as unbounded.
  lower_bounds = -delta_q_min;
  upper_bounds = delta_q_max;

  return {};
}

Eigen::VectorXd PositionLimit::computeViolation(const SceneContext& context) const {
  const auto& res = initializeQMinAndQMax(context);
  if (!res) {
    return Eigen::VectorXd::Constant(num_variables, std::numeric_limits<double>::infinity());
  }

  const auto q_collapsed =
      collapseContinuousJointPositions(context.getScene(), "", context.getJointPositions());

  if (!q_collapsed) {
    return Eigen::VectorXd::Constant(num_variables, std::numeric_limits<double>::infinity());
  }

  const Eigen::VectorXd q_group = q_collapsed.value()(v_indices);
  return (q_min - q_group).cwiseMax(q_group - q_max).cwiseMax(0.0);
}

tl::expected<void, std::string>
PositionLimit::initializeQMinAndQMax(const SceneContext& context) const {
  if (q_min.size() == 0u || q_max.size() == 0u) {
    const auto maybe_position_limits =
        context.getScene().getPositionLimitVectors("", /*collapsed*/ true);
    if (!maybe_position_limits) {
      return tl::make_unexpected("PositionLimit: " + maybe_position_limits.error());
    }

    if (q_min.size() == 0) {
      q_min = maybe_position_limits->first(v_indices);
    }

    if (q_max.size() == 0) {
      q_max = maybe_position_limits->second(v_indices);
    }
  }

  return {};
}

}  // namespace roboplan
