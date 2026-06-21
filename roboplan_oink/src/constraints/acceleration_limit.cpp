#include <roboplan_oink/constraints/acceleration_limit.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <OsqpEigen/OsqpEigen.h>
#include <roboplan/core/scene_utils.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

AccelerationLimit::AccelerationLimit(const Oink& oink, double dt, const Eigen::VectorXd& a_max)
    : dt(dt), a_max(a_max), Delta_q_prev(Eigen::VectorXd::Zero(oink.num_variables)),
      num_variables(oink.num_variables), v_indices(oink.v_indices), delta_q_max(oink.num_variables),
      delta_q_min(oink.num_variables) {
  if (dt <= 0.0) {
    throw std::invalid_argument("AccelerationLimit: dt must be positive, got " +
                                std::to_string(dt));
  }
  if (a_max.size() != static_cast<Eigen::Index>(num_variables)) {
    throw std::invalid_argument("AccelerationLimit: a_max size (" + std::to_string(a_max.size()) +
                                ") does not match oink.num_variables (" +
                                std::to_string(num_variables) + ")");
  }
  for (int i = 0; i < a_max.size(); ++i) {
    if (a_max(i) < 0.0) {
      throw std::invalid_argument("AccelerationLimit: a_max[" + std::to_string(i) +
                                  "] must be non-negative, got " + std::to_string(a_max(i)));
    }
  }
}

void AccelerationLimit::setLastVelocity(const Eigen::VectorXd& v_prev) {
  if (v_prev.size() != static_cast<Eigen::Index>(num_variables)) {
    throw std::invalid_argument("AccelerationLimit: v_prev size (" + std::to_string(v_prev.size()) +
                                ") does not match num_variables (" + std::to_string(num_variables) +
                                ")");
  }
  Delta_q_prev = v_prev * dt;
}

void AccelerationLimit::reset() { Delta_q_prev.setZero(); }

int AccelerationLimit::getNumConstraints(const Scene& /*scene*/) const { return num_variables; }

tl::expected<void, std::string> AccelerationLimit::computeQpConstraints(
    const Scene& scene, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
    Eigen::Ref<Eigen::VectorXd> lower_bounds, Eigen::Ref<Eigen::VectorXd> upper_bounds) const {
  // Validate pre-allocated workspace dimensions.
  if (constraint_matrix.rows() != num_variables || constraint_matrix.cols() != num_variables) {
    return tl::make_unexpected("AccelerationLimit: constraint_matrix size mismatch. Expected (" +
                               std::to_string(num_variables) + " x " +
                               std::to_string(num_variables) + "), got (" +
                               std::to_string(constraint_matrix.rows()) + " x " +
                               std::to_string(constraint_matrix.cols()) + ")");
  }
  if (lower_bounds.size() != num_variables) {
    return tl::make_unexpected("AccelerationLimit: lower_bounds size mismatch. Expected " +
                               std::to_string(num_variables) + ", got " +
                               std::to_string(lower_bounds.size()));
  }
  if (upper_bounds.size() != num_variables) {
    return tl::make_unexpected("AccelerationLimit: upper_bounds size mismatch. Expected " +
                               std::to_string(num_variables) + ", got " +
                               std::to_string(upper_bounds.size()));
  }

  const auto& q = scene.getCurrentJointPositions();
  auto maybe_q_collapsed = collapseContinuousJointPositions(scene, "", q);
  if (!maybe_q_collapsed) {
    return tl::make_unexpected("AccelerationLimit: " + maybe_q_collapsed.error());
  }
  const auto& q_collapsed = maybe_q_collapsed.value();

  // Fetch joint position limits once (used for the braking-distance term).
  if (q_min.size() == 0u) {
    const auto maybe_position_limits = scene.getPositionLimitVectors("", /*collapsed*/ true);
    if (!maybe_position_limits) {
      return tl::make_unexpected("AccelerationLimit: " + maybe_position_limits.error());
    }
    q_min = maybe_position_limits->first;
    q_max = maybe_position_limits->second;
  }

  // Distances to the position limits (clamped at zero to keep the sqrt real if the current
  // configuration sits slightly beyond a limit due to numerical error).
  for (int i = 0; i < num_variables; ++i) {
    const int vi = v_indices(i);
    delta_q_max(i) = std::isfinite(q_max(vi)) ? std::max(0.0, q_max(vi) - q_collapsed(vi))
                                              : std::numeric_limits<double>::infinity();
    delta_q_min(i) = std::isfinite(q_min(vi)) ? std::max(0.0, q_collapsed(vi) - q_min(vi))
                                              : std::numeric_limits<double>::infinity();
  }

  // Box constraints l <= G*dq <= u with G = identity.
  constraint_matrix.setIdentity();

  const double dt_sq = dt * dt;
  for (int i = 0; i < num_variables; ++i) {
    // Acceleration finite-difference bound, centered on the previous displacement.
    const double accel_upper = a_max(i) * dt_sq + Delta_q_prev(i);
    const double accel_lower = a_max(i) * dt_sq - Delta_q_prev(i);

    // Braking-distance bound toward each position limit.
    const double brake_upper = dt * std::sqrt(2.0 * a_max(i) * delta_q_max(i));
    const double brake_lower = dt * std::sqrt(2.0 * a_max(i) * delta_q_min(i));

    // Take the tighter of the two per side. The lower side mirrors Pink's stacked
    // -P*dq <= h form: dq >= -min(accel_lower, brake_lower).
    double upper = std::min(accel_upper, brake_upper);
    double lower = -std::min(accel_lower, brake_lower);

    // Clamp to OSQP's valid range.
    if (!std::isfinite(upper) || upper > OsqpEigen::INFTY) {
      upper = OsqpEigen::INFTY;
    }
    if (!std::isfinite(lower) || lower < -OsqpEigen::INFTY) {
      lower = -OsqpEigen::INFTY;
    }

    upper_bounds(i) = upper;
    lower_bounds(i) = lower;
  }

  return {};
}

}  // namespace roboplan
