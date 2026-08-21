#include <roboplan_oink/constraints/acceleration_limit.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <roboplan/core/scene_utils.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

namespace {

/// dt * sqrt(2 * a * d), the displacement that can still be braked to zero over distance d.
/// Guards the 0 * infinity case, which would otherwise produce a NaN bound.
double brakingDisplacement(double dt, double a, double d) {
  if (std::isinf(a) || std::isinf(d)) {
    return std::numeric_limits<double>::infinity();
  }
  return dt * std::sqrt(2.0 * a * d);
}

}  // namespace

AccelerationLimit::AccelerationLimit(const Oink& oink, double dt, const Eigen::VectorXd& a_max)
    : dt(dt), a_max(a_max), delta_q_prev(Eigen::VectorXd::Zero(oink.num_variables)),
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
  delta_q_prev = v_prev * dt;
}

void AccelerationLimit::setTargetDisplacement(const Eigen::VectorXd& delta_q_target_in) {
  if (delta_q_target_in.size() != static_cast<Eigen::Index>(num_variables)) {
    throw std::invalid_argument(
        "AccelerationLimit: delta_q_target size (" + std::to_string(delta_q_target_in.size()) +
        ") does not match num_variables (" + std::to_string(num_variables) + ")");
  }
  delta_q_target = delta_q_target_in;
}

void AccelerationLimit::clearTargetDisplacement() { delta_q_target.reset(); }

void AccelerationLimit::reset() {
  delta_q_prev.setZero();
  delta_q_target.reset();
}

int AccelerationLimit::getNumConstraints(const SceneContext& /*context*/) const {
  return num_variables;
}

tl::expected<void, std::string> AccelerationLimit::computeQpConstraints(
    const SceneContext& context, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
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

  const auto& q = context.getJointPositions();
  auto maybe_q_collapsed = collapseContinuousJointPositions(context.getScene(), "", q);
  if (!maybe_q_collapsed) {
    return tl::make_unexpected("AccelerationLimit: " + maybe_q_collapsed.error());
  }
  const auto& q_collapsed = maybe_q_collapsed.value();

  // Fetch joint position limits once (used for the braking-distance term).
  if (q_min.size() == 0u) {
    const auto maybe_position_limits =
        context.getScene().getPositionLimitVectors("", /*collapsed*/ true);
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
    // A joint that cannot accelerate cannot move. Pin it instead of emitting a box that the
    // acceleration bound below would make empty as soon as delta_q_prev is nonzero.
    if (a_max(i) <= 0.0) {
      lower_bounds(i) = 0.0;
      upper_bounds(i) = 0.0;
      continue;
    }

    // Acceleration finite-difference bound, centered on the previous displacement. This box
    // is always non-empty: it is delta_q_prev +/- a_max*dt².
    const double accel_lo = delta_q_prev(i) - a_max(i) * dt_sq;
    const double accel_hi = delta_q_prev(i) + a_max(i) * dt_sq;

    // Braking-distance bound toward each position limit.
    double brake_hi = brakingDisplacement(dt, a_max(i), delta_q_max(i));
    double brake_lo = -brakingDisplacement(dt, a_max(i), delta_q_min(i));

    // Braking-distance bound toward the task target, on the side facing it. Absent unless the
    // caller supplied a target this step. Bounding the trailing side too would, right after an
    // overshoot, demand a deceleration larger than a_max and fight the acceleration bound for
    // no benefit.
    if (delta_q_target) {
      const double d = (*delta_q_target)(i);
      if (d > 0.0) {
        brake_hi = std::min(brake_hi, brakingDisplacement(dt, a_max(i), d));
      } else if (d < 0.0) {
        brake_lo = std::max(brake_lo, -brakingDisplacement(dt, a_max(i), -d));
      }
    }

    // A braking bound may not ask for more deceleration than a_max can deliver in one step.
    // Clamping it into the acceleration box guarantees the two always overlap, so the row
    // cannot come out infeasible (which a QP solver would otherwise resolve arbitrarily).
    brake_hi = std::max(brake_hi, accel_lo);
    brake_lo = std::min(brake_lo, accel_hi);

    double upper = std::min(accel_hi, brake_hi);
    double lower = std::max(accel_lo, brake_lo);

    // Unbounded sides (e.g. an unlimited acceleration or a joint without a position limit)
    // are passed through as +/- infinity; the QP solver treats those rows as unbounded.
    if (!std::isfinite(upper)) {
      upper = kInfinity;
    }
    if (!std::isfinite(lower)) {
      lower = -kInfinity;
    }

    upper_bounds(i) = upper;
    lower_bounds(i) = lower;
  }

  return {};
}

}  // namespace roboplan
