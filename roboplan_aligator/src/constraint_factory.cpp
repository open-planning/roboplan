#include <roboplan_aligator/constraint_factory.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <aligator/modelling/constraints/box-constraint.hpp>  // BoxConstraintTpl
#include <aligator/modelling/state-error.hpp>                 // ControlErrorResidualTpl

#include <roboplan_aligator/reduced_group_model.hpp>

namespace roboplan::aligator_detail {

namespace {

using StageFuncPoly = xyz::polymorphic<aligator::StageFunctionTpl<double>>;
using ConstraintSetPoly = xyz::polymorphic<aligator::ConstraintSetTpl<double>>;

using ControlError = aligator::ControlErrorResidualTpl<double>;
using BoxConstraint = aligator::BoxConstraintTpl<double>;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Validates a user-supplied bound: empty is allowed (use the model default); otherwise the size
// must match `expected`.
void checkUserBoundSize(const Eigen::VectorXd& bound, int expected, const char* what) {
  if (bound.size() != 0 && bound.size() != expected) {
    throw std::invalid_argument(std::string(what) + ": bound size " + std::to_string(bound.size()) +
                                " != expected " + std::to_string(expected) + ".");
  }
}

}  // namespace

ConstraintPair buildTorqueLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                const TorqueLimit& spec) {
  const int nv = rgm.nv();
  const int ndx = space.ndx();
  checkUserBoundSize(spec.tau_max, nv, "TorqueLimit tau_max");

  // Default from the reduced model's effort field (no Scene effort accessor exists; design §4.4).
  Eigen::VectorXd lower = rgm.reducedModel().lowerEffortLimit;
  Eigen::VectorXd upper = rgm.reducedModel().upperEffortLimit;

  // A non-finite or zero model effort limit means "unactuated / unspecified" -> treat as unbounded
  // (±inf), never a zero-torque clamp (decision P7).
  for (int i = 0; i < nv; ++i) {
    if (!std::isfinite(upper[i]) || upper[i] == 0.0) {
      upper[i] = kInfinity;
    }
    if (!std::isfinite(lower[i]) || lower[i] == 0.0) {
      lower[i] = -kInfinity;
    }
  }

  // A user tau_max is symmetric [-tau_max, +tau_max], intersected with the model's per-DoF.
  if (spec.tau_max.size() != 0) {
    lower = lower.cwiseMax(-spec.tau_max);
    upper = upper.cwiseMin(spec.tau_max);
  }

  // Control residual: value = u exactly (Euclidean control space, zero reference).
  ControlError control_error(ndx, nv);
  return {.func = StageFuncPoly(control_error),
          .set = ConstraintSetPoly(BoxConstraint(lower, upper))};
}

}  // namespace roboplan::aligator_detail
