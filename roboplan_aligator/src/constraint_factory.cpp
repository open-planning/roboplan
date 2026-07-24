#include "constraint_factory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <aligator/core/unary-function.hpp>                   // UnaryFunctionTpl
#include <aligator/modelling/constraints/box-constraint.hpp>  // BoxConstraintTpl
#include <aligator/modelling/function-xpr-slice.hpp>          // FunctionSliceXprTpl
#include <aligator/modelling/multibody/frame-placement.hpp>   // FramePlacementResidualTpl
#include <aligator/modelling/state-error.hpp>                 // State/ControlErrorResidualTpl
#include <pinocchio/collision/distance.hpp>                   // pinocchio::computeDistances
#include <pinocchio/multibody/geometry.hpp>                   // pinocchio::GeometryData
#include <pinocchio/multibody/model.hpp>                      // pinocchio::Model
#include <pinocchio/spatial/se3.hpp>                          // pinocchio::SE3

#include <roboplan/core/scene.hpp>

#include "collision_residual.hpp"
#include "reduced_group_model.hpp"

namespace roboplan::aligator_detail {

namespace {

using ManifoldPoly = xyz::polymorphic<aligator::ManifoldAbstractTpl<double>>;
using StageFuncPoly = xyz::polymorphic<aligator::StageFunctionTpl<double>>;
using UnaryFuncPoly = xyz::polymorphic<aligator::UnaryFunctionTpl<double>>;
using ConstraintSetPoly = xyz::polymorphic<aligator::ConstraintSetTpl<double>>;

using StateError = aligator::StateErrorResidualTpl<double>;
using ControlError = aligator::ControlErrorResidualTpl<double>;
using FramePlacement = aligator::FramePlacementResidualTpl<double>;
using BoxConstraint = aligator::BoxConstraintTpl<double>;
// StateError is a UnaryFunction, so its output slice uses the UnaryFunction specialization (which
// stays a UnaryFunction — valid at the terminal node, which has no control).
using StateSlice = aligator::FunctionSliceXprTpl<double, aligator::UnaryFunctionTpl<double>>;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Resolves a frame name against the reduced model, throwing a setup-time invariant violation if the
// frame is absent (a constraint referencing a nonexistent frame is a misconfiguration).
pinocchio::FrameIndex resolveFrame(const ReducedGroupModel& rgm, const std::string& frame,
                                   const char* name) {
  const auto frame_id = rgm.frameId(frame);
  if (!frame_id) {
    throw std::invalid_argument(std::string(name) + ": " + frame_id.error());
  }
  return *frame_id;
}

// Remaps a full-model limit vector into reduced-model layout by joint name, mirroring the q0 remap
// in reduced_group_model.cpp (order-robust; does not assume preserved joint ordering).
// `use_tangent` selects the tangent layout (idx_vs/nvs, size nv — velocity / torque) vs the
// configuration layout (idx_qs/nqs, size nq).
Eigen::VectorXd remapFullToReduced(const Eigen::VectorXd& full, const pinocchio::Model& full_model,
                                   const pinocchio::Model& reduced_model, bool use_tangent) {
  const int reduced_size = use_tangent ? reduced_model.nv : reduced_model.nq;
  Eigen::VectorXd reduced = Eigen::VectorXd::Zero(reduced_size);
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(reduced_model.njoints);
       ++j) {
    const pinocchio::JointIndex full_id = full_model.getJointId(reduced_model.names[j]);
    if (use_tangent) {
      reduced.segment(reduced_model.idx_vs[j], reduced_model.nvs[j]) =
          full.segment(full_model.idx_vs[full_id], full_model.nvs[full_id]);
    } else {
      reduced.segment(reduced_model.idx_qs[j], reduced_model.nqs[j]) =
          full.segment(full_model.idx_qs[full_id], full_model.nqs[full_id]);
    }
  }
  return reduced;
}

// Validates a user-supplied bound: empty is allowed (use the model default); otherwise the size
// must match `expected`.
void checkUserBoundSize(const Eigen::VectorXd& bound, int expected, const char* what) {
  if (bound.size() != 0 && bound.size() != expected) {
    throw std::invalid_argument(std::string(what) + ": bound size " + std::to_string(bound.size()) +
                                " != expected " + std::to_string(expected) + ".");
  }
}

}  // namespace

ConstraintPair buildPositionLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                  const Scene& scene, const PositionLimit& spec) {
  const int nq = rgm.nq();
  const int nv = rgm.nv();
  checkUserBoundSize(spec.q_min, nq, "PositionLimit q_min");
  checkUserBoundSize(spec.q_max, nq, "PositionLimit q_max");

  // Model defaults from core (full model, expanded layout), remapped into reduced-model layout.
  const auto model_limits = scene.getPositionLimitVectors("", /*collapsed=*/false);
  if (!model_limits) {
    throw std::invalid_argument("PositionLimit: " + model_limits.error());
  }
  Eigen::VectorXd q_min =
      remapFullToReduced(model_limits->first, rgm.fullModel(), rgm.reducedModel(), false);
  Eigen::VectorXd q_max =
      remapFullToReduced(model_limits->second, rgm.fullModel(), rgm.reducedModel(), false);

  // Clamp any user bound to the model's own per-DoF (a user bound can only tighten; decision P7).
  if (spec.q_min.size() != 0) {
    q_min = q_min.cwiseMax(spec.q_min);
  }
  if (spec.q_max.size() != 0) {
    q_max = q_max.cwiseMin(spec.q_max);
  }

  // The residual value is space.difference(neutral, x); express the q-bounds in that tangent frame
  // (reduces to the raw q-bounds for revolute/prismatic joints, whose neutral is 0).
  const Eigen::VectorXd neutral = space.neutral();
  Eigen::VectorXd x_min(nq + nv);
  x_min << q_min, Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd x_max(nq + nv);
  x_max << q_max, Eigen::VectorXd::Zero(nv);
  const Eigen::VectorXd lower = space.difference(neutral, x_min).head(nv);
  const Eigen::VectorXd upper = space.difference(neutral, x_max).head(nv);

  // State residual sliced to the q-tangent rows {0 .. nv-1} of the ndx-dim state error.
  StateError state_error(ManifoldPoly(space), nv, neutral);
  std::vector<int> q_rows(static_cast<std::size_t>(nv));
  std::iota(q_rows.begin(), q_rows.end(), 0);
  StateSlice slice(UnaryFuncPoly(state_error), q_rows);

  return {.func = StageFuncPoly(slice), .set = ConstraintSetPoly(BoxConstraint(lower, upper))};
}

ConstraintPair buildVelocityLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                  const Scene& scene, const VelocityLimit& spec) {
  const int nv = rgm.nv();
  checkUserBoundSize(spec.v_max, nv, "VelocityLimit v_max");

  const auto model_limits = scene.getVelocityLimitVectors("");
  if (!model_limits) {
    throw std::invalid_argument("VelocityLimit: " + model_limits.error());
  }
  Eigen::VectorXd lower =
      remapFullToReduced(model_limits->first, rgm.fullModel(), rgm.reducedModel(), true);
  Eigen::VectorXd upper =
      remapFullToReduced(model_limits->second, rgm.fullModel(), rgm.reducedModel(), true);

  // A user v_max is symmetric [-v_max, +v_max], intersected with the model's per-DoF (decision P7).
  if (spec.v_max.size() != 0) {
    lower = lower.cwiseMax(-spec.v_max);
    upper = upper.cwiseMin(spec.v_max);
  }

  // State residual sliced to the velocity rows {nv .. 2nv-1}; the v-block is Euclidean (target 0),
  // so its residual value is v directly and the bounds need no tangent conversion.
  const Eigen::VectorXd neutral = space.neutral();
  StateError state_error(ManifoldPoly(space), nv, neutral);
  std::vector<int> v_rows(static_cast<std::size_t>(nv));
  std::iota(v_rows.begin(), v_rows.end(), nv);
  StateSlice slice(UnaryFuncPoly(state_error), v_rows);

  return {.func = StageFuncPoly(slice), .set = ConstraintSetPoly(BoxConstraint(lower, upper))};
}

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

ConstraintPair buildFramePoseConstraint(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                        const FramePoseConstraint& spec) {
  if (spec.tol_pos < 0.0 || spec.tol_rot < 0.0) {
    throw std::invalid_argument("FramePoseConstraint: tol_pos and tol_rot must be nonnegative.");
  }
  const pinocchio::FrameIndex frame_id = resolveFrame(rgm, spec.frame, "FramePoseConstraint");
  const int ndx = space.ndx();
  const int nu = rgm.nv();

  const pinocchio::SE3 target(spec.target);  // explicit SE3 ctor from a 4x4 homogeneous transform
  FramePlacement residual(ndx, nu, rgm.reducedModel(), target, frame_id);

  // 6-D box on the log6 placement error, ordered [translation(3); rotation-log(3)].
  Eigen::VectorXd lower(6);
  lower << -spec.tol_pos, -spec.tol_pos, -spec.tol_pos, -spec.tol_rot, -spec.tol_rot, -spec.tol_rot;
  Eigen::VectorXd upper(6);
  upper << spec.tol_pos, spec.tol_pos, spec.tol_pos, spec.tol_rot, spec.tol_rot, spec.tol_rot;

  return {.func = StageFuncPoly(residual), .set = ConstraintSetPoly(BoxConstraint(lower, upper))};
}

namespace {

// Which candidate collision pairs a collision spec draws from.
enum class CollisionKind { SELF, ENVIRONMENT };

// Expands a collision spec into one single-pair (CollisionDistanceResidual, box[d_min, +inf]) per
// tracked pair (design §5). Classifies candidate pairs by the FULL-model parentJoint of
// each geometry (reliable — the reduced model folds locked robot links onto the base), filters to
// pairs the group can move, ranks by distance at `q_select`, and keeps the closest `n_pairs`
// (all if `n_pairs <= 0`). Runs on the reduced model + reduced collision geometry, so the
// residual's Jacobian columns are already in reduced-`v` (no remap).
std::vector<ConstraintPair> buildCollision(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                           CollisionKind kind, int n_pairs, double d_min,
                                           const Eigen::VectorXd& q_select) {
  const int ndx = space.ndx();
  const int nu = rgm.nv();
  const pinocchio::Model& reduced_model = rgm.reducedModel();
  const pinocchio::GeometryModel& reduced_geom = rgm.reducedCollisionModel();
  const pinocchio::GeometryModel& full_geom = rgm.fullCollisionModel();

  // One-shot distances at the seed configuration, on private build-time scratch (not reused at
  // solve time — each residual owns its own data). buildReducedModel preserves geometry-object
  // order, so a reduced pair's geometry indices are valid in the full collision model too.
  pinocchio::Data data(reduced_model);
  pinocchio::GeometryData geom_data(reduced_geom);
  pinocchio::computeDistances(reduced_model, data, reduced_geom, geom_data, q_select);

  struct Candidate {
    std::size_t pair_id;
    double distance;
  };
  std::vector<Candidate> candidates;
  for (std::size_t k = 0; k < reduced_geom.collisionPairs.size(); ++k) {
    const auto& pair = reduced_geom.collisionPairs[k];
    const std::size_t g1 = pair.first;
    const std::size_t g2 = pair.second;
    // Self vs environment: classify by full-model parent joint (articulated link vs static world).
    const bool g1_articulated = full_geom.geometryObjects[g1].parentJoint > 0;
    const bool g2_articulated = full_geom.geometryObjects[g2].parentJoint > 0;
    const bool is_self = g1_articulated && g2_articulated;
    const bool is_env = g1_articulated != g2_articulated;  // exactly one static
    if (kind == CollisionKind::SELF && !is_self) {
      continue;
    }
    if (kind == CollisionKind::ENVIRONMENT && !is_env) {
      continue;
    }
    // Optimizer-relevance: skip pairs with a constant distance (neither geometry moves with the
    // group — both fold onto the reduced base), which the optimizer cannot affect.
    const bool moves = reduced_geom.geometryObjects[g1].parentJoint > 0 ||
                       reduced_geom.geometryObjects[g2].parentJoint > 0;
    if (!moves) {
      continue;
    }
    candidates.push_back({k, geom_data.distanceResults[k].min_distance});
  }

  const std::size_t take = (n_pairs <= 0)
                               ? candidates.size()
                               : std::min(static_cast<std::size_t>(n_pairs), candidates.size());
  std::partial_sort(candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(take),
                    candidates.end(),
                    [](const Candidate& a, const Candidate& b) { return a.distance < b.distance; });
  candidates.resize(take);

  // Box: signed distance >= d_min, i.e. the 1-D residual value lies in [d_min, +inf).
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(1, d_min);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(1, kInfinity);

  std::vector<ConstraintPair> out;
  out.reserve(take);
  for (const Candidate& c : candidates) {
    CollisionDistanceResidual residual(ndx, nu, reduced_model, reduced_geom, c.pair_id);
    out.push_back(
        {.func = StageFuncPoly(residual), .set = ConstraintSetPoly(BoxConstraint(lower, upper))});
  }
  return out;
}

}  // namespace

std::vector<ConstraintPair> buildSelfCollisionConstraints(const PhaseSpace& space,
                                                          const ReducedGroupModel& rgm,
                                                          const SelfCollisionConstraint& spec,
                                                          const Eigen::VectorXd& q_select) {
  return buildCollision(space, rgm, CollisionKind::SELF, spec.n_pairs, spec.d_min, q_select);
}

std::vector<ConstraintPair> buildCollisionConstraints(const PhaseSpace& space,
                                                      const ReducedGroupModel& rgm,
                                                      const CollisionConstraint& spec,
                                                      const Eigen::VectorXd& q_select) {
  return buildCollision(space, rgm, CollisionKind::ENVIRONMENT, spec.n_pairs, spec.d_min, q_select);
}

}  // namespace roboplan::aligator_detail
