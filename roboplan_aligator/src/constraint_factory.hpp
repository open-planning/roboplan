#pragma once

// INTERNAL helper — NOT public API. Builds the aligator (residual, ConstraintSet) pair for each
// hard-constraint spec (design §4.4). Keeps aligator/pinocchio out of the public headers. Shared by
// trajectory_optimizer.cpp (which attaches the pairs to the resolved stages / terminal) and
// test_constraints.cpp. Verified symbols recorded in API_NOTES.md (§ "Hard constraints", Prompt 7).

#include <aligator/core/constraint-set.hpp>          // ConstraintSetTpl
#include <aligator/core/function-abstract.hpp>       // StageFunctionTpl
#include <aligator/third-party/polymorphic_cxx14.h>  // xyz::polymorphic

#include <roboplan_aligator/constraints.hpp>  // spec types

#include "problem_builder.hpp"  // PhaseSpace

namespace roboplan {

class Scene;              // core (roboplan/core/scene.hpp)
class ReducedGroupModel;  // internal (src/reduced_group_model.hpp)

namespace aligator_detail {

/// @brief A hard constraint as an aligator (residual, constraint-set) pair, both type-erased to the
/// polymorphic bases that StageModel::addConstraint / TrajOptProblem::addTerminalConstraint accept.
/// @details One built pair is reusable across every stage in a window: both `addConstraint` and
/// `addTerminalConstraint` deep-copy (pushBack) their arguments (API_NOTES.md, §3.5-style value
/// polymorphism), so unlike costs there is no in-problem pointer to capture.
struct ConstraintPair {
  xyz::polymorphic<aligator::StageFunctionTpl<double>> func;
  xyz::polymorphic<aligator::ConstraintSetTpl<double>> set;
};

// Each build* resolves the spec against `rgm` (frames) and `scene` (limit defaults, remapped into
// reduced-model layout by joint name — never re-derived), clamps any user bound to the model's own
// per-DoF (maintainer decision, Prompt 7), and returns the (residual, box) pair. A wrong-size spec
// field or an unknown frame throws std::invalid_argument (a setup-time invariant violation).

ConstraintPair buildPositionLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                  const Scene& scene, const PositionLimit& spec);

ConstraintPair buildVelocityLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                  const Scene& scene, const VelocityLimit& spec);

ConstraintPair buildTorqueLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                const TorqueLimit& spec);

ConstraintPair buildFramePoseConstraint(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                        const FramePoseConstraint& spec);

// Collision specs expand to a LIST of single-pair (CollisionDistanceResidual, box[d_min, +inf])
// pairs (design §5, reuse path): the candidate pairs are classified against `rgm` (self = both
// articulated links; collision = robot link vs static), filtered to those the group can move,
// ranked by distance at `q_select` (the optimizer's initial reduced configuration), and the closest
// `spec.n_pairs` (all if `n_pairs <= 0`) are returned. Empty if the model has no matching
// optimizer-relevant pairs.
std::vector<ConstraintPair> buildSelfCollisionConstraints(const PhaseSpace& space,
                                                          const ReducedGroupModel& rgm,
                                                          const SelfCollisionConstraint& spec,
                                                          const Eigen::VectorXd& q_select);

std::vector<ConstraintPair> buildCollisionConstraints(const PhaseSpace& space,
                                                      const ReducedGroupModel& rgm,
                                                      const CollisionConstraint& spec,
                                                      const Eigen::VectorXd& q_select);

}  // namespace aligator_detail

}  // namespace roboplan
