#pragma once

#include <aligator/core/constraint-set.hpp>          // ConstraintSetTpl
#include <aligator/core/function-abstract.hpp>       // StageFunctionTpl
#include <aligator/third-party/polymorphic_cxx14.h>  // xyz::polymorphic

#include <roboplan_aligator/constraints.hpp>  // spec types

#include <roboplan_aligator/problem_builder.hpp>  // PhaseSpace

namespace roboplan {

class Scene;
class ReducedGroupModel;

namespace aligator_detail {

struct ConstraintPair {
  xyz::polymorphic<aligator::StageFunctionTpl<double>> func;
  xyz::polymorphic<aligator::ConstraintSetTpl<double>> set;
};

ConstraintPair buildPositionLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                  const Scene& scene, const PositionLimit& spec);

ConstraintPair buildVelocityLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                  const Scene& scene, const VelocityLimit& spec);

ConstraintPair buildTorqueLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                const TorqueLimit& spec);

ConstraintPair buildFramePoseConstraint(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                        const FramePoseConstraint& spec);

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
