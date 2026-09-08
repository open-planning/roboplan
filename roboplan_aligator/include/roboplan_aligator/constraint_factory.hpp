#pragma once

#include <aligator/core/constraint-set.hpp>          // ConstraintSetTpl
#include <aligator/core/function-abstract.hpp>       // StageFunctionTpl
#include <aligator/third-party/polymorphic_cxx14.h>  // xyz::polymorphic

#include <roboplan_aligator/constraints.hpp>  // spec types

#include <roboplan_aligator/problem_builder.hpp>  // PhaseSpace

namespace roboplan {

class ReducedGroupModel;

namespace aligator_detail {

struct ConstraintPair {
  xyz::polymorphic<aligator::StageFunctionTpl<double>> func;
  xyz::polymorphic<aligator::ConstraintSetTpl<double>> set;
};

ConstraintPair buildTorqueLimit(const PhaseSpace& space, const ReducedGroupModel& rgm,
                                const TorqueLimit& spec);

}  // namespace aligator_detail

}  // namespace roboplan
