#pragma once

#include <Eigen/Dense>

#include <aligator/modelling/costs/sum-of-costs.hpp>  // CostStackTpl

#include <roboplan_aligator/costs/configuration_cost.hpp>
#include <roboplan_aligator/costs/velocity_cost.hpp>

#include <roboplan_aligator/problem_builder.hpp>  // PhaseSpace

namespace roboplan {

class ReducedGroupModel;

namespace aligator_detail {

using CostStack = aligator::CostStackTpl<double>;

// Each attach* function builds the concrete aligator cost from a spec and inserts it into `stack`
// exactly once, at stage-assembly time -- there is no post-insertion mutation, so nothing is
// returned (aligator's own idiom: build each stage's cost fully, once).

void attachConfigurationCost(CostStack& stack, const PhaseSpace& space,
                             const ReducedGroupModel& rgm, const ConfigurationCost& spec,
                             double weight);

void attachVelocityCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                        const VelocityCost& spec, double weight);

}  // namespace aligator_detail

}  // namespace roboplan
