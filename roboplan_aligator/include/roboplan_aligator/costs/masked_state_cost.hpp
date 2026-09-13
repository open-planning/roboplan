#pragma once

#include <Eigen/Dense>

#include <aligator/modelling/costs/sum-of-costs.hpp>  // CostStackTpl

#include <roboplan_aligator/problem_builder.hpp>  // PhaseSpace

namespace roboplan::aligator_detail {

using CostStack = aligator::CostStackTpl<double>;

/// @brief Internal helper shared by `attachConfigurationCost` and `attachVelocityCost`: both
/// insert a `QuadraticStateCostTpl` masked to exactly one tangent block of the state space
/// (ndx = 2*nv), differing only in *which* block is masked and how the target vector is built.
///
/// Builds the ndx x ndx diagonal weight matrix restricted to one tangent block (the
/// configuration-tangent block [0, nv) when `mask_configuration_block` is true, else the
/// velocity-tangent block [nv, ndx)), constructs the cost with `target` (size nq+nv) as its full
/// state target, and inserts it into `stack` with weight `cost_weight`. Called once per stage at
/// assembly time (aligator's own idiom: build each stage's cost fully, once) -- nothing reaches
/// back into `stack` afterward, so there is no return value.
///
/// Not exported as public API -- internal plumbing for the cost factories.
void attachMaskedStateCost(CostStack& stack, const PhaseSpace& space, int nv,
                           const Eigen::VectorXd& target, const Eigen::VectorXd& block_weights,
                           bool mask_configuration_block, double cost_weight);

}  // namespace roboplan::aligator_detail
