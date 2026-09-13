#pragma once

#include <functional>

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
/// velocity-tangent block [nv, ndx)), constructs the cost with `target` (size nq+nv) as its
/// initial full state target, inserts it into `stack` with weight `cost_weight`, recovers the
/// stored copy via `dynamic_cast` (insertion into a `CostStack` copies -- see
/// docs/aligator_structure.md §3), and returns a `setTarget`-style closure that rewrites only the
/// masked block of the target while holding the other block fixed at its value in `target`:
/// - `mask_configuration_block == true`: the closure expects a size-`nq` vector and rewrites the
///   target's head(nq), keeping tail(nv) fixed at `target.tail(nv)`.
/// - `mask_configuration_block == false`: the closure expects a size-`nv` vector and rewrites the
///   target's tail(nv), keeping head(nq) fixed at `target.head(nq)`.
///
/// Not exported as public API -- internal plumbing for the cost factories.
std::function<void(const Eigen::VectorXd&)>
attachMaskedStateCost(CostStack& stack, const PhaseSpace& space, int nq, int nv,
                      const Eigen::VectorXd& target, const Eigen::VectorXd& block_weights,
                      bool mask_configuration_block, double cost_weight);

}  // namespace roboplan::aligator_detail
