#pragma once

#include <memory>

#include <Eigen/Core>

#include <aligator/core/explicit-dynamics.hpp>       // ExplicitDynamicsModelTpl
#include <aligator/core/traj-opt-problem.hpp>        // TrajOptProblemTpl
#include <aligator/modelling/spaces/multibody.hpp>   // MultibodyPhaseSpace, pinocchio::ModelTpl
#include <aligator/third-party/polymorphic_cxx14.h>  // xyz::polymorphic

#include <roboplan_aligator/types.hpp>  // IntegratorType, TrajOptOptions

namespace roboplan::aligator_detail {

/// @brief Reduced-model state space: x = [q; v], nx = nq + nv, ndx = 2*nv.
using PhaseSpace = aligator::MultibodyPhaseSpace<double>;

/// @brief Discretized (integrated) dynamics as the explicit-dynamics base, so both integrator
/// choices (semi-implicit Euler / RK2) share one erased type.
using DiscreteDynamics = xyz::polymorphic<aligator::ExplicitDynamicsModelTpl<double>>;

/// @brief The trajectory-optimization problem type.
using Problem = aligator::TrajOptProblemTpl<double>;

/// @brief Builds the reduced-model phase space from a reduced pinocchio model (copies the model).
PhaseSpace makePhaseSpace(const pinocchio::Model& reduced_model);

/// @brief Builds the discretized free-space multibody dynamics (ABA, actuation B = I so nu = nv),
/// integrated with `type` at step `dt`.
/// @param space The reduced-model phase space (copied into the continuous dynamics).
/// @param type Which integrator discretizes the continuous dynamics.
/// @param dt Time step in seconds (must be > 0).
DiscreteDynamics makeDiscreteDynamics(const PhaseSpace& space, IntegratorType type, double dt);

/// @brief Builds an empty problem shell: no stages yet, an initial-condition constraint at `x0`,
/// and an empty terminal-cost placeholder. Mirrors aligator's own "no pre-allocated stages"
/// constructor (`TrajOptProblemTpl(x0, nu, space, term_cost)`) -- stages are added one at a time,
/// fully formed, via `Problem::addStage` (see `TrajectoryOptimizer::build()`), not built eagerly
/// here and decorated afterward.
/// @param space The reduced-model phase space.
/// @param x0 The fixed initial state [q0; v0] (size nx).
/// @param nu Control dimension (= nv for the fully-actuated group).
/// @return The assembled (stage-less) problem on the heap (returned by unique_ptr so callers need
/// not rely on TrajOptProblemTpl movability).
std::unique_ptr<Problem> buildEmptyProblem(const PhaseSpace& space, const Eigen::VectorXd& x0,
                                           int nu);

}  // namespace roboplan::aligator_detail
