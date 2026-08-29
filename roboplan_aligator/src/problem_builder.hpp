#pragma once

// INTERNAL helper — NOT public API. This header lives under src/, is not installed, and is not
// exposed in the Python bindings. It centralizes ALL aligator problem/dynamics/cost construction
// so aligator (and pinocchio, which aligator links PUBLIC) types never touch a public
// roboplan_aligator header — aligator is linked PRIVATE (design §2.2, roboplan_aligator/CLAUDE.md).
//
// It is shared by two consumers:
//   - TrajectoryOptimizer (src/trajectory_optimizer.cpp): assembles the full problem + solver.
//   - the finite-difference dynamics test (test/test_dynamics.cpp): builds the SAME discretized
//     dynamics so what the test finite-differences is byte-identical to what the optimizer solves.
//
// Every aligator symbol used here is verified against pinned source and recorded in API_NOTES.md
// (§ "Problem shell + solver driver", Prompt 5) before use.

#include <memory>

#include <Eigen/Core>

#include <aligator/core/explicit-dynamics.hpp>       // ExplicitDynamicsModelTpl
#include <aligator/core/traj-opt-problem.hpp>        // TrajOptProblemTpl
#include <aligator/modelling/spaces/multibody.hpp>   // MultibodyPhaseSpace, pinocchio::ModelTpl
#include <aligator/third-party/polymorphic_cxx14.h>  // xyz::polymorphic

#include <roboplan_aligator/types.hpp>  // IntegratorType, TrajOptOptions

namespace roboplan::aligator_detail {

/// @brief Reduced-model state space: x = [q; v], nx = nq + nv, ndx = 2*nv (§3.2).
using PhaseSpace = aligator::MultibodyPhaseSpace<double>;

/// @brief Discretized (integrated) dynamics as the explicit-dynamics base, so both integrator
/// choices (semi-implicit Euler / RK2) share one erased type.
using DiscreteDynamics = xyz::polymorphic<aligator::ExplicitDynamicsModelTpl<double>>;

/// @brief The trajectory-optimization problem type.
using Problem = aligator::TrajOptProblemTpl<double>;

/// @brief Builds the reduced-model phase space from a reduced pinocchio model (copies the model).
PhaseSpace makePhaseSpace(const pinocchio::Model& reduced_model);

/// @brief Builds the discretized free-space multibody dynamics (ABA, actuation B = I so nu = nv,
/// §3.2), integrated with `type` at step `dt`.
/// @param space The reduced-model phase space (copied into the continuous dynamics).
/// @param type Which integrator discretizes the continuous dynamics.
/// @param dt Time step in seconds (must be > 0).
DiscreteDynamics makeDiscreteDynamics(const PhaseSpace& space, IntegratorType type, double dt);

/// @brief Assembles the problem shell (design §3.2): `horizon` identical stages, each an empty
/// cost sum plus the default quadratic control-regularization cost (weight `options.control_reg`,
/// target u = 0; skipped when `control_reg <= 0`), the discretized dynamics, an initial-condition
/// constraint at `x0`, and an empty terminal-cost placeholder.
/// @param space The reduced-model phase space.
/// @param x0 The fixed initial state [q0; v0] (size nx).
/// @param horizon Number of stages N (must be > 0).
/// @param dt Time step in seconds (must be > 0).
/// @param options Discretization/regularization options.
/// @return The assembled problem on the heap (returned by unique_ptr so callers need not rely on
/// TrajOptProblemTpl movability).
std::unique_ptr<Problem> buildProblemShell(const PhaseSpace& space, const Eigen::VectorXd& x0,
                                           int horizon, double dt, const TrajOptOptions& options);

}  // namespace roboplan::aligator_detail
