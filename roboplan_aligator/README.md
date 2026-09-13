# roboplan-aligator

Trajectory optimization (aligator proximal-DDP) satellite for RoboPlan.

Wraps `aligator::SolverProxDDPTpl<double>` over a group-reduced Pinocchio model. The public
class is `roboplan::TrajectoryOptimizer` (Python: `roboplan.aligator.TrajectoryOptimizer`):
set an initial state, add costs and constraints, `build()`, then `solve()`.

Every cost/constraint targets one of three things — every stage (`addCost`/`addConstraint`),
exactly one stage (`addStageCost`/`addStageConstraint(stage, ...)`), or the terminal node
(`addTerminalCost`/`addTerminalConstraint`) — assembled fully, once, inside `build()` (mirroring
aligator's own per-index `addStage` loop). There is no post-build mutation: retargeting means
`resetProblem()`, re-adding with the new value, and `build()` again.

Three ways to express costs/constraints at each of those targets:

- **Spec-based** (common path): plain data structs — `CostSpec` (`ConfigurationCost`,
  `VelocityCost`) and `ConstraintSpec` (`TorqueLimit`).
- **Direct aligator** (advanced, C++ only): subclass `aligator::CostAbstractTpl<double>` /
  `aligator::StageFunctionTpl<double>` / `aligator::ConstraintSetTpl<double>` and pass them
  to the `addCost` / `addConstraint` overloads.
- **Stage authorship** (advanced, C++ only): `setStageFactory` supplies a per-index callback
  that builds the entire aligator `StageModel` itself, with no roboplan translation involved.

The reduced model, phase space, and assembled aligator problem are exposed through
`TrajectoryOptimizer::reducedGroupModel()`, `phaseSpace()`, and `problem()` for power users.

Examples: `roboplan_examples/python/example_aligator_trajopt.py`.

## Known gaps

- **Custom costs/constraints are C++ only.** The direct-aligator overloads take
  `xyz::polymorphic<aligator::CostAbstractTpl<double>>` and
  `(StageFunctionTpl, ConstraintSetTpl)` pairs, for which Python types are not exposed
  (aligator's Python bindings are not built). Python users are limited to the closed
  `CostSpec` / `ConstraintSpec` variants.
- **`CostSpec` / `ConstraintSpec` are closed variants** — there is no `CustomCost` /
  `CustomConstraint` member and no generic "wrap a callable residual" escape hatch.
- **Fixed-base groups only.** Floating-base groups (free-flyer / planar) are rejected by
  `ReducedGroupModel` because aligator's `MultibodyFreeFwdDynamics` is free-space (no contact).
- **aligator is a build-time requirement.** It is resolved via `find_package(aligator)` with a
  FetchContent fallback pinned to a specific commit; it is linked PUBLIC and is part of the
  exported link interface.

## Acknowledgments

This integration is built on top of [aligator](https://github.com/Simple-Robotics/aligator), a proximal-DDP trajectory
optimization library, by Simple-Robotics and LAAS-CNRS.

A sincere appreciation goes out to the original authors for their work and dedication in creating such a sophisticated library!
