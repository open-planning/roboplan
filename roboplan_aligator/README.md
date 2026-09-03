# roboplan-aligator

Trajectory optimization (aligator proximal-DDP) satellite for RoboPlan.

Wraps `aligator::SolverProxDDPTpl<double>` over a group-reduced Pinocchio model. The public
class is `roboplan::TrajectoryOptimizer` (Python: `roboplan.aligator.TrajectoryOptimizer`):
set an initial state, add costs and constraints, `build()`, then `solve()`.

Two ways to express costs/constraints:

- **Spec-based** (common path): plain data structs — `CostSpec` (`FramePoseCost`,
  `FrameAxisCost`, `ConfigurationCost`, `ControlCost`, `VelocityCost`) and `ConstraintSpec`
  (`PositionLimit`, `VelocityLimit`, `TorqueLimit`, `FramePoseConstraint`,
  `SelfCollisionConstraint`, `CollisionConstraint`). `addCost` returns a `CostHandle` for
  hot-path target retargeting between solves.
- **Direct aligator** (advanced, C++ only): subclass `aligator::CostAbstractTpl<double>` /
  `aligator::StageFunctionTpl<double>` / `aligator::ConstraintSetTpl<double>` and pass them
  to the `addCost` / `addConstraint` overloads.

The reduced model, phase space, and assembled aligator problem are exposed through
`TrajectoryOptimizer::reducedGroupModel()`, `phaseSpace()`, and `problem()` for power users.

Examples: `roboplan_examples/python/example_aligator_trajopt.py`,
`roboplan_examples/python/example_aligator_mpc.py`.

## Known gaps

- **Custom costs/constraints are C++ only.** The direct-aligator overloads take
  `xyz::polymorphic<aligator::CostAbstractTpl<double>>` and
  `(StageFunctionTpl, ConstraintSetTpl)` pairs, for which Python types are not exposed
  (aligator's Python bindings are not built). Python users are limited to the closed
  `CostSpec` / `ConstraintSpec` variants.
- **`CostSpec` / `ConstraintSpec` are closed variants** — there is no `CustomCost` /
  `CustomConstraint` member and no generic "wrap a callable residual" escape hatch.
- **Inter-stage collision clearance is not enforced.** `CollisionConstraint` /
  `SelfCollisionConstraint` hold at stage knots only; a straight-line segment whose both
  knots clear `d_min` can pass through a closer interior point (see
  `test_constraints.cpp`, `InterStageClearanceCaveat`).
- **Fixed-base groups only.** Floating-base groups (free-flyer / planar) are rejected by
  `ReducedGroupModel` because aligator's `MultibodyFreeFwdDynamics` is free-space (no contact).
- **aligator is a build-time requirement.** It is resolved via `find_package(aligator)` with a
  FetchContent fallback pinned to a specific commit; it is linked PUBLIC and is part of the
  exported link interface.
