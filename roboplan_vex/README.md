# vex: Sequential Convex Trajectory Optimization

`roboplan_vex` is an offline, kinematic-only trajectory optimizer for a joint group of a
`roboplan::Scene`. It transcribes the trajectory with a double-integrator model (per-node position
and velocity, piecewise-constant acceleration on each segment) and solves it by sequential convex
programming: each iteration solves a sparse convex QP with ProxQP, with ℓ1 penalties for nonconvex
terms, a trust box, and a TrajOpt-style step acceptance test.

The name comes from *con-vex*: every iteration solves a convex QP.
