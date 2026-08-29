Trajectory Optimization
=======================

:doc:`trajectory_generation` covers *timing* a fixed geometric path with TOPP-RA.
This page covers the other trajectory-generation tool in RoboPlan, ``roboplan_aligator``, which
*optimizes* a trajectory: it may reshape the path itself, subject to the robot's forward dynamics,
to satisfy torque limits, windowed costs, and hard via-point constraints.

``roboplan_aligator`` wraps the `aligator <https://github.com/Simple-Robotics/aligator>`_ trajectory
optimizer (a proximal differential-dynamic-programming solver, ``SolverProxDDP``) around a Pinocchio
forward-dynamics model of a joint group. Its ``TrajectoryOptimizer`` takes a seed trajectory and
returns a dynamically-feasible one along with the **torque profile** that produces it.


TOPP-RA vs. ``roboplan_aligator``
---------------------------------

The two tools answer different questions and are often used together, not instead of one another.

- **TOPP-RA asks:** *given this path, how fast can the robot follow it* under velocity and
  acceleration limits? The path geometry is an input and is never changed.
- **Trajectory optimization asks:** *what trajectory* best trades off the objectives (reach this
  pose, keep the tool level over the approach, arrive at rest) while respecting the robot's dynamics
  and limits? The path geometry is an output — it is reshaped by the solve.

.. list-table::
   :header-rows: 1
   :class: comparison-table
   :widths: 22 39 39

   * - Aspect
     - TOPP-RA (``roboplan_toppra``)
     - Trajectory optimization (``roboplan_aligator``)
   * - Input
     - A geometric path (waypoints) plus a fitting mode.
     - A seed trajectory (e.g. a straight-line interpolation of an RRT path) plus costs and
       constraints.
   * - Path geometry
     - Fixed — TOPP-RA only assigns timing; it cannot move a waypoint to, say, respect a torque
       limit.
     - Reshaped — the optimizer moves the trajectory to satisfy costs and constraints.
   * - Dynamics
     - Kinematic only (velocity / acceleration / jerk limits).
     - Full rigid-body forward dynamics; enforces **torque** limits and returns the torque profile.
   * - Constraints
     - Box limits on the timing.
     - Windowed costs and constraints: soft frame-pose/axis/configuration/velocity/effort costs and
       hard position/velocity/torque/frame-pose and (self-)collision constraints, each attachable to
       a sub-window of the horizon.
   * - Output
     - A time-parameterized ``JointTrajectory`` (positions, velocities, accelerations, times).
     - States, velocities, and controls (torques); ``toRoboplan()`` converts to a
       ``JointTrajectory`` (positions + velocities + times).
   * - Online use
     - One-shot timing.
     - Supports receding-horizon re-solving (``shift`` + ``solve(previous_result)``) for MPC-style
       tracking of a moving target.
   * - Cost
     - Cheap and deterministic.
     - More expensive; an iterative nonlinear solve seeded from an initial guess.


Which should you use?
---------------------

.. list-table::
   :header-rows: 1
   :class: comparison-table
   :widths: 20 44 36

   * - Tool
     - Choose it when...
     - Keep in mind...
   * - **TOPP-RA**
     - You already have a good, collision-free path (from RRT, Cartesian planning, or a teach
       pendant) and only need to *time* it — the common case, and the cheaper choice.
     - It cannot change the path. If the timed trajectory exceeds a torque limit or you want to smooth
       out a jerky route, TOPP-RA has no lever to pull; that is a job for trajectory optimization.
   * - **roboplan_aligator**
     - You need the path itself reshaped: to honor **torque**/effort limits, to pass through a hard
       Cartesian via-point (a grasp), to apply costs only over a *window* of the motion, to get a
       torque profile as output, or to re-solve online against a moving target (MPC).
     - It is an iterative nonlinear solve — more expensive than TOPP-RA and dependent on a reasonable
       seed. See the limitations below before relying on it.


Composing with the rest of RoboPlan
-----------------------------------

``roboplan_aligator`` sits **downstream** of the samplers and IK solvers that produce seeds
(``roboplan_rrt``, ``roboplan_oink``) and **parallel** to ``roboplan_toppra``. Two common pipelines:

- **Plan → time:** ``roboplan_rrt`` finds a collision-free path, then TOPP-RA times it. Fast; use when
  the geometric path is already what you want.
- **Plan → optimize:** ``roboplan_rrt`` finds a collision-free path that *seeds* a
  ``TrajectoryOptimizer``, which then refines it into a dynamically-feasible motion respecting a
  torque limit. See ``roboplan_examples/python/example_aligator_trajopt.py`` for a worked,
  side-by-side comparison of the raw RRT path, the RRT + TOPP-RA pipeline, and the RRT + aligator
  pipeline on the same start/goal pair — including a plot of the torque TOPP-RA's trajectory would
  actually require (computed post hoc via inverse dynamics) against the limit aligator enforces
  directly during the solve.


Comparison to STOMP, CHOMP, and TrajOpt
----------------------------------------

Outside of RoboPlan, "trajectory optimization" more often refers to a different family of tools —
STOMP, CHOMP, and TrajOpt (the algorithm behind Tesseract's ``TrajOptMotionPlanner``) — that also
refine or replace a geometric path. They solve a related but narrower problem than
``roboplan_aligator``, so it is worth being precise about the difference before reaching for one over
the other.

**The split that matters most: kinematic vs. dynamic.** STOMP, CHOMP, and TrajOpt all optimize a
sequence of *joint-space waypoints* — positions, with velocity/acceleration derived by finite
differences. None of them treat mass, inertia, or torque as decision-relevant quantities.
``roboplan_aligator`` instead solves a genuine optimal-control problem: state ``(q, v)`` propagated
through Pinocchio's forward dynamics with torque as the control input. That is why it can report a
torque profile and enforce ``TorqueLimit`` as a real dynamic-feasibility constraint, and why its
output is already a *timed* trajectory rather than a geometric path that still needs a separate
retiming step (TOPP-RA/TOTG/Ruckig).

.. list-table::
   :header-rows: 1
   :class: comparison-table
   :widths: 16 21 21 21 21

   * - Aspect
     - STOMP
     - CHOMP
     - TrajOpt / Tesseract
     - ``roboplan_aligator``
   * - Algorithm
     - Gradient-free stochastic sampling (probability-weighted rollout averaging)
     - Covariant functional gradient descent, preconditioned by a smoothness metric
     - Sequential convex optimization with an exact-penalty merit function
     - Proximal augmented-Lagrangian DDP (Riccati recursion)
   * - Dynamics
     - Kinematic; an optional torque *cost* via inverse dynamics, not a constraint
     - Kinematic only
     - Kinematic only
     - Full rigid-body forward dynamics; torque is a control input, not a cost proxy
   * - Collision
     - Precomputed signed distance field; gradient-free, so non-differentiable costs are fine
     - Precomputed signed distance field; requires a differentiable cost and its gradient
     - Convex-convex signed distance, with continuous (swept) checking between waypoints
     - Custom per-pair distance residual; pairs fixed at build time, enforced at stage knots only
   * - Hard constraints
     - Soft costs only
     - Soft costs, with metric-projected equality constraints
     - Soft-but-exact via an :math:`\ell_1` penalty (asymptotically hard as the penalty grows)
     - Soft-but-exact via an augmented Lagrangian (asymptotically hard; see Limitations below)
   * - Output
     - Geometric path; needs external retiming
     - Geometric path; needs external retiming
     - Geometric path; needs external retiming
     - Already time-parameterized: positions, velocities, and torques at fixed ``dt``

The practical upshot: reach for STOMP/CHOMP/TrajOpt (or their MoveIt 2 / Tesseract integrations) when
you want a *kinematic* smoother/refiner over a cluttered scene — especially one where TrajOpt's
continuous collision checking matters more than dynamic feasibility. Reach for ``roboplan_aligator``
when you specifically need dynamic feasibility, a torque profile, hard windowed via-points, or
receding-horizon MPC — and keep in mind that its collision handling (fixed pairs selected at build
time, knot-only enforcement) is currently less battle-tested for tight, cluttered geometry than
TrajOpt's purpose-built continuous-collision machinery.


Limitations
-----------

Trajectory optimization is powerful but has sharp edges. State these honestly when deciding whether to
reach for it:

- **Fixed horizon and time step.** The number of stages ``N`` and the step ``dt`` are set at
  construction; the solver does not add stages or retime the grid.
- **Constraints hold at the stage knots.** Costs and constraints are evaluated at the discrete stages.
  Between two knots the trajectory can still stray — a collision constraint that holds at every knot
  does not guarantee clearance *between* knots. Use a fine enough ``dt`` for the motion.
- **Fixed-base groups only.** The optimized joint group is a fixed-base subset of the model; floating
  bases and contact dynamics are out of scope.
- **Single seed.** A solve runs from one seed. If it does not converge, retrying from a different seed
  (for example a fresh RRT path) is the intended fallback, not an automatic feature.
- **Constraints are satisfied asymptotically.** The augmented-Lagrangian solver drives constraint
  violation down over iterations rather than enforcing it exactly. Always check
  ``result.max_constraint_violation`` against your tolerance — it is reported honestly and can be
  non-zero on an early-terminated or non-converged solve.
- **``toRoboplan()`` drops torques.** The conversion to a core ``JointTrajectory`` keeps positions,
  velocities, and times; accelerations and the torque profile stay on the ``TrajOptResult`` (no
  ``JointTrajectory`` field exists for either).


Practical considerations
-------------------------

Using the output with a position- or velocity-controlled robot
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Most robots are commanded in position or velocity mode, not torque mode — and that is fine.
``TrajOptResult::controls`` (the torque profile) does not need to be sent to the robot at all: use the
optimized ``(q, v)`` trajectory as position/velocity setpoints and let the robot's own servo loop
generate whatever torque is needed to track them. The value of the dynamics-aware solve is not "you
get to command torque" — it is that the reference trajectory is *guaranteed dynamically feasible*
(it will not demand torque beyond what the robot's actual mass, inertia, gravity, and Coriolis terms
allow), which a purely kinematic planner (STOMP/CHOMP/TrajOpt, or an RRT + TOPP-RA pipeline) cannot
promise, since required torque is a function of the full rigid-body dynamics, not just
velocity/acceleration bounds.

``TrajOptResult::toRoboplan()`` populates ``JointTrajectory.positions``, ``.velocities``, and
``.times`` in full-model layout — positions via ``Scene::toFullJointPositions()``, velocities via
``Scene::toFullJointVelocities()`` (non-group entries zero, since a locked joint has no meaningful
velocity in a group-scoped solve). Feed both positions *and* velocities to your trajectory execution
interface (e.g. a ROS 2 ``joint_trajectory_controller``) rather than positions alone: because aligator
integrates real dynamics, consecutive ``(q_k, v_k)`` pairs are dynamically consistent with each other,
which makes them good Hermite-spline endpoints for whatever interpolation your controller does between
the optimizer's (typically coarser) knots and the robot's servo rate.

If your driver supports a combined position/velocity/effort command interface, ``controls`` can ride
along as feedforward torque to reduce tracking error further — useful, but optional.

Model parameter requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An imprecise mass matrix does not make the solver useless, but it is worth knowing what actually
depends on it. Frame costs/constraints, joint/velocity limits, and collision constraints are all
purely kinematic — they depend on the URDF's link and joint geometry, not its inertial parameters —
so they are unaffected by inertial-parameter error. Only two things ride on the mass matrix being
accurate: the torque profile's trustworthiness as a real command or feedforward signal, and whether
``TorqueLimit`` is a *meaningful* dynamic-feasibility guarantee rather than a soft regularizer.

What matters is **validity**, not laboratory-grade accuracy:

- Approximate but physically plausible inertial parameters (CAD-derived, or whatever a manufacturer
  URDF ships) are enough for the solver to converge to smooth, sensible trajectories — the gap
  between computed and real torque scales with the parameter error, but this does not break
  convergence, since ProxDDP optimizes consistently against whatever model it is given.
- Missing or degenerate inertials (for example a placeholder ``mass="0"`` link) are the real failure
  mode: Pinocchio's forward dynamics needs a non-singular mass matrix, and a degenerate inertia can
  produce NaNs or an ill-conditioned solve. Fill in a rough, physically plausible value rather than
  leaving it at zero.
- A missing or zero URDF effort limit does not silently clamp the default ``TorqueLimit`` to zero —
  it is treated as unconstrained, so an incomplete URDF will not spuriously make the problem
  infeasible.

If you do not trust your inertial parameters and are not sending ``controls`` to the robot anyway (the
common case for a position/velocity-controlled robot, above), keep ``TorqueLimit`` loose or omit it,
and rely on the position/velocity output rather than the torque profile for validation. The default
``control_reg`` cost (quadratic torque regularization) still biases the solver toward smooth,
low-effort motion under *any* self-consistent model, so it remains a useful smoothness proxy even when
the model's absolute accuracy is uncertain.
