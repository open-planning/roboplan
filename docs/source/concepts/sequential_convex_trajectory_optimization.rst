Sequential Convex Trajectory Optimization
=========================================

Paths from :doc:`RRT <sampling_based_planning>` are collision-free but long and jerky, and :doc:`path shortcutting <path_shortcutting>` only cuts corners along straight segments.
``Vex`` (package ``roboplan_vex``, Python module ``roboplan.vex``) instead *optimizes* a seed path into a smooth, collision-free trajectory of a given duration.
It follows TrajOpt (Schulman et al.): the nonconvex problem is solved as a sequence of convex quadratic programs (QPs), each a local model around the current trajectory.

Model
-----

The trajectory has :math:`N` nodes, uniformly spaced by :math:`\Delta t = T / (N - 1)` for the duration :math:`T`.
Each node :math:`k` has a position :math:`q_k` and velocity :math:`v_k`, and each segment a constant acceleration :math:`a_k`, with exact double-integrator dynamics

.. math::

   q_{k+1} = q_k + v_k \Delta t + \tfrac{1}{2} a_k \Delta t^2, \qquad v_{k+1} = v_k + a_k \Delta t.

The cost is the exact integral :math:`\int w_v \|v(t)\|^2 + w_a \|a(t)\|^2 \, dt` (``velocity_weight`` and ``acceleration_weight``), which shortens and smooths the trajectory.
The start and goal are reached at rest, and the ``Scene``'s joint position, velocity, and acceleration limits hold at every node.
All of these are linear, so they are hard constraints of every QP.

ℓ1-penalty / trust-box SCP
--------------------------

The seed ``JointPath`` is resampled to :math:`N` nodes and projected onto the hard constraints.
Each iteration then linearizes the nonconvex constraints :math:`g(z) \le 0` at the current trajectory, and solves a sparse QP (with `ProxSuite <https://github.com/Simple-Robotics/proxsuite>`_) in which each linearized row is softened by an ℓ1 penalty slack and the step is bounded by a trust box.
A step is accepted if the true merit (cost plus penalty times the constraint violation) improves by at least ``improve_ratio`` of the predicted improvement; otherwise the trust box shrinks.
When the steps stall while constraints are still violated, the penalty grows (up to ``max_penalty_increases`` times).
The result reports a ``VexStatus``, the iteration count, the final cost, and the largest remaining constraint violation.

Collision handling
------------------

Collision avoidance uses every active collision pair of the ``Scene`` (the same pairs ``hasCollisions`` checks: self, environment, and attached objects).
Each pair must keep a signed distance of at least ``collision_margin``.
The start and goal must already have that clearance on every active pair, since the trajectory begins and ends there: otherwise ``solve`` returns an error (raised as ``RuntimeError`` in Python) naming the endpoint, the two geometries, their distance and the margin, instead of optimizing a problem that cannot be satisfied.
Increase the endpoint's clearance or reduce ``collision_margin``.
It is checked at every interior node and at ``samples_per_segment`` interior points of each segment, on the exact polynomial :math:`q(t) = q_k + v_k t + \tfrac{1}{2} a_k t^2`.
At each sample, every pair closer than ``collision_check_distance`` gets one linearized row, built from the signed-distance gradient (``SceneContext::computeDistanceJacobian``); rows whose gradient over the group's joints is zero (pairs the group cannot move) are constant and stay out of the QP.
The merit used to accept steps evaluates all of these pairs, so a pair the QP did not model can only cause a rejected step, never a silently accepted collision.

The samples alone do not prove the trajectory between them is collision-free.
After the solve, a dense post-check samples every segment so that no joint moves more than 0.01 rad (or m) between consecutive collision checks, and reports the result as ``VexResult.continuous_collision_free``, independently of the status.

Limitations
-----------

This first version is deliberately small:

- Only vector-space joints (:math:`n_q = n_v`) are supported; continuous and floating joints return an error.
- The duration is fixed and must be chosen by the caller. The trajectory is not retimed; use :doc:`trajectory_generation` on its positions if a time-optimal trajectory is needed.
- Every iteration rebuilds and solves a sparse QP whose size grows with ``num_nodes`` and the number of nearby collision pairs, so solve times grow quickly with ``num_nodes`` and ``samples_per_segment``.
- Constraints between samples are only verified by the post-check, and signed distance is only piecewise smooth, so convergence is local and not guaranteed.

Usage
-----

.. code-block:: python

   from roboplan.vex import Vex, VexOptions, VexStatus

   options = VexOptions()
   options.group_name = "arm"
   vex = Vex(scene, options)

   # `path` is a JointPath, e.g. the output of rrt.plan(start, goal).
   result = vex.solve(start, goal, 5.0, path)
   if result.status == VexStatus.Converged and result.continuous_collision_free:
       trajectory = result.trajectory  # a JointTrajectory

``roboplan_examples/python/example_rrt.py --vex`` runs this after RRT and shows the optimized trajectory.
