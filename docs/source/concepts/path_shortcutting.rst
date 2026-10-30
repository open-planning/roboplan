Path Shortcutting
=================

Paths from sampling-based planners such as :doc:`RRT <sampling_based_planning>` are collision-free but tend to wander, with unnecessary corners and detours.
*Path shortcutting* is a fast post-processing step that shortens such a path without re-running the planner.

``PathShortcutter`` (in ``roboplan_core/include/roboplan/core/path_utils.hpp``) shortens a ``JointPath`` using random shortcutting.
Each iteration:

1. **Samples** two configurations at random along the current path.
2. Checks whether they can be connected by a straight, collision-free segment in configuration space.
3. If so, **splices in** that direct connection, discarding the intervening waypoints (the "corner" being cut).

Repeatedly cutting corners monotonically shortens the path while keeping it collision-free.
Because corner-cutting introduces new interpolated vertices, a deterministic **redundant-vertex removal** pass is interleaved periodically (and run once at the end) to collapse vertices whose neighbors have become directly connectable, preventing an accumulation of micro-segments.

This implementation follows `Section 3.5.3 of Motion Planning in Higher Dimensions <https://motion.cs.illinois.edu/RoboticSystems/MotionPlanningHigherDimensions.html>`_.

The shortcutter is configured through ``PathShortcuttingOptions``:

+----------------------------------+-----------------------------------------------------------------+-----------+
| Parameter                        | Description                                                     | Default   |
+==================================+=================================================================+===========+
| ``group_name``                   | Joint group to shortcut                                         | ""        |
+----------------------------------+-----------------------------------------------------------------+-----------+
| ``max_step_size``                | Collision-check step size and minimum separation in a shortcut  | 0.05      |
+----------------------------------+-----------------------------------------------------------------+-----------+
| ``max_iters``                    | Maximum number of random sampling iterations                    | 100       |
+----------------------------------+-----------------------------------------------------------------+-----------+
| ``seed``                         | Random seed (``< 0`` uses a random seed)                        | 0         |
+----------------------------------+-----------------------------------------------------------------+-----------+
| ``max_convergence_iters``        | Stop early after this many consecutive iterations with no       | 20        |
|                                  | shortcut applied (``0`` disables early stopping)                |           |
+----------------------------------+-----------------------------------------------------------------+-----------+
| ``redundant_removal_iters``      | Cadence (in iterations) of the redundant-vertex removal pass    | 20        |
+----------------------------------+-----------------------------------------------------------------+-----------+
| ``constraint_projection``        | Projection options for shortcutting with constraints            |           |
+----------------------------------+-----------------------------------------------------------------+-----------+
| ``constrained_min_detour_ratio`` | Skip projected shortcuts of sections at most this many times    | 1.1       |
|                                  | longer than the straight distance between their ends            |           |
+----------------------------------+-----------------------------------------------------------------+-----------+

Applying the shortcutter to a path is a single call:

.. code-block:: python

   from roboplan.core import PathShortcutter, PathShortcuttingOptions

   options = PathShortcuttingOptions(
       group_name="arm",
       max_step_size=0.05,
       max_iters=100,
   )
   shortcutter = PathShortcutter(scene, options)

   # `path` is a JointPath, e.g. the output of rrt.plan(...).
   shortened_path = shortcutter.shortcut(path)

The result is a collision-free ``JointPath``, ready to be timed with :doc:`trajectory_generation`.

Constrained paths
-----------------

A path from :doc:`constrained planning <sampling_based_planning>` must be shortcut with the same constraints, or the straight shortcuts will leave them:

.. code-block:: python

   shortened_path = shortcutter.shortcut(path, [constraint])

A straight shortcut is then also checked against the constraints.
If it leaves them, the shortcut is instead walked with the planner's constrained extension, in ``constraint_projection.path_step_size`` hops that are each projected back onto the constraints,
as in the shortcut smoothing of `Berenson et al. (2009) <https://personalrobotics.cs.washington.edu/publications/berenson2009cbirrt.pdf>`_.
A projected shortcut can be longer than the section it replaces, so it is only kept if it is shorter,
and sections already within ``constrained_min_detour_ratio`` of the straight distance between their ends are skipped.

The walk matters most for pinned constraints, such as two hands holding one object, which straight shortcuts almost never satisfy.
Constraints with slack let most straight shortcuts through on their own.
