Path Shortcutting
=================

Paths produced by sampling-based planners such as :doc:`RRT <sampling_based_planning>` are collision-free but rarely efficient.
Because the tree grows by random extension, the resulting path tends to wander, with unnecessary corners and detours that a more direct route would avoid.
*Path shortcutting* is a fast post-processing step that smooths such a path without re-running the planner.

RoboPlan provides the ``PathShortcutter`` class (in ``roboplan/include/roboplan/core/path_utils.hpp``), which shortens a ``JointPath`` using random shortcutting.

Each iteration of the shortcutter:

1. **Samples** two configurations at random along the current path.
2. Checks whether they can be connected by a straight, collision-free segment in configuration space.
3. If so, **splices in** that direct connection, discarding the intervening waypoints (the "corner" being cut).

Repeatedly cutting corners this way monotonically shortens the path while keeping it collision-free.
Because corner-cutting introduces new interpolated vertices, a deterministic **redundant-vertex removal** pass is interleaved periodically (and run once at the end) to collapse vertices whose neighbors have become directly connectable, preventing an accumulation of unhelpful micro-segments.

This implementation follows `Section 3.5.3 of Motion Planning in Higher Dimensions <https://motion.cs.illinois.edu/RoboticSystems/MotionPlanningHigherDimensions.html>`_.

The shortcutter is configured through ``PathShortcuttingOptions``:

+-------------------------------+-----------------------------------------------------------------+-----------+
| Parameter                     | Description                                                     | Default   |
+===============================+=================================================================+===========+
| ``group_name``                | Joint group to shortcut                                         | ""        |
+-------------------------------+-----------------------------------------------------------------+-----------+
| ``max_step_size``             | Collision-check step size and minimum separation in a shortcut  | 0.05      |
+-------------------------------+-----------------------------------------------------------------+-----------+
| ``max_iters``                 | Maximum number of random sampling iterations                    | 100       |
+-------------------------------+-----------------------------------------------------------------+-----------+
| ``seed``                      | Random seed (``< 0`` uses a random seed)                        | 0         |
+-------------------------------+-----------------------------------------------------------------+-----------+
| ``max_convergence_iters``     | Stop early after this many consecutive iterations with no       | 20        |
|                               | shortcut applied (``0`` disables early stopping)                |           |
+-------------------------------+-----------------------------------------------------------------+-----------+
| ``redundant_removal_iters``   | Cadence (in iterations) of the redundant-vertex removal pass    | 20        |
+-------------------------------+-----------------------------------------------------------------+-----------+

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

The shortened path is itself a collision-free ``JointPath``, ready to be timed into a smooth trajectory with :doc:`trajectory_generation`.
