Cartesian Planning
==================

The ``roboplan_cartesian_planning`` package traces a Cartesian path in joint space using the :ref:`OInK <oink-solver>` optimal IK solver.

Approach
--------

The planner builds an arc-length SE(3) reference from the waypoints (linear interpolation for position, SLERP for orientation).
It then runs an offline servo loop using the OInK solver.
At each control step, the planner advances the reference, solves one differential-IK step, and integrates the result.
When the robot cannot keep up (e.g., near a singularity or joint velocity limit), the reference feedrate is throttled so that every committed sample stays within tolerance of the geometric path.

Joint **velocity** and **position** limits are enforced inside the QP (and verified per step).

Two speed modes are exposed:

- ``Constant``: the tool moves at the commanded linear/angular speed wherever feasible, respecting joint velocity and position limits.
  This is a velocity-level trace: it does **not** bound joint acceleration, so its ``peak_acceleration_ratio`` can be large.
- ``Toppra``: resolves the path to a dense joint path with the same Oink tracker, then
  time-parameterizes it with :doc:`TOPP-RA <trajectory_generation>` over a straight-segment + circular-blend geometry.
  This way, the trajectory respects joint **velocity and acceleration** limits.
  Unlike ``Constant`` mode, the tool speed varies along the path.

Both modes return a ``JointTrajectory`` plus ``peak_velocity_ratio`` / ``peak_acceleration_ratio`` so the caller can see how close the result is to the joint limits.
Use ``Toppra`` when acceleration feasibility matters (e.g. high commanded speeds, or executing on hardware).
