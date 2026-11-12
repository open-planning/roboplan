# roboplan-ompl

Experimental [OMPL](https://ompl.kavrakilab.org/) integration for RoboPlan.

Wraps a `Scene` joint group as an OMPL state space, state validity checker, and motion validator,
so any OMPL geometric planner (RRT-Connect, AORRTC, BIT*, PRM*, ...) plans against RoboPlan's
collision checking. `OMPLPlanner` selects a planner by name and returns a `JointPath`; the state
space and validators are also public for direct use with OMPL's own API.

Requires OMPL >= 2.0.2 (`ompl` on conda-forge, `ros-<distro>-ompl` on ROS 2).

Notes:

* Bounded joints are real vector coordinates; continuous joints and the rotation of planar joints
  are SO(2) subspaces, so paths wrap around. Floating joints are not supported.
* Every coordinate needs finite position limits, since OMPL samples uniformly within them. Planar
  joint translation without limits gets the same default range as the Scene's random sampling;
  set explicit limits with `Scene::importJointLimitsFromConfig`.
* OMPL's direct informed sampler only supports real vector spaces. Groups with continuous or
  planar joints fall back to rejection sampling for AORRTC, BIT*, and Informed RRT*.
* A collision-free straight line from start to goal is returned without running the planner, as
  the RRT planner does. This also sidesteps an OMPL 2.0.2 AORRTC bug that returns a goal-to-goal
  path in that case.
* Groups with planar joints use the Scene's SE(2) distance, which is not a metric (it can violate
  the triangle inequality). OMPL 2.0.2's AORRTC can then loop forever in its connect step; a fix
  is proposed upstream. The other planners are unaffected.
* VAMP's SIMD collision checking is not integrated: VAMP is a collision backend for its own
  prebaked robot models rather than a planner that accepts an external validator. Its AORRTC
  algorithm is available here through OMPL's implementation.
