# RoboPlan

Modern robot motion planning library based on [Pinocchio](https://github.com/stack-of-tasks/pinocchio).

Refer to the [full documentation](https://roboplan.readthedocs.io) for more information.

> [!WARNING]
> This is an experimental repository!
> Until version 1.0 is released, expect some breaking changes.

---

## Packages list

The main folders found in this repo are as follows.

- `docs` : The documentation source.
- `roboplan` : The core C++ library.
- `roboplan_simple_ik` : A simple inverse kinematics (IK) solver.
- `roboplan_oink` : A task-based Optimal Inverse Kinematics (OInK) solver.
- `roboplan_rrt` : A Rapidly-exploring Random Tree (RRT) based motion planner.
- `roboplan_toppra` : A wrapper around the TOPP-RA algorithm for trajectory timing.
- `roboplan_cartesian_planning` : A Cartesian planner for following task-space paths.
- `roboplan_example_models` : Contains robot models used for testing and examples.
- `roboplan_examples` : Basic examples with real robot models.
- `superbuild` : Shared CMake entry point for local, Conda, and PyPI-wheel builds.

---

<img src="docs/source/media/kinova_ik.gif" alt="Interactive inverse kinematics (IK) with Kinova Gen3 arm." width="600">

<img src="docs/source/media/dual_franka_rrt.gif" alt="Rapidly-exploring random tree (RRT) with dual Franka FR3 arms." width="600">

<img src="docs/source/media/cartesian_planning_ur5.gif" alt="Cartesian path planning with a UR5 arm." width="600">
