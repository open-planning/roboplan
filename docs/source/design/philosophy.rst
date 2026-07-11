Design Philosophy
=================

RoboPlan is a motion planning library, similar to tools like `MoveIt <https://github.com/moveit/moveit2>`_.

The key idea is the use of the `Pinocchio <https://github.com/stack-of-tasks/pinocchio>`_ library to represent robots.
Pinocchio's strength is loading robot models from standard formats such as URDF and MJCF, and being able to represent rigid-body kinematics and dynamics.

RoboPlan can use these models to perform the basic calculations for motion planning and control.
This includes things like collision checking, forward kinematics, and inverse kinematics/dynamics.

What distinguishes RoboPlan from other solutions are the following design points.


Not a monolith
---------------

Several tools optimize their design for runtime configurability via YAML config files and plugins that rely on abstract interface classes for motion planners, IK solvers, etc.
However, not every motion planning solution fits into the same abstraction.
This library shall instead establish *standard data types* for things like joint states, paths, and trajectories.
It is strongly recommended that implementations use these data types in their interfaces as much as possible.

This does mean that switching to different planners (for example) requires changing your code.
But, grounded in a decade of experience developing runtime-configurable systems, we consider this worthwhile to keep the codebase simple and flexible.


Middleware is optional
----------------------

The core library is standalone.
Middleware such as ROS, and all its specific tools (message definitions, pub/sub, parameters, etc.) shall be available as *optional*, lightweight wrappers around the core... in a separate repository.

Consequently, community contributors can leverage the core library in any project as-is and can consider middleware connections tailored to their use-cases.

The `roboplan-ros <https://github.com/open-planning/roboplan-ros>`_ repository contains ROS wrappers for RoboPlan.


Bindings are top priority
-------------------------

Users should be able to easily install the Python bindings and get to hacking, debugging, and visualizing as easily as possible.
New users can develop directly using Python, whereas intermediate/advanced users can directly use C++ for performance.
Contributors are expected to implement new features in C++ and provide working Python bindings.
