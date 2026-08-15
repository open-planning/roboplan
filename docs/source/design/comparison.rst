Comparison to Other Tools
=========================

There are several excellent open-source tools for robot motion planning, each with its own design philosophy.
This page compares RoboPlan to the most common alternatives so you can pick the right tool for your application.

As described in the :doc:`Design Philosophy </design/philosophy>`, RoboPlan differs from most frameworks in three ways:

- **Not a monolith**: Concrete, shared data types instead of abstract plugin interfaces. Swapping components means changing code, not YAML.
- **Middleware is optional**: The core library is standalone; ROS 2 wrappers live in the separate `roboplan-ros <https://github.com/open-planning/roboplan-ros>`_ repository.
- **Bindings are top priority**: Core algorithms are implemented in C++ and ship with typed Python bindings, installable via ``pip`` and ``conda``.

.. note::

   This comparison reflects the state of each project at the time of writing.
   All of these tools are actively developed, so check their documentation for the latest details.
   If you see anything that is incorrect or missing, please submit an issue or pull request.


At a Glance
-----------

.. list-table::
   :header-rows: 1
   :class: comparison-table
   :widths: 14 22 22 22 22

   * - Tool
     - Scope
     - ROS coupling
     - Configuration style
     - Python
   * - **RoboPlan**
     - Motion planning library built on `Pinocchio <https://github.com/stack-of-tasks/pinocchio>`__: IK, sampling-based planning, Cartesian planning, and trajectory timing.
     - Optional — ROS 2 wrappers in a separate repository.
     - Code-first: concrete data types and per-component options structs.
     - nanobind bindings with typed stubs; installable via pip and conda-forge.
   * - `MoveIt 2 <https://moveit.ai/>`__
     - Full ROS 2 manipulation framework: planning, IK, perception, execution, task planning, and GUI tooling.
     - Required — built on ROS 2 nodes, pluginlib, and parameters.
     - Runtime-configurable: nearly everything is a plugin behind abstract interfaces, selected via YAML.
     - ``moveit_py`` bindings based on pybind11; distributed as ROS 2 packages (no pip wheels; conda only via RoboStack channels).
   * - `Tesseract <https://github.com/tesseract-robotics/tesseract>`__
     - Industrial motion planning framework (SwRI / ROS-Industrial): environment model, planners, and task pipelines.
     - Optional — ROS-free core with separate ``tesseract_ros2`` wrappers.
     - Runtime-configurable: plugin factories, YAML-defined task pipelines, and named planner profiles.
     - SWIG-based bindings (maintained separately from the core); pip wheels available.
   * - `OMPL <https://ompl.kavrakilab.org/>`__
     - Library of ~60 sampling-based planning algorithms; deliberately excludes robot models, collision checking, and visualization.
     - None — plain C++ library, wrapped by MoveIt and Tesseract.
     - Code-first: subclass state spaces, validity checkers, and planners.
     - nanobind bindings (since 2.0); pip wheels since 1.7.
   * - `Drake <https://drake.mit.edu/>`__
     - Model-based design toolbox: multibody dynamics, systems framework, and optimization, with planning tools built on top (trajectory optimization, GCS, IRIS).
     - Optional (experimental) — ``drake-ros`` integration in a separate repository.
     - Code-first: compose systems and optimization problems in C++/Python.
     - ``pydrake``; mature and pip-installable.
   * - `cuRobo <https://curobo.org/>`__
     - GPU-accelerated motion generation: batched IK, collision checking, trajectory optimization, and MPC. Requires an NVIDIA GPU.
     - None — ROS 2 / MoveIt 2 integration via NVIDIA Isaac ROS cuMotion.
     - Python API plus YAML robot configurations (curated collision spheres).
     - Python-first (PyTorch); installed from source.


Feature Comparison
------------------

A closer look at RoboPlan and the frameworks with the most overlapping scope.

.. list-table::
   :header-rows: 1
   :class: comparison-table
   :widths: 12 20 20 20 20

   * - Feature
     - RoboPlan
     - MoveIt 2
     - Tesseract
     - OMPL
   * - Robot modeling
     - Pinocchio (URDF + SRDF), including mimic joints
     - URDF + SRDF (``RobotModel``)
     - Scene graph with native URDF + SRDF parsing
     - None — user-defined state spaces
   * - Collision checking
     - Coal: discrete checks and distance queries, with per-thread scene contexts for concurrent queries
     - FCL (default) or Bullet plugins
     - Bullet (discrete + continuous swept) or FCL (discrete) plugins
     - None — user-supplied validity checkers
   * - Inverse kinematics
     - SimpleIK (damped least squares) and OInK (QP-based with task priorities and control barrier functions)
     - Plugins: KDL, TRAC-IK, bio_ik, IKFast
     - Plugins: KDL, OPW, UR, IKFast
     - None
   * - Sampling-based planning
     - RRT, RRT-Connect, RRT*, and constrained planning on pose constraints (CBiRRT2); k-d tree neighbor search with continuous/planar joint topologies
     - Via OMPL plugin
     - Via OMPL wrapper
     - ~60 planners, including asymptotically optimal and constrained (manifold) planning
   * - Optimization-based planning
     - Not yet
     - CHOMP, STOMP
     - TrajOpt, TrajOpt-IFOPT
     - None (sampling-based only)
   * - Cartesian planning
     - Multi-waypoint, multi-end-effector planner: QP-based IK resolves the task-space path, then TOPP-RA times it under the joint limits
     - Pilz industrial planner (LIN/CIRC), Cartesian interpolation
     - Descartes ladder-graph planner; its command language mixes freespace and Cartesian segments
     - None
   * - Task composition
     - Not yet — components are composed directly in code
     - MoveIt Task Constructor: powerful multi-stage task planning (e.g., pick-and-place) built from generator, propagator, and connector stages
     - Task Composer: YAML-defined pipelines (planning, validation, smoothing) executed as parallel task graphs
     - None
   * - Path post-processing
     - Shortcutting with redundant vertex removal; uniform resampling
     - Planning request/response adapters
     - Task composer nodes (e.g., contact-check validation, smoothing)
     - Path simplification utilities
   * - Trajectory timing
     - TOPP-RA with four path-fitting modes
     - TOTG and Ruckig adapters
     - TOTG, ISP, Ruckig
     - None — outputs geometric paths only
   * - Perception
     - Octrees from point clouds as collision geometry (no live sensor pipeline)
     - Live octomap updates from depth sensors
     - Octree (octomap) collision geometry
     - None
   * - Servoing / teleoperation
     - No dedicated servo component, but OInK supports servo-style control (see the teleoperation example)
     - MoveIt Servo — a dedicated component, though in practice it tends to require substantial tuning to get good results
     - None
     - None
   * - Visualization
     - Viser web viewer with interactive gizmos; matplotlib plots
     - RViz plugin and Setup Assistant GUI
     - ``tesseract_qt`` / Tesseract Studio; RViz plugins
     - None (the OMPL.app GUI was removed in OMPL 2.0)


Which Tool Should You Use?
--------------------------

.. list-table::
   :header-rows: 1
   :class: comparison-table
   :widths: 12 44 44

   * - Tool
     - Choose it when...
     - Keep in mind...
   * - **RoboPlan**
     - You want a lean, code-first library for manipulator IK, planning, and trajectory generation that works in C++ or Python, with or without ROS.
     - Pre-1.0 with breaking changes and an intentionally small algorithm surface; no optimization-based planners, perception pipeline, or online execution layer (yet!).
   * - MoveIt 2
     - You are building a ROS 2 system and want a batteries-included stack: GUI setup, perception, execution, and swappable planner/IK plugins without recompiling.
     - Requires ROS 2. Behavior is spread across many YAML/SRDF/launch files, and the abstract plugin interfaces can make debugging and customization harder. Open-source features and maintenance have slowed recently.
   * - Tesseract
     - You target industrial Cartesian processes (welding, painting, machining) that need mixed freespace/Cartesian programs, TrajOpt, and continuous collision checking — with Python bindings and optional ROS integration, like RoboPlan.
     - Pre-1.0 with breaking changes. Shares MoveIt's plugin/YAML indirection (by design), and the Python bindings are low-level SWIG wrappers.
   * - OMPL
     - You need a battle-tested catalog of sampling-based planners, constrained planning on manifolds, or planner benchmarking tools.
     - It is an algorithms library, not a framework: you (or a framework like MoveIt or Tesseract) must supply the robot model, collision checking, and trajectory timing.
   * - Drake
     - You want optimization-first planning (trajectory optimization, Graphs of Convex Sets) and high-fidelity dynamics with analytical gradients.
     - Steep learning curve; its planning tools are building blocks you compose yourself, not an end-to-end manipulation stack.
   * - cuRobo
     - You need very fast, batched motion generation and IK for learning pipelines or data generation, and have NVIDIA hardware.
     - Hard CUDA dependency with no CPU fallback; each robot needs a curated collision-sphere configuration.
