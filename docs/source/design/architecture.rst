System Architecture
===================

RoboPlan is organized as a set of packages around a common core.
The core package defines the :doc:`standard data types </design/philosophy>` (e.g., joint information, paths, trajectories) and the ``Scene``, which represents the robot and its environment.
Each algorithm lives in its own package, consumes the ``Scene`` and the standard types, and can be installed and used independently.

The diagram below shows the major components, the external libraries they build on, and how the Python bindings fit in.
Colors indicate the implementation language of each component: C++ only, Python only, or C++ with Python bindings.

.. mermaid::

   flowchart TD
       USER["User code (Python or C++)"]

       subgraph PYONLY["Python-only modules"]
           VIZ["roboplan.visualization"]
           INTERP["roboplan.interpolation"]
       end

       subgraph ALGOS["Algorithm packages"]
           CART["CartesianPathPlanner<br/>(roboplan.cartesian_planning)"]
           SINK["SimpleIK<br/>(roboplan.simple_ik)"]
           OINK["OInK<br/>(roboplan.optimal_ik)"]
           RRT["RRT<br/>(roboplan.rrt)"]
           TOPPRA["TOPP-RA<br/>(roboplan.toppra)"]
       end

       subgraph CORE["Core package (roboplan.core)"]
           SCENE["Scene<br/>(robot model, collision, sampling)"]
           TYPES["Standard types<br/>(JointPath, JointTrajectory, ...)"]
           SHORT["Path utilities<br/>(shortcutting, resampling)"]
       end

       subgraph EXT["External libraries"]
           PIN["Pinocchio"]
           COAL["Coal"]
           PROXSUITE["ProxSuite"]
           TOPPRALIB["toppra"]
           DYNO["dynotree"]
           VISER["Viser"]
           MPL["matplotlib"]
       end

       USER --> PYONLY
       USER --> ALGOS
       USER --> CORE

       CART --> OINK
       CART --> TOPPRA
       SINK --> CORE
       OINK --> CORE
       RRT --> CORE
       TOPPRA --> CORE
       CART --> CORE
       SHORT --> SCENE

       OINK --> PROXSUITE
       RRT --> DYNO
       TOPPRA --> TOPPRALIB
       SCENE --> PIN
       SCENE --> COAL
       VIZ --> VISER
       VIZ --> MPL

       subgraph LEGEND["Legend"]
           LCPP["C++ only"]
           LBOTH["C++ with Python bindings"]
           LPY["Python only"]
       end

       EXT ~~~ LPY & LBOTH & LCPP

       classDef cpp fill:#6da7ec,stroke:#00599c,color:#111111
       classDef python fill:#ffd43b,stroke:#8a6d00,color:#111111
       classDef both fill:#6fce89,stroke:#1b5e20,color:#111111
       classDef neutral fill:#ffffff,stroke:#666666,color:#111111

       class LCPP,PROXSUITE,TOPPRALIB,DYNO cpp
       class LPY,VIZ,INTERP,VISER,MPL python
       class LBOTH,CART,SINK,OINK,RRT,TOPPRA,SCENE,TYPES,SHORT,PIN,COAL both
       class USER neutral

Note that all components use `Eigen <https://eigen.tuxfamily.org/>`_ for linear algebra; it is omitted from the diagram for clarity.


Packages
--------

Each C++ package ships its own Python bindings, exposed as a submodule of the ``roboplan`` namespace package.

.. list-table::
   :header-rows: 1
   :widths: 24 24 52

   * - Package
     - Python module
     - Role
   * - ``roboplan``
     - ``roboplan.core``, ``roboplan.filters``
     - ``Scene``, standard data types, collision checking, sampling, path utilities, and signal filters.
   * - ``roboplan_simple_ik``
     - ``roboplan.simple_ik``
     - Damped least-squares inverse kinematics (:doc:`SimpleIK </concepts/inverse_kinematics>`).
   * - ``roboplan_oink``
     - ``roboplan.optimal_ik``
     - QP-based inverse kinematics with tasks, constraints, and barriers (:ref:`OInK <oink-solver>`).
   * - ``roboplan_rrt``
     - ``roboplan.rrt``
     - :doc:`Sampling-based motion planning </concepts/sampling_based_planning>` with RRT.
   * - ``roboplan_toppra``
     - ``roboplan.toppra``
     - Time-optimal :doc:`trajectory generation </concepts/trajectory_generation>` using TOPP-RA.
   * - ``roboplan_cartesian_planning``
     - ``roboplan.cartesian_planning``
     - :doc:`Cartesian path planning </concepts/cartesian_planning>`, composing OInK and TOPP-RA.
   * - ``roboplan_example_models``
     - ``roboplan.example_models``
     - Example robot models (URDF/SRDF) used in examples and tests.
   * - ``roboplan_examples``
     - (scripts only)
     - Runnable C++ and Python examples.


The core package
----------------

The ``Scene`` is the central object in RoboPlan.
It owns the Pinocchio robot model and data, the collision geometry model, and planning-relevant information such as joint groups, joint limits, and dynamic obstacles.
On top of these it provides the queries every algorithm needs: forward kinematics, frame Jacobians, joint limits and groups, collision and distance checks, random and collision-free sampling, and interpolation/integration that respects the configuration space topology.

Algorithms take a ``Scene`` (usually as a ``std::shared_ptr``) and work with the standard data types, though they can also extract the underlying Pinocchio model information directly from the scene as necessary.
Following the design of Pinocchio, the ``Scene`` itself splits into:

- An immutable robot description (the model, the collision geometry, the frame/joint/group lookups)
- Per-query scratch (Pinocchio data, geometry data, the broadphase AABB tree, the RNG, the current configuration).

Only the first half is safe to share, so algorithms can snapshot a private ``SceneContext`` from the ``Scene`` and run every scratch-writing query (e.g., collision checks, forward kinematics, frame Jacobians, random sampling)  against that.
Give each thread its own ``SceneContext`` and any number of algorithms can work against one shared ``Scene`` concurrently; the Python bindings release the GIL for the long-running entry points, so this holds from Python too.
A context also takes advantage of Pinocchio's broadphase manager for faster collision checking.

A ``SceneContext`` carries the configuration an algorithm is working at, which is what keeps that configuration from travelling between subsystems through the shared ``Scene``.
``Scene::setJointPositions`` and ``Scene::getCurrentJointPositions`` remain for interactive and scripting use; library code reads the current positions at most once, on entry to a call, to seed a context.

A context borrows the scene's collision geometry and sizes its own scratch from it when built.
Adding or removing geometry, or changing collision pairs, leaves that scratch stale: the collision queries report the mismatch rather than answering against geometry they were not sized for, while kinematics and sampling are unaffected.
Algorithms can build a context per call pick up the change automatically, or hold one for their lifetime and refresh it as needed.
Moving an existing geometry with ``updateGeometryPlacement`` does not invalidate anything.

The core package also provides post-processing utilities that operate on paths, such as :doc:`path shortcutting </concepts/path_shortcutting>` and uniform resampling.


Algorithm packages
------------------

Each algorithm package builds on the core package, bringing in its own external solver where needed:

- **SimpleIK** iterates a damped least-squares update using Jacobians from the ``Scene``.
- **OInK** formulates IK as a quadratic program over tasks, constraints, and control barrier functions, and solves it with `ProxSuite <https://github.com/Simple-Robotics/proxsuite>`_.
- **RRT** grows search trees in configuration space, using the ``Scene`` for sampling and collision checks and the vendored `dynotree <https://github.com/quimortiz/dynotree>`_ k-d tree for nearest-neighbor lookups.
- **TOPP-RA** wraps the `toppra <https://github.com/hungpham2511/toppra>`_ library to time-parameterize joint paths subject to the velocity and acceleration limits stored in the ``Scene``.
- **CartesianPathPlanner** is the main integration point: it resolves a task-space path into a joint path with an internal OInK solver, then times that path with TOPP-RA.

See the :doc:`Concepts </concepts/index>` section for a detailed description of each algorithm.


Python bindings and visualization
---------------------------------

The bindings are a first-class deliverable (see :doc:`Design Philosophy </design/philosophy>`).
Each package binds its C++ API with `nanobind <https://github.com/wjakob/nanobind>`_ and installs typed stubs, and the per-package modules combine into the single ``roboplan`` namespace package shown in the table above.

Two modules are implemented in pure Python on top of the bindings:

- ``roboplan.visualization`` renders scenes, paths, and trajectories in the browser using `Viser <https://viser.studio/>`_ (through Pinocchio's ``ViserVisualizer``), and plots trajectories with matplotlib.
- ``roboplan.interpolation`` provides helpers for sampling trajectories and Cartesian paths at arbitrary times.

The Python examples in ``roboplan_examples`` tie everything together: they load a model from ``roboplan.example_models``, build a ``Scene``, run a subset of algorithms, and visualize the results.

.. note::

   Because Pinocchio's Python bindings do not yet use nanobind, the ``Scene``'s C++ Pinocchio model cannot be passed directly to Pinocchio's Python API.
   Visualization therefore builds a separate Pinocchio model on the Python side, loaded from the same URDF as the ``Scene``.
   Pinocchio is actively working on switching its bindings to nanobind; once that lands, the Pinocchio model will be extracted directly from the ``Scene``.


ROS integration
---------------

The core library and all algorithm packages run standalone.
Following the :doc:`Design Philosophy </design/philosophy>`, optional ROS 2 wrappers are maintained in the separate `roboplan-ros <https://github.com/open-planning/roboplan-ros>`_ repository,
which builds on the same standard data types and Python bindings.
