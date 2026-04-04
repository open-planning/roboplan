Inverse Kinematics
==================

RoboPlan provides two inverse kinematics solvers: a simple Jacobian-based solver and an optimal solver based on quadratic programming.

SimpleIK: Jacobian-Based Solver
--------------------------------

SimpleIK is a lightweight inverse kinematics solver using the damped least squares (DLS) method, also known as the Levenberg-Marquardt algorithm.

Algorithm
^^^^^^^^^

At each iteration, SimpleIK computes joint velocities that minimize the Cartesian error:

.. math::

   \dot{q} = -J^T (J J^T + \lambda I)^{-1} e

Where:

- :math:`e = \log_6(T_{\text{goal}}^{-1} T_{\text{current}})` — 6D Cartesian error (position + orientation)
- :math:`J` — Frame Jacobian in ``LOCAL`` reference frame
- :math:`\lambda` — Damping factor for regularization

The joint configuration is updated via integration:

.. math::

   q \leftarrow q \oplus (\dot{q} \cdot \text{step\_size})

**Properties:**

- Simple and efficient — minimal computational overhead
- Supports multiple simultaneous goal frames
- Collision checking with random restarts on failure
- Convergence monitoring based on separate linear and angular error thresholds

Configuration
^^^^^^^^^^^^^

+-----------------------------+--------------------------------------+-----------+
| Parameter                   | Description                          | Default   |
+=============================+======================================+===========+
| ``group_name``              | Joint group to control               | ""        |
+-----------------------------+--------------------------------------+-----------+
| ``max_iters``               | Maximum iterations per attempt       | 100       |
+-----------------------------+--------------------------------------+-----------+
| ``max_time``                | Maximum computation time (seconds)   | 0.01      |
+-----------------------------+--------------------------------------+-----------+
| ``max_restarts``            | Number of random restarts on failure | 2         |
+-----------------------------+--------------------------------------+-----------+
| ``step_size``               | Integration step size                | 0.01      |
+-----------------------------+--------------------------------------+-----------+
| ``damping``                 | Damping factor :math:`\lambda`       | 0.001     |
+-----------------------------+--------------------------------------+-----------+
| ``max_linear_error_norm``   | Convergence threshold (meters)       | 0.001     |
+-----------------------------+--------------------------------------+-----------+
| ``max_angular_error_norm``  | Convergence threshold (radians)      | 0.001     |
+-----------------------------+--------------------------------------+-----------+
| ``check_collisions``        | Enable collision checking            | true      |
+-----------------------------+--------------------------------------+-----------+

Usage Example
^^^^^^^^^^^^^

.. code-block:: python

   import numpy as np
   from roboplan.core import Scene, JointConfiguration, CartesianConfiguration
   from roboplan.simple_ik import SimpleIkOptions, SimpleIk

   # Setup
   scene = Scene("robot", urdf_path, srdf_path, package_paths)

   options = SimpleIkOptions(
       group_name="arm",
       step_size=0.25,
       max_linear_error_norm=0.001,
       max_angular_error_norm=0.001,
       check_collisions=True
   )
   ik_solver = SimpleIk(scene, options)

   # Define goal
   goal = CartesianConfiguration()
   goal.base_frame = "base"
   goal.tip_frame = "tool0"
   goal.tform = target_transform  # 4x4 SE(3) matrix

   # Solve
   start = JointConfiguration()
   start.positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # initial configuration
   solution = JointConfiguration()

   success = ik_solver.solveIk(goal, start, solution)

OInK: Optimal Inverse Kinematics
---------------------------------

The OInK solver uses Quadratic Programming (QP) to compute joint displacements that achieve multiple objectives while respecting constraints and safety barriers.

QP Problem Formulation
^^^^^^^^^^^^^^^^^^^^^^

OInK solves the following QP at each control step:

.. math::

   \min_{\Delta q} \quad \underbrace{\frac{1}{2} \sum_{k} \| W_k (J_k \Delta q + \alpha_k e_k) \|^2}_{\text{Tasks}} + \underbrace{\frac{\lambda}{2} \|\Delta q\|^2}_{\text{Regularization}} + \underbrace{\sum_{b} \frac{r_b}{2\|J_b\|^2} \|\Delta q - \Delta q_{\text{safe}}\|^2}_{\text{Barrier Regularization}}

Subject to:

.. math::

   \underbrace{l \leq G_c \Delta q \leq u}_{\text{Hard Constraints}} \quad \text{and} \quad \underbrace{G_b \Delta q \leq h_b}_{\text{Barrier Constraints}}
^^^^^^^^^^^^^^^^

Reformulated as:

.. math::

   \min_{\Delta q} \quad \frac{1}{2} \Delta q^T H \Delta q + c^T \Delta q

Where:

.. math::

   H = \lambda I + \sum_k (J_k^T W_k^T W_k J_k + \mu_k I) + \sum_b \frac{r_b}{\|J_b\|^2} I

.. math::

   c = \sum_k (-\alpha_k J_k^T W_k^T W_k e_k) + \sum_b \frac{-r_b}{\|J_b\|^2} \Delta q_{\text{safe}}

+---------------------+------------------------------------------+--------------+
| Symbol              | Description                              | Source       |
+=====================+==========================================+==============+
| :math:`\Delta q`    | Joint displacement (decision variable)   | —            |
+---------------------+------------------------------------------+--------------+
| :math:`J_k, e_k,`   | Task Jacobian, error, weight matrix      | Tasks        |
| :math:`W_k`         |                                          |              |
+---------------------+------------------------------------------+--------------+
| :math:`\alpha_k`    | Task gain (low-pass filter)              | Tasks        |
+---------------------+------------------------------------------+--------------+
| :math:`\mu_k`       | Levenberg-Marquardt damping              | Tasks        |
+---------------------+------------------------------------------+--------------+
| :math:`\lambda`     | Tikhonov regularization                  | Solver       |
+---------------------+------------------------------------------+--------------+
| :math:`G_c, l, u`   | Hard constraint matrix and bounds        | Constraints  |
+---------------------+------------------------------------------+--------------+
| :math:`G_b, h_b`    | Barrier constraint matrix and bounds     | Barriers     |
+---------------------+------------------------------------------+--------------+
| :math:`r_b, J_b`    | Safe displacement gain and barrier       | Barriers     |
|                     | Jacobian                                 |              |
+---------------------+------------------------------------------+--------------+

Tasks
^^^^^

Tasks define optimization objectives through Jacobians and errors.

FrameTask
"""""""""

Tracks a target 6-DOF pose (position + orientation).

**Error computation:**

.. math::

   e_{\text{pos}} = p_{\text{target}} - p_{\text{current}}

.. math::

   e_{\text{rot}} = R_{\text{current}} \cdot \log_3(R_{\text{current}}^T R_{\text{target}})

**Error saturation** (prevents large jumps that invalidate CBF linearization):

.. math::

   e_{\text{saturated}} = e_{\max} \cdot \tanh\left(\frac{\|e\|}{e_{\max}}\right) \cdot \frac{e}{\|e\|}

**Jacobian:** Frame Jacobian in ``LOCAL_WORLD_ALIGNED`` coordinates, negated so QP moves toward target.

**Weight matrix:**

.. math::

   W = \text{diag}(\sqrt{w_{\text{pos}}} \cdot I_3, \sqrt{w_{\text{rot}}} \cdot I_3)

+--------------------------+-----------------------------------+-----------+
| Parameter                | Description                       | Default   |
+==========================+===================================+===========+
| ``position_cost``        | Position error weight             | 1.0       |
+--------------------------+-----------------------------------+-----------+
| ``orientation_cost``     | Orientation error weight          | 1.0       |
+--------------------------+-----------------------------------+-----------+
| ``task_gain``            | Low-pass filter gain              | 1.0       |
|                          | :math:`\alpha`                    |           |
+--------------------------+-----------------------------------+-----------+
| ``lm_damping``           | Levenberg-Marquardt damping       | 0.0       |
+--------------------------+-----------------------------------+-----------+
| ``max_position_error``   | Saturation limit (meters)         | ∞         |
+--------------------------+-----------------------------------+-----------+
| ``max_rotation_error``   | Saturation limit (radians)        | ∞         |
+--------------------------+-----------------------------------+-----------+

ConfigurationTask
"""""""""""""""""

Drives toward a target joint configuration (null-space regularization).

**Error:** Manifold-aware difference: :math:`e = \text{difference}(q, q_{\text{target}})`

**Jacobian:** :math:`J = -I` (negative identity)

**Weight matrix:** :math:`W = \text{diag}(\sqrt{w_1}, \ldots, \sqrt{w_{n_v}})`

Constraints vs Barriers
^^^^^^^^^^^^^^^^^^^^^^^

Hard Constraints
""""""""""""""""

Hard constraints enforce **strict bounds** that the QP solver cannot violate:

.. math::

   G \cdot \Delta q \leq h

**Properties:**

- Exact enforcement — the solution always satisfies the constraint
- State-independent bounds — same restriction regardless of distance to limit
- No Jacobian needed for joint-space constraints
- QP becomes infeasible if constraints conflict

**Use for:** Physical actuator limits, joint position/velocity bounds

Control Barrier Functions (Barriers)
"""""""""""""""""""""""""""""""""""""

Barriers enforce **forward invariance** of a safe set through a differential condition:

.. math::

   \dot{h}(q) + \alpha(h(q)) \geq 0

In discrete time:

.. math::

   \frac{J_h \cdot \Delta q}{\Delta t} + \alpha(h(q)) \geq 0 \quad \Rightarrow \quad -J_h \cdot \Delta q \leq \Delta t \cdot \alpha(h(q))

**Properties:**

- State-dependent bounds — more freedom far from boundary, tighter near it
- Smooth behavior — graceful slowdown instead of abrupt stop
- Requires Jacobian computation
- Always feasible (soft constraint via class-K function)
- Subject to linearization error in discrete time

**Use for:** Cartesian position bounds, collision avoidance, workspace constraints

Comparison
""""""""""

+------------------------+------------------------------------+--------------------------------------+
| Aspect                 | Hard Constraint                    | Barrier (CBF)                        |
+========================+====================================+======================================+
| **Formulation**        | :math:`\Delta q \leq` constant     | :math:`J \cdot \Delta q \leq`        |
|                        |                                    | :math:`f(\text{distance})`           |
+------------------------+------------------------------------+--------------------------------------+
| **Near boundary**      | Same restriction                   | Tighter (slows down)                 |
+------------------------+------------------------------------+--------------------------------------+
| **Far from boundary**  | Same restriction                   | Looser (more freedom)                |
+------------------------+------------------------------------+--------------------------------------+
| **Enforcement**        | Exact                              | Approximate (linearization)          |
+------------------------+------------------------------------+--------------------------------------+
| **Feasibility**        | Can fail                           | Always feasible                      |
+------------------------+------------------------------------+--------------------------------------+
| **Behavior**           | Abrupt at limit                    | Smooth approach                      |
+------------------------+------------------------------------+--------------------------------------+
| **Computation**        | Simple                             | Requires Jacobian                    |
+------------------------+------------------------------------+--------------------------------------+

Constraint Details
^^^^^^^^^^^^^^^^^^

VelocityLimit
"""""""""""""

Enforces maximum joint velocities as hard bounds:

.. math::

   -\Delta t \cdot v_{\max} \leq \Delta q \leq \Delta t \cdot v_{\max}

PositionLimit
"""""""""""""

Restricts motion based on distance to joint limits:

.. math::

   -\gamma (q - q_{\min}) \leq \Delta q \leq \gamma (q_{\max} - q)

The gain :math:`\gamma \in (0, 1]` controls aggressiveness. As :math:`q \to q_{\max}`, the upper bound :math:`\to 0`.

Barrier Details
^^^^^^^^^^^^^^^

PositionBarrier
"""""""""""""""

Keeps a frame within an axis-aligned bounding box using CBF constraints.

**Barrier function** (for lower bound on axis :math:`i`):

.. math::

   h_i(q) = p_i(q) - p_{\min,i}

**Barrier Jacobian:**

.. math::

   J_{h_i} = J_{\text{frame},i}(q) \quad \text{(row } i \text{ of frame Jacobian)}

**QP constraint** (using saturating class-K function):

.. math::

   -J_{h_i} \cdot \Delta q \leq \Delta t \cdot \gamma \cdot \frac{h_i}{1 + |h_i|} - m

Where:

- :math:`\gamma` — barrier gain (aggressiveness)
- :math:`m` — safety margin (conservative buffer for linearization error)

**Safe displacement regularization** adds to the objective:

.. math::

   \frac{r}{2\|J_h\|^2} \|\Delta q - \Delta q_{\text{safe}}\|^2

This encourages motion toward a safe configuration when near boundaries.

+-------------------------------+-------------------------------------+-----------+
| Parameter                     | Description                         | Default   |
+===============================+=====================================+===========+
| ``gain``                      | Class-K function gain               | 1.0       |
|                               | :math:`\gamma`                      |           |
+-------------------------------+-------------------------------------+-----------+
| ``dt``                        | Control timestep                    | required  |
+-------------------------------+-------------------------------------+-----------+
| ``safe_displacement_gain``    | Regularization weight :math:`r`     | 1.0       |
+-------------------------------+-------------------------------------+-----------+
| ``safety_margin``             | Conservative buffer :math:`m`       | 0.0       |
+-------------------------------+-------------------------------------+-----------+
| ``axis_selection``            | Enable/disable per-axis constraints | all       |
+-------------------------------+-------------------------------------+-----------+

Linearization Error and ``enforceBarriers()``
""""""""""""""""""""""""""""""""""""""""""""""

The CBF constraint is based on a first-order Taylor expansion:

.. math::

   h(q + \Delta q) \approx h(q) + J_h \cdot \Delta q

This has :math:`O(\|\Delta q\|^2)` error. Near boundaries with large commands, the linearized constraint can be satisfied while the actual barrier is violated.

``enforceBarriers()`` provides a post-solve safety check using forward kinematics:

.. code-block:: cpp

   // After solving QP
   oink.solveIk(tasks, constraints, barriers, scene, delta_q);

   // Validate using FK: if h(q + delta_q) < -tolerance, set delta_q = 0
   oink.enforceBarriers(barriers, scene, delta_q, tolerance);

Implementation Notes
^^^^^^^^^^^^^^^^^^^^

Numerical Properties
""""""""""""""""""""

- **Positive definiteness**: Guaranteed by Tikhonov regularization :math:`\lambda I`
- **Convexity**: Quadratic objective + linear constraints → unique global optimum
- **Weight scaling**: Applied as :math:`\sqrt{w}` for better conditioning

Solver
""""""

OInK uses `OSQP <https://osqp.org/>`_ with:

- Dense accumulation of :math:`H` and :math:`c`
- Sparse conversion for solving
- Warm-starting between iterations
- Workspace caching for constraints

Usage Example
^^^^^^^^^^^^^

.. code-block:: python

   import numpy as np
   from roboplan_ext import Scene
   from roboplan_ext.optimal_ik import (
       Oink, FrameTask, FrameTaskOptions,
       VelocityLimit, PositionLimit, PositionBarrier
   )

   # Setup
   scene = Scene("robot", urdf_path, srdf_path, package_paths)
   nv = scene.model.nv
   dt = 0.01

   # Tasks
   task = FrameTask(target_pose, nv, FrameTaskOptions(
       position_cost=1.0,
       orientation_cost=1.0,
       max_position_error=0.1  # Prevents large jumps
   ))

   # Hard constraints (exact enforcement)
   vel_limit = VelocityLimit(nv, dt, v_max=np.ones(nv))
   pos_limit = PositionLimit(nv, gain=0.95)

   # Barriers (smooth task-space safety)
   barrier = PositionBarrier(
       frame_name="tool0",
       p_min=np.array([-0.5, -0.5, 0.1]),
       p_max=np.array([0.5, 0.5, 1.0]),
       num_variables=nv,
       dt=dt,
       gain=5.0,
       safety_margin=0.01
   )

   # Solve
   oink = Oink(nv)
   delta_q = np.zeros(nv)

   oink.solveIk([task], [vel_limit, pos_limit], [barrier], scene, delta_q)
   oink.enforceBarriers([barrier], scene, delta_q)  # FK validation

   q = scene.integrate(q, delta_q)
