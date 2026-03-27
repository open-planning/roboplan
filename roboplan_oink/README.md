# OInK: Optimal Inverse Kinematics

Mathematical formulation of the OInK solver, which uses Quadratic Programming (QP) to compute joint displacements that achieve multiple objectives while respecting constraints and safety barriers.

## QP Problem Formulation

OInK solves the following QP at each control step:

$$
\min_{\Delta q} \quad \underbrace{\frac{1}{2} \sum_{k} \| W_k (J_k \Delta q + \alpha_k e_k) \|^2}_{\text{Tasks}} + \underbrace{\frac{\lambda}{2} \|\Delta q\|^2}_{\text{Regularization}} + \underbrace{\sum_{b} \frac{r_b}{2\|J_b\|^2} \|\Delta q - \Delta q_{\text{safe}}\|^2}_{\text{Barrier Regularization}}
$$

Subject to:

$$
\underbrace{l \leq G_c \Delta q \leq u}_{\text{Hard Constraints}} \quad \text{and} \quad \underbrace{G_b \Delta q \leq h_b}_{\text{Barrier Constraints}}
$$

### Standard QP Form

Reformulated as:

$$
\min_{\Delta q} \quad \frac{1}{2} \Delta q^T H \Delta q + c^T \Delta q
$$

Where:

$$
H = \lambda I + \sum_k (J_k^T W_k^T W_k J_k + \mu_k I) + \sum_b \frac{r_b}{\|J_b\|^2} I
$$

$$
c = \sum_k (-\alpha_k J_k^T W_k^T W_k e_k) + \sum_b \frac{-r_b}{\|J_b\|^2} \Delta q_{\text{safe}}
$$

| Symbol | Description | Source |
|--------|-------------|--------|
| $\Delta q$ | Joint displacement (decision variable) | — |
| $J_k, e_k, W_k$ | Task Jacobian, error, weight matrix | Tasks |
| $\alpha_k$ | Task gain (low-pass filter) | Tasks |
| $\mu_k$ | Levenberg-Marquardt damping | Tasks |
| $\lambda$ | Tikhonov regularization | Solver |
| $G_c, l, u$ | Hard constraint matrix and bounds | Constraints |
| $G_b, h_b$ | Barrier constraint matrix and bounds | Barriers |
| $r_b, J_b$ | Safe displacement gain and barrier Jacobian | Barriers |

---

## Tasks

Tasks define optimization objectives through Jacobians and errors.

### FrameTask

Tracks a target 6-DOF pose (position + orientation).

**Error computation:**

$$
e_{\text{pos}} = p_{\text{target}} - p_{\text{current}}
$$

$$
e_{\text{rot}} = R_{\text{current}} \cdot \log_3(R_{\text{current}}^T R_{\text{target}})
$$

**Error saturation** (prevents large jumps that invalidate CBF linearization):

$$
e_{\text{saturated}} = e_{\max} \cdot \tanh\left(\frac{\|e\|}{e_{\max}}\right) \cdot \frac{e}{\|e\|}
$$

**Jacobian:** Frame Jacobian in `LOCAL_WORLD_ALIGNED` coordinates, negated so QP moves toward target.

**Weight matrix:**

$$
W = \text{diag}(\sqrt{w_{\text{pos}}} \cdot I_3, \sqrt{w_{\text{rot}}} \cdot I_3)
$$

| Parameter | Description | Default |
|-----------|-------------|---------|
| `position_cost` | Position error weight | 1.0 |
| `orientation_cost` | Orientation error weight | 1.0 |
| `task_gain` | Low-pass filter gain $\alpha$ | 1.0 |
| `lm_damping` | Levenberg-Marquardt damping | 0.0 |
| `max_position_error` | Saturation limit (meters) | ∞ |
| `max_rotation_error` | Saturation limit (radians) | ∞ |

### ConfigurationTask

Drives toward a target joint configuration (null-space regularization).

**Error:** Manifold-aware difference: $e = \text{difference}(q, q_{\text{target}})$

**Jacobian:** $J = -I$ (negative identity)

**Weight matrix:** $W = \text{diag}(\sqrt{w_1}, \ldots, \sqrt{w_{n_v}})$

---

## Constraints vs Barriers

### Hard Constraints

Hard constraints enforce **strict bounds** that the QP solver cannot violate:

$$
G \cdot \Delta q \leq h
$$

**Properties:**
- Exact enforcement — the solution always satisfies the constraint
- State-independent bounds — same restriction regardless of distance to limit
- No Jacobian needed for joint-space constraints
- QP becomes infeasible if constraints conflict

**Use for:** Physical actuator limits, joint position/velocity bounds

### Control Barrier Functions (Barriers)

Barriers enforce **forward invariance** of a safe set through a differential condition:

$$
\dot{h}(q) + \alpha(h(q)) \geq 0
$$

In discrete time:

$$
\frac{J_h \cdot \Delta q}{\Delta t} + \alpha(h(q)) \geq 0 \quad \Rightarrow \quad -J_h \cdot \Delta q \leq \Delta t \cdot \alpha(h(q))
$$

**Properties:**
- State-dependent bounds — more freedom far from boundary, tighter near it
- Smooth behavior — graceful slowdown instead of abrupt stop
- Requires Jacobian computation
- Always feasible (soft constraint via class-K function)
- Subject to linearization error in discrete time

**Use for:** Cartesian position bounds, collision avoidance, workspace constraints

### Comparison

| Aspect | Hard Constraint | Barrier (CBF) |
|--------|-----------------|---------------|
| **Formulation** | $\Delta q \leq \text{constant}$ | $J \cdot \Delta q \leq f(\text{distance})$ |
| **Near boundary** | Same restriction | Tighter (slows down) |
| **Far from boundary** | Same restriction | Looser (more freedom) |
| **Enforcement** | Exact | Approximate (linearization) |
| **Feasibility** | Can fail | Always feasible |
| **Behavior** | Abrupt at limit | Smooth approach |
| **Computation** | Simple | Requires Jacobian |

---

## Constraint Details

### VelocityLimit

Enforces maximum joint velocities as hard bounds:

$$
-\Delta t \cdot v_{\max} \leq \Delta q \leq \Delta t \cdot v_{\max}
$$

### PositionLimit

Restricts motion based on distance to joint limits:

$$
-\gamma (q - q_{\min}) \leq \Delta q \leq \gamma (q_{\max} - q)
$$

The gain $\gamma \in (0, 1]$ controls aggressiveness. As $q \to q_{\max}$, the upper bound $\to 0$.

---

## Barrier Details

### PositionBarrier

Keeps a frame within an axis-aligned bounding box using CBF constraints.

**Barrier function** (for lower bound on axis $i$):

$$
h_i(q) = p_i(q) - p_{\min,i}
$$

**Barrier Jacobian:**

$$
J_{h_i} = J_{\text{frame},i}(q) \quad \text{(row } i \text{ of frame Jacobian)}
$$

**QP constraint** (using saturating class-K function):

$$
-J_{h_i} \cdot \Delta q \leq \Delta t \cdot \gamma \cdot \frac{h_i}{1 + |h_i|} - m
$$

Where:
- $\gamma$ — barrier gain (aggressiveness)
- $m$ — safety margin (conservative buffer for linearization error)

**Safe displacement regularization** adds to the objective:

$$
\frac{r}{2\|J_h\|^2} \|\Delta q - \Delta q_{\text{safe}}\|^2
$$

This encourages motion toward a safe configuration when near boundaries.

| Parameter | Description | Default |
|-----------|-------------|---------|
| `gain` | Class-K function gain $\gamma$ | 1.0 |
| `dt` | Control timestep | required |
| `safe_displacement_gain` | Regularization weight $r$ | 1.0 |
| `safety_margin` | Conservative buffer $m$ | 0.0 |
| `axis_selection` | Enable/disable per-axis constraints | all |

### Linearization Error and `enforceBarriers()`

The CBF constraint is based on a first-order Taylor expansion:

$$
h(q + \Delta q) \approx h(q) + J_h \cdot \Delta q
$$

This has $O(\|\Delta q\|^2)$ error. Near boundaries with large commands, the linearized constraint can be satisfied while the actual barrier is violated.

`enforceBarriers()` provides a post-solve safety check using forward kinematics:

```cpp
// After solving QP
oink.solveIk(tasks, constraints, barriers, scene, delta_q);

// Validate using FK: if h(q + delta_q) < -tolerance, set delta_q = 0
oink.enforceBarriers(barriers, scene, delta_q, tolerance);
```

## Implementation Notes

### Numerical Properties

- **Positive definiteness**: Guaranteed by Tikhonov regularization $\lambda I$
- **Convexity**: Quadratic objective + linear constraints → unique global optimum
- **Weight scaling**: Applied as $\sqrt{w}$ for better conditioning

### Solver

OInK uses [OSQP](https://osqp.org/) with:
- Dense accumulation of $H$ and $c$
- Sparse conversion for solving
- Warm-starting between iterations
- Workspace caching for constraints

---

## Usage Example

```python
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
```

---

## TODO

- Support floating base robots (`nq != nv`) in position limits
- Hierarchical task prioritization via null-space projection
- Adaptive LM damping near singularities
- Constraint scaling for mixed constraint types
- Self-collision avoidance barrier
- Singularity avoidance barrier

## Acknowledgments

This project is inspired by and adapts code and concepts from the following fantastic open-source libraries:
- [Pink](https://github.com/stephane-caron/pink) by Stéphane Caron.
- [OSCBF](https://github.com/StanfordASL/oscbf) by Daniel Morton and Marco Pavone.
- [Placo](https://github.com/Rhoban/placo) by Marc Duclusaud, Grégoire Passault, Vincent Padois and Olivier Ly.

A sincere appreciation goes out to the original authors for their work and dedication in creating such valuable resources!
