#!/usr/bin/env python3

import threading
import time
from collections import deque
from typing import Literal

import numpy as np
import pinocchio as pin
import plotly.graph_objects as go
import tyro
import xacro
from plotly.subplots import make_subplots
from pinocchio.visualize import ViserVisualizer

from common import get_model_data
from roboplan.core import CartesianConfiguration, Scene
from roboplan.example_models import get_package_share_dir
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskOptions,
    FrameTask,
    FrameTaskOptions,
    ManipulabilityBarrier,
    Oink,
    PositionLimit,
    VelocityLimit,
)

# Joint-space waypoints for each singularity type (UR5, 6 arm joints).
# The trajectory thread interpolates linearly between them in joint space and
# calls pinocchio FK to obtain the Cartesian target at each step.  This forces
# OInK to track a path whose pre-image includes the singular configuration.
_WAYPOINTS: dict[str, list[np.ndarray]] = {
    "wrist": [
        # Home: wrist_2 at -90° — well away from singularity
        np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]),
        # Approach: wrist_2 at -45°
        np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 4, np.pi / 4]),
        # Wrist singularity: wrist_2 = 0 → wrist_1 and wrist_3 axes align, rank drops by 1
        np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, 0.0, np.pi / 2]),
        # Depart: wrist_2 at +45°
        np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 4, np.pi / 4]),
        # Past singularity: wrist_2 at +90°
        np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, 0.0]),
    ],
    "elbow": [
        # Start: elbow at 90° — arm clearly bent
        np.array([0.0, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, 0.0]),
        # Extending: elbow at 45°
        np.array([0.0, -np.pi / 8, np.pi / 4, -np.pi / 8, -np.pi / 2, 0.0]),
        # Elbow singularity: elbow ≈ 0 → arm nearly fully extended, rank drops by 1
        np.array([0.0, -0.05, 0.1, -0.05, -np.pi / 2, 0.0]),
        # Bending back: elbow at 45°
        np.array([0.0, -np.pi / 8, np.pi / 4, -np.pi / 8, -np.pi / 2, 0.0]),
        # Return: elbow at 90°
        np.array([0.0, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, 0.0]),
    ],
    "shoulder": [
        # Start: j1=+45°, arm angled — wrist center clearly off j1 axis
        np.array([np.pi / 4, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]),
        # Approach: j1 sweeping toward 0, arm rising
        np.array([np.pi / 8, -2 * np.pi / 3, np.pi / 3, np.pi / 3, -np.pi / 2, 0.0]),
        # Shoulder singularity: shoulder_lift + elbow ≈ -π/2 → wrist center on j1 axis.
        # Rotating j1 no longer changes the wrist center position, rank drops by 1.
        np.array([0.0, -3 * np.pi / 4, np.pi / 4, np.pi / 2, -np.pi / 2, 0.0]),
        # Depart: j1 sweeping past, arm lowering
        np.array([-np.pi / 8, -2 * np.pi / 3, np.pi / 3, np.pi / 3, -np.pi / 2, 0.0]),
        # End: j1=-45°, arm angled — wrist center off j1 axis again
        np.array([-np.pi / 4, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]),
    ],
}

_LABELS: dict[str, list[str]] = {
    "shoulder": [
        "Start (j1=+45°, arm angled)",
        "Approach (arm rising, j1→0)",
        "AT SHOULDER SINGULARITY (wrist on j1 axis)",
        "Depart (arm lowering, j1→-45°)",
        "End (j1=-45°, arm angled)",
    ],
    "wrist": [
        "Home (wrist_2=-90°)",
        "Approach (wrist_2=-45°)",
        "AT WRIST SINGULARITY (wrist_2=0°)",
        "Depart (wrist_2=+45°)",
        "Past singularity (wrist_2=+90°)",
    ],
    "elbow": [
        "Elbow bent (90°)",
        "Extending (45°)",
        "AT ELBOW SINGULARITY (~0°)",
        "Bending back (45°)",
        "Return (90°)",
    ],
}

_SINGULAR_THRESHOLD = 0.05
_PLOT_HZ = 10  # sidebar plot refresh rate
_PLOT_WINDOW_S = 15.0  # seconds of history shown in plots
_JOINT_COLORS = ("#e74c3c", "#e67e22", "#f0c030", "#2ecc71", "#3498db", "#9b59b6")


def _arm_jacobian_metrics(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    frame_name: str,
    v_indices: np.ndarray,
) -> tuple[float, float]:
    """Returns (manipulability, sigma_min) for the arm-joint Jacobian at `frame_name`.

    manipulability = product of all singular values = sqrt(det(J J^T))
    sigma_min      = smallest singular value — goes to 0 at a singularity
    """
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    frame_id = model.getFrameId(frame_name)
    J = pin.getFrameJacobian(
        model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )
    J_arm = J[:, v_indices]
    sv = np.linalg.svd(J_arm, compute_uv=False)
    return float(np.prod(sv)), float(sv[-1])


def main(
    singularity: Literal["wrist", "elbow", "shoulder"] = "wrist",
    task_gain: float = 1.0,
    lm_damping: float = 0.001,
    no_damping: bool = False,
    regularization: float = 1e-6,
    control_freq: float = 100.0,
    waypoint_duration: float = 5.0,
    manipulability_barrier: bool = False,
    sigma_safe: float = 0.1,
    barrier_gain: float = 10.0,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Move a UR5 through a kinematic singularity with OInK and display real-time
    joint-space tracking error and Jacobian metrics in the Viser sidebar.
    The trajectory ping-pongs continuously between waypoints.

    The trajectory is defined in joint space: at each control step the target is
    recomputed via FK from a linearly-interpolated joint configuration.  This
    guarantees OInK must track through the exact singular joint pose.

    Parameters:
        singularity: Which singularity to traverse.
            "wrist"    – wrist_2 (j5) through 0°, wrist_1/wrist_3 axes align.
            "elbow"    – elbow (j3) near 0°, arm nearly fully extended.
            "shoulder" – shoulder_lift+elbow≈-90°, wrist center on j1 axis.
        task_gain: IK task gain α ∈ (0, 1].
        lm_damping: Levenberg-Marquardt damping λ added to each task's Hessian
            (H = Jᵀ J + λ I). Prevents joint-velocity blow-up near rank loss at
            the cost of task-tracking accuracy. Set to 0 via --no_damping.
        no_damping: Disable all regularization (forces lm_damping=0,
            regularization=1e-12). Near the singularity the QP gradient is
            ill-conditioned: VelocityLimit still prevents divergence, but the
            commanded direction becomes poorly aligned with the task and the
            robot stalls or oscillates. Watch σ_min drop while the joints chatter.
        regularization: Tikhonov regularization on the full QP Hessian diagonal.
        control_freq: Control loop frequency in Hz.
        waypoint_duration: Seconds to travel between adjacent waypoints.
        manipulability_barrier: Enable CBF-based singularity avoidance. After
            each OInK solve the resulting delta_q is projected onto the half-space
            defined by ∇σ_min · Δq ≥ −γ·dt·(σ_min − σ_safe), preventing σ_min
            from decreasing faster than the barrier allows.  The robot will stall
            before reaching the singular configuration rather than passing through.
        sigma_safe: Minimum σ_min the barrier enforces (metres/rad scale of the
            Jacobian).  Tune upward if the robot still gets close to the
            singularity; lower values allow closer approach.
        barrier_gain: Class-K gain γ controlling how aggressively the barrier
            pushes back. Higher values give a stiffer wall.
        host: ViserVisualizer host.
        port: ViserVisualizer port.
    """
    if no_damping:
        lm_damping = 0.0
        regularization = 1e-12

    model_data = get_model_data()["ur5"]
    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "singularity_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )

    joint_names = scene.getJointGroupInfo(model_data.default_joint_group).joint_names
    print(f"\n=== UR5 Singularity Demo: {singularity} ===")
    print(f"Arm joints : {joint_names}")
    print(f"lm_damping : {lm_damping}" + ("  (DISABLED)" if no_damping else ""))
    print(f"regulariz. : {regularization}")
    if manipulability_barrier:
        print(f"CBF barrier: ON  σ_safe={sigma_safe}  gain={barrier_gain}")
    else:
        print("CBF barrier: OFF")
    print()

    model_pin = pin.buildModelFromXML(urdf_xml, mimic=True)
    # Two separate Data objects — one per thread — to avoid races.
    data_man = model_pin.createData()  # manipulability (control thread)
    data_traj = model_pin.createData()  # FK for trajectory  (trajectory thread)

    collision_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    viz = ViserVisualizer(model_pin, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    oink = Oink(scene, model_data.default_joint_group)
    num_variables = len(oink.v_indices)
    dt = 1.0 / control_freq

    position_limit = PositionLimit(oink, gain=1.0)
    v_max = np.hstack(
        [scene.getJointInfo(name).limits.max_velocity for name in joint_names]
    )
    velocity_limit = VelocityLimit(oink, dt, v_max)
    constraints = [position_limit, velocity_limit]

    waypoints_q = _WAYPOINTS[singularity]
    ee_frame_id = model_pin.getFrameId("tool0")

    # Initialise the scene at the first waypoint.
    q_template = scene.getCurrentJointPositions().copy()
    q_init = q_template.copy()
    q_init[oink.q_indices] = waypoints_q[0]
    scene.setJointPositions(q_init)
    scene.forwardKinematics(q_init, "tool0")

    # Compute the initial Cartesian target via pinocchio (no scene state change).
    pin.forwardKinematics(model_pin, data_traj, q_init)
    pin.updateFramePlacements(model_pin, data_traj)
    initial_target = data_traj.oMf[ee_frame_id].homogeneous.copy()

    config_task = ConfigurationTask(
        oink,
        waypoints_q[0],
        np.full(num_variables, 0.05),
        ConfigurationTaskOptions(task_gain=0.1, lm_damping=0.0),
    )

    goal = CartesianConfiguration()
    goal.base_frame = model_data.base_link
    goal.tip_frame = "tool0"
    frame_task = FrameTask(
        oink,
        scene,
        goal,
        FrameTaskOptions(
            position_cost=1.0,
            orientation_cost=0.1,
            task_gain=task_gain,
            lm_damping=lm_damping,
        ),
    )
    tasks = [frame_task, config_task]

    # ── Sidebar plots ────────────────────────────────────────────────────────
    buf_len = int(_PLOT_WINDOW_S * _PLOT_HZ)
    t_buf: deque[float] = deque(maxlen=buf_len)
    err_bufs: list[deque[float]] = [deque(maxlen=buf_len) for _ in range(num_variables)]
    sigma_buf: deque[float] = deque(maxlen=buf_len)
    buf_lock = threading.Lock()

    def _make_figure(
        t_arr: np.ndarray,
        e_arrays: tuple,
        s_arr: np.ndarray,
    ) -> go.Figure:
        fig = make_subplots(
            rows=1,
            cols=2,
            subplot_titles=("Joint Error [rad]", "σ_min"),
            horizontal_spacing=0.12,
        )
        for i, (name, color) in enumerate(zip(joint_names, _JOINT_COLORS)):
            fig.add_trace(
                go.Scatter(
                    x=t_arr,
                    y=e_arrays[i],
                    name=name,
                    showlegend=True,
                    line=dict(color=color, width=2),
                    legendgroup=name,
                ),
                row=1,
                col=1,
            )
        fig.add_trace(
            go.Scatter(
                x=t_arr,
                y=s_arr,
                name="σ_min",
                line=dict(color="#e74c3c", width=2.5),
                showlegend=False,
            ),
            row=1,
            col=2,
        )
        fig.add_trace(
            go.Scatter(
                x=t_arr,
                y=np.full_like(s_arr, _SINGULAR_THRESHOLD),
                name="singular threshold",
                line=dict(color="#7f8c8d", dash="dash", width=1.5),
                showlegend=False,
            ),
            row=1,
            col=2,
        )
        if manipulability_barrier:
            fig.add_trace(
                go.Scatter(
                    x=t_arr,
                    y=np.full_like(s_arr, sigma_safe),
                    name="σ_safe (barrier)",
                    line=dict(color="#f39c12", dash="dot", width=2),
                    showlegend=False,
                ),
                row=1,
                col=2,
            )
        _axis = dict(
            showgrid=True,
            gridcolor="rgba(255,255,255,0.07)",
            gridwidth=1,
            zeroline=True,
            zerolinecolor="rgba(255,255,255,0.15)",
            zerolinewidth=1,
            linecolor="rgba(255,255,255,0.2)",
            tickfont=dict(size=11, color="#aaaaaa"),
            title_font=dict(size=12, color="#cccccc"),
        )
        fig.update_layout(
            template="plotly_dark",
            height=1400,
            width=1400,
            margin=dict(l=60, r=30, t=60, b=120),
            paper_bgcolor="#1a1a2e",
            plot_bgcolor="#16213e",
            font=dict(family="Inter, sans-serif", size=12, color="#cccccc"),
            legend=dict(
                orientation="h",
                y=-0.18,
                x=0.0,
                font=dict(size=11, color="#cccccc"),
                bgcolor="rgba(0,0,0,0)",
            ),
        )
        fig.update_annotations(font=dict(size=13, color="#ffffff"))
        fig.update_xaxes(**_axis, title_text="time [s]", type="linear")
        fig.update_yaxes(**_axis)
        return fig

    plot_handle = viz.viewer.gui.add_plotly(
        figure=_make_figure(
            np.array([0.0]),
            tuple(np.zeros(1) for _ in range(num_variables)),
            np.array([1.0]),
        ),
        aspect=1.4,
    )
    # ─────────────────────────────────────────────────────────────────────────

    # Shared state between threads.
    current_target = [initial_target]
    current_q_arm_target = [
        waypoints_q[0].copy()
    ]  # joint-space reference for error plot
    active_segment = [0]
    metrics = [(0.0, 0.0, False)]  # (manipulability, sigma_min, barrier_active)
    scene_lock = threading.Lock()
    target_lock = threading.Lock()
    running = True

    # Build the C++ barrier list once — passed into every solveIk call.
    barriers = []
    if manipulability_barrier:
        barriers.append(
            ManipulabilityBarrier(
                oink,
                scene,
                "tool0",
                dt,
                sigma_safe,
                gain=barrier_gain,
                safe_displacement_gain=1.0,
                safety_margin=0.0,
            )
        )

    plot_downsample = max(1, round(control_freq / _PLOT_HZ))
    tick = [0]
    t_start = time.time()

    def control_loop():
        delta_q = np.zeros(num_variables)
        delta_q_full = np.zeros(model_pin.nv)
        while running:
            t0 = time.time()
            with scene_lock:
                q_cur = scene.getCurrentJointPositions()
                with target_lock:
                    frame_task.setTargetFrameTransform(current_target[0])
                    q_arm_target = current_q_arm_target[0].copy()
                try:
                    oink.solveIk(
                        scene, tasks, constraints, barriers, delta_q, regularization
                    )
                except RuntimeError as e:
                    delta_q[:] = 0.0
                    print(f"\nWarning: IK solver failed: {e}")

                delta_q_full[:] = 0.0
                delta_q_full[oink.v_indices] = delta_q
                if barriers:
                    oink.enforceBarriers(scene, barriers, delta_q_full, tolerance=0.0)

                q_cur = scene.integrate(q_cur, delta_q_full)
                scene.setJointPositions(q_cur)
                scene.forwardKinematics(q_cur, "tool0")
                manip, sigma_min = _arm_jacobian_metrics(
                    model_pin, data_man, q_cur, "tool0", oink.v_indices
                )
                metrics[0] = (
                    manip,
                    sigma_min,
                    manipulability_barrier and sigma_min < sigma_safe,
                )

                # Downsample into plot buffers.
                tick[0] += 1
                if tick[0] % plot_downsample == 0:
                    t_now = time.time() - t_start
                    q_arm = q_cur[oink.q_indices]
                    q_err = q_arm - q_arm_target
                    with buf_lock:
                        t_buf.append(t_now)
                        for i in range(num_variables):
                            err_bufs[i].append(float(q_err[i]))
                        sigma_buf.append(sigma_min)

                viz.display(q_cur)
            time.sleep(max(0.0, dt - (time.time() - t0)))

    def trajectory_loop():
        """Ping-pongs through waypoints in joint space, computing FK targets each step.

        Joint-space interpolation guarantees the target path passes through the
        exact singular joint configuration, forcing OInK to encounter the rank drop.
        Ping-pong (forward then reverse) gives seamless continuous looping.
        """
        steps = max(1, int(waypoint_duration * control_freq))
        n = len(waypoints_q)
        q_test = q_template.copy()  # pre-allocated, reused each step

        # Alternate between forward (0→n-1) and reverse (n-1→0) passes.
        while running:
            for forward in (True, False):
                seg_range = range(n - 1) if forward else range(n - 2, -1, -1)
                for seg in seg_range:
                    q0 = waypoints_q[seg if forward else seg + 1]
                    q1 = waypoints_q[seg + 1 if forward else seg]
                    for step in range(steps):
                        if not running:
                            return
                        alpha = step / steps
                        q_arm = (1.0 - alpha) * q0 + alpha * q1
                        q_test[:] = q_template
                        q_test[oink.q_indices] = q_arm
                        pin.forwardKinematics(model_pin, data_traj, q_test)
                        pin.updateFramePlacements(model_pin, data_traj)
                        T_target = data_traj.oMf[ee_frame_id].homogeneous.copy()
                        with target_lock:
                            current_target[0] = T_target
                            current_q_arm_target[0] = q_arm.copy()
                        active_segment[0] = seg
                        time.sleep(dt)

    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    traj_thread = threading.Thread(target=trajectory_loop, daemon=True)
    ctrl_thread.start()
    traj_thread.start()

    viz.display(q_init)

    labels = _LABELS[singularity]
    n_buf = 0
    try:
        while True:
            # Snapshot buffers and push to plots.
            with buf_lock:
                n_buf = len(t_buf)
                if n_buf >= 2:
                    t_arr = np.array(t_buf)
                    e_arrays = tuple(
                        np.array(err_bufs[i]) for i in range(num_variables)
                    )
                    s_arr = np.array(sigma_buf)

            if n_buf >= 2:
                plot_handle.figure = _make_figure(t_arr, e_arrays, s_arr)

            # Terminal status line.
            seg = active_segment[0]
            label = labels[min(seg, len(labels) - 1)]
            manip, sigma_min, barrier_active = metrics[0]
            tag = ""
            if barrier_active:
                tag = "  <<< BARRIER"
            elif sigma_min < _SINGULAR_THRESHOLD:
                tag = "  <<< SINGULAR"
            print(
                f"\r[{singularity}] {label:<44s}  σ_min={sigma_min:.4f}  manip={manip:.3e}{tag}",
                end="",
                flush=True,
            )
            time.sleep(1.0 / _PLOT_HZ)
    except KeyboardInterrupt:
        running = False
        print()
        ctrl_thread.join(timeout=1.0)
        traj_thread.join(timeout=1.0)


if __name__ == "__main__":
    tyro.cli(main)
