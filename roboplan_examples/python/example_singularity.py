#!/usr/bin/env python3

import threading
import time
from typing import Literal

import numpy as np
import pinocchio as pin
import tyro
import xacro
from pinocchio.visualize import ViserVisualizer

from common import get_model_data
from roboplan.core import CartesianConfiguration, Scene
from roboplan.example_models import get_package_share_dir
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskOptions,
    FrameTask,
    FrameTaskOptions,
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

# σ_min below this value is printed as "SINGULAR"
_SINGULAR_THRESHOLD = 0.05


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
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_arm = J[:, v_indices]
    sv = np.linalg.svd(J_arm, compute_uv=False)
    return float(np.prod(sv)), float(sv[-1])


def main(
    singularity: Literal["wrist", "elbow", "shoulder"] = "wrist",
    task_gain: float = 1.0,
    lm_damping: float = 0.01,
    regularization: float = 1e-6,
    control_freq: float = 100.0,
    waypoint_duration: float = 4.0,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Move a UR5 through a kinematic singularity with OInK and display
    the manipulability and minimum Jacobian singular value in real time.

    The trajectory is defined in joint space: at each control step the target
    is recomputed via FK from a linearly-interpolated joint configuration.
    This guarantees OInK must track through the exact singular joint pose.

    Parameters:
        singularity: Singularity type to traverse.
            "wrist"    – joint 5 (wrist_2) passes through 0°, aligning the
                         wrist_1 and wrist_3 rotation axes (rank drops by 1).
            "elbow"    – joint 3 (elbow) approaches 0°, the arm is nearly
                         fully extended (rank drops by 1 locally).
            "shoulder" – shoulder_lift + elbow ≈ -90°, placing the wrist
                         center directly on the shoulder_pan (j1) axis.
                         Rotating j1 no longer displaces the TCP (rank drops).
        task_gain: IK task gain α ∈ (0, 1].
        lm_damping: Levenberg-Marquardt damping λ. Raise this (e.g. 0.1) to
            observe how OInK trades off task accuracy for joint-velocity
            stability near singularities. With lm_damping=0 and a singularity
            the QP solution is still bounded by VelocityLimit, but the robot
            will stall / oscillate rather than drift gracefully.
        regularization: Tikhonov regularization on the QP Hessian diagonal.
        control_freq: Control loop frequency in Hz.
        waypoint_duration: Seconds to travel between adjacent waypoints.
        host: ViserVisualizer host.
        port: ViserVisualizer port.
    """
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
    print(f"\n=== UR5 Singularity Demo: {singularity} singularity ===")
    print(f"Arm joints: {joint_names}")
    print(f"lm_damping={lm_damping}  (raise to observe graceful singularity handling)")
    print()

    model_pin = pin.buildModelFromXML(urdf_xml, mimic=True)
    # Separate data objects — one per thread to avoid races.
    data_man = model_pin.createData()   # manipulability (control thread)
    data_traj = model_pin.createData()  # trajectory FK  (trajectory thread)

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

    # Compute the initial Cartesian target for the frame task.
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

    # Shared state between threads (protected by locks).
    current_target = [initial_target]
    active_segment = [0]
    metrics = [(0.0, 0.0)]  # (manipulability, sigma_min)
    scene_lock = threading.Lock()
    target_lock = threading.Lock()
    running = True

    def control_loop():
        delta_q = np.zeros(num_variables)
        delta_q_full = np.zeros(model_pin.nv)
        while running:
            t0 = time.time()
            with scene_lock:
                q_cur = scene.getCurrentJointPositions()
                with target_lock:
                    frame_task.setTargetFrameTransform(current_target[0])
                try:
                    oink.solveIk(scene, tasks, constraints, delta_q, regularization)
                except RuntimeError as e:
                    delta_q[:] = 0.0
                    print(f"\nWarning: IK solver failed: {e}")
                delta_q_full[:] = 0.0
                delta_q_full[oink.v_indices] = delta_q
                q_cur = scene.integrate(q_cur, delta_q_full)
                scene.setJointPositions(q_cur)
                scene.forwardKinematics(q_cur, "tool0")
                metrics[0] = _arm_jacobian_metrics(
                    model_pin, data_man, q_cur, "tool0", oink.v_indices
                )
                viz.display(q_cur)
            time.sleep(max(0.0, dt - (time.time() - t0)))

    def trajectory_loop():
        """Interpolates linearly in joint space and converts to FK targets each step.

        Joint-space interpolation guarantees the target path passes through the
        exact singular joint configuration, forcing OInK to encounter the rank drop.
        """
        steps = max(1, int(waypoint_duration * control_freq))
        n = len(waypoints_q)
        q_test = q_template.copy()  # pre-allocated, reused each step
        while running:
            for seg in range(n - 1):
                q0 = waypoints_q[seg]
                q1 = waypoints_q[seg + 1]
                for step in range(steps):
                    if not running:
                        return
                    alpha = step / steps
                    # Linear joint-space interpolation (valid for revolute joints).
                    q_test[:] = q_template
                    q_test[oink.q_indices] = (1.0 - alpha) * q0 + alpha * q1
                    pin.forwardKinematics(model_pin, data_traj, q_test)
                    pin.updateFramePlacements(model_pin, data_traj)
                    T_target = data_traj.oMf[ee_frame_id].homogeneous.copy()
                    with target_lock:
                        current_target[0] = T_target
                    active_segment[0] = seg
                    time.sleep(dt)
            active_segment[0] = n - 1
            time.sleep(1.5)  # brief pause before looping back

    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    traj_thread = threading.Thread(target=trajectory_loop, daemon=True)
    ctrl_thread.start()
    traj_thread.start()

    viz.display(q_init)

    labels = _LABELS[singularity]
    try:
        while True:
            seg = active_segment[0]
            label = labels[min(seg, len(labels) - 1)]
            manip, sigma_min = metrics[0]
            near = "  <<< SINGULAR" if sigma_min < _SINGULAR_THRESHOLD else ""
            print(
                f"\r[{singularity}] {label:<44s}  σ_min={sigma_min:.4f}  manip={manip:.3e}{near}",
                end="",
                flush=True,
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        running = False
        print()
        ctrl_thread.join(timeout=1.0)
        traj_thread.join(timeout=1.0)


if __name__ == "__main__":
    tyro.cli(main)
