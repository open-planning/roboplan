"""
Track mock learned policy action chunks.
This example demonstrates how a learned policy action chunk can be treated as a
short-horizon command sequence, interpolated at a smaller control timestep, and
tracked through OInK for Cartesian actions or directly integrated for joint-space actions.

Supported chunk types:
  1. Cartesian/end-effector targets: sparse absolute SE(3) target poses
  2. Joint-space targets: sparse absolute joint configurations

The main idea is:
  sparse policy-like action chunk
      -> sparse target poses/configurations
      -> dense interpolated targets at control_dt
      -> Cartesian actions: OInK tracking with PositionLimit + VelocityLimit
      -> joint-space actions: direct interpolated configuration trajectory
      -> visualization

For multi-arm models such as "dual", Cartesian chunks create one frame task per
end effector, and joint-space chunks visualize trails for all configured end
effectors.
"""

import sys
import time
from typing import Literal

import numpy as np
import pinocchio as pin
import tyro
import xacro
from pinocchio.visualize import ViserVisualizer

from common import get_home_configuration, get_model_data
from roboplan.core import (
    CartesianConfiguration,
    CartesianTrajectory,
    JointTrajectory,
    Scene,
)
from roboplan.example_models import get_package_share_dir
from roboplan.interpolation import (
    interpolateCartesianTrajectory,
    interpolateJointTrajectory,
)
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskOptions,
    FrameTask,
    FrameTaskOptions,
    Oink,
    PositionLimit,
    VelocityLimit,
)
from roboplan.visualization import visualizePositionTrace

ActionSpace = Literal["cartesian", "joint"]


def make_mock_cartesian_trajectory(
    scene: Scene,
    q_start: np.ndarray,
    ee_frame_names: list[str],
    base_frame: str,
    horizon: int,
    segment_time: float,
    translation_scale: float = 0.04,
    action_scale: float = 1.0,
) -> CartesianTrajectory:
    """Create a sparse Cartesian target trajectory for all EE frames.

    Args:
        scene: RoboPlan scene used to compute starting end-effector poses.
        q_start: Full starting robot configuration.
        ee_frame_names: End-effector frame names for the trajectory.
        base_frame: Reference frame for the Cartesian trajectory.
        horizon: Number of sparse target poses after the starting pose.
        segment_time: Duration between consecutive sparse Cartesian waypoints, in seconds.
        translation_scale: Per-step translation scale in meters.
        action_scale: Scale applied to the mock Cartesian target paths.

    Returns:
        Sparse Cartesian trajectory containing one transform sequence per EE frame.
    """
    tforms_by_frame = []

    for ee_idx, ee_frame_name in enumerate(ee_frame_names):
        start_pose = scene.forwardKinematics(q_start, ee_frame_name)
        start_rotation = start_pose[:3, :3]
        start_translation = start_pose[:3, 3]

        lateral_offset = (
            0.03 * (ee_idx - 0.5 * (len(ee_frame_names) - 1))
            if len(ee_frame_names) > 1
            else 0.0
        )

        targets = [start_pose.copy()]

        for i in range(horizon):
            phase = (i + 1) / float(horizon)

            local_translation_offset = action_scale * np.array(
                [
                    translation_scale * (i + 1),
                    0.012 * np.sin(np.pi * phase),
                    -0.010 * np.sin(0.5 * np.pi * phase),
                ],
                dtype=float,
            )

            if lateral_offset:
                local_translation_offset += np.array(
                    [0.0, lateral_offset * phase, 0.0],
                    dtype=float,
                )

            target = np.eye(4)
            target[:3, :3] = start_rotation
            target[:3, 3] = (
                start_translation + start_rotation @ local_translation_offset
            )
            targets.append(target)

        tforms_by_frame.append(targets)

    return CartesianTrajectory(
        base_frames=[base_frame] * len(ee_frame_names),
        tip_frames=ee_frame_names,
        times=[idx * segment_time for idx in range(horizon + 1)],
        tforms=tforms_by_frame,
    )


def make_mock_joint_targets(
    scene: Scene,
    q_start: np.ndarray,
    v_indices: np.ndarray,
    num_velocity_variables: int,
    horizon: int,
    joint_delta_scale: float = 0.05,
    action_scale: float = 1.0,
) -> list[np.ndarray]:
    """Create sparse absolute joint-space targets.

    Args:
        scene: RoboPlan scene used to integrate joint offsets.
        q_start: Full starting robot configuration.
        v_indices: Velocity indices for the selected joint group.
        num_velocity_variables: Size of the full robot velocity/tangent vector.
        horizon: Number of sparse target configurations after the start.
        joint_delta_scale: Scale of the joint-space target offsets.
        action_scale: Global scale applied on top of joint_delta_scale.

    Returns:
        Sparse full-configuration joint targets.
    """
    targets = [q_start.copy()]
    delta_full = np.zeros(num_velocity_variables)

    for i in range(horizon):
        phase = (i + 1) / float(horizon)
        direction = np.sin(np.linspace(0.0, np.pi, len(v_indices)) + np.pi * phase)
        delta_full[v_indices] = action_scale * joint_delta_scale * (i + 1) * direction
        targets.append(scene.integrate(q_start, delta_full))

    return targets


def compute_cartesian_trajectory_positions(
    trajectory: CartesianTrajectory,
) -> dict[str, np.ndarray]:
    """Extract xyz positions from a Cartesian trajectory.

    Args:
        trajectory: Cartesian trajectory to extract positions from.

    Returns:
        Dictionary mapping tip frame names to position arrays.
    """
    return {
        tip_frame: np.array([tform[:3, 3].copy() for tform in tforms])
        for tip_frame, tforms in zip(trajectory.tip_frames, trajectory.tforms)
    }


def compute_end_effector_positions(
    scene: Scene,
    configurations: list[np.ndarray],
    ee_frame_names: list[str],
) -> dict[str, np.ndarray]:
    """Compute end-effector xyz positions for multiple frames.

    Args:
        scene: RoboPlan scene used for forward kinematics.
        configurations: Joint configurations to evaluate.
        ee_frame_names: End-effector frame names.

    Returns:
        Dictionary mapping end-effector frame names to position arrays.
    """
    return {
        name: np.array(
            [scene.forwardKinematics(q, name)[:3, 3].copy() for q in configurations]
        )
        for name in ee_frame_names
    }


def visualize_ee_traces(
    viz,
    ee_frame_name: str,
    sparse_pos: np.ndarray,
    dense_pos: np.ndarray,
    executed_pos: np.ndarray,
    executed_color: tuple[int, int, int],
) -> None:
    """Visualize sparse waypoints, dense references, and executed trajectory for one EE.

    Args:
        viz: ViserVisualizer instance.
        ee_frame_name: End-effector frame name used for scene path naming.
        sparse_pos: Sparse policy waypoint positions.
        dense_pos: Dense interpolated target positions.
        executed_pos: Executed end-effector trajectory positions.
        executed_color: RGB color for the executed trajectory trace.
    """
    visualizePositionTrace(
        viz,
        sparse_pos,
        trace_name=f"/action_chunk/{ee_frame_name}/sparse_policy_waypoints/trace",
        waypoint_root=f"/action_chunk/{ee_frame_name}/sparse_policy_waypoints/markers",
        trace_color=(255, 160, 0),
        waypoint_color=(255, 160, 0),
        line_width=1.0,
        waypoint_radius=0.006,
        draw_trace=False,
        draw_waypoints=True,
    )

    visualizePositionTrace(
        viz,
        dense_pos,
        trace_name=f"/action_chunk/{ee_frame_name}/dense_interpolated_targets/trace",
        waypoint_root=f"/action_chunk/{ee_frame_name}/dense_interpolated_targets/markers",
        trace_color=(90, 90, 90),
        waypoint_color=(90, 90, 90),
        line_width=3.0,
        waypoint_radius=0.004,
        draw_trace=True,
        draw_waypoints=True,
        waypoint_stride=10,
    )

    visualizePositionTrace(
        viz,
        executed_pos,
        trace_name=f"/action_chunk/{ee_frame_name}/executed_trace/trace",
        waypoint_root=f"/action_chunk/{ee_frame_name}/executed_trace/markers",
        trace_color=executed_color,
        waypoint_color=executed_color,
        line_width=10.0,
        waypoint_radius=0.01,
        draw_trace=True,
        draw_waypoints=False,
    )


def main(
    model: str = "ur5",
    action_space: ActionSpace = "cartesian",
    chunk_horizon: int = 6,
    action_scale: float = 1.0,
    segment_time: float = 0.2,
    control_freq: float = 100.0,
    task_gain: float = 1.0,
    lm_damping: float = 0.01,
    regularization: float = 1e-6,
    sleep: bool = False,
    playback_speed: float = 1.0,
    host: str = "localhost",
    port: str = "8000",
):
    """Track a mock policy action chunk.

    Args:
        model: Robot model name from roboplan_examples/python/common.py.
        action_space: Whether to use Cartesian or joint-space mock action chunks.
        chunk_horizon: Number of sparse targets in the mock policy-like chunk.
        action_scale: Scale applied to the mock target chunk to make the motion shorter or longer.
        segment_time: Duration between consecutive sparse action waypoints, in seconds.
        control_freq: Dense interpolation/tracking frequency, in Hz.
        task_gain: Cartesian OInK task gain.
        lm_damping: Cartesian frame-task Levenberg-Marquardt damping.
        regularization: Tikhonov regularization passed to OInK in Cartesian mode.
        sleep: If true, sleep between dense tracking steps while initially generating the trajectory.
        playback_speed: Playback speed multiplier for GUI animation.
        host: Viser host.
        port: Viser port.
    """
    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        print(f"Available models: {list(get_model_data().keys())}")
        sys.exit(1)

    if control_freq <= 0.0:
        raise ValueError("control_freq must be positive.")
    if segment_time <= 0.0:
        raise ValueError("segment_time must be positive.")
    if playback_speed <= 0.0:
        raise ValueError("playback_speed must be positive.")

    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "policy_action_chunk_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )

    joint_group = model_data.default_joint_group
    joint_names = scene.getJointGroupInfo(joint_group).joint_names

    print(f"\n=== Model: {model} ===")
    print(f"Joint group: {joint_group}")
    print(f"Joint names: {joint_names}")
    print(f"Action space: {action_space}")
    print(f"Action scale: {action_scale}")

    # Create a redundant Pinocchio model just for visualization with mimic joints.
    model_pin = pin.buildModelFromXML(urdf_xml, mimic=True)
    q_start = get_home_configuration(scene, model_data)

    collision_model = pin.buildGeomFromUrdfString(
        model_pin,
        urdf_xml,
        pin.GeometryType.COLLISION,
        package_dirs=package_paths,
    )
    visual_model = pin.buildGeomFromUrdfString(
        model_pin,
        urdf_xml,
        pin.GeometryType.VISUAL,
        package_dirs=package_paths,
    )

    viz = ViserVisualizer(model_pin, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)
    viz.display(q_start)

    scene.setJointPositions(q_start)

    dt = 1.0 / control_freq
    animation_dt = dt / playback_speed

    print(f"Dense control dt: {dt:.4f} s")
    print(f"Sparse segment time: {segment_time:.4f} s")
    print(
        f"Interpolation intervals per segment: {max(1, int(np.ceil(segment_time / dt)))}"
    )

    ee_frame_names = model_data.ee_names
    if not ee_frame_names:
        raise ValueError(f"Model '{model}' has no configured end-effectors.")
    print(f"End-effectors: {ee_frame_names}")

    if action_space == "cartesian":
        oink = Oink(scene, joint_group)
        num_variables = len(oink.v_indices)

        v_max = np.hstack(
            [scene.getJointInfo(name).limits.max_velocity for name in joint_names]
        )
        constraints = [
            PositionLimit(oink, gain=1.0),
            VelocityLimit(oink, dt, v_max),
        ]

        print(f"Velocity variables: {num_variables}")

        joint_weights = np.full(num_variables, 1e-4)
        config_options = ConfigurationTaskOptions(task_gain=1e-4, lm_damping=0.0)
        config_task = ConfigurationTask(
            oink,
            q_start[oink.q_indices],
            joint_weights,
            config_options,
        )

        task_options = FrameTaskOptions(
            position_cost=1.0,
            orientation_cost=0.1,
            task_gain=task_gain,
            lm_damping=lm_damping,
        )

        frame_tasks = []
        for ee_frame_name in ee_frame_names:
            goal = CartesianConfiguration()
            goal.base_frame = model_data.base_link
            goal.tip_frame = ee_frame_name
            frame_tasks.append(FrameTask(oink, scene, goal, task_options))

        sparse_cartesian_trajectory = make_mock_cartesian_trajectory(
            scene,
            q_start,
            ee_frame_names,
            model_data.base_link,
            chunk_horizon,
            segment_time,
            action_scale=action_scale,
        )
        dense_cartesian_trajectory = interpolateCartesianTrajectory(
            sparse_cartesian_trajectory,
            dt,
        )

        sparse_target_positions_by_frame = compute_cartesian_trajectory_positions(
            sparse_cartesian_trajectory
        )
        dense_target_positions_by_frame = compute_cartesian_trajectory_positions(
            dense_cartesian_trajectory
        )

        tasks = [*frame_tasks, config_task]

    else:  # Joint action space
        joint_group_info = scene.getJointGroupInfo(joint_group)
        joint_velocity_indices = np.asarray(joint_group_info.v_indices)

        sparse_targets = make_mock_joint_targets(
            scene,
            q_start,
            joint_velocity_indices,
            model_pin.nv,
            chunk_horizon,
            action_scale=action_scale,
        )
        sparse_trajectory = JointTrajectory()
        sparse_trajectory.joint_names = joint_names
        sparse_trajectory.times = [
            idx * segment_time for idx in range(len(sparse_targets))
        ]
        sparse_trajectory.positions = sparse_targets

        dense_targets = interpolateJointTrajectory(scene, sparse_trajectory, dt)

        sparse_target_positions_by_frame = compute_end_effector_positions(
            scene, sparse_targets, ee_frame_names
        )
        dense_target_positions_by_frame = compute_end_effector_positions(
            scene, dense_targets, ee_frame_names
        )

    print(f"Sparse targets: {len(sparse_target_positions_by_frame[ee_frame_names[0]])}")
    print(f"Dense targets:  {len(dense_target_positions_by_frame[ee_frame_names[0]])}")

    # Trajectory rollout starts from the fixed start configuration
    scene.setJointPositions(q_start)

    if action_space == "cartesian":
        q_current = q_start.copy()
        trajectory = [q_current.copy()]
        delta_q = np.zeros(num_variables, dtype=float)
        # Non-group indices are always zero; only group indices are written each step.
        delta_q_full = np.zeros(model_pin.nv, dtype=float)

        for idx in range(len(dense_cartesian_trajectory.times)):
            loop_start = time.time()

            for frame_task, frame_tforms in zip(
                frame_tasks, dense_cartesian_trajectory.tforms
            ):
                frame_task.setTargetFrameTransform(frame_tforms[idx])

            try:
                oink.solveIk(scene, tasks, constraints, delta_q, regularization)
            except RuntimeError as exc:
                print(f"Warning: OInK failed at dense step {idx}: {exc}")
                delta_q[:] = 0.0

            delta_q_full[oink.v_indices] = delta_q
            q_current = scene.integrate(q_current, delta_q_full)
            scene.setJointPositions(q_current)
            trajectory.append(q_current.copy())

            if sleep:
                viz.display(q_current)
                elapsed = time.time() - loop_start
                time.sleep(max(0.0, dt - elapsed))

    else:
        trajectory = dense_targets
        if sleep:
            for q in trajectory:
                viz.display(q)
                time.sleep(dt)

    print("Finished tracking action chunk.")
    print(f"Generated trajectory with {len(trajectory)} configurations.")

    executed_ee_positions_by_frame = compute_end_effector_positions(
        scene,
        trajectory,
        ee_frame_names,
    )

    trace_colors = [
        (0, 80, 255),
        (0, 180, 120),
        (180, 0, 255),
        (255, 80, 80),
    ]

    current_ee_markers = {}

    for ee_idx, ee_frame_name in enumerate(ee_frame_names):
        executed_color = trace_colors[ee_idx % len(trace_colors)]

        # Visualize 1. sparse policy waypoints,
        # 2. dense interpolated references,
        # 3. final executed trajectory
        visualize_ee_traces(
            viz,
            ee_frame_name,
            sparse_target_positions_by_frame[ee_frame_name],
            dense_target_positions_by_frame[ee_frame_name],
            executed_ee_positions_by_frame[ee_frame_name],
            executed_color,
        )

        current_ee_markers[ee_frame_name] = viz.viewer.scene.add_icosphere(
            f"/action_chunk/{ee_frame_name}/current_ee",
            radius=0.022,
            position=executed_ee_positions_by_frame[ee_frame_name][0],
            color=(255, 0, 0),
        )

    print("Visualization added:")
    print("  orange: sparse policy waypoints shown as small markers only")
    print("  gray:   dense interpolated targets")
    if action_space == "cartesian":
        print("  colored traces: executed OInK-constrained end-effector trajectories")
    else:
        print("  colored traces: executed joint-space end-effector trajectories")
    print("  red spheres: current end-effector positions")
    print("Use the Viser GUI controls to animate, scrub, or reset the trajectory.")

    animating = False

    animate_button = viz.viewer.gui.add_button("Animate action chunk")
    reset_button = viz.viewer.gui.add_button("Reset")
    step_slider = viz.viewer.gui.add_slider(
        "Trajectory step",
        min=0,
        max=len(trajectory) - 1,
        step=1,
        initial_value=0,
    )

    def display_step(target_step_idx: int, update_slider: bool = True):
        """Display one tracked configuration by index."""
        target_step_idx = max(0, min(target_step_idx, len(trajectory) - 1))
        viz.display(trajectory[target_step_idx])
        for ee_frame_name, marker in current_ee_markers.items():
            marker.position = executed_ee_positions_by_frame[ee_frame_name][
                target_step_idx
            ]
        if update_slider:
            step_slider.value = target_step_idx

    @animate_button.on_click
    def animate_action_chunk(_):
        nonlocal animating
        if animating:
            return

        animating = True
        animate_button.disabled = True
        step_slider.disabled = True
        reset_button.disabled = True

        start_step_idx = int(step_slider.value)
        if start_step_idx >= len(trajectory) - 1:
            start_step_idx = 0

        for idx in range(start_step_idx, len(trajectory)):
            display_step(idx)
            time.sleep(animation_dt)

        animating = False
        animate_button.disabled = False
        step_slider.disabled = False
        reset_button.disabled = False

    @step_slider.on_update
    def update_step_from_slider(_):
        if animating:
            return
        display_step(int(step_slider.value), update_slider=False)

    @reset_button.on_click
    def reset(_):
        if animating:
            return
        display_step(0)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    tyro.cli(main)
