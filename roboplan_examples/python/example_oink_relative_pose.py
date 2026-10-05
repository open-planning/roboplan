#!/usr/bin/env python3

import threading
import time

import numpy as np
import pinocchio as pin
import tyro
import xacro
from common import get_model_data
from pinocchio.visualize import ViserVisualizer

from roboplan.core import (
    CartesianConfiguration,
    Scene,
    loadJointLimitsConfig,
    loadUrdfSceneDescriptionFromXml,
)
from roboplan.example_models import get_package_share_dir
from roboplan.filters import SE3LowPassFilter
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskOptions,
    FrameTask,
    FrameTaskOptions,
    Oink,
    PositionLimit,
    RelativePoseConstraint,
    SelfCollisionBarrier,
    SelfCollisionBarrierOptions,
    VelocityLimit,
)


def main(
    use_relative_pose_constraint: bool = True,
    position_tolerance: float = 0.005,
    orientation_tolerance: float = 0.05,
    bar_radius: float = 0.02,
    config_task_weight: float = 0.05,
    self_collision_num_pairs: int = 4,
    control_freq: float = 100.0,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Move both arms of the dual Franka in tandem with a RelativePoseConstraint.

    The grippers hold a bar, and a marker sits at its center.
    Moving the marker drags both grippers along.

    Parameters:
        use_relative_pose_constraint: If true, only the left gripper tracks the marker and the
            constraint carries the right gripper along. If false, each gripper tracks the marker
            with its own FrameTask and no constraint couples them.
        position_tolerance: Per-axis relative position tolerance, in meters.
        orientation_tolerance: Per-axis relative orientation tolerance, in radians.
        bar_radius: Radius of the held bar, in meters.
        config_task_weight: Weight of a priority-2 ConfigurationTask pulling toward the start.
        self_collision_num_pairs: Number of closest collision pairs constrained by the
            self-collision barrier; 0 disables the barrier.
        control_freq: Control loop frequency in Hz.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
    """
    model_data = get_model_data()["dual"]
    left_tcp, right_tcp = "left_fr3_hand_tcp", "right_fr3_hand_tcp"
    package_paths = [get_package_share_dir()]
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "oink_relative_pose_scene",
        loadUrdfSceneDescriptionFromXml(urdf_xml, package_paths),
    )
    scene.importJointLimitsFromConfig(
        loadJointLimitsConfig(model_data.yaml_config_path)
    )
    scene.importSrdf(srdf_xml)

    # Use initial joint positions for arms facing inward and wrists turned so the fingers close across the bar.
    q_start = np.array(model_data.starting_joint_config)
    q_start[[0, 8]] = [-np.pi / 6, np.pi / 6]
    q_start[[6, 14]] = [-np.pi / 4 - np.pi / 6, -np.pi / 4 + np.pi / 6]
    q_start[[7, 15]] = bar_radius
    scene.setJointPositions(q_start)

    model_pin = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )
    viz = ViserVisualizer(model_pin, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    oink = Oink(scene, model_data.default_joint_group)
    dt = 1.0 / control_freq
    scene_lock = threading.Lock()

    joint_names = scene.getJointGroupInfo(model_data.default_joint_group).joint_names
    v_max = np.hstack(
        [scene.getJointInfo(name).limits.max_velocity for name in joint_names]
    )

    # Hold the right gripper at its starting pose relative to the left gripper.
    T_left = scene.forwardKinematics(q_start, left_tcp)
    T_right = scene.forwardKinematics(q_start, right_tcp)
    relative_pose = RelativePoseConstraint(
        oink,
        scene,
        left_tcp,
        right_tcp,
        np.linalg.inv(T_left) @ T_right,
        position_tolerance=np.full(3, position_tolerance),
        orientation_tolerance=np.full(3, orientation_tolerance),
    )
    constraints = [
        PositionLimit(oink, gain=1.0),
        VelocityLimit(oink, dt, v_max),
    ]
    if use_relative_pose_constraint:
        constraints.append(relative_pose)
    barriers = []
    if self_collision_num_pairs > 0:
        barriers.append(
            SelfCollisionBarrier(
                oink,
                scene,
                dt=dt,
                options=SelfCollisionBarrierOptions(
                    n_collision_pairs=self_collision_num_pairs,
                    safe_displacement_gain=0.001,
                    d_min=0.02,
                ),
            )
        )

    task_options = FrameTaskOptions(
        position_cost=1.0, orientation_cost=0.1, task_gain=1.0, lm_damping=0.01
    )
    frame_tasks = []
    for name in (left_tcp,) if use_relative_pose_constraint else (left_tcp, right_tcp):
        goal = CartesianConfiguration()
        goal.tip_frame = name
        goal.tform = scene.forwardKinematics(q_start, name)
        frame_tasks.append(FrameTask(oink, scene, goal, task_options))

    config_task = ConfigurationTask(
        oink,
        q_start[oink.q_indices],
        np.full(oink.num_variables, config_task_weight),
        ConfigurationTaskOptions(priority=2),
    )

    # The bar frame starts midway between the grippers, world-aligned, with the bar along y.
    # The bar is rigidly attached to the left gripper, and the marker tracks the bar frame. Each
    # gripper's target is the marker pose composed with its fixed offset from the bar.
    T_bar = pin.SE3(np.eye(3), 0.5 * (T_left[:3, 3] + T_right[:3, 3])).homogeneous
    bar_T_tcp = {
        left_tcp: np.linalg.inv(T_bar) @ T_left,
        right_tcp: np.linalg.inv(T_bar) @ T_right,
    }
    left_T_bar = np.linalg.inv(bar_T_tcp[left_tcp])
    bar_T_cylinder = pin.SE3(pin.utils.rotate("x", np.pi / 2), np.zeros(3)).homogeneous
    bar = viz.viewer.scene.add_cylinder(
        "/bar",
        radius=bar_radius,
        height=np.linalg.norm(T_right[:3, 3] - T_left[:3, 3]) + 0.2,
        color=(200, 120, 40),
    )
    marker = viz.viewer.scene.add_transform_controls(
        "/ik_marker", depth_test=False, scale=0.2, disable_sliders=True
    )
    marker_target = T_bar
    reference_filter = SE3LowPassFilter(tau=0.1)

    if use_relative_pose_constraint:
        pos_slider = viz.viewer.gui.add_slider(
            "Position tol. (mm)", 0.0, 50.0, 0.5, position_tolerance * 1000.0
        )
        rot_slider = viz.viewer.gui.add_slider(
            "Orientation tol. (deg)", 0.0, 30.0, 0.5, np.rad2deg(orientation_tolerance)
        )

        @pos_slider.on_update
        def _(_):
            with scene_lock:
                relative_pose.position_tolerance = np.full(3, pos_slider.value / 1000.0)

        @rot_slider.on_update
        def _(_):
            with scene_lock:
                relative_pose.orientation_tolerance = np.full(
                    3, np.deg2rad(rot_slider.value)
                )

    config_checkbox = viz.viewer.gui.add_checkbox("Configuration task", True)
    error_text = viz.viewer.gui.add_markdown("")
    reset_button = viz.viewer.gui.add_button("Reset Marker")

    @marker.on_update
    def _(_):
        nonlocal marker_target
        with scene_lock:
            marker_target = pin.SE3(
                pin.Quaternion(marker.wxyz[[1, 2, 3, 0]]), marker.position
            ).homogeneous

    @reset_button.on_click
    def reset_marker(_):
        nonlocal marker_target
        with scene_lock:
            q = scene.getCurrentJointPositions()
            marker_target = scene.forwardKinematics(q, left_tcp) @ left_T_bar
            reference_filter.reset(marker_target)
            marker.position = marker_target[:3, 3]
            marker.wxyz = pin.Quaternion(marker_target[:3, :3]).coeffs()[[3, 0, 1, 2]]

    running = True

    def control_loop():
        delta_q = np.zeros(oink.num_variables)
        last_display = 0.0
        while running:
            loop_start = time.time()
            with scene_lock:
                T_marker = reference_filter.update(marker_target, dt)
                for task in frame_tasks:
                    task.setTargetFrameTransform(T_marker @ bar_T_tcp[task.frame_name])

                tasks = (
                    frame_tasks + [config_task]
                    if config_checkbox.value
                    else frame_tasks
                )
                q = scene.getCurrentJointPositions()
                try:
                    oink.solveIk(q, tasks, constraints, barriers, delta_q, 1e-3)
                except RuntimeError as e:
                    delta_q[:] = 0.0
                    print(f"Warning: IK solver failed: {e}")
                q = scene.integrate(
                    q,
                    scene.toFullJointVelocities(
                        model_data.default_joint_group, delta_q
                    ),
                )
                scene.setJointPositions(q)
                T_l = scene.forwardKinematics(q, left_tcp)
                T_r = scene.forwardKinematics(q, right_tcp)
                T_err = pin.SE3(
                    np.linalg.inv(relative_pose.target_pose) @ np.linalg.inv(T_l) @ T_r
                )
                T_cylinder = T_l @ left_T_bar @ bar_T_cylinder

            if loop_start - last_display >= 1.0 / 30.0:
                viz.display(q)
                bar.position = T_cylinder[:3, 3]
                bar.wxyz = pin.Quaternion(T_cylinder[:3, :3]).coeffs()[[3, 0, 1, 2]]
                error_text.content = (
                    f"Relative error: {1000.0 * np.abs(T_err.translation).max():.1f} mm, "
                    f"{np.rad2deg(np.abs(pin.log3(T_err.rotation)).max()):.1f} deg "
                    "(max per axis)"
                )
                last_display = loop_start
            time.sleep(max(0.0, dt - (time.time() - loop_start)))

    reset_marker(None)
    viz.display(q_start)
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        running = False
        control_thread.join(timeout=1.0)


if __name__ == "__main__":
    tyro.cli(main)
