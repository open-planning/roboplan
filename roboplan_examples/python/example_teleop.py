#!/usr/bin/env python3

import sys
import threading
import time
import tyro
import xacro

import numpy as np
import pinocchio as pin

from common import MODELS
from roboplan.core import Scene, CartesianConfiguration
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
from roboplan.viser_visualizer import ViserVisualizer
from teleop import (
    CommandFilter,
    KeyboardInput,
    TeleopConfig,
    TeleopController,
    TeleopState,
    TwistCommand,
)


def main(
    model: str = "ur5",
    teleop: str = "keyboard",
    task_gain: float = 1.0,
    lm_damping: float = 0.01,
    regularization: float = 1e-6,
    control_freq: float = 100.0,
    max_joint_velocity: float = 1.0,
    linear_sensitivity: float = 0.12,
    angular_sensitivity: float = 0.60,
    input_timeout_s: float = 0.20,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Run keyboard teleoperation with OInK-based constrained tracking.

    Controls (terminal keyboard, unless viser key callbacks are available):
      Translation: w/s (+/-x), a/d (+/-y), q/e (+/-z)
      Rotation: i/k (+/-roll), j/l (+/-pitch), u/o (+/-yaw)
      Gripper: g/h
      Pause toggle: space
      Reset home: r
      Reset target to current pose: t
      Quit: x
    """

    if model not in MODELS:
        print(f"Invalid model requested: {model}")
        sys.exit(1)
    if teleop != "keyboard":
        print(
            f"Unsupported teleop mode '{teleop}'. Milestone 1 currently supports only 'keyboard'."
        )
        sys.exit(1)

    model_data = MODELS[model]
    if len(model_data.ee_names) != 1:
        print(
            f"Model '{model}' has {len(model_data.ee_names)} end-effectors. "
            "Milestone 1 supports single-EE models only."
        )
        sys.exit(1)

    package_paths = [get_package_share_dir()]
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "teleop_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )

    model_pin = pin.buildModelFromXML(urdf_xml)
    collision_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    viz = ViserVisualizer(model_pin, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    q_current = scene.getCurrentJointPositions()
    ee_name = model_data.ee_names[0]
    dt = 1.0 / control_freq
    num_variables = scene.computeFrameJacobian(q_current, ee_name).shape[1]

    oink = Oink(num_variables)
    task_options = FrameTaskOptions(
        position_cost=1.0,
        orientation_cost=0.1,
        task_gain=task_gain,
        lm_damping=lm_damping,
    )
    goal = CartesianConfiguration()
    goal.base_frame = model_data.base_link
    goal.tip_frame = ee_name
    frame_task = FrameTask(goal, num_variables, task_options)

    q_home = np.array(model_data.starting_joint_config, dtype=float)
    if len(q_home) != len(q_current):
        print(
            f"Warning: starting_joint_config has size {len(q_home)}, expected {len(q_current)}. "
            "Using current scene configuration as home."
        )
        q_home = q_current.copy()

    config_task = ConfigurationTask(
        q_home,
        np.full(num_variables, 0.05, dtype=float),
        ConfigurationTaskOptions(task_gain=0.1, lm_damping=0.0),
    )
    tasks = [frame_task, config_task]

    v_max = np.full(num_variables, max_joint_velocity, dtype=float)
    constraints = [
        PositionLimit(num_variables, gain=1.0),
        VelocityLimit(num_variables, dt, v_max),
    ]

    teleop_config = TeleopConfig(
        linear_sensitivity=linear_sensitivity,
        angular_sensitivity=angular_sensitivity,
        input_timeout_s=input_timeout_s,
    )
    # Keyboard mode should feel immediate; smoothing can be tuned later.
    teleop_config.smoothing_alpha = 1.0
    teleop_input = KeyboardInput(
        viewer=viz.viewer,
        key_mapping=teleop_config.key_mapping,
        key_hold_s=teleop_config.key_hold_s,
        default_frame=teleop_config.default_frame,
    )
    teleop_state = TeleopState(active_device=f"keyboard ({teleop_input.backend_name})")
    command_filter = CommandFilter()

    scene_lock = threading.Lock()
    state_lock = threading.Lock()
    running = True
    paused = False
    request_reset_home = False
    request_reset_target = False

    with scene_lock:
        scene.setJointPositions(q_home)
        initial_target = scene.forwardKinematics(q_home, ee_name)
    controller = TeleopController(
        initial_target=initial_target,
        workspace_min_xyz=teleop_config.workspace_min_xyz,
        workspace_max_xyz=teleop_config.workspace_max_xyz,
        frame_mode=teleop_config.default_frame,
    )
    frame_task.setTargetFrameTransform(controller.get_target())
    viz.display(q_home)

    enable_checkbox = viz.viewer.gui.add_checkbox("Enable Teleop", initial_value=True)
    pause_button = viz.viewer.gui.add_button("Pause / Resume")
    linear_slider = viz.viewer.gui.add_slider(
        "Linear Sensitivity (m/s)",
        min=0.01,
        max=0.40,
        step=0.01,
        initial_value=teleop_config.linear_sensitivity,
    )
    angular_slider = viz.viewer.gui.add_slider(
        "Angular Sensitivity (rad/s)",
        min=0.05,
        max=2.00,
        step=0.05,
        initial_value=teleop_config.angular_sensitivity,
    )
    frame_dropdown = viz.viewer.gui.add_dropdown(
        "Control Frame",
        options=("ee", "world"),
        initial_value=teleop_config.default_frame,
    )
    reset_home_button = viz.viewer.gui.add_button("Reset to Home")
    reset_current_button = viz.viewer.gui.add_button("Reset Target to Current")
    status_md = viz.viewer.gui.add_markdown("Initializing teleop status...")

    @enable_checkbox.on_update
    def _toggle_enable(event):
        nonlocal paused
        teleop_state.enabled = bool(event.target.value)
        with state_lock:
            if teleop_state.enabled:
                paused = False
                teleop_state.mode = "velocity_drive"
            else:
                paused = True
                teleop_state.mode = "paused"
                command_filter.reset()

    @pause_button.on_click
    def _toggle_pause(_):
        nonlocal paused
        with state_lock:
            paused = not paused
            teleop_state.mode = "paused" if paused else "velocity_drive"
            if paused:
                command_filter.reset()

    @linear_slider.on_update
    def _update_linear_gain(event):
        teleop_config.linear_sensitivity = float(event.target.value)

    @angular_slider.on_update
    def _update_angular_gain(event):
        teleop_config.angular_sensitivity = float(event.target.value)

    @frame_dropdown.on_update
    def _update_frame(event):
        teleop_config.default_frame = str(event.target.value)
        controller.set_frame_mode(teleop_config.default_frame)

    @reset_home_button.on_click
    def _reset_home(_):
        nonlocal request_reset_home
        with state_lock:
            request_reset_home = True

    @reset_current_button.on_click
    def _reset_target(_):
        nonlocal request_reset_target
        with state_lock:
            request_reset_target = True

    print(
        "Teleop started. Focus terminal and use keyboard controls. "
        f"Input backend: {teleop_input.backend_name}"
    )
    if teleop_input.backend_note:
        print(teleop_input.backend_note)

    def hold_current_pose() -> None:
        q_hold = scene.getCurrentJointPositions()
        hold_pose = scene.forwardKinematics(q_hold, ee_name)
        controller.reset_to(hold_pose)
        frame_task.setTargetFrameTransform(controller.get_target())
        viz.display(q_hold)

    def control_loop() -> None:
        nonlocal running, paused, request_reset_home, request_reset_target
        delta_q = np.zeros(num_variables, dtype=float)
        last_status_update = time.monotonic()

        while running:
            loop_start = time.monotonic()
            now_ns = time.monotonic_ns()

            if teleop_input.quit_requested:
                running = False
                break
            if teleop_input.consume_pause_toggle():
                with state_lock:
                    paused = not paused
                    teleop_state.mode = "paused" if paused else "velocity_drive"
                    if paused:
                        command_filter.reset()
            if teleop_input.consume_reset_home():
                with state_lock:
                    request_reset_home = True
            if teleop_input.consume_reset_target():
                with state_lock:
                    request_reset_target = True

            with state_lock:
                enabled = teleop_state.enabled
                paused_local = paused
                do_reset_home = request_reset_home
                do_reset_target = request_reset_target
                request_reset_home = False
                request_reset_target = False

            with scene_lock:
                if do_reset_home:
                    scene.setJointPositions(q_home)
                    current_pose = scene.forwardKinematics(q_home, ee_name)
                    controller.reset_to(current_pose)
                    frame_task.setTargetFrameTransform(controller.get_target())
                    command_filter.reset()
                    viz.display(q_home)
                    continue

                q_current_local = scene.getCurrentJointPositions()
                current_pose = scene.forwardKinematics(q_current_local, ee_name)

                if do_reset_target:
                    controller.reset_to(current_pose)
                    frame_task.setTargetFrameTransform(controller.get_target())
                    command_filter.reset()

                teleop_state.input_alive = teleop_input.is_active()
                raw_cmd = teleop_input.read()
                raw_cmd.frame = teleop_config.default_frame

                stale = (now_ns - raw_cmd.stamp_ns) > int(
                    teleop_config.input_timeout_s * 1e9
                )
                if stale:
                    raw_cmd = TwistCommand.zero(
                        frame=teleop_config.default_frame, stamp_ns=now_ns
                    )

                if (not enabled) or paused_local:
                    teleop_state.mode = "paused"
                    command_filter.reset()
                    hold_current_pose()
                else:
                    teleop_state.mode = "velocity_drive"
                    filtered_cmd = command_filter.filter(raw_cmd, teleop_config)
                    controller.update(filtered_cmd, dt)
                    frame_task.setTargetFrameTransform(controller.get_target())

                    try:
                        oink.solveIk(tasks, constraints, scene, delta_q, regularization)
                    except RuntimeError as err:
                        print(f"Warning: IK solve failed, zeroing delta_q: {err}")
                        delta_q[:] = 0.0

                    q_next = scene.integrate(q_current_local, delta_q)
                    scene.setJointPositions(q_next)
                    scene.forwardKinematics(q_next, ee_name)
                    viz.display(q_next)

                now = time.monotonic()
                if now - last_status_update > 0.1:
                    q_status = scene.getCurrentJointPositions()
                    ee_status = scene.forwardKinematics(q_status, ee_name)
                    ee_xyz = ee_status[:3, 3]
                    status_md.content = (
                        f"Device: `{teleop_state.active_device}`  \n"
                        f"Input alive: `{teleop_state.input_alive}`  \n"
                        f"Mode: `{teleop_state.mode}`  \n"
                        f"Frame: `{teleop_config.default_frame}`  \n"
                        f"Linear sensitivity: `{teleop_config.linear_sensitivity:.3f}` m/s  \n"
                        f"Angular sensitivity: `{teleop_config.angular_sensitivity:.3f}` rad/s  \n"
                        f"EE position: `[{ee_xyz[0]:.3f}, {ee_xyz[1]:.3f}, {ee_xyz[2]:.3f}]`  \n"
                        f"Keyboard events: `{teleop_input.key_event_count}`"
                    )
                    last_status_update = now

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, dt - elapsed))

    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    try:
        while running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        running = False
    finally:
        control_thread.join(timeout=1.0)
        teleop_input.close()


if __name__ == "__main__":
    tyro.cli(main)
