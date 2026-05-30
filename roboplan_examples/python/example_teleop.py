"""
Keyboard teleoperation example for robot end-effector jogging.

Use keyboard input to move one or more robot end-effectors in Cartesian space,
tracked through OInK with joint position and velocity limits. A mild
ConfigurationTask regularization prevents unwanted nullspace drift.

Keyboard controls (focus the terminal window first):

    Translation:
        w / s  ->  +x / -x
        a / d  ->  +y / -y
        q / e  ->  +z / -z

    Rotation:
        i / k  ->  +roll / -roll
        j / l  ->  +pitch / -pitch
        u / o  ->  +yaw / -yaw

    Other:
        space  ->  pause / resume
        r      ->  reset robot to home position
        t      ->  reset target to current end-effector pose
        x      ->  quit

Example commands:

    python example_teleop.py --model ur5 --device keyboard
    python example_teleop.py --model franka --device keyboard
    python example_teleop.py --model dual --device keyboard --ee-mode all
"""

import sys
import termios
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np
import pinocchio as pin
import tyro
import xacro
from pinocchio.visualize import ViserVisualizer
from pynput import keyboard

from common import get_home_configuration, get_model_data
from roboplan.visualization import se3_to_viser_wxyz
from roboplan.core import CartesianConfiguration, Scene
from roboplan.example_models import get_package_share_dir
from roboplan.filters import SE3LowPassFilter
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskOptions,
    FrameTask,
    FrameTaskOptions,
    Oink,
    PositionLimit,
    VelocityLimit,
)


@dataclass
class TeleopKeyboardState:
    """Tracks which teleop keys are currently held or triggered.

    Shared between the keyboard listener thread and the control loop.
    Access all fields under ``lock``.
    """

    lock: threading.Lock = field(default_factory=threading.Lock)

    # Translation keys (held)
    pos_x: bool = False  # w
    neg_x: bool = False  # s
    pos_y: bool = False  # a
    neg_y: bool = False  # d
    pos_z: bool = False  # q
    neg_z: bool = False  # e

    # Rotation keys (held)
    pos_roll: bool = False  # i
    neg_roll: bool = False  # k
    pos_pitch: bool = False  # j
    neg_pitch: bool = False  # l
    pos_yaw: bool = False  # u
    neg_yaw: bool = False  # o

    # One-shot triggers. Set on press, cleared by control loop after reading.
    reset_home: bool = False
    reset_target: bool = False
    quit: bool = False

    # Toggle. Flipped on each space press.
    paused: bool = False


_MOTION_KEY_MAP: dict[str, str] = {
    "w": "pos_x",
    "s": "neg_x",
    "a": "pos_y",
    "d": "neg_y",
    "q": "pos_z",
    "e": "neg_z",
    "i": "pos_roll",
    "k": "neg_roll",
    "j": "pos_pitch",
    "l": "neg_pitch",
    "u": "pos_yaw",
    "o": "neg_yaw",
}


def make_keyboard_listener(state: TeleopKeyboardState) -> keyboard.Listener:
    """Create a pynput keyboard listener that updates shared teleop state.

    Args:
        state: Shared keyboard state to update on key events.

    Returns:
        A configured pynput Listener.
    """

    def on_press(key: keyboard.Key | keyboard.KeyCode) -> None:
        try:
            ch = key.char
        except AttributeError:
            if key == keyboard.Key.space:
                with state.lock:
                    state.paused = not state.paused
                    print("Paused." if state.paused else "Resumed.")
            return

        with state.lock:
            if ch in _MOTION_KEY_MAP:
                setattr(state, _MOTION_KEY_MAP[ch], True)
            elif ch == "r":
                state.reset_home = True
            elif ch == "t":
                state.reset_target = True
            elif ch == "x":
                state.quit = True

    def on_release(key: keyboard.Key | keyboard.KeyCode) -> None:
        try:
            ch = key.char
        except AttributeError:
            return

        with state.lock:
            if ch in _MOTION_KEY_MAP:
                setattr(state, _MOTION_KEY_MAP[ch], False)

    return keyboard.Listener(on_press=on_press, on_release=on_release)


def _suppress_terminal_echo() -> tuple[int, list]:
    """Disable terminal echo so held keys do not flood the terminal output."""
    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    new_attrs = termios.tcgetattr(fd)
    new_attrs[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, new_attrs)
    return fd, old_attrs


def compute_cartesian_delta(
    state: TeleopKeyboardState,
    linear_sensitivity: float,
    angular_sensitivity: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute linear and angular displacement deltas from key state.

    Args:
        state: Current keyboard state.
        linear_sensitivity: Linear speed in m/s at full key deflection.
        angular_sensitivity: Angular speed in rad/s at full key deflection.
        dt: Control timestep in seconds.

    Returns:
        Tuple of ``(linear_delta, angular_delta)``, each a 3-element array.
    """
    with state.lock:
        dx = float(state.pos_x) - float(state.neg_x)
        dy = float(state.pos_y) - float(state.neg_y)
        dz = float(state.pos_z) - float(state.neg_z)
        droll = float(state.pos_roll) - float(state.neg_roll)
        dpitch = float(state.pos_pitch) - float(state.neg_pitch)
        dyaw = float(state.pos_yaw) - float(state.neg_yaw)

    linear_delta = linear_sensitivity * dt * np.array([dx, dy, dz])
    angular_delta = angular_sensitivity * dt * np.array([droll, dpitch, dyaw])
    return linear_delta, angular_delta


def apply_cartesian_delta(
    target: np.ndarray,
    linear_delta: np.ndarray,
    angular_delta: np.ndarray,
    control_frame: str,
) -> np.ndarray:
    """Apply a Cartesian displacement to a 4x4 SE(3) target transform.

    Args:
        target: Current 4x4 homogeneous target transform.
        linear_delta: 3D linear displacement in meters.
        angular_delta: 3D axis-angle rotation in radians.
        control_frame: ``"world"`` or ``"ee"``.

    Returns:
        Updated 4x4 SE(3) target transform.
    """
    new_target = target.copy()
    rot = target[:3, :3]

    if control_frame == "world":
        new_target[:3, 3] += linear_delta
        if np.any(angular_delta != 0.0):
            new_target[:3, :3] = pin.exp3(angular_delta) @ rot
    else:
        new_target[:3, 3] += rot @ linear_delta
        if np.any(angular_delta != 0.0):
            new_target[:3, :3] = rot @ pin.exp3(angular_delta)

    return new_target


def reset_targets_to_current_ee_poses(
    scene: Scene,
    q_current: np.ndarray,
    ee_frame_names: list[str],
    target_poses: dict[str, np.ndarray],
    target_frame_handles: dict[str, Any],
    target_filters: dict[str, SE3LowPassFilter] | None = None,
) -> None:
    """Reset all target poses and their Viser markers to the current EE poses.

    If ``target_filters`` is provided, each filter is also reset to the new
    target pose so there is no lag or snap on resumption.

    Args:
        scene: Scene used for forward kinematics.
        q_current: Current joint configuration.
        ee_frame_names: End-effector frame names to reset.
        target_poses: Dict of EE frame name to current target SE(3) transform.
            Updated in place.
        target_frame_handles: Dict of EE frame name to Viser frame handle.
            Updated in place.
        target_filters: Optional dict of EE frame name to SE3LowPassFilter.
            When provided, each filter is reset to the new target pose.
    """
    for ee_frame_name in ee_frame_names:
        target_poses[ee_frame_name] = scene.forwardKinematics(
            q_current,
            ee_frame_name,
        ).copy()
        position, wxyz = se3_to_viser_wxyz(target_poses[ee_frame_name])
        target_frame_handles[ee_frame_name].position = position
        target_frame_handles[ee_frame_name].wxyz = wxyz
        if target_filters:
            target_filters[ee_frame_name].reset(target_poses[ee_frame_name])


def main(
    model: str = "ur5",
    device: str = "keyboard",
    ee_mode: Literal["first", "all"] = "first",
    control_freq: float = 50.0,
    linear_sensitivity: float = 0.3,
    angular_sensitivity: float = 0.5,
    reference_filter_tau: float = 0.1,
    task_gain: float = 1.0,
    lm_damping: float = 0.01,
    regularization: float = 1e-6,
    config_task_gain: float = 1e-4,
    target_axes_length: float = 0.1,
    target_axes_radius: float = 0.005,
    host: str = "localhost",
    port: str = "8000",
) -> None:
    """Teleoperate robot end-effectors using keyboard input.

    Focus the terminal window to capture keyboard input while Viser is open in
    the browser.

    Args:
        model: Robot model name from roboplan_examples/python/common.py.
        device: Teleop device. Currently only ``keyboard`` is supported.
        ee_mode: ``first`` controls the first end-effector only. ``all`` uses
            all end-effectors, useful for multi-arm models such as ``dual``.
        control_freq: Control loop frequency, in Hz.
        linear_sensitivity: End-effector linear speed in m/s at full key deflection.
        angular_sensitivity: End-effector angular speed in rad/s at full key deflection.
        reference_filter_tau: Initial time constant for SE(3) target low-pass
            filtering, in seconds. Adjustable at runtime via the Viser GUI.
            Set to 0 to disable filtering.
        task_gain: OInK FrameTask gain.
        lm_damping: OInK FrameTask Levenberg-Marquardt damping.
        regularization: Tikhonov regularization for OInK.
        config_task_gain: Per-DoF gain for the mild ConfigurationTask regularization.
            Keep this small (e.g. 1e-4) so it reduces nullspace drift without
            fighting the Cartesian frame tasks.
        target_axes_length: Length of the target frame axes in Viser, in meters.
        target_axes_radius: Radius of the target frame axes in Viser, in meters.
        host: Viser host.
        port: Viser port.
    """
    if device != "keyboard":
        raise ValueError(
            f"Unsupported teleop device '{device}'. "
            "Only 'keyboard' is currently supported."
        )

    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model: {model}")
        print(f"Available models: {list(get_model_data().keys())}")
        sys.exit(1)

    if not model_data.ee_names:
        raise ValueError(f"Model '{model}' has no configured end-effectors.")

    if control_freq <= 0.0:
        raise ValueError("control_freq must be positive.")

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

    joint_group = model_data.default_joint_group
    joint_names = scene.getJointGroupInfo(joint_group).joint_names

    if ee_mode == "first":
        ee_frame_names = [model_data.ee_names[0]]
    else:
        ee_frame_names = list(model_data.ee_names)

    print(f"\n=== Model: {model} ===")
    print(f"Device:          {device}")
    print(f"Joint group:     {joint_group}")
    print(f"End-effector(s): {ee_frame_names}")
    print(f"Control freq:    {control_freq} Hz")
    print("\nKeyboard controls (focus this terminal window):")
    print("  w/s: +x/-x   a/d: +y/-y   q/e: +z/-z")
    print("  i/k: +roll/-roll   j/l: +pitch/-pitch   u/o: +yaw/-yaw")
    print("  space: pause/resume   r: reset home   t: reset target   x: quit\n")

    # Build Pinocchio model for visualization.
    model_pin = pin.buildModelFromXML(urdf_xml)
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

    q_home = get_home_configuration(scene, model_data)
    scene.setJointPositions(q_home)
    viz.display(q_home)

    # Set up OInK solver.
    oink = Oink(scene, joint_group)
    num_variables = len(oink.v_indices)
    dt = 1.0 / control_freq

    v_max = np.hstack(
        [scene.getJointInfo(name).limits.max_velocity for name in joint_names]
    )
    constraints = [
        PositionLimit(oink, gain=1.0),
        VelocityLimit(oink, dt, v_max),
    ]

    # One FrameTask per controlled end effector.
    frame_task_options = FrameTaskOptions(
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
        frame_tasks.append(FrameTask(oink, scene, goal, frame_task_options))

    # Mild ConfigurationTask regularization to reduce nullspace drift.
    # config_task_gain is kept small (default 1e-4) so it does not compete
    # with the Cartesian frame tasks; it only acts in the null space.
    config_task = ConfigurationTask(
        oink,
        q_home[oink.q_indices],
        np.full(num_variables, config_task_gain),
        ConfigurationTaskOptions(
            task_gain=config_task_gain,
            lm_damping=0.0,
        ),
    )

    tasks = [*frame_tasks, config_task]

    # Viser GUI controls.
    linear_slider = viz.viewer.gui.add_slider(
        "Linear sensitivity (m/s)",
        min=0.01,
        max=1.0,
        step=0.01,
        initial_value=linear_sensitivity,
    )
    angular_slider = viz.viewer.gui.add_slider(
        "Angular sensitivity (rad/s)",
        min=0.01,
        max=2.0,
        step=0.01,
        initial_value=angular_sensitivity,
    )
    # tau=0 means pass-through (no filtering); the slider min is set to 0
    # so the user can disable filtering at runtime without restarting.
    filter_tau_slider = viz.viewer.gui.add_slider(
        "Reference filter tau (s)",
        min=0.0,
        max=1.0,
        step=0.01,
        initial_value=reference_filter_tau,
    )
    control_frame_dropdown = viz.viewer.gui.add_dropdown(
        "Control frame",
        options=["world", "ee"],
        initial_value="world",
    )

    # Only show the active-EE selector when there are multiple end-effectors
    # to choose from. For single-EE models it would be meaningless.
    active_ee_dropdown = None
    if len(ee_frame_names) > 1:
        active_ee_dropdown = viz.viewer.gui.add_dropdown(
            "Active end-effector",
            options=["all", *ee_frame_names],
            initial_value="all",
        )

    reset_home_button = viz.viewer.gui.add_button("Reset to home")
    reset_target_button = viz.viewer.gui.add_button("Reset target to current EE")

    # Threading events for GUI-triggered resets.
    gui_reset_home = threading.Event()
    gui_reset_target = threading.Event()

    @reset_home_button.on_click
    def _(_) -> None:
        gui_reset_home.set()

    @reset_target_button.on_click
    def _(_) -> None:
        gui_reset_target.set()

    # One target pose and one Viser frame marker per end effector.
    target_poses: dict[str, np.ndarray] = {
        ee_frame_name: scene.forwardKinematics(q_home, ee_frame_name).copy()
        for ee_frame_name in ee_frame_names
    }

    # Filters are always created so tau can be adjusted at runtime via the
    # slider without restarting. When tau=0 the filter output equals the
    # input (pass-through), which is checked in the control loop.
    target_filters: dict[str, SE3LowPassFilter] = {}
    for ee_frame_name, target_pose in target_poses.items():
        target_filter = SE3LowPassFilter(tau=reference_filter_tau)
        target_filter.reset(target_pose)
        target_filters[ee_frame_name] = target_filter

    target_frame_handles: dict[str, Any] = {}
    for ee_frame_name, target_pose in target_poses.items():
        position, wxyz = se3_to_viser_wxyz(target_pose)
        target_frame_handles[ee_frame_name] = viz.viewer.scene.add_frame(
            f"/teleop/target_{ee_frame_name}",
            axes_length=target_axes_length,
            axes_radius=target_axes_radius,
            position=position,
            wxyz=wxyz,
        )

    # Shared state and control loop buffers.
    kb_state = TeleopKeyboardState()
    q_current = q_home.copy()
    delta_q = np.zeros(num_variables, dtype=float)
    delta_q_full = np.zeros(model_pin.nv, dtype=float)

    listener = make_keyboard_listener(kb_state)
    fd, _old_term_attrs = _suppress_terminal_echo()
    listener.start()

    print("Teleop running. Press x in this terminal to quit.")

    try:
        while True:
            loop_start = time.time()

            # --- GUI-triggered resets ---
            if gui_reset_home.is_set():
                gui_reset_home.clear()
                q_current = q_home.copy()
                scene.setJointPositions(q_current)
                reset_targets_to_current_ee_poses(
                    scene,
                    q_current,
                    ee_frame_names,
                    target_poses,
                    target_frame_handles,
                    target_filters,
                )
                viz.display(q_current)
                continue

            if gui_reset_target.is_set():
                gui_reset_target.clear()
                reset_targets_to_current_ee_poses(
                    scene,
                    q_current,
                    ee_frame_names,
                    target_poses,
                    target_frame_handles,
                    target_filters,
                )
                continue

            # --- Keyboard one-shot actions ---
            with kb_state.lock:
                do_quit = kb_state.quit
                do_reset_home = kb_state.reset_home
                do_reset_target = kb_state.reset_target
                paused = kb_state.paused
                kb_state.reset_home = False
                kb_state.reset_target = False

            if do_quit:
                print("Quit requested.")
                break

            if do_reset_home:
                q_current = q_home.copy()
                scene.setJointPositions(q_current)
                reset_targets_to_current_ee_poses(
                    scene,
                    q_current,
                    ee_frame_names,
                    target_poses,
                    target_frame_handles,
                    target_filters,
                )
                viz.display(q_current)
                continue

            if do_reset_target:
                reset_targets_to_current_ee_poses(
                    scene,
                    q_current,
                    ee_frame_names,
                    target_poses,
                    target_frame_handles,
                    target_filters,
                )
                continue

            if paused:
                time.sleep(dt)
                continue

            # --- Read current GUI values ---
            lin_sens = float(linear_slider.value)
            ang_sens = float(angular_slider.value)
            filter_tau = float(filter_tau_slider.value)
            ctrl_frame = str(control_frame_dropdown.value)

            # Determine which EEs the keyboard commands apply to this step.
            if active_ee_dropdown is None or active_ee_dropdown.value == "all":
                controlled_ee_names = ee_frame_names
            else:
                controlled_ee_names = [str(active_ee_dropdown.value)]

            # --- Compute and apply Cartesian delta to selected target(s) ---
            linear_delta, angular_delta = compute_cartesian_delta(
                kb_state, lin_sens, ang_sens, dt
            )
            for ee_frame_name in controlled_ee_names:
                target_poses[ee_frame_name] = apply_cartesian_delta(
                    target_poses[ee_frame_name],
                    linear_delta,
                    angular_delta,
                    ctrl_frame,
                )
                position, wxyz = se3_to_viser_wxyz(target_poses[ee_frame_name])
                target_frame_handles[ee_frame_name].position = position
                target_frame_handles[ee_frame_name].wxyz = wxyz

            # Update all frame task targets before solving. Even when only one
            # EE is being commanded, all tasks must receive their current target
            # so OInK does not try to move uncontrolled EEs back to stale poses.
            for frame_task, ee_frame_name in zip(frame_tasks, ee_frame_names):
                if filter_tau > 0.0:
                    # Recreate the filter if tau changed via the slider, preserving
                    # the current pose so there is no discontinuity.
                    if target_filters[ee_frame_name].tau != filter_tau:
                        current_pose = target_filters[ee_frame_name].update(
                            target_poses[ee_frame_name], dt
                        )
                        target_filters[ee_frame_name] = SE3LowPassFilter(tau=filter_tau)
                        target_filters[ee_frame_name].reset(current_pose)
                    filtered_target = target_filters[ee_frame_name].update(
                        target_poses[ee_frame_name],
                        dt,
                    )
                    frame_task.setTargetFrameTransform(filtered_target)
                else:
                    frame_task.setTargetFrameTransform(target_poses[ee_frame_name])

            # --- Solve OInK and integrate ---
            try:
                oink.solveIk(scene, tasks, constraints, delta_q, regularization)
            except RuntimeError as exc:
                print(f"Warning: OInK failed: {exc}")
                delta_q[:] = 0.0

            delta_q_full[:] = 0.0
            delta_q_full[oink.v_indices] = delta_q
            q_current = scene.integrate(q_current, delta_q_full)
            scene.setJointPositions(q_current)
            viz.display(q_current)

            # --- Maintain control rate ---
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, dt - elapsed))

    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, _old_term_attrs)
        print("Teleop stopped.")


if __name__ == "__main__":
    tyro.cli(main)
