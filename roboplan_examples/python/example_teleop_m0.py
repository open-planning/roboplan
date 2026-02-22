#!/usr/bin/env python3

"""Milestone 0 teleop spike.

Scope:
1. Verify keyboard input path feasibility with current viser version.
2. Run a 100 Hz OInK loop and report jitter statistics.
3. Validate EE/world frame integration math using Pinocchio exp6.
4. Provide a minimal keyboard-to-twist teleop demo for UR5 in simulation.

Keyboard controls (terminal focus required):
  Translation: w/s (+/-x), a/d (+/-y), q/e (+/-z)
  Rotation:    i/k (+/-roll), j/l (+/-pitch), u/o (+/-yaw)
  Toggle pause: space
  Toggle frame: f (ee/world)
  Quit: x
"""

import dataclasses
import select
import sys
import termios
import threading
import time
import tty
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


@dataclasses.dataclass
class ControlCommand:
    linear: np.ndarray
    angular: np.ndarray
    stamp: float


class LoopTimingStats:
    def __init__(self):
        self.samples: list[float] = []

    def add(self, dt: float) -> None:
        self.samples.append(dt)

    def summarize_and_clear(self) -> str | None:
        if not self.samples:
            return None
        arr = np.asarray(self.samples, dtype=float)
        mean_s = float(np.mean(arr))
        p95_s = float(np.percentile(arr, 95))
        max_s = float(np.max(arr))
        self.samples.clear()
        return (
            f"loop dt stats: mean={mean_s * 1000.0:.2f} ms, "
            f"p95={p95_s * 1000.0:.2f} ms, max={max_s * 1000.0:.2f} ms, "
            f"mean_hz={1.0 / max(mean_s, 1e-9):.1f}"
        )


class TerminalKeyboardReader:
    """Non-blocking terminal keyboard reader with short command hold."""

    def __init__(self, key_hold_s: float = 0.12):
        self.key_hold_s = key_hold_s
        self.latest_press: dict[str, float] = {}
        self._lock = threading.Lock()
        self.paused = False
        self.frame_mode = "ee"
        self.quit_requested = False
        self.key_event_count = 0
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._old_term = None

    def start(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleop requires a TTY terminal.")
        self._old_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self._thread.start()

    def close(self) -> None:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=0.5)
        if self._old_term is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)

    def _run(self) -> None:
        while self._running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not ready:
                continue
            char = sys.stdin.read(1).lower()
            now = time.monotonic()
            with self._lock:
                self.key_event_count += 1
                if char == " ":
                    self.paused = not self.paused
                elif char == "f":
                    self.frame_mode = "world" if self.frame_mode == "ee" else "ee"
                elif char == "x":
                    self.quit_requested = True
                elif char in "wsadqeijkluo":
                    self.latest_press[char] = now

    def read_command(self) -> ControlCommand:
        now = time.monotonic()
        lin = np.zeros(3, dtype=float)
        ang = np.zeros(3, dtype=float)

        key_to_axis = {
            "w": (lin, 0, +1.0),
            "s": (lin, 0, -1.0),
            "a": (lin, 1, +1.0),
            "d": (lin, 1, -1.0),
            "q": (lin, 2, +1.0),
            "e": (lin, 2, -1.0),
            "i": (ang, 0, +1.0),
            "k": (ang, 0, -1.0),
            "j": (ang, 1, +1.0),
            "l": (ang, 1, -1.0),
            "u": (ang, 2, +1.0),
            "o": (ang, 2, -1.0),
        }

        with self._lock:
            stale_keys = []
            for key, stamp in self.latest_press.items():
                if now - stamp > self.key_hold_s:
                    stale_keys.append(key)
                    continue
                vec, idx, sign = key_to_axis[key]
                vec[idx] += sign
            for key in stale_keys:
                self.latest_press.pop(key, None)

        lin = np.clip(lin, -1.0, 1.0)
        ang = np.clip(ang, -1.0, 1.0)
        return ControlCommand(linear=lin, angular=ang, stamp=now)


def assert_frame_integration_math() -> None:
    """Validate expected difference between EE/world composition order."""
    t_des = pin.SE3.Identity().homogeneous
    dt = 0.02
    cmd_linear = np.array([0.1, 0.0, 0.0])
    cmd_angular = np.array([0.0, 0.0, 0.5])
    twist = np.concatenate([cmd_linear, cmd_angular]) * dt

    d_tform = pin.exp6(pin.Motion(twist)).homogeneous

    # With identity, both orders should be exactly equal.
    ee_next = t_des @ d_tform
    world_next = d_tform @ t_des
    if not np.allclose(ee_next, world_next, atol=1e-10):
        raise RuntimeError("Frame integration validation failed at identity pose.")

    # With non-identity orientation, composition order should differ.
    rot = pin.rpy.rpyToMatrix(0.3, -0.2, 0.1)
    t_non_identity = pin.SE3(rot, np.array([0.2, -0.1, 0.4])).homogeneous
    ee_next = t_non_identity @ d_tform
    world_next = d_tform @ t_non_identity
    if np.allclose(ee_next, world_next, atol=1e-8):
        raise RuntimeError("EE/world integration order unexpectedly identical.")


def check_viser_keyboard_path(viz: ViserVisualizer) -> str:
    """Probe current viser API for keyboard event hooks."""
    available = []
    for attr in ["on_key_down", "on_key_up", "on_keydown", "on_keyup"]:
        if hasattr(viz.viewer, attr):
            available.append(f"viewer.{attr}")
    clients = viz.viewer.get_clients()
    for client in clients.values():
        for attr in ["on_key_down", "on_key_up", "on_keydown", "on_keyup"]:
            if hasattr(client, attr):
                available.append(f"client.{attr}")

    if available:
        return "Keyboard hooks found in viser API: " + ", ".join(sorted(set(available)))
    return (
        "No keyboard keydown/keyup hooks found in viser API for this environment; "
        "using terminal keyboard fallback for Milestone 0."
    )


def main(
    model: str = "ur5",
    control_freq: float = 100.0,
    linear_speed: float = 0.12,
    angular_speed: float = 0.60,
    key_hold_s: float = 0.12,
    max_joint_velocity: float = 1.0,
    task_gain: float = 1.0,
    lm_damping: float = 0.01,
    regularization: float = 1e-6,
    host: str = "localhost",
    port: str = "8000",
):
    if model != "ur5":
        raise ValueError("Milestone 0 scope is ur5 only. Use --model ur5.")

    model_data = MODELS[model]
    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "teleop_m0_scene",
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

    # M0.1: keyboard path probe
    keyboard_path_msg = check_viser_keyboard_path(viz)
    print(keyboard_path_msg)

    # M0.3: frame integration validation
    assert_frame_integration_math()
    print("Frame integration validation passed (EE/world composition difference confirmed).")

    q_current = scene.getCurrentJointPositions()
    ee_name = model_data.ee_names[0]

    dt = 1.0 / control_freq
    jac = scene.computeFrameJacobian(q_current, ee_name)
    num_variables = jac.shape[1]

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
        q_home = q_current.copy()
    config_options = ConfigurationTaskOptions(task_gain=0.1, lm_damping=0.0)
    config_task = ConfigurationTask(
        q_home,
        np.full(num_variables, 0.05, dtype=float),
        config_options,
    )
    tasks = [frame_task, config_task]

    v_max = np.full(num_variables, max_joint_velocity)
    constraints = [PositionLimit(num_variables, gain=1.0), VelocityLimit(num_variables, dt, v_max)]
    delta_q = np.zeros(num_variables)

    keyboard = TerminalKeyboardReader(key_hold_s=key_hold_s)
    keyboard.start()
    print(
        "Terminal keyboard teleop active. Focus this terminal while pressing keys. "
        "Press 'x' to quit."
    )

    frame_label = viz.viewer.gui.add_markdown(f"Frame mode: `{keyboard.frame_mode}`")
    pause_label = viz.viewer.gui.add_markdown(f"Paused: `{keyboard.paused}`")
    key_count_label = viz.viewer.gui.add_markdown("Terminal key events: `0`")
    timing_label = viz.viewer.gui.add_markdown("Loop stats: `starting...`")
    diag_label = viz.viewer.gui.add_markdown(keyboard_path_msg)

    running = True
    stats = LoopTimingStats()
    last_stats_log = time.monotonic()
    scene.setJointPositions(q_home)
    desired_tform = scene.forwardKinematics(q_home, ee_name)
    frame_task.setTargetFrameTransform(desired_tform)
    viz.display(q_home)

    try:
        while running:
            loop_start = time.monotonic()
            cmd = keyboard.read_command()

            if keyboard.quit_requested:
                running = False
                continue

            frame_label.content = f"Frame mode: `{keyboard.frame_mode}`"
            pause_label.content = f"Paused: `{keyboard.paused}`"
            key_count_label.content = f"Terminal key events: `{keyboard.key_event_count}`"
            diag_label.content = keyboard_path_msg

            q_current = scene.getCurrentJointPositions()
            if not keyboard.paused:
                linear = cmd.linear * linear_speed
                angular = cmd.angular * angular_speed
                twist = np.concatenate([linear, angular]) * dt
                d_tform = pin.exp6(pin.Motion(twist)).homogeneous
                if keyboard.frame_mode == "ee":
                    desired_tform = desired_tform @ d_tform
                else:
                    desired_tform = d_tform @ desired_tform

                frame_task.setTargetFrameTransform(desired_tform)

                try:
                    oink.solveIk(tasks, constraints, scene, delta_q, regularization)
                except RuntimeError as err:
                    print(f"Warning: IK solver failed: {err}")
                    delta_q[:] = 0.0

                q_current = scene.integrate(q_current, delta_q)
                scene.setJointPositions(q_current)
                scene.forwardKinematics(q_current, ee_name)
                viz.display(q_current)

            elapsed = time.monotonic() - loop_start
            stats.add(elapsed)
            now = time.monotonic()
            if now - last_stats_log > 2.0:
                summary = stats.summarize_and_clear()
                if summary is not None:
                    print(summary)
                    timing_label.content = "Loop stats: `" + summary + "`"
                last_stats_log = now

            time.sleep(max(0.0, dt - elapsed))
    finally:
        keyboard.close()


if __name__ == "__main__":
    tyro.cli(main)
