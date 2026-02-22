from __future__ import annotations

import numpy as np
import pinocchio as pin

from .types import CommandFrame, TwistCommand


class TeleopController:
    """Maintains and updates the target SE(3) pose from twist commands."""

    def __init__(
        self,
        initial_target: np.ndarray,
        workspace_min_xyz: tuple[float, float, float],
        workspace_max_xyz: tuple[float, float, float],
        frame_mode: CommandFrame = "ee",
    ):
        if initial_target.shape != (4, 4):
            raise ValueError("initial_target must be a 4x4 homogeneous transform.")

        self._target = np.array(initial_target, dtype=float, copy=True)
        self._workspace_min = np.asarray(workspace_min_xyz, dtype=float)
        self._workspace_max = np.asarray(workspace_max_xyz, dtype=float)
        self._frame_mode = frame_mode

    def set_frame_mode(self, frame_mode: CommandFrame) -> None:
        self._frame_mode = frame_mode

    def reset_to(self, pose: np.ndarray) -> None:
        if pose.shape != (4, 4):
            raise ValueError("pose must be a 4x4 homogeneous transform.")
        self._target = np.array(pose, dtype=float, copy=True)
        self._clamp_workspace()

    def _clamp_workspace(self) -> None:
        self._target[:3, 3] = np.clip(self._target[:3, 3], self._workspace_min, self._workspace_max)

    def update(self, cmd: TwistCommand, dt: float) -> None:
        if dt <= 0.0:
            return

        twist = np.concatenate([cmd.linear_xyz, cmd.angular_xyz]) * dt
        if np.linalg.norm(twist) <= 1e-12:
            return

        delta = pin.exp6(pin.Motion(twist)).homogeneous
        active_frame = cmd.frame if cmd.frame in ("ee", "world") else self._frame_mode
        if active_frame == "ee":
            self._target = self._target @ delta
        else:
            self._target = delta @ self._target

        self._clamp_workspace()

    def get_target(self) -> np.ndarray:
        return np.array(self._target, copy=True)
