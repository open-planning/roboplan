import numpy as np

from .config import TeleopConfig
from .types import TwistCommand


class CommandFilter:
    """Applies deadzone, scaling, clipping, and smoothing to teleop commands."""

    def __init__(self):
        self._prev_linear = np.zeros(3, dtype=float)
        self._prev_angular = np.zeros(3, dtype=float)

    def reset(self) -> None:
        self._prev_linear[:] = 0.0
        self._prev_angular[:] = 0.0

    def _apply_deadzone(self, values: np.ndarray, threshold: float) -> np.ndarray:
        out = values.copy()
        out[np.abs(out) < threshold] = 0.0
        return out

    def filter(self, raw_cmd: TwistCommand, config: TeleopConfig) -> TwistCommand:
        linear = self._apply_deadzone(
            np.asarray(raw_cmd.linear_xyz, dtype=float), config.linear_deadzone
        )
        angular = self._apply_deadzone(
            np.asarray(raw_cmd.angular_xyz, dtype=float), config.angular_deadzone
        )

        linear = np.clip(linear, -1.0, 1.0) * config.linear_sensitivity
        angular = np.clip(angular, -1.0, 1.0) * config.angular_sensitivity

        linear = np.clip(linear, -config.max_linear_speed, config.max_linear_speed)
        angular = np.clip(angular, -config.max_angular_speed, config.max_angular_speed)

        alpha = float(np.clip(config.smoothing_alpha, 0.0, 1.0))
        linear = alpha * linear + (1.0 - alpha) * self._prev_linear
        angular = alpha * angular + (1.0 - alpha) * self._prev_angular

        self._prev_linear = linear
        self._prev_angular = angular

        return TwistCommand(
            linear_xyz=linear,
            angular_xyz=angular,
            gripper=float(np.clip(raw_cmd.gripper, -1.0, 1.0)),
            frame=raw_cmd.frame,
            stamp_ns=raw_cmd.stamp_ns,
        )
