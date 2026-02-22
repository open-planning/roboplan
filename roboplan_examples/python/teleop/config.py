from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal


@dataclass
class KeyboardKeyMapping:
    """Default keyboard mapping for 6-DOF teleoperation."""

    tx_pos: str = "w"
    tx_neg: str = "s"
    ty_pos: str = "a"
    ty_neg: str = "d"
    tz_pos: str = "q"
    tz_neg: str = "e"
    rx_pos: str = "i"
    rx_neg: str = "k"
    ry_pos: str = "j"
    ry_neg: str = "l"
    rz_pos: str = "u"
    rz_neg: str = "o"
    gripper_open: str = "g"
    gripper_close: str = "h"
    pause_toggle: str = "space"
    reset_home: str = "r"
    reset_target_to_current: str = "t"


@dataclass
class TeleopConfig:
    """Configuration for keyboard teleoperation behavior (units noted per field)."""

    linear_sensitivity: float = 0.12  # [m/s] translation speed at unit input
    angular_sensitivity: float = 0.60  # [rad/s] angular speed at unit input
    linear_deadzone: float = 0.05  # [unitless] normalized input threshold in [0, 1]
    angular_deadzone: float = 0.05  # [unitless] normalized input threshold in [0, 1]
    smoothing_alpha: float = 0.35  # [unitless] EMA blend factor in [0, 1]
    max_linear_speed: float = 0.25  # [m/s] post-filter translational speed clamp
    max_angular_speed: float = 1.50  # [rad/s] post-filter angular speed clamp
    input_timeout_s: float = 0.20  # [s] stale-input timeout before zeroing command
    workspace_min_xyz: tuple[float, float, float] = (-1.2, -1.2, -0.05)  # [m]
    workspace_max_xyz: tuple[float, float, float] = (1.2, 1.2, 1.6)  # [m]
    key_hold_s: float = 0.12  # [s] terminal key press hold duration
    default_frame: Literal["ee", "world"] = "ee"
    key_mapping: KeyboardKeyMapping = field(default_factory=KeyboardKeyMapping)
