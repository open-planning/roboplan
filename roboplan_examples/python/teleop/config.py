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
    """Configuration for keyboard teleoperation behavior."""

    linear_sensitivity: float = 0.12
    angular_sensitivity: float = 0.60
    linear_deadzone: float = 0.05
    angular_deadzone: float = 0.05
    smoothing_alpha: float = 0.35
    max_linear_speed: float = 0.25
    max_angular_speed: float = 1.50
    input_timeout_s: float = 0.20
    workspace_min_xyz: tuple[float, float, float] = (-1.2, -1.2, -0.05)
    workspace_max_xyz: tuple[float, float, float] = (1.2, 1.2, 1.6)
    key_hold_s: float = 0.12
    default_frame: Literal["ee", "world"] = "ee"
    key_mapping: KeyboardKeyMapping = field(default_factory=KeyboardKeyMapping)
