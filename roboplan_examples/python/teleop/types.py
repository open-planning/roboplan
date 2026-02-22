from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Literal

import numpy as np
from numpy.typing import NDArray


CommandFrame = Literal["ee", "world"]
TeleopMode = Literal["velocity_drive", "position_hold", "paused"]


def _zero3() -> NDArray[np.float64]:
    return np.zeros(3, dtype=float)


@dataclass
class TwistCommand:
    """Timestamped Cartesian command for teleoperation."""

    linear_xyz: NDArray[np.float64] = field(default_factory=_zero3)
    angular_xyz: NDArray[np.float64] = field(default_factory=_zero3)
    gripper: float = 0.0
    frame: CommandFrame = "ee"
    stamp_ns: int = field(default_factory=time.monotonic_ns)

    @staticmethod
    def zero(frame: CommandFrame = "ee", stamp_ns: int | None = None) -> "TwistCommand":
        if stamp_ns is None:
            stamp_ns = time.monotonic_ns()
        return TwistCommand(frame=frame, stamp_ns=stamp_ns)

    def is_zero(self, tol: float = 1e-12) -> bool:
        return (
            float(np.linalg.norm(self.linear_xyz)) <= tol
            and float(np.linalg.norm(self.angular_xyz)) <= tol
            and abs(self.gripper) <= tol
        )


@dataclass
class TeleopState:
    """Runtime state for user-facing teleop status."""

    enabled: bool = True
    input_alive: bool = True
    active_device: str = "keyboard"
    mode: TeleopMode = "velocity_drive"
