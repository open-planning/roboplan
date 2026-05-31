import enum

import roboplan.core._core_ext


class SplineFittingMode(enum.Enum):
    """Enumeration for TOPP-RA spline fitting mode."""

    Hermite = 0

    Cubic = 1

    Adaptive = 2

class PathParameterizerTOPPRA:
    """Trajectory time parameterizer using the TOPP-RA algorithm."""

    def __init__(self, scene: roboplan.core._core_ext.Scene, group_name: str = '') -> None: ...

    def generate(self, path: roboplan.core._core_ext.JointPath, dt: float, mode: SplineFittingMode = SplineFittingMode.Hermite, velocity_scale: float = 1.0, acceleration_scale: float = 1.0, max_adaptive_iterations: int = 10, max_adaptive_step_size: float = 0.05) -> roboplan.core._core_ext.JointTrajectory:
        """Time-parameterizes a joint-space path using TOPP-RA."""
