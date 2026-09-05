import enum
from typing import overload

import roboplan.core._core_ext


class SplineFittingMode(enum.Enum):
    """Enumeration for TOPP-RA spline fitting mode."""

    Hermite = 0

    Cubic = 1

    Adaptive = 2

    LinearBlend = 3

class TOPPRAOptions:
    """Options controlling TOPP-RA time parameterization."""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, dt: float = 0.01, mode: SplineFittingMode = SplineFittingMode.Hermite, velocity_scale: float = 1.0, acceleration_scale: float = 1.0, max_adaptive_iterations: int = 10, max_adaptive_step_size: float = 0.05, max_blend_deviation: float = 0.01) -> None: ...

    @property
    def dt(self) -> float: ...

    @dt.setter
    def dt(self, arg: float, /) -> None: ...

    @property
    def mode(self) -> SplineFittingMode: ...

    @mode.setter
    def mode(self, arg: SplineFittingMode, /) -> None: ...

    @property
    def velocity_scale(self) -> float: ...

    @velocity_scale.setter
    def velocity_scale(self, arg: float, /) -> None: ...

    @property
    def acceleration_scale(self) -> float: ...

    @acceleration_scale.setter
    def acceleration_scale(self, arg: float, /) -> None: ...

    @property
    def max_adaptive_iterations(self) -> int: ...

    @max_adaptive_iterations.setter
    def max_adaptive_iterations(self, arg: int, /) -> None: ...

    @property
    def max_adaptive_step_size(self) -> float: ...

    @max_adaptive_step_size.setter
    def max_adaptive_step_size(self, arg: float, /) -> None: ...

    @property
    def max_blend_deviation(self) -> float: ...

    @max_blend_deviation.setter
    def max_blend_deviation(self, arg: float, /) -> None: ...

class PathParameterizerTOPPRA:
    """Trajectory time parameterizer using the TOPP-RA algorithm."""

    def __init__(self, scene: roboplan.core._core_ext.Scene, group_name: str = '') -> None: ...

    def generate(self, path: roboplan.core._core_ext.JointPath, options: TOPPRAOptions = ...) -> roboplan.core._core_ext.JointTrajectory:
        """Time-parameterizes a joint-space path using TOPP-RA."""
