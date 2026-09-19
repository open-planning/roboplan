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
    def dt(self) -> float:
        """
        The sample time of the output trajectory, in seconds. Must be strictly positive.
        """

    @dt.setter
    def dt(self, arg: float, /) -> None: ...

    @property
    def mode(self) -> SplineFittingMode:
        """
        The mode to use for spline fitting the path. Hermite fits a cubic Hermite spline with zero velocity at all waypoints, which can be slow but adheres to the path exactly. Cubic fits a smoother spline with zero acceleration only at the endpoints; it can deviate from the path, so it is collision checked and falls back to Hermite. Adaptive uses Cubic but adds intermediate points where it finds collisions, up to max_adaptive_iterations, then falls back to Hermite. LinearBlend joins straight-line segments with circular corner blends (see max_blend_deviation), is collision checked, and falls back to Hermite.
        """

    @mode.setter
    def mode(self, arg: SplineFittingMode, /) -> None: ...

    @property
    def velocity_scale(self) -> float:
        """A scaling factor in (0, 1] for velocity limits."""

    @velocity_scale.setter
    def velocity_scale(self, arg: float, /) -> None: ...

    @property
    def acceleration_scale(self) -> float:
        """A scaling factor in (0, 1] for acceleration limits."""

    @acceleration_scale.setter
    def acceleration_scale(self, arg: float, /) -> None: ...

    @property
    def max_adaptive_iterations(self) -> int:
        """Maximum number of adaptive iterations, if adaptive mode is enabled."""

    @max_adaptive_iterations.setter
    def max_adaptive_iterations(self, arg: int, /) -> None: ...

    @property
    def max_adaptive_step_size(self) -> float:
        """
        If adaptive mode is enabled, the maximum joint configuration step size to sample generated splines for collision checking.
        """

    @max_adaptive_step_size.setter
    def max_adaptive_step_size(self, arg: float, /) -> None: ...

    @property
    def max_blend_deviation(self) -> float:
        """
        Maximum distance a corner blend may deviate from the sharp corner, in the joint configuration's units. Only used by LinearBlend. Larger values round corners more (faster, but the path strays further from the waypoints); values <= 0 disable blending.
        """

    @max_blend_deviation.setter
    def max_blend_deviation(self, arg: float, /) -> None: ...

class PathParameterizerTOPPRA:
    """Trajectory time parameterizer using the TOPP-RA algorithm."""

    def __init__(self, scene: roboplan.core._core_ext.Scene, group_name: str = '') -> None: ...

    def generate(self, path: roboplan.core._core_ext.JointPath, options: TOPPRAOptions = ...) -> roboplan.core._core_ext.JointTrajectory:
        """Time-parameterizes a joint-space path using TOPP-RA."""
