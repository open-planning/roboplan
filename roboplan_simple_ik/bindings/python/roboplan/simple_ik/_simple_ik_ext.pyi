from collections.abc import Sequence
from typing import overload

import roboplan.core._core_ext


class SimpleIkOptions:
    """Options struct for simple IK solver."""

    def __init__(self, group_name: str = '', max_iters: int = 100, max_time: float = 0.005, max_restarts: int = 2, step_size: float = 0.25, damping: float = 0.001, max_linear_error_norm: float = 0.001, max_angular_error_norm: float = 0.001, check_collisions: bool = True, fast_return: bool = True) -> None: ...

    @property
    def group_name(self) -> str:
        """The joint group name to be used by the solver."""

    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...

    @property
    def max_iters(self) -> int:
        """Max iterations for one try of the solver."""

    @max_iters.setter
    def max_iters(self, arg: int, /) -> None: ...

    @property
    def max_time(self) -> float:
        """Max total computation time, in seconds."""

    @max_time.setter
    def max_time(self, arg: float, /) -> None: ...

    @property
    def max_restarts(self) -> int:
        """Maximum number of restarts until success."""

    @max_restarts.setter
    def max_restarts(self, arg: int, /) -> None: ...

    @property
    def step_size(self) -> float:
        """The integration step for the solver."""

    @step_size.setter
    def step_size(self, arg: float, /) -> None: ...

    @property
    def damping(self) -> float:
        """Damping value for the Jacobian pseudoinverse."""

    @damping.setter
    def damping(self, arg: float, /) -> None: ...

    @property
    def max_linear_error_norm(self) -> float:
        """The maximum linear error norm, in meters."""

    @max_linear_error_norm.setter
    def max_linear_error_norm(self, arg: float, /) -> None: ...

    @property
    def max_angular_error_norm(self) -> float:
        """The maximum angular error norm, in radians."""

    @max_angular_error_norm.setter
    def max_angular_error_norm(self, arg: float, /) -> None: ...

    @property
    def check_collisions(self) -> bool:
        """Whether to check collisions."""

    @check_collisions.setter
    def check_collisions(self, arg: bool, /) -> None: ...

    @property
    def fast_return(self) -> bool:
        """If true, returns when the first ik solution is found."""

    @fast_return.setter
    def fast_return(self, arg: bool, /) -> None: ...

class SimpleIk:
    """
    Simple inverse kinematics (IK) solver based on the Jacobian pseudoinverse.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, options: SimpleIkOptions) -> None: ...

    @overload
    def solveIk(self, goal: roboplan.core._core_ext.CartesianConfiguration, start: roboplan.core._core_ext.JointConfiguration, solution: roboplan.core._core_ext.JointConfiguration) -> bool:
        """Solves inverse kinematics (single goal)."""

    @overload
    def solveIk(self, goals: Sequence[roboplan.core._core_ext.CartesianConfiguration], start: roboplan.core._core_ext.JointConfiguration, solution: roboplan.core._core_ext.JointConfiguration) -> bool:
        """Solves inverse kinematics (multiple goal)."""
