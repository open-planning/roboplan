from collections.abc import Sequence
from typing import Annotated

import numpy
from numpy.typing import NDArray

import roboplan_ext.core


class Task:
    @property
    def gain(self) -> float: ...

    @property
    def weight(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]: ...

    @property
    def lm_damping(self) -> float: ...

class FrameTask(Task):
    def __init__(self, frame_name: str, target_pose: roboplan_ext.core.CartesianConfiguration, task_weight: float = 1.0, position_cost: float = 1.0, orientation_cost: float = 1.0, task_gain: float = 1.0, lm_damping: float = 0.0) -> None: ...

    @property
    def frame_name(self) -> str: ...

    @frame_name.setter
    def frame_name(self, arg: str, /) -> None: ...

    @property
    def target_pose(self) -> roboplan_ext.core.CartesianConfiguration: ...

    @target_pose.setter
    def target_pose(self, arg: roboplan_ext.core.CartesianConfiguration, /) -> None: ...

    @property
    def position_cost(self) -> float: ...

    @position_cost.setter
    def position_cost(self, arg: float, /) -> None: ...

    @property
    def orientation_cost(self) -> float: ...

    @orientation_cost.setter
    def orientation_cost(self, arg: float, /) -> None: ...

    def computeError(self, arg0: roboplan_ext.core.Scene, arg1: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> "tl::expected<void, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > >": ...

    def computeJacobian(self, arg0: roboplan_ext.core.Scene, arg1: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')], /) -> "tl::expected<void, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > >": ...

class Constraints:
    pass

class PositionLimit(Constraints):
    def __init__(self, gain: float = 1.0) -> None: ...

    @property
    def config_limit_gain(self) -> float: ...

    @config_limit_gain.setter
    def config_limit_gain(self, arg: float, /) -> None: ...

    def computeQpConstraints(self, arg0: roboplan_ext.core.Scene, arg1: Annotated[NDArray[numpy.float64], dict(shape=(None, None))], arg2: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], arg3: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> "tl::expected<void, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > >": ...

class VelocityLimit(Constraints):
    def __init__(self, dt: float, v_max: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None: ...

    @property
    def dt(self) -> float: ...

    @dt.setter
    def dt(self, arg: float, /) -> None: ...

    @property
    def v_max(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @v_max.setter
    def v_max(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    def computeQpConstraints(self, arg0: roboplan_ext.core.Scene, arg1: Annotated[NDArray[numpy.float64], dict(shape=(None, None))], arg2: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], arg3: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> "tl::expected<void, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > >": ...

class Oink:
    def __init__(self, num_variables: int) -> None: ...

    def solveIk(self, tasks: Sequence[Task], constraints: Sequence[Constraints], scene: roboplan_ext.core.Scene) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Solve inverse kinematics with constraints and return delta_q. Raises RuntimeError on failure.
        """
