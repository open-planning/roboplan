from collections.abc import Sequence
from typing import Annotated

import numpy
from numpy.typing import NDArray

import roboplan_ext.core


class Task:
    """Abstract base class for IK tasks."""

    @property
    def gain(self) -> float:
        """Task gain for low-pass filtering."""

    @property
    def weight(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]:
        """Weight matrix for cost normalization."""

    @property
    def lm_damping(self) -> float:
        """Levenberg-Marquardt damping."""

    @property
    def num_variables(self) -> int:
        """Number of optimization variables."""

class FrameTaskOptions:
    """Options for FrameTask."""

    def __init__(self, position_cost: float = 1.0, orientation_cost: float = 1.0, task_gain: float = 1.0, lm_damping: float = 0.0) -> None: ...

    @property
    def position_cost(self) -> float:
        """Position cost weight."""

    @position_cost.setter
    def position_cost(self, arg: float, /) -> None: ...

    @property
    def orientation_cost(self) -> float:
        """Orientation cost weight."""

    @orientation_cost.setter
    def orientation_cost(self, arg: float, /) -> None: ...

    @property
    def task_gain(self) -> float:
        """Task gain for low-pass filtering."""

    @task_gain.setter
    def task_gain(self, arg: float, /) -> None: ...

    @property
    def lm_damping(self) -> float:
        """Levenberg-Marquardt damping."""

    @lm_damping.setter
    def lm_damping(self, arg: float, /) -> None: ...

class FrameTask(Task):
    """Task to reach a target pose for a specified frame."""

    def __init__(self, frame_name: str, target_pose: roboplan_ext.core.CartesianConfiguration, num_variables: int, options: FrameTaskOptions = ...) -> None: ...

    @property
    def frame_name(self) -> str:
        """Name of the frame to control."""

    @frame_name.setter
    def frame_name(self, arg: str, /) -> None: ...

    @property
    def target_pose(self) -> roboplan_ext.core.CartesianConfiguration:
        """Target pose for the frame."""

    @target_pose.setter
    def target_pose(self, arg: roboplan_ext.core.CartesianConfiguration, /) -> None: ...

class ConfigurationTaskOptions:
    """Options for ConfigurationTask."""

    def __init__(self, task_gain: float = 1.0, lm_damping: float = 0.0) -> None: ...

    @property
    def task_gain(self) -> float:
        """Task gain for low-pass filtering."""

    @task_gain.setter
    def task_gain(self, arg: float, /) -> None: ...

    @property
    def lm_damping(self) -> float:
        """Levenberg-Marquardt damping."""

    @lm_damping.setter
    def lm_damping(self, arg: float, /) -> None: ...

class ConfigurationTask(Task):
    """Task to reach a target joint configuration."""

    def __init__(self, target_q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], joint_weights: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], options: ConfigurationTaskOptions = ...) -> None: ...

    @property
    def target_q(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Target joint configuration."""

    @target_q.setter
    def target_q(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def joint_weights(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Weights for each joint in the configuration task."""

    @joint_weights.setter
    def joint_weights(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class Constraints:
    """Abstract base class for IK constraints."""

class PositionLimit(Constraints):
    """Constraint to enforce joint position limits."""

    def __init__(self, num_variables: int, gain: float = 1.0) -> None: ...

    @property
    def config_limit_gain(self) -> float:
        """Gain for position limit enforcement."""

    @config_limit_gain.setter
    def config_limit_gain(self, arg: float, /) -> None: ...

class VelocityLimit(Constraints):
    """Constraint to enforce joint velocity limits."""

    def __init__(self, num_variables: int, dt: float, v_max: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None: ...

    @property
    def dt(self) -> float:
        """Time step for velocity calculation."""

    @dt.setter
    def dt(self, arg: float, /) -> None: ...

    @property
    def v_max(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Maximum joint velocities."""

    @v_max.setter
    def v_max(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class Oink:
    """Optimal inverse kinematics solver using quadratic programming."""

    def __init__(self, num_variables: int) -> None: ...

    def solveIk(self, tasks: Sequence[Task], constraints: Sequence[Constraints], scene: roboplan_ext.core.Scene) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Solve inverse kinematics with constraints and return delta_q. Raises RuntimeError on failure.
        """
