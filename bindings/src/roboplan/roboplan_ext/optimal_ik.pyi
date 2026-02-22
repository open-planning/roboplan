from collections.abc import Sequence
import enum
from typing import Annotated, overload

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
    """Parameters for FrameTask."""

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

    def __init__(self, target_pose: roboplan_ext.core.CartesianConfiguration, num_variables: int, options: FrameTaskOptions = ...) -> None: ...

    @property
    def frame_name(self) -> str:
        """Name of the frame to control."""

    @property
    def frame_id(self) -> "std::optional<unsigned long>":
        """Index of the frame in the scene's Pinocchio model."""

    @property
    def target_pose(self) -> roboplan_ext.core.CartesianConfiguration:
        """Target pose for the frame."""

    def setTargetFrameTransform(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], /) -> None:
        """Sets the target transform for this frame task."""

class ConfigurationTaskOptions:
    """Parameters for ConfigurationTask."""

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

class CommandFilterConfig:
    """Parameters for teleop command filtering."""

    def __init__(self) -> None: ...

    @property
    def linear_deadzone(self) -> float:
        """Linear deadzone threshold (unitless, [0, 1])."""

    @linear_deadzone.setter
    def linear_deadzone(self, arg: float, /) -> None: ...

    @property
    def angular_deadzone(self) -> float:
        """Angular deadzone threshold (unitless, [0, 1])."""

    @angular_deadzone.setter
    def angular_deadzone(self, arg: float, /) -> None: ...

    @property
    def linear_sensitivity(self) -> float:
        """Linear sensitivity (m/s at unit input)."""

    @linear_sensitivity.setter
    def linear_sensitivity(self, arg: float, /) -> None: ...

    @property
    def angular_sensitivity(self) -> float:
        """Angular sensitivity (rad/s at unit input)."""

    @angular_sensitivity.setter
    def angular_sensitivity(self, arg: float, /) -> None: ...

    @property
    def max_linear_speed(self) -> float:
        """Maximum translational speed (m/s)."""

    @max_linear_speed.setter
    def max_linear_speed(self, arg: float, /) -> None: ...

    @property
    def max_angular_speed(self) -> float:
        """Maximum angular speed (rad/s)."""

    @max_angular_speed.setter
    def max_angular_speed(self, arg: float, /) -> None: ...

    @property
    def smoothing_alpha(self) -> float:
        """EMA smoothing coefficient (unitless, [0, 1])."""

    @smoothing_alpha.setter
    def smoothing_alpha(self, arg: float, /) -> None: ...

class CommandFilter:
    """Applies deadzone, scaling, clamping, and smoothing."""

    def __init__(self, config: CommandFilterConfig = ...) -> None: ...

    def filter(self, raw_twist: Annotated[NDArray[numpy.float64], dict(shape=(None,))]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Filter raw normalized twist [vx, vy, vz, wx, wy, wz] into physical units.
        """

    def reset(self) -> None:
        """Reset internal smoothing state."""

    def setConfig(self, arg: CommandFilterConfig, /) -> None:
        """Set filter configuration."""

    def getConfig(self) -> CommandFilterConfig:
        """Get filter configuration."""

class TeleopFrame(enum.Enum):
    """Frame used for teleop pose integration."""

    EndEffector = 0

    World = 1

class WorkspaceBounds:
    """Axis-aligned workspace bounds."""

    def __init__(self) -> None: ...

    @property
    def min_xyz(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Lower XYZ bound in meters."""

    @min_xyz.setter
    def min_xyz(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def max_xyz(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Upper XYZ bound in meters."""

    @max_xyz.setter
    def max_xyz(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

class TeleopTarget:
    """Maintains and updates a target pose from twist commands."""

    def __init__(self, initial_pose: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], bounds: WorkspaceBounds = ..., frame: TeleopFrame = TeleopFrame.EndEffector) -> None: ...

    @overload
    def update(self, twist: Annotated[NDArray[numpy.float64], dict(shape=(None,))], dt: float) -> None:
        """Integrate twist into target pose using default frame."""

    @overload
    def update(self, twist: Annotated[NDArray[numpy.float64], dict(shape=(None,))], dt: float, frame: TeleopFrame) -> None:
        """Integrate twist into target pose using explicit frame."""

    def resetTo(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], /) -> None:
        """Reset target pose to a specified transform."""

    def getTarget(self) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
        """Get current target pose as a 4x4 transform."""

    def setFrame(self, arg: TeleopFrame, /) -> None:
        """Set default integration frame."""

    def getFrame(self) -> TeleopFrame:
        """Get default integration frame."""

    def setBounds(self, arg: WorkspaceBounds, /) -> None:
        """Set workspace bounds."""

    def getBounds(self) -> WorkspaceBounds:
        """Get workspace bounds."""

class Oink:
    """Optimal Inverse Kinematics solver."""

    def __init__(self, num_variables: int) -> None: ...

    def solveIk(self, tasks: Sequence[Task], constraints: Sequence[Constraints], scene: roboplan_ext.core.Scene, delta_q: Annotated[NDArray[numpy.float64], dict(shape=(None,))], regularization: float = 1e-12) -> None:
        """
        Solve inverse kinematics for given tasks and constraints.

        Solves a QP optimization problem to compute the joint velocity that minimizes
        weighted task errors while satisfying all constraints. The result is written
        directly into the provided delta_q buffer.

        Args:
            tasks: List of weighted tasks to optimize for.
            constraints: List of constraints to satisfy.
            scene: Scene containing robot model and state.
            delta_q: Pre-allocated numpy array for output (size = num_variables).
                     Must be a contiguous float64 array. Modified in-place.
            regularization: Tikhonov regularization weight for the QP Hessian.
                            Higher values improve numerical stability but may reduce
                            task tracking accuracy. Default: 1e-12.

        Raises:
            RuntimeError: If the QP solver fails to find a solution.

        Example:
            delta_q = np.zeros(oink.num_variables)
            oink.solveIk(tasks, constraints, scene, delta_q)
        """
