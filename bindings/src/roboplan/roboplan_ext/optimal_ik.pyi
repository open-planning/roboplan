from collections.abc import Sequence
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

    def __init__(self, position_cost: float = 1.0, orientation_cost: float = 1.0, task_gain: float = 1.0, lm_damping: float = 0.0, max_position_error: float = float('inf'), max_rotation_error: float = float('inf')) -> None:
        """Constructor with custom parameters."""

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

    @property
    def max_position_error(self) -> float:
        """Maximum position error magnitude (meters). Infinite = no limit."""

    @max_position_error.setter
    def max_position_error(self, arg: float, /) -> None: ...

    @property
    def max_rotation_error(self) -> float:
        """Maximum rotation error magnitude (radians). Infinite = no limit."""

    @max_rotation_error.setter
    def max_rotation_error(self, arg: float, /) -> None: ...

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

    @property
    def max_position_error(self) -> float:
        """Maximum position error magnitude (meters)."""

    @property
    def max_rotation_error(self) -> float:
        """Maximum rotation error magnitude (radians)."""

    def setTargetFrameTransform(self, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]) -> None:
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

class Barrier:
    """Abstract base class for Control Barrier Functions."""

    def get_num_barriers(self, scene: roboplan_ext.core.Scene) -> int:
        """Get the number of barrier constraints."""

    @property
    def gain(self) -> float:
        """Barrier gain (gamma)."""

    @property
    def dt(self) -> float:
        """Timestep."""

    @property
    def safe_displacement_gain(self) -> float:
        """Gain for safe displacement regularization."""

    @property
    def safety_margin(self) -> float:
        """Conservative margin for hard constraints."""

class PositionBarrier(Barrier):
    """
    Position barrier constraint that keeps a frame within an axis-aligned bounding box.
    """

    @overload
    def __init__(self, frame_name: str, p_min: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], p_max: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], num_variables: int, gain: float = 1.0, dt: float = 0.01, safe_displacement_gain: float = 1.0, safety_margin: float = 0.0) -> None:
        """Create a position barrier for all 3 axes (x, y, z)."""

    @overload
    def __init__(self, frame_name: str, indices: Sequence[int], p_min: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], p_max: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], num_variables: int, gain: float = 1.0, dt: float = 0.01, safe_displacement_gain: float = 1.0, safety_margin: float = 0.0) -> None:
        """Create a position barrier for selected axes only."""

    def get_frame_position(self, scene: roboplan_ext.core.Scene) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Get the current frame position in world coordinates."""

    @property
    def frame_name(self) -> str:
        """Name of the constrained frame."""

    @property
    def indices(self) -> list[int]:
        """Constrained axis indices."""

    @property
    def p_min(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Minimum position bounds."""

    @property
    def p_max(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Maximum position bounds."""

class Oink:
    """Optimal Inverse Kinematics solver."""

    def __init__(self, num_variables: int) -> None:
        """Constructor with number of optimization variables."""

    @property
    def num_variables(self) -> int:
        """Number of optimization variables."""

    def solveIk(self, tasks: Sequence[Task], constraints: Sequence[Constraints], barriers: Sequence[Barrier], scene: roboplan_ext.core.Scene, delta_q: Annotated[NDArray[numpy.float64], dict(shape=(None,))], regularization: float = 1e-12) -> None:
        """
        Solve inverse kinematics for given tasks, constraints, and optional barriers.

        Solves a QP optimization problem to compute the joint velocity that minimizes
        weighted task errors while satisfying all constraints and barrier functions.
        The result is written directly into the provided delta_q buffer.

        Args:
            tasks: List of weighted tasks to optimize for.
            constraints: List of constraints to satisfy.
            barriers: List of barrier functions for safety constraints (default: []).
            scene: Scene containing robot model and state.
            delta_q: Pre-allocated numpy array for output (size = num_variables).
                     Must be a contiguous float64 array. Modified in-place.
            regularization: Tikhonov regularization weight for the QP Hessian
                            (default: 1e-12). Higher values improve numerical stability
                            but may reduce task tracking accuracy.

        Raises:
            RuntimeError: If the QP solver fails to find a solution.

        Examples:
            # Without barriers:
            delta_q = np.zeros(oink.num_variables)
            oink.solveIk(tasks, constraints, [], scene, delta_q)

            # With barriers:
            oink.solveIk(tasks, constraints, barriers, scene, delta_q)

            # With custom regularization:
            oink.solveIk(tasks, constraints, barriers, scene, delta_q, 1e-6)
        """
