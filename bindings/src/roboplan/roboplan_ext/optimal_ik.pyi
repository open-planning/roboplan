"""Optimal IK solver module"""

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

    def __init__(self, oink: Oink, scene: roboplan_ext.core.Scene, target_pose: roboplan_ext.core.CartesianConfiguration, options: FrameTaskOptions = ...) -> None: ...

    @property
    def frame_name(self) -> str:
        """Name of the frame to control."""

    @property
    def frame_id(self) -> int:
        """Index of the frame in the scene's Pinocchio model."""

    @property
    def v_indices(self) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """Velocity vector indices for the joint group."""

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

    def __init__(self, oink: Oink, target_q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], joint_weights: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], options: ConfigurationTaskOptions = ...) -> None: ...

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

    def __init__(self, oink: Oink, gain: float = 1.0) -> None: ...

    @property
    def config_limit_gain(self) -> float:
        """Gain for position limit enforcement."""

    @config_limit_gain.setter
    def config_limit_gain(self, arg: float, /) -> None: ...

class VelocityLimit(Constraints):
    """Constraint to enforce joint velocity limits."""

    def __init__(self, oink: Oink, dt: float, v_max: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None: ...

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

class ConstraintAxisSelection:
    """Axis selection for position barrier constraints."""

    def __init__(self, x: bool = True, y: bool = True, z: bool = True) -> None:
        """Constructor with axis enable flags."""

    @property
    def x(self) -> bool:
        """Constrain X axis."""

    @x.setter
    def x(self, arg: bool, /) -> None: ...

    @property
    def y(self) -> bool:
        """Constrain Y axis."""

    @y.setter
    def y(self, arg: bool, /) -> None: ...

    @property
    def z(self) -> bool:
        """Constrain Z axis."""

    @z.setter
    def z(self, arg: bool, /) -> None: ...

class PositionBarrier(Barrier):
    """
    Position barrier constraint that keeps a frame within an axis-aligned bounding box.
    """

    def __init__(self, oink: Oink, scene: roboplan_ext.core.Scene, frame_name: str, p_min: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], p_max: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], dt: float, axis_selection: ConstraintAxisSelection = ..., gain: float = 1.0, safe_displacement_gain: float = 1.0, safety_margin: float = 0.0) -> None:
        """Create a position barrier with optional axis selection."""

    def get_frame_position(self, scene: roboplan_ext.core.Scene) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Get the current frame position in world coordinates."""

    @property
    def frame_name(self) -> str:
        """Name of the constrained frame."""

    @property
    def axis_selection(self) -> ConstraintAxisSelection:
        """Axis selection for constraints."""

    @property
    def p_min(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Minimum position bounds."""

    @property
    def p_max(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Maximum position bounds."""

class Oink:
    """Optimal Inverse Kinematics solver."""

    @overload
    def __init__(self, scene: roboplan_ext.core.Scene, group_name: str) -> None:
        """Constructor for a named joint group."""

    @overload
    def __init__(self, scene: roboplan_ext.core.Scene) -> None:
        """Constructor for the full robot (all joints)."""

    @property
    def num_variables(self) -> int:
        """Number of optimization variables."""

    @property
    def q_indices(self) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """Position indices of the joint group."""

    @property
    def v_indices(self) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """Velocity indices of the joint group."""

    @overload
    def solveIk(self, scene: roboplan_ext.core.Scene, tasks: Sequence[Task], constraints: Sequence[Constraints], barriers: Sequence[Barrier], delta_q: Annotated[NDArray[numpy.float64], dict(shape=(None,))], regularization: float = 1e-12) -> None:
        """
        Solve inverse kinematics for given tasks, constraints, and optional barriers.

        Solves a QP optimization problem to compute the joint velocity that minimizes
        weighted task errors while satisfying all constraints and barrier functions.
        The result is written directly into the provided delta_q buffer.

        Args:
            tasks: List of weighted tasks to optimize for.
            constraints: List of constraints to satisfy.
            barriers: List of barrier functions for safety constraints (default: []).
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
            oink.solveIk(scene, tasks, constraints, [], delta_q)

            # With barriers:
            oink.solveIk(scene, tasks, constraints, barriers, delta_q)

            # With custom regularization:
            oink.solveIk(scene, tasks, constraints, barriers, delta_q, 1e-6)
        """

    @overload
    def solveIk(self, scene: roboplan_ext.core.Scene, tasks: Sequence[Task], delta_q: Annotated[NDArray[numpy.float64], dict(shape=(None,))], regularization: float = 1e-12) -> None:
        """
        Solve inverse kinematics for tasks only (no constraints or barriers).

        Args:
            tasks: List of weighted tasks to optimize for.
            delta_q: Pre-allocated numpy array for output (size = num_variables).
            regularization: Tikhonov regularization weight (default: 1e-12).

        Example:
            delta_q = np.zeros(oink.num_variables)
            oink.solveIk(scene, tasks, delta_q)
        """

    @overload
    def solveIk(self, scene: roboplan_ext.core.Scene, tasks: Sequence[Task], constraints: Sequence[Constraints], delta_q: Annotated[NDArray[numpy.float64], dict(shape=(None,))], regularization: float = 1e-12) -> None:
        """
        Solve inverse kinematics for tasks with constraints (no barriers).

        Args:
            tasks: List of weighted tasks to optimize for.
            constraints: List of constraints to satisfy.
            delta_q: Pre-allocated numpy array for output (size = num_variables).
            regularization: Tikhonov regularization weight (default: 1e-12).

        Example:
            delta_q = np.zeros(oink.num_variables)
            oink.solveIk(scene, tasks, constraints, delta_q)
        """

    @overload
    def solveIk(self, scene: roboplan_ext.core.Scene, tasks: Sequence[Task], barriers: Sequence[Barrier], delta_q: Annotated[NDArray[numpy.float64], dict(shape=(None,))], regularization: float = 1e-12) -> None:
        """
        Solve inverse kinematics for tasks with barriers (no constraints).

        Args:
            tasks: List of weighted tasks to optimize for.
            barriers: List of barrier functions for safety constraints.
            delta_q: Pre-allocated numpy array for output (size = num_variables).
            regularization: Tikhonov regularization weight (default: 1e-12).

        Example:
            delta_q = np.zeros(oink.num_variables)
            oink.solveIk(scene, tasks, barriers, delta_q)
        """

    def enforceBarriers(self, scene: roboplan_ext.core.Scene, barriers: Sequence[Barrier], delta_q: Annotated[NDArray[numpy.float64], dict(shape=(None,))], tolerance: float = 0.0) -> None:
        """
        Validate delta_q against barriers using forward kinematics.

        This method provides a post-solve safety check by evaluating the actual barrier
        values at the candidate configuration (q + delta_q). If any barrier would be
        violated, delta_q is set to zero to prevent unsafe motion.

        This is a backup safety mechanism for cases where the linearized CBF constraint
        in the QP has significant error (e.g., large jumps, near-boundary configurations).

        Args:
            barriers: List of barrier functions to check.
            delta_q: Configuration displacement to validate. Modified in place: set to
                     zero if barrier violation is detected.
            tolerance: Tolerance for barrier violation detection. A barrier is considered
                       violated if h(q + delta_q) < -tolerance. Default is 0.0.

        Raises:
            RuntimeError: If barrier evaluation fails (e.g., frame not found).

        Example:
            delta_q = np.zeros(oink.num_variables)
            oink.solveIk(scene, tasks, constraints, barriers, delta_q)
            oink.enforceBarriers(scene, barriers, delta_q)
        """
