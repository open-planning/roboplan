from collections.abc import Sequence
import enum
from typing import overload

import roboplan.core._core_ext
import roboplan.optimal_ik._optimal_ik_ext


class CartesianSpeedMode(enum.Enum):
    """Selects how the planner assigns speed/timing along the path."""

    Bounded = 0
    """Trace the path under bounded Cartesian velocity and acceleration."""

    TimeOptimal = 1
    """
    Time-optimal re-timing respecting joint velocity and acceleration limits.
    """

class CartesianPlannerOptions:
    """Options for the Cartesian path planner."""

    def __init__(self, group_name: str = '', dt: float = 0.01, speed_mode: CartesianSpeedMode = CartesianSpeedMode.Bounded, max_linear_speed: float = 0.1, max_angular_speed: float = 0.5, max_linear_acceleration: float = 0.5, max_angular_acceleration: float = 2.5, max_position_error: float = 0.005, max_orientation_error: float = 0.01, position_cost: float = 1.0, orientation_cost: float = 1.0, task_gain: float = 1.0, lm_damping: float = 0.01, regularization: float = 1e-06, config_task_weight: float = 0.05, velocity_scale: float = 1.0, acceleration_scale: float = 1.0, limit_ratio_tolerance: float = 1.05, toppra_blend_deviation: float = 0.05, position_limit_gain: float = 1.0, max_attempts_per_step: int = 16) -> None: ...

    @property
    def group_name(self) -> str:
        """Joint group name."""

    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...

    @property
    def dt(self) -> float:
        """Output trajectory sample period (s)."""

    @dt.setter
    def dt(self, arg: float, /) -> None: ...

    @property
    def speed_mode(self) -> CartesianSpeedMode:
        """Speed/timing strategy."""

    @speed_mode.setter
    def speed_mode(self, arg: CartesianSpeedMode, /) -> None: ...

    @property
    def max_linear_speed(self) -> float:
        """Maximum linear tool speed (m/s)."""

    @max_linear_speed.setter
    def max_linear_speed(self, arg: float, /) -> None: ...

    @property
    def max_angular_speed(self) -> float:
        """Maximum angular tool speed (rad/s)."""

    @max_angular_speed.setter
    def max_angular_speed(self, arg: float, /) -> None: ...

    @property
    def max_linear_acceleration(self) -> float:
        """Maximum linear tool acceleration (m/s^2), Bounded mode."""

    @max_linear_acceleration.setter
    def max_linear_acceleration(self, arg: float, /) -> None: ...

    @property
    def max_angular_acceleration(self) -> float:
        """Maximum angular tool acceleration (rad/s^2), Bounded mode."""

    @max_angular_acceleration.setter
    def max_angular_acceleration(self, arg: float, /) -> None: ...

    @property
    def max_position_error(self) -> float:
        """Maximum position deviation from the path (m)."""

    @max_position_error.setter
    def max_position_error(self, arg: float, /) -> None: ...

    @property
    def max_orientation_error(self) -> float:
        """Maximum orientation deviation from the path (rad)."""

    @max_orientation_error.setter
    def max_orientation_error(self, arg: float, /) -> None: ...

    @property
    def position_cost(self) -> float:
        """Oink frame task position cost."""

    @position_cost.setter
    def position_cost(self, arg: float, /) -> None: ...

    @property
    def orientation_cost(self) -> float:
        """Oink frame task orientation cost."""

    @orientation_cost.setter
    def orientation_cost(self, arg: float, /) -> None: ...

    @property
    def task_gain(self) -> float:
        """Oink frame task gain."""

    @task_gain.setter
    def task_gain(self, arg: float, /) -> None: ...

    @property
    def lm_damping(self) -> float:
        """Oink frame task Levenberg-Marquardt damping."""

    @lm_damping.setter
    def lm_damping(self, arg: float, /) -> None: ...

    @property
    def regularization(self) -> float:
        """Tikhonov regularization for the Oink QP."""

    @regularization.setter
    def regularization(self, arg: float, /) -> None: ...

    @property
    def config_task_weight(self) -> float:
        """Weight of the nullspace configuration-regularization task."""

    @config_task_weight.setter
    def config_task_weight(self, arg: float, /) -> None: ...

    @property
    def velocity_scale(self) -> float:
        """Scaling factor for joint velocity limits."""

    @velocity_scale.setter
    def velocity_scale(self, arg: float, /) -> None: ...

    @property
    def acceleration_scale(self) -> float:
        """
        Scaling factor for joint acceleration limits (TimeOptimal re-timing and Bounded joint-acceleration throttle).
        """

    @acceleration_scale.setter
    def acceleration_scale(self, arg: float, /) -> None: ...

    @property
    def limit_ratio_tolerance(self) -> float:
        """
        Acceptance tolerance (>= 1.0) for the Bounded mode's slow-down retry; the peak velocity/acceleration ratios may exceed the (scaled) limits by up to this factor.
        """

    @limit_ratio_tolerance.setter
    def limit_ratio_tolerance(self, arg: float, /) -> None: ...

    @property
    def toppra_blend_deviation(self) -> float:
        """
        Corner-rounding tolerance (rad) for the TimeOptimal mode's TOPP-RA line+blend geometry.
        """

    @toppra_blend_deviation.setter
    def toppra_blend_deviation(self, arg: float, /) -> None: ...

    @property
    def position_limit_gain(self) -> float:
        """Gain for the joint position-limit constraint."""

    @position_limit_gain.setter
    def position_limit_gain(self, arg: float, /) -> None: ...

    @property
    def max_attempts_per_step(self) -> int:
        """Maximum feedrate-throttling attempts per control step."""

    @max_attempts_per_step.setter
    def max_attempts_per_step(self, arg: int, /) -> None: ...

class CartesianPlannerComponents:
    """
    Caller-supplied OInK solver and IK objectives for the Cartesian path planner.
    """

    def __init__(self) -> None: ...

    @property
    def oink(self) -> roboplan.optimal_ik._optimal_ik_ext.Oink:
        """The OInK solver to use."""

    @oink.setter
    def oink(self, arg: roboplan.optimal_ik._optimal_ik_ext.Oink, /) -> None: ...

    @property
    def tracking_tasks(self) -> list[roboplan.optimal_ik._optimal_ik_ext.FrameTask]:
        """
        FrameTasks that the planner updates each step, one per end-effector (ordered to match the path's tip frames).
        """

    @tracking_tasks.setter
    def tracking_tasks(self, arg: Sequence[roboplan.optimal_ik._optimal_ik_ext.FrameTask], /) -> None: ...

    @property
    def extra_tasks(self) -> list[roboplan.optimal_ik._optimal_ik_ext.Task]:
        """Additional tasks solved alongside the tracking tasks."""

    @extra_tasks.setter
    def extra_tasks(self, arg: Sequence[roboplan.optimal_ik._optimal_ik_ext.Task], /) -> None: ...

    @property
    def constraints(self) -> list[roboplan.optimal_ik._optimal_ik_ext.Constraints]:
        """Constraints applied at every control step."""

    @constraints.setter
    def constraints(self, arg: Sequence[roboplan.optimal_ik._optimal_ik_ext.Constraints], /) -> None: ...

    @property
    def barriers(self) -> list[roboplan.optimal_ik._optimal_ik_ext.Barrier]:
        """Control barrier functions applied at every control step."""

    @barriers.setter
    def barriers(self, arg: Sequence[roboplan.optimal_ik._optimal_ik_ext.Barrier], /) -> None: ...

class CartesianPathPlanner:
    """
    Offline Cartesian path planner that traces a CartesianPath in joint space using Oink.
    """

    @overload
    def __init__(self, scene: roboplan.core._core_ext.Scene, options: CartesianPlannerOptions) -> None: ...

    @overload
    def __init__(self, scene: roboplan.core._core_ext.Scene, options: CartesianPlannerOptions, components: CartesianPlannerComponents) -> None:
        """
        Constructs a planner that uses a caller-supplied OInK solver and objectives.
        """

    def plan(self, path: roboplan.core._core_ext.CartesianPath, q_start: roboplan.core._core_ext.JointConfiguration) -> roboplan.core._core_ext.JointTrajectory:
        """Plans a joint trajectory that traces the provided Cartesian path."""

    def compute_peak_limit_ratios(self, trajectory: roboplan.core._core_ext.JointTrajectory) -> tuple[float, float]:
        """
        Computes the (peak velocity / limit, peak acceleration / limit) ratios over a trajectory.
        """

    def compute_achieved_path_length(self, trajectory: roboplan.core._core_ext.JointTrajectory, path: roboplan.core._core_ext.CartesianPath) -> float:
        """
        Computes the achieved Cartesian path length (m) traced by the path's tip frames.
        """
