import enum

import roboplan.core._core_ext


class CartesianSpeedMode(enum.Enum):
    """Selects how the planner assigns speed/timing along the path."""

    Constant = 0
    """Trace the path at a (roughly) constant Cartesian tool speed."""

    Toppra = 1
    """Time-optimal re-timing respecting joint limits (not yet implemented)."""

class CartesianPlannerOptions:
    """Options for the Cartesian path planner."""

    def __init__(self, group_name: str = '', dt: float = 0.01, linear_speed: float = 0.1, angular_speed: float = 0.5, max_position_error: float = 0.005, max_orientation_error: float = 0.01, speed_mode: CartesianSpeedMode = CartesianSpeedMode.Constant, position_cost: float = 1.0, orientation_cost: float = 1.0, task_gain: float = 1.0, lm_damping: float = 0.01, regularization: float = 1e-06, config_task_weight: float = 0.05, velocity_scale: float = 1.0, acceleration_scale: float = 1.0, toppra_blend_deviation: float = 0.05, position_limit_gain: float = 1.0, max_attempts_per_step: int = 16) -> None: ...

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
    def linear_speed(self) -> float:
        """Commanded linear tool speed (m/s)."""

    @linear_speed.setter
    def linear_speed(self, arg: float, /) -> None: ...

    @property
    def angular_speed(self) -> float:
        """Commanded angular tool speed (rad/s)."""

    @angular_speed.setter
    def angular_speed(self, arg: float, /) -> None: ...

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
    def speed_mode(self) -> CartesianSpeedMode:
        """Speed/timing strategy."""

    @speed_mode.setter
    def speed_mode(self, arg: CartesianSpeedMode, /) -> None: ...

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
        """Scaling factor for joint acceleration limits (Toppra mode)."""

    @acceleration_scale.setter
    def acceleration_scale(self, arg: float, /) -> None: ...

    @property
    def toppra_blend_deviation(self) -> float:
        """Corner-rounding tolerance (rad) for the Toppra line+blend geometry."""

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

class CartesianPlanResult:
    """Result of a successful Cartesian plan."""

    @property
    def trajectory(self) -> roboplan.core._core_ext.JointTrajectory:
        """The joint trajectory that traces the path."""

    @property
    def achieved_path_length(self) -> float:
        """Achieved Cartesian path length (m)."""

    @property
    def feedrate_efficiency(self) -> float:
        """Fraction of control steps run at the full commanded feedrate."""

    @property
    def peak_velocity_ratio(self) -> float:
        """Peak |joint velocity| / velocity-limit ratio over the trajectory."""

    @property
    def peak_acceleration_ratio(self) -> float:
        """
        Peak |joint acceleration| / acceleration-limit ratio over the trajectory.
        """

class CartesianPathPlanner:
    """
    Offline Cartesian path planner that traces a CartesianPath in joint space using Oink.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, options: CartesianPlannerOptions) -> None: ...

    def plan(self, path: roboplan.core._core_ext.CartesianPath, q_start: roboplan.core._core_ext.JointConfiguration) -> CartesianPlanResult:
        """Plans a joint trajectory that traces the provided Cartesian path."""
