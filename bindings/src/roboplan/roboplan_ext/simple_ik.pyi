import roboplan_ext.core


class SimpleIkOptions:
    """Options struct for simple IK solver."""

    def __init__(self) -> None: ...

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
    def max_error_norm(self) -> float:
        """The maximum error norm."""

    @max_error_norm.setter
    def max_error_norm(self, arg: float, /) -> None: ...

    @property
    def check_collisions(self) -> bool:
        """Whether to check collisions."""

    @check_collisions.setter
    def check_collisions(self, arg: bool, /) -> None: ...

class SimpleIk:
    """
    Simple inverse kinematics (IK) solver based on the Jacobian pseudoinverse.
    """

    def __init__(self, scene: roboplan_ext.core.Scene, options: SimpleIkOptions) -> None: ...

    def solveIk(self, goal: roboplan_ext.core.CartesianConfiguration, start: roboplan_ext.core.JointConfiguration, solution: roboplan_ext.core.JointConfiguration) -> bool:
        """Solves inverse kinematics."""
