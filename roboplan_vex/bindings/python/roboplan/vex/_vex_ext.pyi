import enum

import roboplan.core._core_ext


class VexStatus(enum.Enum):
    """Termination status of a vex solve."""

    Converged = 0
    """
    Converged with all nonconvex constraints satisfied within the tolerance.
    """

    ConvergedInfeasible = 1
    """
    Converged, but nonconvex constraints are still violated after the maximum number of penalty increases.
    """

    MaxIterations = 2
    """Stopped after `max_iterations` QP iterations."""

    QpFailure = 3
    """A convex subproblem could not be solved."""

    Timeout = 4
    """Stopped after `max_time` seconds."""

class VexOptions:
    """
    Options of the vex trajectory optimizer. Joint limits are read from the Scene.
    """

    def __init__(self) -> None: ...

    @property
    def group_name(self) -> str:
        """The name of the joint group to optimize."""

    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...

    @property
    def num_nodes(self) -> int:
        """The number of trajectory nodes N (>= 3), uniformly spaced in time."""

    @num_nodes.setter
    def num_nodes(self, arg: int, /) -> None: ...

    @property
    def samples_per_segment(self) -> int:
        """
        Collision samples per segment, in addition to the nodes (0 = nodes only).
        """

    @samples_per_segment.setter
    def samples_per_segment(self, arg: int, /) -> None: ...

    @property
    def velocity_weight(self) -> float:
        """Weight of the path-energy cost, the integral of ||v(t)||^2."""

    @velocity_weight.setter
    def velocity_weight(self, arg: float, /) -> None: ...

    @property
    def acceleration_weight(self) -> float:
        """Weight of the smoothing cost, the integral of ||a(t)||^2."""

    @acceleration_weight.setter
    def acceleration_weight(self, arg: float, /) -> None: ...

    @property
    def collision_margin(self) -> float:
        """
        Minimum signed distance d_safe (m) between every active collision pair.
        """

    @collision_margin.setter
    def collision_margin(self, arg: float, /) -> None: ...

    @property
    def collision_check_distance(self) -> float:
        """
        Pairs closer than this distance d_check (m) enter the QP; must exceed d_safe.
        """

    @collision_check_distance.setter
    def collision_check_distance(self, arg: float, /) -> None: ...

    @property
    def initial_penalty(self) -> float:
        """Initial l1 penalty weight on nonconvex constraint violations."""

    @initial_penalty.setter
    def initial_penalty(self, arg: float, /) -> None: ...

    @property
    def penalty_increase(self) -> float:
        """Factor by which the penalty grows when converged but still infeasible."""

    @penalty_increase.setter
    def penalty_increase(self, arg: float, /) -> None: ...

    @property
    def max_penalty_increases(self) -> int:
        """Maximum number of penalty increases."""

    @max_penalty_increases.setter
    def max_penalty_increases(self, arg: int, /) -> None: ...

    @property
    def initial_trust_box(self) -> float:
        """Initial half-width of the trust box, in position units."""

    @initial_trust_box.setter
    def initial_trust_box(self, arg: float, /) -> None: ...

    @property
    def trust_shrink(self) -> float:
        """Factor applied to the trust box after a rejected step."""

    @trust_shrink.setter
    def trust_shrink(self, arg: float, /) -> None: ...

    @property
    def trust_expand(self) -> float:
        """Factor applied to the trust box after an accepted step."""

    @trust_expand.setter
    def trust_expand(self, arg: float, /) -> None: ...

    @property
    def improve_ratio(self) -> float:
        """
        Minimum ratio of actual to predicted merit improvement to accept a step.
        """

    @improve_ratio.setter
    def improve_ratio(self, arg: float, /) -> None: ...

    @property
    def max_iterations(self) -> int:
        """
        Maximum number of SCP iterations (QP solves after the initial projection).
        """

    @max_iterations.setter
    def max_iterations(self, arg: int, /) -> None: ...

    @property
    def max_time(self) -> float:
        """Maximum wall time of the SCP loop, in seconds."""

    @max_time.setter
    def max_time(self, arg: float, /) -> None: ...

    @property
    def tolerance(self) -> float:
        """
        Tolerance on constraint violation, predicted merit improvement and trust box size.
        """

    @tolerance.setter
    def tolerance(self, arg: float, /) -> None: ...

class VexResult:
    """Result of a vex solve."""

    @property
    def trajectory(self) -> roboplan.core._core_ext.JointTrajectory:
        """The optimized trajectory over the group joints. Not retimed."""

    @property
    def status(self) -> VexStatus:
        """The termination status."""

    @property
    def iterations(self) -> int:
        """The number of SCP iterations."""

    @property
    def cost(self) -> float:
        """The final convex (smoothness) cost."""

    @property
    def max_violation(self) -> float:
        """The final maximum violation of the nonconvex constraints."""

    @property
    def continuous_collision_free(self) -> bool:
        """Whether a dense collision post-check over all active pairs passed."""

class Vex:
    """
    Offline, kinematic-only trajectory optimizer based on sequential convex programming.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, options: VexOptions) -> None: ...

    def solve(self, start: roboplan.core._core_ext.JointConfiguration, goal: roboplan.core._core_ext.JointConfiguration, duration: float, seed: roboplan.core._core_ext.JointPath) -> VexResult:
        """
        Optimizes a rest-to-rest trajectory of the given duration (s) from start to goal, starting from the seed path. Raises RuntimeError for invalid input, a start or goal in collision, or an infeasible problem.
        """
