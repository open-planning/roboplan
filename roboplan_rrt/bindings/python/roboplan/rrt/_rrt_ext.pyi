from collections.abc import Sequence
import roboplan.core._core_ext
from typing import Annotated

import numpy
from numpy.typing import NDArray


class Constraint:
    """
    Base class for kinematic constraints that planners enforce by projection.
    """

    def dimension(self) -> int:
        """The number of residual coordinates this constraint contributes."""

    def getGroupName(self) -> str:
        """
        The joint group whose degrees of freedom a projection is allowed to move.
        """

    def computeError(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Computes the constraint residual at a configuration, which is zero where the constraint is satisfied and otherwise the signed overshoot past the nearest bound.
        """

    def satisfies(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> bool:
        """Checks whether a configuration satisfies this constraint."""

class PoseConstraint(Constraint):
    """
    Constrains the pose of a robot frame to a box of allowed task space displacements.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, group_name: str, frame_name: str, lower_bounds: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')] = ..., upper_bounds: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')] = ..., tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')] = ..., reference_frame: str = '', position_tolerance: float = 0.001, orientation_tolerance: float = 0.005) -> None: ...

    def computeDisplacement(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]:
        """
        Computes the constrained frame's [x, y, z, roll, pitch, yaw] displacement from the region frame.
        """

    @property
    def frame_name(self) -> str:
        """The name of the constrained frame."""

    @property
    def reference_frame(self) -> str:
        """The name of the frame the region transform is expressed in."""

    @property
    def tform(self) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
        """The pose of the region frame relative to the reference frame."""

    @property
    def lower_bounds(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]:
        """The lower bounds on the frame's task space displacement."""

    @property
    def upper_bounds(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]:
        """The upper bounds on the frame's task space displacement."""

    @property
    def position_tolerance(self) -> float:
        """
        The position residual norm, in meters, below which the constraint is satisfied.
        """

    @property
    def orientation_tolerance(self) -> float:
        """
        The orientation residual norm, in radians, below which the constraint is satisfied.
        """

class ConstraintProjectorOptions:
    """Options struct for constraint projection."""

    def __init__(self, path_step_size: float = 0.1, max_iters: int = 50, correction_step_size: float = 1.0, damping: float = 1e-06, convergence_ratio: float = 0.1) -> None: ...

    @property
    def path_step_size(self) -> float:
        """The configuration-space step size taken between projections."""

    @path_step_size.setter
    def path_step_size(self, arg: float, /) -> None: ...

    @property
    def max_iters(self) -> int:
        """The maximum number of projection iterations before giving up."""

    @max_iters.setter
    def max_iters(self, arg: int, /) -> None: ...

    @property
    def correction_step_size(self) -> float:
        """
        The fraction of each computed correction to apply per projection iteration.
        """

    @correction_step_size.setter
    def correction_step_size(self, arg: float, /) -> None: ...

    @property
    def damping(self) -> float:
        """Damping value for the Jacobian pseudoinverse."""

    @damping.setter
    def damping(self, arg: float, /) -> None: ...

    @property
    def convergence_ratio(self) -> float:
        """
        The fraction of each constraint's tolerance the projection converges to, leaving headroom for the interpolation between projected configurations.
        """

    @convergence_ratio.setter
    def convergence_ratio(self, arg: float, /) -> None: ...

class ConstraintProjector:
    """
    Projects joint configurations onto the intersection of a set of constraints.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, group_name: str, constraints: Sequence[Constraint], options: ConstraintProjectorOptions = ...) -> None: ...

    def satisfies(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> bool:
        """Checks whether a configuration satisfies every constraint."""

    def project(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None:
        """
        Projects a configuration onto the constraints, returning None if the projection did not converge.
        """

    def satisfiesAlongPath(self, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], max_step_size: float, check_endpoints: bool = True) -> bool:
        """
        Checks whether a straight-line configuration-space path stays inside the constraints.
        """

    def empty(self) -> bool:
        """Whether the projector holds no constraints, and is therefore a no-op."""

class Node:
    """Defines a graph node for search-based planners."""

    def __init__(self, config: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], parent_id: int) -> None: ...

    @property
    def config(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The configuration (e.g., joint positions) of this node."""

    @property
    def parent_id(self) -> int:
        """The parent node ID."""

    @property
    def cost(self) -> float:
        """The cost-to-come from the tree root to this node (RRT* only)."""

class RRTOptions:
    """Options struct for RRT planner."""

    def __init__(self, group_name: str = '', max_nodes: int = 1000, max_connection_distance: float = 3.0, collision_check_step_size: float = 0.05, collision_check_use_bisection: bool = True, goal_biasing_probability: float = 0.15, max_planning_time: float = 0.0, rrt_connect: bool = False, rrt_star: bool = False, rewire_distance: float = 5.0, fast_return: bool = True, constraint_projection: ConstraintProjectorOptions = ...) -> None: ...

    @property
    def group_name(self) -> str:
        """The joint group name to be used by the planner."""

    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...

    @property
    def max_nodes(self) -> int:
        """The maximum number of nodes to sample."""

    @max_nodes.setter
    def max_nodes(self, arg: int, /) -> None: ...

    @property
    def max_connection_distance(self) -> float:
        """The maximum configuration distance between two nodes."""

    @max_connection_distance.setter
    def max_connection_distance(self, arg: float, /) -> None: ...

    @property
    def collision_check_step_size(self) -> float:
        """The configuration-space step size for collision checking along edges."""

    @collision_check_step_size.setter
    def collision_check_step_size(self, arg: float, /) -> None: ...

    @property
    def collision_check_use_bisection(self) -> bool:
        """
        If true, uses bisection instead of linear search for collision checking along edges.
        """

    @collision_check_use_bisection.setter
    def collision_check_use_bisection(self, arg: bool, /) -> None: ...

    @property
    def goal_biasing_probability(self) -> float:
        """The probability of sampling the goal node instead of a random node."""

    @goal_biasing_probability.setter
    def goal_biasing_probability(self, arg: float, /) -> None: ...

    @property
    def max_planning_time(self) -> float:
        """The maximum amount of time to allow for planning, in seconds."""

    @max_planning_time.setter
    def max_planning_time(self, arg: float, /) -> None: ...

    @property
    def rrt_connect(self) -> bool:
        """If true, use the RRT-Connect algorithm to grow the search trees."""

    @rrt_connect.setter
    def rrt_connect(self, arg: bool, /) -> None: ...

    @property
    def rrt_star(self) -> bool:
        """If true, use the RRT* algorithm to grow asymptotically optimal trees."""

    @rrt_star.setter
    def rrt_star(self, arg: bool, /) -> None: ...

    @property
    def rewire_distance(self) -> float:
        """
        The configuration-space radius used to find neighbors for RRT* rewiring.
        """

    @rewire_distance.setter
    def rewire_distance(self, arg: float, /) -> None: ...

    @property
    def fast_return(self) -> bool:
        """
        If true, return on the first path found; if false, plan until the budget is exhausted and return the lowest-cost path.
        """

    @fast_return.setter
    def fast_return(self, arg: bool, /) -> None: ...

    @property
    def constraint_projection(self) -> ConstraintProjectorOptions:
        """
        Options for the projection that pulls sampled configurations onto the constraints.
        """

    @constraint_projection.setter
    def constraint_projection(self, arg: ConstraintProjectorOptions, /) -> None: ...

class RRT:
    """
    Motion planner based on the Rapidly-exploring Random Tree (RRT) algorithm.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, options: RRTOptions) -> None: ...

    def setOptions(self, options: RRTOptions) -> None:
        """Sets or updates the options for the RRT planner."""

    def plan(self, start: roboplan.core._core_ext.JointConfiguration, goal: roboplan.core._core_ext.JointConfiguration, constraints: Sequence[Constraint] = []) -> roboplan.core._core_ext.JointPath:
        """
        Plan a path from start to goal, optionally subject to constraints that every configuration on the path must satisfy.
        """

    def setRngSeed(self, seed: int) -> None:
        """Sets the seed for the random number generator (RNG)."""

    def getNodes(self) -> tuple[list[Node], list[Node]]:
        """
        Returns the start and goal trees' node vectors, for visualization purposes.
        """
