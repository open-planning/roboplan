from collections.abc import Sequence
from typing import Annotated

import numpy
from numpy.typing import NDArray
import roboplan.core._core_ext


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
        """
        The cost-to-come from the tree root to this node. Zero unless rrt_star is true or fast_return is false.
        """

class RRTOptions:
    """Options struct for RRT planner."""

    def __init__(self, group_name: str = '', max_nodes: int = 1000, max_connection_distance: float = 3.0, collision_check_step_size: float = 0.05, collision_check_use_bisection: bool = True, goal_biasing_probability: float = 0.15, max_planning_time: float = 0.0, rrt_connect: bool = False, rrt_star: bool = False, rewire_distance: float = 5.0, fast_return: bool = True, constraint_projection: roboplan.core._core_ext.ConstraintProjectorOptions = ...) -> None: ...

    @property
    def group_name(self) -> str:
        """The joint group name to be used by the planner."""

    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...

    @property
    def max_nodes(self) -> int:
        """
        The maximum number of nodes to sample. This includes the start and all goal configurations.
        """

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
        """
        The probability of sampling the goal node instead of a random node. Must be between 0 and 1. Ignored when `rrt_connect` is true.
        """

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
    def constraint_projection(self) -> roboplan.core._core_ext.ConstraintProjectorOptions:
        """
        Options for the projection that pulls sampled configurations onto the constraints. Only used when `plan` or `planToAny` is given constraints.
        """

    @constraint_projection.setter
    def constraint_projection(self, arg: roboplan.core._core_ext.ConstraintProjectorOptions, /) -> None: ...

class RRTPlan:
    """The result of planning to one of a set of goals."""

    @property
    def path(self) -> roboplan.core._core_ext.JointPath:
        """The joint-space path from the start to the reached goal."""

    @property
    def goal_index(self) -> int:
        """
        The index of the goal the path ends at, in the order the goals were given.
        """

class RRT:
    """
    Motion planner based on the Rapidly-exploring Random Tree (RRT) algorithm.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, options: RRTOptions) -> None: ...

    def setOptions(self, options: RRTOptions) -> None:
        """Sets or updates the options for the RRT planner."""

    def plan(self, start: roboplan.core._core_ext.JointConfiguration, goal: roboplan.core._core_ext.JointConfiguration, constraints: Sequence[roboplan.core._core_ext.Constraint] = []) -> roboplan.core._core_ext.JointPath:
        """
        Plan a path from start to goal, optionally subject to constraints that every configuration on the path must satisfy.
        """

    def planToAny(self, start: roboplan.core._core_ext.JointConfiguration, goals: Sequence[roboplan.core._core_ext.JointConfiguration], constraints: Sequence[roboplan.core._core_ext.Constraint] = []) -> RRTPlan:
        """
        Plan a path from start to any one of a set of goals, optionally subject to constraints that every configuration on the path must satisfy. Returns the path along with the index of the goal it reached. If any goal can be reached by a direct connection, the closest such goal is returned immediately, regardless of fast_return.
        """

    def setRngSeed(self, seed: int) -> None:
        """Sets the seed for the random number generator (RNG)."""

    def getNodes(self) -> tuple[list[Node], list[Node]]:
        """
        Returns the start and goal trees' node vectors, for visualization purposes.
        """
