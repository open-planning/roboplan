from typing import Annotated

import numpy
from numpy.typing import NDArray

import roboplan_ext.core


class Node:
    """Defines a graph node for search-based planners."""

    def __init__(self, config: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], parent_id: int) -> None: ...

    @property
    def config(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The configuration (e.g., joint positions) of this node."""

    @property
    def parent_id(self) -> int:
        """The parent node ID."""

class RRTOptions:
    """Options struct for RRT planner."""

    def __init__(self) -> None: ...

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

class RRT:
    """
    Motion planner based on the Rapidly-exploring Random Tree (RRT) algorithm.
    """

    def __init__(self, scene: roboplan_ext.core.Scene, options: RRTOptions) -> None: ...

    def plan(self, start: roboplan_ext.core.JointConfiguration, goal: roboplan_ext.core.JointConfiguration) -> roboplan_ext.core.JointPath:
        """Plan a path from start to goal."""

    def setRngSeed(self, seed: int) -> None:
        """Sets the seed for the random number generator (RNG)."""

    def getNodes(self) -> tuple[list[Node], list[Node]]:
        """
        Returns the start and goal trees' node vectors, for visualization purposes.
        """
