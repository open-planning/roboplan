from collections.abc import Mapping
from typing import Annotated

import numpy
from numpy.typing import NDArray
import roboplan.core._core_ext


class OMPLPlannerOptions:
    """Options struct for the OMPL planner wrapper."""

    def __init__(self, group_name: str = '', planner_name: str = 'RRTConnect', max_planning_time: float = 5.0, collision_check_step_size: float = 0.05, collision_check_use_bisection: bool = True, simplify_path: bool = False, max_simplification_time: float = 0.0, planner_params: Mapping[str, str] = {}) -> None: ...

    @property
    def group_name(self) -> str:
        """The joint group name to be used by the planner."""

    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...

    @property
    def planner_name(self) -> str:
        """
        The OMPL planner to use. See OMPLPlanner.plannerNames() for the supported names.
        """

    @planner_name.setter
    def planner_name(self, arg: str, /) -> None: ...

    @property
    def max_planning_time(self) -> float:
        """
        The planning time budget, in seconds. Anytime planners such as AORRTC use all of it.
        """

    @max_planning_time.setter
    def max_planning_time(self, arg: float, /) -> None: ...

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
    def simplify_path(self) -> bool:
        """If true, runs OMPL's path simplifier on the solution."""

    @simplify_path.setter
    def simplify_path(self, arg: bool, /) -> None: ...

    @property
    def max_simplification_time(self) -> float:
        """
        The time budget for path simplification, in seconds. If <= 0, simplifies until the path stops improving.
        """

    @max_simplification_time.setter
    def max_simplification_time(self, arg: float, /) -> None: ...

    @property
    def planner_params(self) -> dict[str, str]:
        """
        Planner-specific parameters, forwarded by name to the OMPL planner (e.g. 'range').
        """

    @planner_params.setter
    def planner_params(self, arg: Mapping[str, str], /) -> None: ...

class OMPLPlanStats:
    """Timing and bookkeeping from the most recent plan."""

    @property
    def planning_time(self) -> float:
        """The time the planner spent searching, in seconds."""

    @property
    def simplification_time(self) -> float:
        """The time spent simplifying the path, in seconds."""

    @property
    def num_state_checks(self) -> int:
        """The number of single-state validity checks."""

    @property
    def num_motion_checks(self) -> int:
        """The number of edge (motion) validity checks."""

class OMPLPlanner:
    """
    Motion planner that runs OMPL's sampling-based planners against a roboplan Scene.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, options: OMPLPlannerOptions) -> None: ...

    def setOptions(self, options: OMPLPlannerOptions) -> None:
        """
        Sets or updates the planner options. Rebuilds the state space if the group changed.
        """

    def plan(self, start: roboplan.core._core_ext.JointConfiguration, goal: roboplan.core._core_ext.JointConfiguration) -> roboplan.core._core_ext.JointPath:
        """Plans a path from start to goal."""

    @staticmethod
    def setRngSeed(seed: int) -> None:
        """
        Seeds OMPL's random number generation. This is global to all OMPL planners in the process and only takes effect if called before any of them has run.
        """

    def getLastStats(self) -> OMPLPlanStats:
        """Timing and bookkeeping from the most recent plan."""

    def getNodes(self) -> list[tuple[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], int]]:
        """
        The vertices of the search graph from the last plan, as (expanded group positions, OMPL vertex tag) pairs. RRT-Connect tags its start tree 1 and its goal tree 2.
        """

    def getEdges(self) -> list[tuple[int, int]]:
        """
        The edges of the search graph from the last plan, as index pairs into getNodes().
        """

    @staticmethod
    def plannerNames() -> list[str]:
        """The OMPL planner names accepted by OMPLPlannerOptions.planner_name."""
