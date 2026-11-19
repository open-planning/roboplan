from roboplan_common import add_dll_directories

_dll_directories = add_dll_directories()

import numpy as np
from pinocchio.visualize import ViserVisualizer

# The `roboplan.ompl` bindings, backed by the compiled `_ompl_ext` module.
# Import core first to guarantee its types are registered before use.
from roboplan.core import Scene, computeFramePath

from ._ompl_ext import *
from ._ompl_ext import __version__  # noqa: F401


def visualizeGraph(
    viz: ViserVisualizer,
    scene: Scene,
    planner: OMPLPlanner,
    group_name: str,
    frame_names: list,
    max_step_size: float,
    start_tree_color: tuple = (0, 100, 100),
    start_tree_name: str = "/ompl/start_tree",
    goal_tree_color: tuple = (100, 0, 100),
    goal_tree_name: str = "/ompl/goal_tree",
) -> None:
    """
    Helper function to visualize the search graph from an OMPL planner's last plan.

    Edges from vertices OMPL tags as the goal tree (RRT-Connect's tag 2) are drawn as the goal
    tree; everything else is drawn as the start tree.

    Args:
        viz: The Viser visualizer instance.
        scene: The scene instance.
        planner: The OMPL planner instance.
        group_name: The joint group the planner was configured with.
        frame_names: List of frame names to use for forward kinematics.
        max_step_size: The maximum step size between joint configurations when interpolating edges.
        start_tree_color: The color of the rendered start tree.
        start_tree_name: The name of the start tree in Viser.
        goal_tree_color: The color of the rendered goal tree.
        goal_tree_name: The name of the goal tree in Viser.
    """
    nodes = planner.getNodes()
    q_full = [scene.toFullJointPositions(group_name, q) for q, _ in nodes]
    segments = {start_tree_name: [], goal_tree_name: []}
    for frame_name in frame_names:
        for i, j in planner.getEdges():
            name = goal_tree_name if nodes[i][1] == 2 else start_tree_name
            frame_path = computeFramePath(
                scene, q_full[i], q_full[j], frame_name, max_step_size
            )
            for idx in range(len(frame_path) - 1):
                segments[name].append(
                    [frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]]
                )

    colors = {start_tree_name: start_tree_color, goal_tree_name: goal_tree_color}
    for name, segs in segments.items():
        if segs:
            viz.viewer.scene.add_line_segments(
                name, points=np.array(segs), colors=colors[name], line_width=1.0
            )
