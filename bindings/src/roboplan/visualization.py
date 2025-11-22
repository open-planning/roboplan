from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np

from roboplan.core import Scene, computeFramePath, JointPath, JointTrajectory
from roboplan.rrt import RRT
from roboplan.viser_visualizer import ViserVisualizer


def visualizePath(
    viz: ViserVisualizer,
    scene: Scene,
    path: JointPath,
    frame_names: list,
    max_step_size: float,
    color: tuple = (100, 0, 0),
    name: str = "/rrt/path",
) -> None:
    """
    Helper function to visualize an RRT path.

    Args
        viz: The viser visualizer instance.
        scene: The scene instance.
        path: The joint path to visualize.
        frame_names: The list of frame names to use for forward kinematics.
        max_step_size: The maximum step size between joint configurations when interpolating paths.
        color: The color of the rendered path.
        name: The name of the path in the vizer window.
    """
    q_start = scene.getCurrentJointPositions()
    q_end = scene.getCurrentJointPositions()
    q_indices = scene.getJointPositionIndices(path.joint_names)
    path_segments = []
    if path is not None:
        for frame_name in frame_names:
            for idx in range(len(path.positions) - 1):
                q_start[q_indices] = path.positions[idx]
                q_end[q_indices] = path.positions[idx + 1]
                frame_path = computeFramePath(
                    scene, q_start, q_end, frame_name, max_step_size
                )
                for idx in range(len(frame_path) - 1):
                    path_segments.append(
                        [frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]]
                    )

    if path_segments:
        viz.viewer.scene.add_line_segments(
            name,
            points=np.array(path_segments),
            colors=color,
            line_width=3.0,
        )


def visualizeTree(
    viz: ViserVisualizer,
    scene: Scene,
    rrt: RRT,
    frame_names: list,
    max_step_size: float,
    start_tree_color: tuple = (0, 100, 100),
    start_tree_name: str = "/rrt/start_tree",
    goal_tree_color: tuple = (100, 0, 100),
    goal_tree_name: str = "/rrt/goal_tree",
) -> None:
    """
    Helper function to visualize the start and goal trees from an RRT planner.

    Args
        viz: The viser visualizer instance.
        scene: The scene instance.
        rrt: The RRT planner instance.
        path: The joint path to visualize. If None, does not visualize the path.
        frame_names: List of frame names to use for forward kinematics.
        max_step_size: The maximum step size between joint configurations when interpolating paths.
        start_tree_color: The color of the rendered start tree.
        start_tree_name: The name of the start tree in the vizer window.
        goal_tree_color: The color of the rendered goal tree.
        goal_tree_name: The name of the goal tree in the vizer window.
    """
    start_nodes, goal_nodes = rrt.getNodes()

    start_segments = []
    for frame_name in frame_names:
        for node in start_nodes[1:]:
            q_start = start_nodes[node.parent_id].config
            q_end = node.config
            frame_path = computeFramePath(
                scene, q_start, q_end, frame_name, max_step_size
            )
            for idx in range(len(frame_path) - 1):
                start_segments.append(
                    [frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]]
                )

    goal_segments = []
    for frame_name in frame_names:
        for node in goal_nodes[1:]:
            q_start = goal_nodes[node.parent_id].config
            q_end = node.config
            frame_path = computeFramePath(
                scene, q_start, q_end, frame_name, max_step_size
            )
            for idx in range(len(frame_path) - 1):
                goal_segments.append(
                    [frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]]
                )

    if start_segments:
        viz.viewer.scene.add_line_segments(
            start_tree_name,
            points=np.array(start_segments),
            colors=start_tree_color,
            line_width=1.0,
        )
    if goal_segments:
        viz.viewer.scene.add_line_segments(
            goal_tree_name,
            points=np.array(goal_segments),
            colors=goal_tree_color,
            line_width=1.0,
        )


def visualizeJointTrajectory(
    trajectory: JointTrajectory, scene: Scene, plot_title: str = "Joint Trajectory"
) -> Figure:
    """
    Visualize a joint trajectory as a plot of joint positions over time.

    Args
        trajectory: The trajectory object to be visualized.
        scene: The Scene object used to get joint information.
        plot_title: The title of the plot.

    Returns
        The matplotlib figure object. Use plt.show() to display it.
    """
    plt.ion()
    plt.plot(trajectory.times, trajectory.positions)
    plt.xlabel("Time")
    plt.ylabel("Joint positions")
    plt.title(plot_title)

    dof_names = []
    for name in trajectory.joint_names:
        nq = scene.getJointInfo(name).num_position_dofs
        if nq == 1:
            dof_names.append(name)
        else:
            dof_names.extend(f"{name}:{idx}" for idx in range(nq))

    plt.legend(dof_names)

    return plt.gcf()
