from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

from roboplan.core import Scene, computeFramePath, JointPath, JointTrajectory
from roboplan.rrt import RRT


# TODO: Remove this function when this OcTree visualization support in Viser is added into pinocchio.
# For more information, see https://github.com/stack-of-tasks/pinocchio/issues/2868
# Inspired from https://github.com/stack-of-tasks/pinocchio/blob/655877b314baed68c7e2d4dd56b0a0200bb9f98e/bindings/python/pinocchio/visualize/meshcat_visualizer.py#L235-L295
def visualizeOcTree(
    viz: ViserVisualizer,
    octree_geometry: pin.GeometryObject,
    prefix: str | None,
) -> None:
    """
    Helper function to visualize octree geometries on Viser.

    Args:
        viz: The viser visualizer instance.
        octree_geometry: the octree geometry object
        prefix: the prefix for geometry name
    """
    name = octree_geometry.name
    if prefix:
        name = prefix + "/" + name

    geom = octree_geometry.geometry
    color = octree_geometry.meshColor

    boxes = geom.toBoxes()
    if len(boxes) == 0:
        return

    bs = boxes[0][3] / 2.0
    num_boxes = len(boxes)

    box_corners = np.array(
        [
            [bs, bs, bs],
            [bs, bs, -bs],
            [bs, -bs, bs],
            [bs, -bs, -bs],
            [-bs, bs, bs],
            [-bs, bs, -bs],
            [-bs, -bs, bs],
            [-bs, -bs, -bs],
        ]
    )

    all_points = np.empty((8 * num_boxes, 3))
    all_faces = np.empty((12 * num_boxes, 3), dtype=int)
    face_id = 0
    for box_id, box_properties in enumerate(boxes):
        box_center = box_properties[:3]

        corners = box_corners + box_center
        point_range = range(box_id * 8, (box_id + 1) * 8)
        all_points[point_range, :] = corners

        A = box_id * 8
        B = A + 1
        C = B + 1
        D = C + 1
        E = D + 1
        F = E + 1
        G = F + 1
        H = G + 1

        all_faces[face_id] = np.array([C, D, B])
        all_faces[face_id + 1] = np.array([B, A, C])
        all_faces[face_id + 2] = np.array([A, B, F])
        all_faces[face_id + 3] = np.array([F, E, A])
        all_faces[face_id + 4] = np.array([E, F, H])
        all_faces[face_id + 5] = np.array([H, G, E])
        all_faces[face_id + 6] = np.array([G, H, D])
        all_faces[face_id + 7] = np.array([D, C, G])
        # # top
        all_faces[face_id + 8] = np.array([A, E, G])
        all_faces[face_id + 9] = np.array([G, C, A])
        # # bottom
        all_faces[face_id + 10] = np.array([B, H, F])
        all_faces[face_id + 11] = np.array([H, B, D])

        face_id += 12

    frame = viz.viewer.scene.add_mesh_simple(
        name,
        all_points,
        all_faces,
        color=color[:3],
        opacity=color[3],
    )

    viz.frames[name] = frame


def visualizePath(
    viz: ViserVisualizer,
    scene: Scene,
    path: JointPath | None,
    frame_names: list,
    max_step_size: float,
    color: tuple = (100, 0, 0),
    name: str = "/path",
) -> None:
    """
    Helper function to visualize a sparse joint path in Cartesian space, using interpolation.

    Args:
        viz: The Viser visualizer instance.
        scene: The scene instance.
        path: The joint path to visualize.
        frame_names: The list of frame names to use for forward kinematics.
        max_step_size: The maximum step size between joint configurations when interpolating paths.
        color: The color of the rendered path.
        name: The name of the path in Viser.
    """
    if path is None:
        return

    q_start = scene.getCurrentJointPositions()
    q_end = scene.getCurrentJointPositions()
    q_indices = scene.getJointPositionIndices(path.joint_names)
    path_segments = []
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

    viz.viewer.scene.add_line_segments(
        name,
        points=np.array(path_segments),
        colors=color,
        line_width=3.0,
    )


def visualizeJointTrajectory(
    viz: ViserVisualizer,
    scene: Scene,
    traj: JointTrajectory | None,
    frame_names: list,
    color: tuple = (100, 0, 0),
    name: str = "/trajectory",
) -> None:
    """
    Helper function to visualize a joint trajectory in Cartesian space.

    Args:
        viz: The Viser visualizer instance.
        scene: The scene instance.
        traj: The joint trajectory to visualize.
        frame_names: The list of frame names to use for forward kinematics.
        color: The color of the rendered trajectory.
        name: The name of the trajectory in Viser.
    """
    if traj is None:
        return

    q_indices = scene.getJointPositionIndices(traj.joint_names)
    q_vec = np.tile(scene.getCurrentJointPositions(), (len(traj.positions), 1))
    q_vec[:, q_indices] = traj.positions

    path_segments = []
    for frame_name in frame_names:
        frame_path = computeFramePath(scene, q_vec, frame_name)
        for idx in range(len(frame_path) - 1):
            path_segments.append([frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]])

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

    Args:
        viz: The Viser visualizer instance.
        scene: The scene instance.
        rrt: The RRT planner instance.
        frame_names: List of frame names to use for forward kinematics.
        max_step_size: The maximum step size between joint configurations when interpolating paths.
        start_tree_color: The color of the rendered start tree.
        start_tree_name: The name of the start tree in Viser.
        goal_tree_color: The color of the rendered goal tree.
        goal_tree_name: The name of the goal tree in Viser.
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


def plotJointTrajectory(
    trajectory: JointTrajectory, scene: Scene, plot_title: str = "Joint Trajectory"
) -> Figure:
    """
    Plot a joint trajectory of positions over time.

    Args:
        trajectory: The trajectory object to visualize.
        scene: The Scene object used to get joint information.
        plot_title: The title of the plot.

    Returns:
        The matplotlib figure object. Use ``matplotlib.pyplot.show()`` to display it.
    """
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
