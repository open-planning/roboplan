from __future__ import annotations

from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

from roboplan.core import (
    Scene,
    computeFramePath,
    collapseContinuousJointPositions,
    JointPath,
    JointTrajectory,
)


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


def plotJointTrajectory(
    trajectory: JointTrajectory,
    scene: Scene,
    group_name: str = "",
    title: str = "Joint Trajectory",
    positions: bool = True,
    velocities: bool = False,
    accelerations: bool = False,
) -> Figure:
    """
    Plot a joint trajectory over time.

    By default only positions are plotted. Enabling velocities and/or
    accelerations adds them as additional stacked subplots sharing a common
    time axis.

    Positions are plotted in collapsed (velocity-DOF) coordinates so they line up
    with the velocity and acceleration curves: each continuous joint's (cos, sin)
    position pair is collapsed back to a single angle.

    Args:
        trajectory: The trajectory object to visualize.
        scene: The Scene object used to get joint information.
        group_name: The joint group the trajectory belongs to, used to collapse the
            continuous-joint positions. Empty means the full robot.
        title: The title of the plot.
        positions: Whether to add a subplot of joint positions.
        velocities: Whether to add a subplot of joint velocities.
        accelerations: Whether to add a subplot of joint accelerations.

    Returns:
        The matplotlib figure object. Use ``matplotlib.pyplot.show()`` to display it.
    """
    dof_names = []
    for name in trajectory.joint_names:
        nv = scene.getJointInfo(name).num_velocity_dofs
        if nv == 1:
            dof_names.append(name)
        else:
            dof_names.extend(f"{name}:{idx}" for idx in range(nv))

    subplots = []
    if positions:
        # Trajectory positions are stored expanded (continuous joints as cos/sin pairs);
        # collapse them to velocity-DOF coordinates so they match the velocity/acceleration
        # curves and the dof_names above.
        collapsed_positions = [
            collapseContinuousJointPositions(scene, group_name, q)
            for q in trajectory.positions
        ]
        subplots.append(("Joint positions", collapsed_positions))
    if velocities:
        subplots.append(("Joint velocities", trajectory.velocities))
    if accelerations:
        subplots.append(("Joint accelerations", trajectory.accelerations))
    if not subplots:
        raise ValueError(
            "At least one of positions, velocities, or accelerations must be True."
        )

    fig = plt.gcf()
    fig.clear()
    axes = fig.subplots(len(subplots), 1, sharex=True, squeeze=False)[:, 0]

    for ax, (ylabel, values) in zip(axes, subplots):
        ax.plot(trajectory.times, values)
        ax.set_ylabel(ylabel)
        ax.legend(dof_names)

    axes[-1].set_xlabel("Time")
    fig.suptitle(title)

    return fig


def addPositionPolyline(
    viz: ViserVisualizer,
    name: str,
    positions: np.ndarray,
    color: tuple[int, int, int],
    line_width: float,
) -> None:
    """
    Draw a Cartesian position trajectory as straight line segments.

    Args:
        viz: The Viser visualizer instance.
        name: The name of the trace in Viser.
        positions: Array of xyz positions with shape (N, 3).
        color: RGB color tuple.
        line_width: Width of the rendered line.
    """
    if positions is None or len(positions) < 2:
        return

    line_segments = np.stack([positions[:-1], positions[1:]], axis=1)

    viz.viewer.scene.add_line_segments(
        name,
        points=line_segments,
        colors=np.asarray(color, dtype=np.uint8),
        line_width=line_width,
    )


def visualizePositionTrace(
    viz: ViserVisualizer,
    positions: np.ndarray,
    trace_name: str,
    waypoint_root: str,
    trace_color: tuple[int, int, int],
    waypoint_color: tuple[int, int, int],
    line_width: float = 5.0,
    waypoint_radius: float = 0.01,
    draw_trace: bool = True,
    draw_waypoints: bool = True,
    waypoint_stride: int = 1,
) -> None:
    """
    Visualize a Cartesian position trace and optional waypoint markers.

    Args:
        viz: The Viser visualizer instance.
        positions: Array of xyz positions with shape (N, 3).
        trace_name: Name of the line trace in Viser.
        waypoint_root: Root name for waypoint markers in Viser.
        trace_color: RGB color tuple for the trace.
        waypoint_color: RGB color tuple for waypoint markers.
        line_width: Width of the rendered line.
        waypoint_radius: Radius of waypoint marker spheres.
        draw_trace: Whether to draw the line trace.
        draw_waypoints: Whether to draw waypoint markers.
        waypoint_stride: Draw one waypoint marker every this many positions.
    """
    if positions is None or len(positions) == 0:
        return

    if draw_trace:
        addPositionPolyline(
            viz,
            trace_name,
            positions,
            trace_color,
            line_width,
        )

    if not draw_waypoints:
        return

    stride = max(1, waypoint_stride)
    waypoint_indices = list(range(0, len(positions), stride))

    if waypoint_indices[-1] != len(positions) - 1:
        waypoint_indices.append(len(positions) - 1)

    for idx in range(0, len(positions), stride):
        viz.viewer.scene.add_icosphere(
            f"{waypoint_root}/{idx}",
            radius=waypoint_radius,
            color=waypoint_color,
            position=positions[idx],
        )


def se3_to_viser_wxyz(transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Extract Viser-compatible position and wxyz quaternion from an SE(3) matrix.

    Args:
        transform: 4x4 homogeneous transform.

    Returns:
        Tuple of ``(position, wxyz)`` as numpy arrays.
    """
    position = transform[:3, 3].copy()
    quat = pin.Quaternion(transform[:3, :3])
    wxyz = np.array([quat.w, quat.x, quat.y, quat.z])
    return position, wxyz
