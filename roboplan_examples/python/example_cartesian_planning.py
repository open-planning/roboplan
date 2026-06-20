#!/usr/bin/env python3

import sys
import time
import tyro
import xacro

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

from common import get_home_configuration, get_model_data
from roboplan.core import Scene, JointConfiguration, CartesianPath
from roboplan.example_models import get_package_share_dir
from roboplan.cartesian_planning import (
    CartesianPathPlanner,
    CartesianPlannerOptions,
    CartesianSpeedMode,
)
from roboplan.visualization import (
    plotJointTrajectory,
    visualizeJointTrajectory,
    visualizePositionTrace,
)


def round_corners(
    vertices: list[np.ndarray],
    radius: float,
    max_arc_step_deg: float = 1.0,
    min_arc_length: float = 0.0,
) -> list[np.ndarray]:
    """
    Rounds the interior corners of a polyline (list of 3D points) with circular arcs of the
    given radius (meters), returning a denser list of points that traces straight legs joined
    by tangent arcs. This is the task-space "blend": each corner is replaced by an arc of the
    requested radius, tangent to both adjacent legs. The tangent length is clamped to half of
    each adjacent segment so neighbouring arcs never overlap (the effective radius shrinks at
    corners whose legs are too short). A radius <= 0 leaves the corners sharp.

    Corners whose rounded arc would be shorter than `min_arc_length` meters are left sharp
    instead. Such a sub-resolution arc cannot be traced as a blend: the whole direction change
    happens within roughly one control step, spiking the joint acceleration and forcing the
    Bounded planner to slow the entire motion down. A sharp corner is handled better there (its
    single large-deflection vertex is caught by the corner-speed cap). Pass one control step of
    tool travel (max_linear_speed * dt) to snap exactly the arcs the planner cannot resolve.
    """
    if radius <= 0.0 or len(vertices) < 3:
        return vertices

    out = [vertices[0]]
    for i in range(1, len(vertices) - 1):
        prev_v, corner, next_v = vertices[i - 1], vertices[i], vertices[i + 1]
        in_vec, out_vec = corner - prev_v, next_v - corner
        len_in, len_out = np.linalg.norm(in_vec), np.linalg.norm(out_vec)
        if len_in < 1e-9 or len_out < 1e-9:
            out.append(corner)
            continue
        d_in, d_out = in_vec / len_in, out_vec / len_out
        deflection = np.arccos(np.clip(d_in @ d_out, -1.0, 1.0))
        if deflection < 1e-6 or deflection > np.pi - 1e-6:
            out.append(corner)  # straight or reversal: nothing to round
            continue

        half = 0.5 * deflection
        # Tangent length for the requested radius, clamped to half of each leg.
        tangent = min(radius * np.tan(half), 0.5 * len_in, 0.5 * len_out)
        eff_radius = tangent / np.tan(half)
        # Snap sub-resolution arcs to a sharp corner (see the note in the docstring).
        if eff_radius * deflection < min_arc_length:
            out.append(corner)
            continue
        center = corner + (d_out - d_in) / np.linalg.norm(d_out - d_in) * (
            eff_radius / np.cos(half)
        )
        tangent_in = corner - tangent * d_in
        x = (tangent_in - center) / np.linalg.norm(tangent_in - center)
        y = d_in  # unit tangent at the arc start

        num_arc = max(1, int(np.ceil(np.degrees(deflection) / max_arc_step_deg)))
        for k in range(num_arc + 1):
            theta = deflection * k / num_arc
            out.append(center + eff_radius * (x * np.cos(theta) + y * np.sin(theta)))
    out.append(vertices[-1])
    return out


def make_lawnmower_path(
    scene: Scene,
    base_link: str,
    tip_frames: list[str],
    q_full: np.ndarray,
    path_size: float = 0.15,
    path_num_passes: int = 5,
    path_corner_radius: float = 0.0,
    path_corner_arc_step_deg: float = 1.0,
    path_corner_min_arc_length: float = 0.0,
) -> CartesianPath:
    """
    Builds a "lawnmower" (boustrophedon) Cartesian path with one waypoint list per end-effector
    in `tip_frames`. Each end-effector traces an identically shaped lawnmower that zigzags
    `path_num_passes` times across a square region starting at its own current pose and extending
    up and to the right, so multi-arm robots execute a coordinated sweep. The square lies in the
    base-frame y-z plane. Each pass sweeps across the square along the in-plane "u" (y) axis,
    alternating direction, and steps over along the "v" (z) axis between passes. Interior corners
    are rounded with circular arcs of `path_corner_radius` meters (0 leaves them sharp); arcs
    shorter than `path_corner_min_arc_length` meters are snapped back to sharp.
    """
    u_dir = np.array([0.0, 1.0, 0.0])
    v_dir = np.array([0.0, 0.0, 1.0])

    # Corner vertices (positions relative to a start pose), then round them in task space. The
    # square starts at the start pose and extends up and to the right (0 to path_size on both
    # in-plane axes); the same offsets are applied from each end-effector's start pose.
    vertices = []
    for i in range(path_num_passes):
        v = path_size * i / (path_num_passes - 1) if path_num_passes > 1 else 0.0
        # Alternate the sweep direction each pass to zigzag instead of retracing.
        u_values = (0.0, path_size) if i % 2 == 0 else (path_size, 0.0)
        for u in u_values:
            vertices.append(u * u_dir + v * v_dir)
    positions = round_corners(
        vertices,
        path_corner_radius,
        path_corner_arc_step_deg,
        path_corner_min_arc_length,
    )

    tforms = []
    for tip_frame in tip_frames:
        start = scene.forwardKinematics(q_full, tip_frame, base_link)
        waypoints = []
        for offset in positions:
            pose = start.copy()
            pose[:3, 3] += offset
            waypoints.append(pose)
        tforms.append(waypoints)

    base_frames = [base_link] * len(tip_frames)
    return CartesianPath(base_frames, tip_frames, tforms)


def main(
    model: str = "ur5",
    speed_mode: CartesianSpeedMode = CartesianSpeedMode.TimeOptimal,
    max_linear_speed: float = 0.1,
    max_angular_speed: float = 0.5,
    max_linear_acceleration: float = 0.5,
    max_angular_acceleration: float = 2.5,
    max_position_error: float = 0.01,
    max_orientation_error: float = 0.1,
    velocity_scale: float = 1.0,
    acceleration_scale: float = 1.0,
    dt: float = 0.01,
    path_size: float = 0.15,
    path_num_passes: int = 5,
    path_corner_radius: float = 0.0,
    path_corner_arc_step_deg: float = 1.0,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Plan a Cartesian path and visualize it.

    Parameters:
        model: The name of the model to use.
        speed_mode: Bounded for a bounded-acceleration Cartesian tool speed (ramps up/down
            within the commanded Cartesian acceleration maxima, capped at the commanded speeds,
            and slowed further to respect joint velocity/acceleration limits), or TimeOptimal for a
            time-optimal re-timing that respects joint velocity and acceleration limits.
        max_linear_speed: Maximum linear tool speed along the path (m/s). Bounded mode only.
        max_angular_speed: Maximum angular tool speed along the path (rad/s). Bounded mode only.
        max_linear_acceleration: Maximum linear tool acceleration along the path (m/s^2).
            Bounded mode only.
        max_angular_acceleration: Maximum angular tool acceleration along the path (rad/s^2).
            Bounded mode only.
        max_position_error: Maximum position deviation from the path (m).
        max_orientation_error: Maximum orientation deviation from the path (rad).
        velocity_scale: Scaling (0, 1] applied to joint velocity limits.
        acceleration_scale: Scaling (0, 1] applied to joint acceleration limits (TimeOptimal mode).
        dt: Output trajectory sample period (s).
        path_size: Side length of the square region the lawnmower covers (m).
        path_num_passes: Number of zigzag passes across the square.
        path_corner_radius: Task-space radius (m) used to round the lawnmower corners. Larger values
            round the corners more, letting the tool carry speed through them (0 = sharp corners,
            clamped per corner so adjacent arcs do not overlap).
        path_corner_arc_step_deg: Angular step (deg) used to discretize each rounded corner arc into
            chords. Coarse values (e.g. 15) facet the arc into a few straight segments whose kinks
            show up as a jagged velocity profile; use a small value (~1-2) so the arc is smooth and
            the tool carries speed cleanly through the corner. Ignored when path_corner_radius=0.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
    """
    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

    # Pre-process with xacro. This is not necessary for raw URDFs.
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()
    package_paths = [get_package_share_dir()]

    scene = Scene(
        "cartesian_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )

    # Place the robot at its home configuration, which serves as the IK seed.
    q_full = get_home_configuration(scene, model_data)
    scene.setJointPositions(q_full)

    base_link = model_data.base_link
    tip_frames = model_data.ee_names
    # Snap corner arcs that the planner cannot resolve (shorter than one control step of tool
    # travel) back to sharp corners: a sub-step arc spikes the joint acceleration and makes the
    # Bounded planner slow the whole motion down, whereas a sharp corner is handled cleanly.
    path = make_lawnmower_path(
        scene,
        base_link,
        tip_frames,
        q_full,
        path_size=path_size,
        path_num_passes=path_num_passes,
        path_corner_radius=path_corner_radius,
        path_corner_arc_step_deg=path_corner_arc_step_deg,
        path_corner_min_arc_length=max_linear_speed * dt,
    )

    options = CartesianPlannerOptions(
        group_name=model_data.default_joint_group,
        dt=dt,
        max_linear_speed=max_linear_speed,
        max_angular_speed=max_angular_speed,
        max_linear_acceleration=max_linear_acceleration,
        max_angular_acceleration=max_angular_acceleration,
        max_position_error=max_position_error,
        max_orientation_error=max_orientation_error,
        velocity_scale=velocity_scale,
        acceleration_scale=acceleration_scale,
        speed_mode=speed_mode,
    )
    planner = CartesianPathPlanner(scene, options)

    q_start = JointConfiguration()
    q_start.positions = q_full

    print(
        f"Planning a {path_num_passes}-pass lawnmower over a {path_size} m square "
        f"Cartesian path for {len(tip_frames)} end-effector(s) "
        f"({', '.join(tip_frames)}) in {speed_mode.name} mode..."
    )
    t0 = time.time()
    try:
        result = planner.plan(path, q_start)
    except RuntimeError as e:
        print(f"  Planning failed: {e}")
        sys.exit(1)
    elapsed = time.time() - t0

    traj = result
    peak_velocity_ratio, peak_acceleration_ratio = planner.compute_peak_limit_ratios(
        traj
    )
    print(f"  Planned in {elapsed * 1e3:.1f} ms")
    print(f"  Trajectory samples: {len(traj.times)}")
    print(f"  Trajectory duration: {traj.times[-1]:.3f} s")
    print(
        f"  Achieved Cartesian path length: "
        f"{planner.compute_achieved_path_length(traj, path):.4f} m"
    )
    print(f"  Peak velocity / limit:     {peak_velocity_ratio:.2f}")
    print(f"  Peak acceleration / limit: {peak_acceleration_ratio:.2f}")

    # Plot the planned joint trajectory over time.
    fig = plotJointTrajectory(
        traj,
        scene,
        group_name=model_data.default_joint_group,
        title="Cartesian Path Joint Trajectory",
        positions=True,
        velocities=True,
    )

    # Visualize: build a redundant Pinocchio model for rendering with mimic joints.
    model_pin = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )
    viz = ViserVisualizer(model_pin, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)
    viz.display(q_full)

    # Draw the reference path (commanded waypoints, green) vs. the actual traced
    # path (forward kinematics of the planned trajectory, red), once per end-effector.
    #
    # The CartesianPath waypoints are expressed in the base frame, so map them into
    # the world frame for visualization. The base frame is fixed relative to the
    # world, so a single forward-kinematics call suffices.
    world_T_base = scene.forwardKinematics(q_full, base_link)
    for i, tip_frame in enumerate(tip_frames):
        reference_positions = np.array(
            [(world_T_base @ waypoint)[:3, 3] for waypoint in path.tforms[i]]
        )
        visualizePositionTrace(
            viz,
            reference_positions,
            trace_name=f"/reference_path/{tip_frame}",
            waypoint_root=f"/reference_waypoints/{tip_frame}",
            trace_color=(40, 180, 40),
            waypoint_color=(40, 180, 40),
            line_width=4.0,
            waypoint_radius=0.0025,
        )
    visualizeJointTrajectory(
        viz,
        scene,
        traj,
        tip_frames,
        color=(220, 40, 40),
        name="/actual_path",
    )

    # Show the trajectory plot without blocking so the animation loop can run.
    plt.ion()
    plt.show(block=False)
    plt.pause(0.2)

    print("Playing back the trajectory. Press Ctrl+C to exit.")
    try:
        while True:
            for group_positions in traj.positions:
                t_start = time.perf_counter()
                q_play = scene.toFullJointPositions(
                    model_data.default_joint_group, group_positions
                )
                viz.display(q_play)
                fig.canvas.flush_events()
                time.sleep(max(0.0, dt - (time.perf_counter() - t_start)))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    tyro.cli(main)
