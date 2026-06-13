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


def round_corners(vertices, radius, max_arc_step_deg=15.0):
    """
    Rounds the interior corners of a polyline (list of 3D points) with circular arcs of the
    given radius (meters), returning a denser list of points that traces straight legs joined
    by tangent arcs. This is the task-space "blend": each corner is replaced by an arc of the
    requested radius, tangent to both adjacent legs. The tangent length is clamped to half of
    each adjacent segment so neighbouring arcs never overlap (the effective radius shrinks at
    corners whose legs are too short). A radius <= 0 leaves the corners sharp.
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
    scene, base_link, tip_frame, q_full, side=0.15, num_passes=5, corner_radius=0.0
):
    """
    Builds a lawnmower (boustrophedon) Cartesian path that zigzags `num_passes` times
    across a square region, starting from the current tool pose. The square lies in the
    plane spanned by the base-frame (1, 1, 0)/sqrt(2) and z directions. Each pass sweeps
    across the square along the in-plane "u" axis, alternating direction, and steps over
    along the "v" axis between passes. Interior corners are rounded with circular arcs of
    `corner_radius` meters (0 leaves them sharp). Returns a CartesianPath with one tip frame.
    """
    start = scene.forwardKinematics(q_full, tip_frame, base_link)
    # In-plane orthonormal axes: u sweeps across each pass, v steps over between passes.
    u_dir = np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0)
    v_dir = np.array([0.0, 0.0, 1.0])

    # Corner vertices (positions relative to the start pose), then round them in task space.
    vertices = []
    for i in range(num_passes):
        v = side * i / (num_passes - 1) if num_passes > 1 else 0.0
        # Alternate the sweep direction each pass to zigzag instead of retracing.
        u_values = (0.0, side) if i % 2 == 0 else (side, 0.0)
        for u in u_values:
            vertices.append(u * u_dir + v * v_dir)
    positions = round_corners(vertices, corner_radius)

    waypoints = []
    for offset in positions:
        pose = start.copy()
        pose[:3, 3] += offset
        waypoints.append(pose)
    return CartesianPath([base_link], [tip_frame], [waypoints])


def main(
    model: str = "ur5",
    speed_mode: str = "toppra",
    linear_speed: float = 0.1,
    angular_speed: float = 0.5,
    max_position_error: float = 0.01,
    max_orientation_error: float = 0.1,
    velocity_scale: float = 1.0,
    acceleration_scale: float = 1.0,
    dt: float = 0.01,
    side: float = 0.15,
    num_passes: int = 5,
    corner_radius: float = 0.0,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Plan a Cartesian path with the Oink-based planner and (optionally) play it back.

    Parameters:
        model: The name of the model to use.
        speed_mode: "constant" for a constant Cartesian tool speed (velocity-level, does
            not bound acceleration), or "toppra" for a time-optimal re-timing that
            respects joint velocity and acceleration limits (tool speed varies).
        linear_speed: Commanded linear tool speed along the path (m/s). Constant mode only.
        angular_speed: Commanded angular tool speed along the path (rad/s). Constant mode only.
        max_position_error: Maximum position deviation from the path (m).
        max_orientation_error: Maximum orientation deviation from the path (rad).
        velocity_scale: Scaling (0, 1] applied to joint velocity limits.
        acceleration_scale: Scaling (0, 1] applied to joint acceleration limits (toppra mode).
        dt: Output trajectory sample period (s).
        side: Side length of the square region the lawnmower covers (m).
        num_passes: Number of zigzag passes across the square.
        corner_radius: Task-space radius (m) used to round the lawnmower corners. Larger values
            round the corners more, letting the tool carry speed through them (0 = sharp corners,
            clamped per corner so adjacent arcs do not overlap).
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
    tip_frame = model_data.ee_names[0]
    path = make_lawnmower_path(
        scene,
        base_link,
        tip_frame,
        q_full,
        side=side,
        num_passes=num_passes,
        corner_radius=corner_radius,
    )

    speed_mode_map = {
        "constant": CartesianSpeedMode.ConstantCartesianSpeed,
        "toppra": CartesianSpeedMode.TimeOptimalToppra,
    }
    if speed_mode not in speed_mode_map:
        print(f"Invalid speed_mode '{speed_mode}'. Choose from {list(speed_mode_map)}.")
        sys.exit(1)

    options = CartesianPlannerOptions(
        group_name=model_data.default_joint_group,
        dt=dt,
        linear_speed=linear_speed,
        angular_speed=angular_speed,
        max_position_error=max_position_error,
        max_orientation_error=max_orientation_error,
        velocity_scale=velocity_scale,
        acceleration_scale=acceleration_scale,
        speed_mode=speed_mode_map[speed_mode],
    )
    planner = CartesianPathPlanner(scene, options)

    q_start = JointConfiguration()
    q_start.positions = q_full

    print(
        f"Planning a {num_passes}-pass lawnmower over a {side} m square "
        f"Cartesian path ({speed_mode} mode)..."
    )
    t0 = time.time()
    try:
        result = planner.plan(path, q_start)
    except RuntimeError as e:
        print(f"  Planning failed: {e}")
        sys.exit(1)
    elapsed = time.time() - t0

    traj = result.trajectory
    print(f"  Planned in {elapsed * 1e3:.1f} ms")
    print(f"  Trajectory samples: {len(traj.times)}")
    print(f"  Trajectory duration: {traj.times[-1]:.3f} s")
    print(f"  Achieved Cartesian path length: {result.achieved_path_length:.4f} m")
    print(f"  Feedrate efficiency: {result.feedrate_efficiency * 100:.1f}%")
    print(f"  Peak velocity / limit:     {result.peak_velocity_ratio:.2f}")
    print(f"  Peak acceleration / limit: {result.peak_acceleration_ratio:.2f}")
    # Constant mode is a velocity-level trace and does not bound joint acceleration, so it
    # can exceed the acceleration limits; flag that and point at the toppra mode.
    if speed_mode == "constant" and result.peak_acceleration_ratio > 1.25:
        print(
            "  NOTE: this velocity-level trajectory exceeds the joint acceleration limits. "
            "Use --speed-mode toppra for an acceleration-limited, time-optimal re-timing."
        )

    # Plot the planned joint trajectory over time.
    fig = plotJointTrajectory(traj, scene, plot_title="Cartesian Path Joint Trajectory")

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
    # path (forward kinematics of the planned trajectory, red).
    #
    # The CartesianPath waypoints are expressed in the base frame, so map them into
    # the world frame for visualization. The base frame is fixed relative to the
    # world, so a single forward-kinematics call suffices.
    world_T_base = scene.forwardKinematics(q_full, base_link)
    reference_positions = np.array(
        [(world_T_base @ waypoint)[:3, 3] for waypoint in path.tforms[0]]
    )
    visualizePositionTrace(
        viz,
        reference_positions,
        trace_name="/reference_path",
        waypoint_root="/reference_waypoints",
        trace_color=(40, 180, 40),
        waypoint_color=(40, 180, 40),
        line_width=4.0,
        waypoint_radius=0.0025,
    )
    visualizeJointTrajectory(
        viz,
        scene,
        traj,
        [tip_frame],
        color=(220, 40, 40),
        name="/actual_path",
    )

    # Show the trajectory plot without blocking so the animation loop can run.
    plt.ion()
    plt.show(block=False)
    plt.pause(0.2)

    print("Reference path: green. Actual traced path: red.")
    print("Playing back the trajectory. Press Ctrl+C to exit.")
    try:
        while True:
            for group_positions in traj.positions:
                q_play = scene.toFullJointPositions(
                    model_data.default_joint_group, group_positions
                )
                viz.display(q_play)
                time.sleep(dt)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    tyro.cli(main)
