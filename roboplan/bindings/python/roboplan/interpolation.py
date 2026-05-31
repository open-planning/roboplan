import math

import numpy as np
import pinocchio as pin

from roboplan.core import CartesianPath, CartesianTrajectory, JointTrajectory, Scene


def computeStepsPerSegment(segment_time: float, control_dt: float) -> int:
    """Compute the number of interpolation intervals in one segment.

    Args:
        segment_time: Duration of one waypoint-to-waypoint segment, in seconds.
        control_dt: Desired interpolation sample period, in seconds.

    Returns:
        Number of interpolation intervals for the segment.
    """
    if segment_time <= 0.0:
        raise ValueError("segment_time must be positive.")
    if control_dt <= 0.0:
        raise ValueError("control_dt must be positive.")

    return max(1, int(math.ceil(segment_time / control_dt)))


def interpolateConfigurationWaypoints(
    scene: Scene,
    waypoints: list[np.ndarray],
    segment_time: float,
    control_dt: float,
) -> list[np.ndarray]:
    """Interpolate configuration waypoints using Scene.interpolate().

    Args:
        scene: RoboPlan scene used to interpolate between configurations.
        waypoints: Sparse configuration waypoints.
        segment_time: Duration of each waypoint-to-waypoint segment, in seconds.
        control_dt: Desired interpolation sample period, in seconds.

    Returns:
        Dense configuration waypoints sampled approximately every control_dt seconds.
    """
    if len(waypoints) < 2:
        return [w.copy() for w in waypoints]

    steps_per_segment = computeStepsPerSegment(segment_time, control_dt)
    dense_waypoints = []

    for idx in range(len(waypoints) - 1):
        start = waypoints[idx]
        end = waypoints[idx + 1]

        for step in range(steps_per_segment + 1):
            if idx > 0 and step == 0:
                continue

            alpha = step / steps_per_segment
            dense_waypoints.append(scene.interpolate(start, end, alpha))

    return dense_waypoints


def interpolateJointTrajectory(
    scene: Scene,
    trajectory: JointTrajectory,
    control_dt: float,
) -> list[np.ndarray]:
    """Interpolate a JointTrajectory using its waypoint times.

    Args:
        scene: RoboPlan scene used to interpolate between configurations.
        trajectory: Sparse joint trajectory with positions and waypoint times.
        control_dt: Desired interpolation sample period, in seconds.

    Returns:
        Dense configuration waypoints sampled according to the trajectory times.
    """
    if control_dt <= 0.0:
        raise ValueError("control_dt must be positive.")

    if len(trajectory.positions) != len(trajectory.times):
        raise ValueError(
            "JointTrajectory positions and times must have the same length."
        )

    if len(trajectory.positions) < 2:
        return [np.asarray(position).copy() for position in trajectory.positions]

    dense_waypoints = []

    for idx in range(len(trajectory.positions) - 1):
        # JointTrajectory.positions comes from nanobind/Eigen vectors, so convert
        # each position to a NumPy array before using Scene.interpolate().
        start = np.asarray(trajectory.positions[idx])
        end = np.asarray(trajectory.positions[idx + 1])
        segment_time = trajectory.times[idx + 1] - trajectory.times[idx]

        if segment_time <= 0.0:
            raise ValueError("JointTrajectory times must be strictly increasing.")

        steps_per_segment = computeStepsPerSegment(segment_time, control_dt)

        for step in range(steps_per_segment + 1):
            if idx > 0 and step == 0:
                continue

            alpha = step / steps_per_segment
            dense_waypoints.append(scene.interpolate(start, end, alpha))

    return dense_waypoints


def interpolateCartesianTrajectory(
    trajectory: CartesianTrajectory,
    control_dt: float,
) -> CartesianTrajectory:
    """Interpolate a CartesianTrajectory using its waypoint times.

    Args:
        trajectory: Sparse Cartesian trajectory with transforms and waypoint times.
        control_dt: Desired interpolation sample period, in seconds.

    Returns:
        Dense Cartesian trajectory sampled according to the sparse waypoint times.
    """
    if control_dt <= 0.0:
        raise ValueError("control_dt must be positive.")

    if len(trajectory.base_frames) != len(trajectory.tip_frames):
        raise ValueError("base_frames and tip_frames must have the same length.")

    if len(trajectory.tforms) != len(trajectory.tip_frames):
        raise ValueError("tforms must have one transform sequence per tip frame.")

    for frame_tforms in trajectory.tforms:
        if len(frame_tforms) != len(trajectory.times):
            raise ValueError(
                "Each transform sequence must have the same length as times."
            )

    if len(trajectory.times) < 2:
        return CartesianTrajectory(
            base_frames=trajectory.base_frames,
            tip_frames=trajectory.tip_frames,
            times=trajectory.times,
            tforms=[
                [np.asarray(tform).copy() for tform in frame_tforms]
                for frame_tforms in trajectory.tforms
            ],
        )

    # Pre-convert all transforms to SE3 once, avoiding repeated conversions in the loop.
    se3_tforms_by_frame = [
        [pin.SE3(np.asarray(tform)) for tform in frame_tforms]
        for frame_tforms in trajectory.tforms
    ]

    dense_times = []
    dense_tforms_by_frame = [[] for _ in trajectory.tip_frames]

    for idx in range(len(trajectory.times) - 1):
        segment_time = trajectory.times[idx + 1] - trajectory.times[idx]
        if segment_time <= 0.0:
            raise ValueError("CartesianTrajectory times must be strictly increasing.")

        steps_per_segment = computeStepsPerSegment(segment_time, control_dt)

        for step in range(steps_per_segment + 1):
            if idx > 0 and step == 0:
                continue

            alpha = step / steps_per_segment
            dense_times.append(trajectory.times[idx] + alpha * segment_time)

            for frame_idx, frame_se3s in enumerate(se3_tforms_by_frame):
                dense_tforms_by_frame[frame_idx].append(
                    pin.SE3.Interpolate(
                        frame_se3s[idx],
                        frame_se3s[idx + 1],
                        alpha,
                    ).homogeneous
                )

    return CartesianTrajectory(
        base_frames=trajectory.base_frames,
        tip_frames=trajectory.tip_frames,
        times=dense_times,
        tforms=dense_tforms_by_frame,
    )


def interpolateCartesianPath(
    path: CartesianPath,
    waypoint_times: list[float],
    control_dt: float,
) -> CartesianTrajectory:
    """Interpolate a CartesianPath using waypoint times.

    Prefer interpolateCartesianTrajectory() when the input already includes times.

    Args:
        path: Cartesian path containing one transform sequence per end-effector frame.
        waypoint_times: Times corresponding to the path waypoints.
        control_dt: Desired dense interpolation sample period, in seconds.

    Returns:
        Dense Cartesian trajectory sampled according to the waypoint times.
    """
    sparse_trajectory = CartesianTrajectory(
        base_frames=path.base_frames,
        tip_frames=path.tip_frames,
        times=waypoint_times,
        tforms=path.tforms,
    )

    return interpolateCartesianTrajectory(sparse_trajectory, control_dt)
