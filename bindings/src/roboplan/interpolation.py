import math

import numpy as np
import pinocchio as pin

from roboplan.core import JointTrajectory, Scene


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
        return waypoints.copy()

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


def interpolateSE3Waypoints(
    transforms: list[np.ndarray],
    waypoint_times: list[float],
    control_dt: float,
) -> list[np.ndarray]:
    """Interpolate SE(3) waypoints using Pinocchio SE(3) interpolation.

    Args:
        transforms: Sparse SE(3) waypoints as 4x4 homogeneous matrices.
        waypoint_times: End time of each waypoint, in seconds.
        control_dt: Desired interpolation sample period, in seconds.

    Returns:
        Dense SE(3) waypoints as 4x4 homogeneous matrices sampled approximately
        every control_dt seconds.
    """
    if len(transforms) < 2:
        return transforms.copy()

    transforms_se3 = [pin.SE3(tform) for tform in transforms]
    dense_transforms = []

    for idx in range(len(transforms) - 1):
        start = transforms_se3[idx]
        end = transforms_se3[idx + 1]
        segment_time = waypoint_times[idx + 1] - waypoint_times[idx]
        steps_per_segment = computeStepsPerSegment(segment_time, control_dt)
        for step in range(steps_per_segment + 1):
            # Ensure waypoints are not duplicated.
            if idx > 0 and step == 0:
                continue

            alpha = step / steps_per_segment
            dense_transforms.append(pin.SE3.Interpolate(start, end, alpha).homogeneous)

    return dense_transforms
