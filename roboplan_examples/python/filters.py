import numpy as np
import pinocchio as pin


class SE3LowPassFilter:
    """First-order filter for SE3 poses to prevent sudden jumps in reference commands.

    Smoothly interpolates between the current filtered pose and the target pose
    using exponential filtering for position and SLERP for orientation.
    """

    def __init__(self, tau: float = 0.1):
        """Initialize the filter.

        Args:
            tau: Time constant in seconds. Larger values = slower, smoother tracking.
                 tau=0.1 means ~63% of the step is taken per 0.1 second.
        """
        self.tau = tau
        self.filtered_position: np.ndarray | None = None
        self.filtered_quaternion: pin.Quaternion | None = None

    def reset(self, pose: np.ndarray) -> None:
        """Reset the filter state to a specific pose.

        Args:
            pose: 4x4 homogeneous transformation matrix.
        """
        self.filtered_position = pose[:3, 3].copy()
        self.filtered_quaternion = pin.Quaternion(pose[:3, :3])

    def update(self, target_pose: np.ndarray, dt: float) -> np.ndarray:
        """Filter the target pose and return the smoothed result.

        Args:
            target_pose: 4x4 homogeneous transformation matrix (target).
            dt: Time step in seconds.

        Returns:
            4x4 homogeneous transformation matrix (filtered).
        """
        target_position = target_pose[:3, 3]
        target_quaternion = pin.Quaternion(target_pose[:3, :3])

        # Initialize on first call
        if self.filtered_position is None:
            self.filtered_position = target_position.copy()
            self.filtered_quaternion = target_quaternion
            return target_pose.copy()

        # Exponential filter coefficient: alpha = 1 - exp(-dt/tau)
        # alpha -> 0 as tau -> inf (slower), alpha -> 1 as tau -> 0 (faster)
        alpha = 1.0 - np.exp(-dt / self.tau)

        # Filter position (linear interpolation)
        self.filtered_position = self.filtered_position + alpha * (
            target_position - self.filtered_position
        )

        # Filter orientation (SLERP)
        self.filtered_quaternion = self.filtered_quaternion.slerp(
            alpha, target_quaternion
        )

        # Construct filtered pose
        filtered_pose = np.eye(4)
        filtered_pose[:3, :3] = self.filtered_quaternion.toRotationMatrix()
        filtered_pose[:3, 3] = self.filtered_position

        return filtered_pose
