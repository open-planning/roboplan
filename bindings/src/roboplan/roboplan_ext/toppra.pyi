import roboplan_ext.core


class PathParameterizerTOPPRA:
    """Trajectory time parameterizer using the TOPP-RA algorithm."""

    def __init__(self, scene: roboplan_ext.core.Scene, group_name: str = '') -> None: ...

    def generate(self, path: roboplan_ext.core.JointPath, dt: float, velocity_scale: float = 1.0, acceleration_scale: float = 1.0) -> roboplan_ext.core.JointTrajectory:
        """Time-parameterizes a joint-space path using TOPP-RA."""
