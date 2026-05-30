from typing import Annotated

import numpy
from numpy.typing import NDArray


class SE3LowPassFilter:
    """First-order low-pass filter for SE3 poses."""

    def __init__(self, tau: float = 0.1) -> None: ...

    def reset(self, pose: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]) -> None:
        """Resets the filter state to a specific pose."""

    def update(self, target_pose: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], dt: float) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
        """Updates the filtered state toward a target pose."""

    def tau(self) -> float:
        """Returns the filter time constant in seconds."""

    def setTau(self, tau: float) -> None:
        """Sets the filter time constant."""

    def isInitialized(self) -> bool:
        """Checks whether the filter has an active filtered state."""
