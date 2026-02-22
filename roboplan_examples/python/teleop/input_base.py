from __future__ import annotations

from abc import ABC, abstractmethod

from .types import TwistCommand


class TeleopInput(ABC):
    """Abstract input adapter for teleop commands."""

    @abstractmethod
    def read(self) -> TwistCommand:
        """Return latest command without blocking."""

    @abstractmethod
    def is_active(self) -> bool:
        """True when input adapter is alive and usable."""

    @abstractmethod
    def close(self) -> None:
        """Release any resources and stop background polling."""
