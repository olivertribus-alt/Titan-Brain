"""Message priority definitions."""

from __future__ import annotations

from enum import IntEnum


class Priority(IntEnum):
    """Priority levels used by the CognitiveBus dispatcher."""

    BACKGROUND = 0
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4
    EMERGENCY = 5

    @property
    def is_realtime(self) -> bool:
        """Return whether the priority requires real-time handling."""
        return self >= Priority.CRITICAL

    @property
    def is_background(self) -> bool:
        """Return whether the priority is suitable for background work."""
        return self is Priority.BACKGROUND

    @property
    def requires_immediate_dispatch(self) -> bool:
        """Return whether the priority bypasses normal queueing."""
        return self >= Priority.HIGH
