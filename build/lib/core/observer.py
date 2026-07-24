"""Passive observer contract for CognitiveBus events."""

from __future__ import annotations

from typing import Protocol

from core.types.delivery import DeliveryReport
from core.types.message import CognitiveMessage
from core.types.payload import BasePayload


class BusObserver(Protocol):
    """Asynchronous observer isolated from the dispatch critical path."""

    async def on_message_received(self, message: CognitiveMessage[BasePayload]) -> None:
        """Observe a message after envelope and payload validation."""

    async def on_dispatch_complete(
        self,
        message: CognitiveMessage[BasePayload],
        report: DeliveryReport,
        latency_ns: int,
    ) -> None:
        """Observe the outcome of a completed message dispatch."""
