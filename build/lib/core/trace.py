"""Bounded in-memory tracing observer for CognitiveBus dispatches."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass

from core.observer import BusObserver
from core.types.delivery import DeliveryFailure, DeliveryReport
from core.types.message import CognitiveMessage
from core.types.payload import BasePayload


@dataclass(frozen=True)
class TraceRecord:
    """Immutable record of a completed message dispatch."""

    message_id: str
    trace_id: str
    payload_type: str
    targets: tuple[str, ...]
    latency_ns: int
    failures: tuple[DeliveryFailure, ...]


class TraceRegistry(BusObserver):
    """Store a bounded FIFO history of completed message dispatches."""

    def __init__(self, max_records: int = 1000) -> None:
        if max_records < 1:
            raise ValueError("max_records must be greater than or equal to one")
        self._max_records = max_records
        self._records: OrderedDict[str, TraceRecord] = OrderedDict()

    async def on_message_received(self, message: CognitiveMessage[BasePayload]) -> None:
        """Accept the validated-message event without retaining transient state."""
        return None

    async def on_dispatch_complete(
        self,
        message: CognitiveMessage[BasePayload],
        report: DeliveryReport,
        latency_ns: int,
    ) -> None:
        """Record one completed dispatch and evict excess historical entries."""
        record = TraceRecord(
            message_id=str(message.message_id),
            trace_id=str(message.trace_id),
            payload_type=message.payload.payload_type,
            targets=tuple(str(module_id) for module_id in report.delivered_to),
            latency_ns=latency_ns,
            failures=report.failures,
        )
        self._records[record.message_id] = record
        self._records.move_to_end(record.message_id)
        while len(self._records) > self._max_records:
            self._records.popitem(last=False)

    def get(self, message_id: str) -> TraceRecord | None:
        """Return one trace record by message identifier, if retained."""
        return self._records.get(message_id)

    def recent(self) -> tuple[TraceRecord, ...]:
        """Return retained trace records from oldest to newest."""
        return tuple(self._records.values())
