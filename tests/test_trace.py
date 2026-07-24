"""Tests for TraceRegistry and TraceRecord behavior."""

from __future__ import annotations

import asyncio
from typing import Literal

import pytest
from pydantic import ConfigDict

from core.trace import TraceRegistry, TraceRecord
from core.types.identifiers import (
    create_message_id,
    create_module_id,
    create_trace_id,
)
from core.types.delivery import DeliveryReport
from core.types.message import Broadcast, CognitiveMessage, IncomingMessageEnvelope
from core.types.payload import BasePayload
from core.types.priority import Priority
from core.types.telemetry import CognitiveTelemetry, RuntimeTelemetry, Telemetry


class DummyPayload(BasePayload):
    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)
    schema_version: Literal[1] = 1
    payload_type: Literal["dummy"] = "dummy"


def test_trace_registry_invalid_max_records() -> None:
    with pytest.raises(ValueError, match="max_records must be greater than or equal to one"):
        TraceRegistry(max_records=0)


def test_trace_registry_records_dispatch() -> None:
    registry = TraceRegistry(max_records=2)

    msg_id = create_message_id()
    trace_id = create_trace_id()
    message = CognitiveMessage(
        message_id=msg_id,
        trace_id=trace_id,
        source=create_module_id("Source"),
        destination=Broadcast(),
        timestamp_ns=1,
        priority=Priority.NORMAL,
        telemetry=Telemetry(
            module_id=create_module_id("Source"),
            timestamp_ns=1,
            runtime=RuntimeTelemetry(
                latency_ms=1.0, execution_time_ms=1.0, queue_depth=0,
                cpu_usage_percent=1.0, memory_usage_mb=1.0
            ),
            cognitive=CognitiveTelemetry(
                confidence=0.5, entropy=0.1, prediction_error=0.1,
                novelty=0.1, uncertainty=0.1
            ),
        ),
        payload=DummyPayload(),
    )

    report = DeliveryReport(message_id=msg_id, delivered_to=(create_module_id("Receiver"),), failures=())

    asyncio.run(registry.on_message_received(message))
    asyncio.run(registry.on_dispatch_complete(message, report, latency_ns=500))

    record = registry.get(str(msg_id))
    assert record is not None
    assert record.message_id == str(msg_id)
    assert record.trace_id == str(trace_id)
    assert record.payload_type == "dummy"
    assert record.targets == ("Receiver",)
    assert record.latency_ns == 500
    assert record.failures == ()

    recent_records = registry.recent()
    assert len(recent_records) == 1
    assert recent_records[0].message_id == str(msg_id)


def test_trace_registry_fifo_eviction() -> None:
    registry = TraceRegistry(max_records=1)

    def _make_msg_and_report() -> tuple[CognitiveMessage[BasePayload], DeliveryReport]:
        mid = create_message_id()
        msg = CognitiveMessage(
            message_id=mid,
            trace_id=create_trace_id(),
            source=create_module_id("Source"),
            destination=Broadcast(),
            timestamp_ns=1,
            priority=Priority.NORMAL,
            telemetry=Telemetry(
                module_id=create_module_id("Source"),
                timestamp_ns=1,
                runtime=RuntimeTelemetry(
                    latency_ms=1.0, execution_time_ms=1.0, queue_depth=0,
                    cpu_usage_percent=1.0, memory_usage_mb=1.0
                ),
                cognitive=CognitiveTelemetry(
                    confidence=0.5, entropy=0.1, prediction_error=0.1,
                    novelty=0.1, uncertainty=0.1
                ),
            ),
            payload=DummyPayload(),
        )
        rep = DeliveryReport(message_id=mid, delivered_to=(), failures=())
        return msg, rep

    msg1, rep1 = _make_msg_and_report()
    msg2, rep2 = _make_msg_and_report()

    asyncio.run(registry.on_dispatch_complete(msg1, rep1, 100))
    assert registry.get(str(msg1.message_id)) is not None

    asyncio.run(registry.on_dispatch_complete(msg2, rep2, 200))
    assert registry.get(str(msg1.message_id)) is None
    assert registry.get(str(msg2.message_id)) is not None
