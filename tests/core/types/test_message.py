"""Tests for two-phase message validation."""

from __future__ import annotations

import json

from core.types.message import Broadcast, IncomingMessageEnvelope


def test_incoming_envelope_validates_json_payload() -> None:
    raw_message = {
        "message_id": "message-1",
        "trace_id": "trace-1",
        "source": "Analyzer",
        "destination": {"type": "BROADCAST"},
        "timestamp_ns": 1,
        "priority": 2,
        "telemetry": {
            "schema_version": 1,
            "module_id": "Analyzer",
            "timestamp_ns": 1,
            "runtime": {
                "latency_ms": 1.0,
                "execution_time_ms": 1.0,
                "queue_depth": 0,
                "cpu_usage_percent": 1.0,
                "memory_usage_mb": 1.0,
            },
            "cognitive": {
                "confidence": 0.5,
                "entropy": 0.1,
                "prediction_error": 0.1,
                "novelty": 0.1,
                "uncertainty": 0.1,
            },
        },
        "payload": {"schema_version": 1, "payload_type": "analysis"},
    }

    message = IncomingMessageEnvelope.model_validate_json(json.dumps(raw_message))

    assert isinstance(message.destination, Broadcast)
    assert message.payload["payload_type"] == "analysis"
