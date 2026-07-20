"""Strongly typed identifiers and identifier factories."""

from __future__ import annotations

import re
import uuid
from typing import NewType

MessageId = NewType("MessageId", str)
TraceId = NewType("TraceId", str)
ModuleId = NewType("ModuleId", str)
CapabilityId = NewType("CapabilityId", str)

_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def _generate_unique_id() -> str:
    return str(uuid.uuid4())


def _validate_name(name: str, identifier_name: str) -> str:
    if not _IDENTIFIER_PATTERN.fullmatch(name):
        raise ValueError(f"Invalid {identifier_name}: {name!r}")
    return name


def create_message_id() -> MessageId:
    return MessageId(_generate_unique_id())


def create_trace_id() -> TraceId:
    return TraceId(_generate_unique_id())


def create_module_id(name: str) -> ModuleId:
    return ModuleId(_validate_name(name, "ModuleId"))


def create_capability_id(name: str) -> CapabilityId:
    return CapabilityId(_validate_name(name, "CapabilityId"))
