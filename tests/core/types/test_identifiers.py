"""Tests for identifier factories."""

from __future__ import annotations

import uuid

import pytest

from core.types.identifiers import (
    create_capability_id,
    create_message_id,
    create_module_id,
    create_trace_id,
)


def test_unique_identifier_factories_create_uuid4_values() -> None:
    assert uuid.UUID(create_message_id()).version == 4
    assert uuid.UUID(create_trace_id()).version == 4


@pytest.mark.parametrize("name", ["Module_1", "A", "analysisModule"])
def test_module_identifier_accepts_valid_names(name: str) -> None:
    assert create_module_id(name) == name


@pytest.mark.parametrize("name", ["", "1module", "bad-name", "with space"])
def test_named_identifiers_reject_invalid_names(name: str) -> None:
    with pytest.raises(ValueError):
        create_module_id(name)
    with pytest.raises(ValueError):
        create_capability_id(name)
