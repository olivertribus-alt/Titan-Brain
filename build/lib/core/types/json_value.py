"""Strict type aliases for JSON-serializable structures."""

from __future__ import annotations

from typing_extensions import TypeAliasType

JsonValue = TypeAliasType(
    "JsonValue",
    "str | int | float | bool | None | list[JsonValue] | dict[str, JsonValue]",
)
