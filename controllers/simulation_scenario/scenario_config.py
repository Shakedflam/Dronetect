"""Scenario configuration for Webots background actors."""

from __future__ import annotations

from dataclasses import dataclass
import json
from json import JSONDecodeError
from pathlib import Path
from typing import Any


DEFAULT_TRAFFIC_DRONES = 3
DEFAULT_BIRDS = 3
MAX_TRAFFIC_DRONES = 5
MAX_BIRDS = 3


@dataclass(frozen=True, slots=True)
class ScenarioConfig:
    traffic_drone_count: int = DEFAULT_TRAFFIC_DRONES
    bird_count: int = DEFAULT_BIRDS


def load_scenario_config(path: Path) -> ScenarioConfig:
    if not path.exists():
        return ScenarioConfig()

    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except JSONDecodeError:
        return ScenarioConfig()
    if not isinstance(raw, dict):
        return ScenarioConfig()

    return ScenarioConfig(
        traffic_drone_count=_read_count(
            raw,
            "traffic_drones",
            DEFAULT_TRAFFIC_DRONES,
            MAX_TRAFFIC_DRONES,
        ),
        bird_count=_read_count(raw, "birds", DEFAULT_BIRDS, MAX_BIRDS),
    )


def _read_count(raw: dict[str, Any], section: str, default: int, maximum: int) -> int:
    section_value = raw.get(section)
    if not isinstance(section_value, dict):
        return default

    count = section_value.get("count", default)
    if isinstance(count, bool):
        return default
    try:
        return min(max(0, int(count)), maximum)
    except (TypeError, ValueError):
        return default
