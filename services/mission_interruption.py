"""Generic mission execution state preserved across target pursuit."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any


Position = tuple[float, float, float]


class InterruptedMissionType(StrEnum):
    PATROL = "PATROL"
    ROI = "ROI"
    FLIGHT_STATE = "FLIGHT_STATE"


@dataclass(frozen=True, slots=True)
class MissionInterruptionContext:
    """Mission-neutral resume envelope with an opaque progress token."""

    source_type: InterruptedMissionType
    source_state: str
    interruption_position: Position
    active_id: str | None
    active_waypoint: Position | None
    remaining_path: tuple[Position, ...]
    progress: Any = None

