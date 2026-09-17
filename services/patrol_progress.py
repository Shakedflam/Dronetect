"""Resumable execution state for an interrupted normal-patrol coverage path."""

from __future__ import annotations

from dataclasses import dataclass
import math
from collections.abc import Sequence


Position = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class PausedPatrolCoverage:
    """The exact unfinished portion of a normal-patrol coverage session."""

    region_id: str
    interruption_position: Position
    active_waypoint: Position
    remaining_waypoints: tuple[Position, ...]

    def resumed_waypoints(
        self,
        return_route: Sequence[Position],
    ) -> tuple[Position, ...]:
        """Route back through the interruption point, then finish coverage."""
        ordered = [
            *return_route,
            self.interruption_position,
            self.active_waypoint,
            *self.remaining_waypoints,
        ]
        deduplicated: list[Position] = []
        for waypoint in ordered:
            if not deduplicated or _distance(deduplicated[-1], waypoint) > 1e-7:
                deduplicated.append(waypoint)
        return tuple(deduplicated)


def _distance(first: Position, second: Position) -> float:
    return math.dist(first, second)
