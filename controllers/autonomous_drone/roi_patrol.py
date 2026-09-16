"""Simple, replaceable waypoint planning for patrol inside a circular ROI."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol


Position = tuple[float, float, float]


class RoiPatrolPlan(Protocol):
    """Minimal interface consumed by the flight state machine."""

    def reset(
        self,
        center: tuple[float, float],
        radius_m: float,
        altitude_m: float,
        current_position: Position,
    ) -> Position: ...

    def next_waypoint(self) -> Position: ...

    def clear(self) -> None: ...


@dataclass(frozen=True, slots=True)
class RoiPatrolConfig:
    """Tuning for the deliberately simple V1 sweep pattern."""

    lane_count: int = 5
    boundary_margin_m: float = 2.0

    def __post_init__(self) -> None:
        if isinstance(self.lane_count, bool) or self.lane_count < 2:
            raise ValueError("lane_count must be at least 2")
        if (
            isinstance(self.boundary_margin_m, bool)
            or not math.isfinite(self.boundary_margin_m)
            or self.boundary_margin_m < 0.0
        ):
            raise ValueError("boundary_margin_m must be finite and non-negative")


class LawnMowerRoiPatrolPlanner:
    """Cycle forever over horizontal chords contained by a circular ROI.

    The generated polyline is fully contained by the ROI: its waypoints lie
    on an inset circle and every connecting segment lies inside that convex
    circle. Starting at the closest waypoint avoids an unnecessary cross-ROI
    leg when patrol begins.
    """

    def __init__(self, config: RoiPatrolConfig | None = None) -> None:
        self.config = config or RoiPatrolConfig()
        self._waypoints: tuple[Position, ...] = ()
        self._next_index = 0

    @property
    def waypoints(self) -> tuple[Position, ...]:
        return self._waypoints

    def reset(
        self,
        center: tuple[float, float],
        radius_m: float,
        altitude_m: float,
        current_position: Position,
    ) -> Position:
        if not math.isfinite(radius_m) or radius_m < 0.0:
            raise ValueError("radius_m must be finite and non-negative")

        usable_radius = max(0.0, radius_m - self.config.boundary_margin_m)
        if usable_radius == 0.0:
            self._waypoints = ((center[0], center[1], altitude_m),)
        else:
            # Avoid tangent-only lanes at +/- radius; they add no coverage and
            # produce duplicate endpoints.
            lane_step = 2.0 * usable_radius / (self.config.lane_count + 1)
            points: list[Position] = []
            for lane_index in range(self.config.lane_count):
                y_offset = -usable_radius + lane_step * (lane_index + 1)
                x_extent = math.sqrt(
                    max(0.0, usable_radius * usable_radius - y_offset * y_offset)
                )
                left = (center[0] - x_extent, center[1] + y_offset, altitude_m)
                right = (center[0] + x_extent, center[1] + y_offset, altitude_m)
                points.extend((left, right) if lane_index % 2 == 0 else (right, left))
            self._waypoints = tuple(points)

        closest_index = min(
            range(len(self._waypoints)),
            key=lambda index: _horizontal_distance(
                current_position, self._waypoints[index]
            ),
        )
        first = self._waypoints[closest_index]
        self._next_index = (closest_index + 1) % len(self._waypoints)
        return first

    def next_waypoint(self) -> Position:
        if not self._waypoints:
            raise RuntimeError("ROI patrol planner must be reset before use")
        waypoint = self._waypoints[self._next_index]
        self._next_index = (self._next_index + 1) % len(self._waypoints)
        return waypoint

    def clear(self) -> None:
        self._waypoints = ()
        self._next_index = 0


def _horizontal_distance(first: Position, second: Position) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])
