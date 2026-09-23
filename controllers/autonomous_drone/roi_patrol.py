"""Simple, replaceable waypoint planning for patrol inside a circular ROI."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from shapely import wkt
from shapely.geometry import LineString, Point, Polygon

from config.shared_config import default_operating_config


Position = tuple[float, float, float]


class RoiPatrolPlan(Protocol):
    """Minimal interface consumed by the flight state machine."""

    # The operating config swap replaces this wholesale when the flight radius
    # changes, so it is part of the interface rather than an implementation
    # detail of the lawn-mower planner.
    config: "RoiPatrolConfig"

    def reset(
        self,
        center: tuple[float, float],
        radius_m: float,
        altitude_m: float,
        current_position: Position,
    ) -> Position: ...

    def next_waypoint(self) -> Position: ...

    # True once every waypoint of the current route has been handed out once.
    # Only consulted when FlightConfig.roi_single_cycle is set, so continuous
    # planners stay unaffected.
    @property
    def cycle_complete(self) -> bool: ...

    def entry_target(self, center: tuple[float, float], altitude_m: float) -> Position: ...

    def contains(self, position: Position) -> bool: ...

    def closest_waypoint(self, position: Position) -> Position: ...

    def retarget_altitude(self, new_altitude_m: float) -> None: ...

    def clear(self) -> None: ...

    def snapshot(self) -> "RoiPatrolProgress": ...

    def restore(self, progress: "RoiPatrolProgress") -> None: ...


@dataclass(frozen=True, slots=True)
class RoiPatrolProgress:
    """Exact generated ROI path and the next waypoint cursor."""

    waypoints: tuple[Position, ...]
    next_index: int
    allowed_area_wkt: str | None = None

    @property
    def remaining_path(self) -> tuple[Position, ...]:
        if not self.waypoints:
            return ()
        return (
            self.waypoints[self.next_index :]
            + self.waypoints[: self.next_index]
        )


@dataclass(frozen=True, slots=True)
class RoiPatrolConfig:
    """Tuning for the deliberately simple V1 sweep pattern."""

    lane_count: int = 5
    boundary_margin_m: float = 2.0
    max_flight_radius_m: float = default_operating_config.max_flight_radius_m

    def __post_init__(self) -> None:
        if isinstance(self.lane_count, bool) or self.lane_count < 2:
            raise ValueError("lane_count must be at least 2")
        if (
            isinstance(self.boundary_margin_m, bool)
            or not math.isfinite(self.boundary_margin_m)
            or self.boundary_margin_m < 0.0
        ):
            raise ValueError("boundary_margin_m must be finite and non-negative")
        if (
            isinstance(self.max_flight_radius_m, bool)
            or not math.isfinite(self.max_flight_radius_m)
            or self.max_flight_radius_m <= 0.0
        ):
            raise ValueError("max_flight_radius_m must be finite and positive")


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
        self._delivered = 0
        self._allowed_area: Polygon | None = None

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

        if radius_m == 0.0:
            if math.hypot(center[0], center[1]) > self.config.max_flight_radius_m:
                raise ValueError("ROI does not overlap the maximum flight radius")
            self._allowed_area = Point(center).buffer(1e-6)
            self._waypoints = ((center[0], center[1], altitude_m),)
            self._next_index = 0
            return self._waypoints[0]

        home_area = Point(0.0, 0.0).buffer(
            self.config.max_flight_radius_m,
            quad_segs=48,
        )
        roi_area = Point(center).buffer(radius_m, quad_segs=48)
        allowed = home_area.intersection(roi_area)
        if allowed.is_empty or allowed.area <= 1e-7:
            raise ValueError("ROI does not overlap the maximum flight radius")
        if allowed.geom_type != "Polygon":
            raise ValueError("ROI flight area must be one continuous polygon")
        self._allowed_area = allowed

        fully_inside_home = math.hypot(center[0], center[1]) + radius_m <= (
            self.config.max_flight_radius_m + 1e-7
        )
        usable_radius = max(0.0, radius_m - self.config.boundary_margin_m)
        if usable_radius == 0.0:
            if fully_inside_home:
                self._waypoints = ((center[0], center[1], altitude_m),)
            else:
                representative = allowed.representative_point()
                self._waypoints = ((representative.x, representative.y, altitude_m),)
        elif fully_inside_home:
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
        else:
            inset = allowed.buffer(-self.config.boundary_margin_m)
            usable = inset if not inset.is_empty and inset.area > 1e-7 else allowed
            min_x, min_y, max_x, max_y = usable.bounds
            lane_step = (max_y - min_y) / (self.config.lane_count + 1)
            points: list[Position] = []
            for lane_index in range(self.config.lane_count):
                y = min_y + lane_step * (lane_index + 1)
                scan = usable.intersection(
                    LineString([(min_x - 1.0, y), (max_x + 1.0, y)])
                )
                segments = (
                    [scan]
                    if scan.geom_type == "LineString"
                    else list(getattr(scan, "geoms", ()))
                )
                segments = [
                    segment for segment in segments
                    if segment.geom_type == "LineString" and segment.length > 1e-7
                ]
                segments.sort(key=lambda segment: segment.bounds[0])
                if lane_index % 2:
                    segments.reverse()
                for segment in segments:
                    coordinates = list(segment.coords)
                    start = coordinates[0] if lane_index % 2 == 0 else coordinates[-1]
                    end = coordinates[-1] if lane_index % 2 == 0 else coordinates[0]
                    points.extend(
                        (
                            (float(start[0]), float(start[1]), altitude_m),
                            (float(end[0]), float(end[1]), altitude_m),
                        )
                    )
            if not points:
                representative = usable.representative_point()
                points = [(representative.x, representative.y, altitude_m)]
            self._waypoints = tuple(points)

        closest_index = min(
            range(len(self._waypoints)),
            key=lambda index: _horizontal_distance(
                current_position, self._waypoints[index]
            ),
        )
        first = self._waypoints[closest_index]
        self._next_index = (closest_index + 1) % len(self._waypoints)
        self._delivered = 1
        return first

    def entry_target(
        self,
        center: tuple[float, float],
        altitude_m: float,
    ) -> Position:
        if not self._waypoints:
            raise RuntimeError("ROI patrol planner must be reset before use")
        if math.hypot(center[0], center[1]) <= self.config.max_flight_radius_m:
            return (center[0], center[1], altitude_m)
        return self._waypoints[0]

    def contains(self, position: Position) -> bool:
        return bool(
            self._allowed_area is not None
            and self._allowed_area.buffer(1e-7).covers(Point(position[0], position[1]))
        )

    def closest_waypoint(self, position: Position) -> Position:
        if not self._waypoints:
            raise RuntimeError("ROI patrol planner must be reset before use")
        return min(
            self._waypoints,
            key=lambda waypoint: _horizontal_distance(position, waypoint),
        )

    def next_waypoint(self) -> Position:
        if not self._waypoints:
            raise RuntimeError("ROI patrol planner must be reset before use")
        waypoint = self._waypoints[self._next_index]
        self._next_index = (self._next_index + 1) % len(self._waypoints)
        self._delivered += 1
        return waypoint

    @property
    def cycle_complete(self) -> bool:
        """Every waypoint has been handed out once, so reaching the current
        target closes one full lawn-mower cycle. The cursor still wraps: only
        FlightConfig.roi_single_cycle decides whether to act on this."""
        return bool(self._waypoints) and self._delivered >= len(self._waypoints)

    def retarget_altitude(self, new_altitude_m: float) -> None:
        """Rewrite every waypoint's Z in place, preserving XY order and the cursor."""
        self._waypoints = tuple(
            (x, y, new_altitude_m) for x, y, _ in self._waypoints
        )

    def clear(self) -> None:
        self._waypoints = ()
        self._next_index = 0
        self._delivered = 0
        self._allowed_area = None

    def snapshot(self) -> RoiPatrolProgress:
        return RoiPatrolProgress(
            self._waypoints,
            self._next_index,
            None if self._allowed_area is None else self._allowed_area.wkt,
        )

    def restore(self, progress: RoiPatrolProgress) -> None:
        if progress.waypoints and not 0 <= progress.next_index < len(progress.waypoints):
            raise ValueError("ROI progress next_index is outside its waypoint path")
        if not progress.waypoints and progress.next_index != 0:
            raise ValueError("Empty ROI progress must use next_index zero")
        self._waypoints = progress.waypoints
        self._next_index = progress.next_index
        self._allowed_area = (
            None
            if progress.allowed_area_wkt is None
            else wkt.loads(progress.allowed_area_wkt)
        )


def _horizontal_distance(first: Position, second: Position) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])
