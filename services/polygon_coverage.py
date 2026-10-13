"""Deterministic zigzag coverage paths for geographic polygons."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from collections.abc import Sequence

from shapely import affinity
from shapely.geometry import GeometryCollection, LineString, MultiLineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from services.gps import (
    LocalPosition,
    Wgs84Point,
    local_meters_to_wgs84,
    wgs84_to_local_meters,
)


Point2 = tuple[float, float]


class CoveragePathUnavailable(RuntimeError):
    """Raised when a polygon cannot be covered by one safe continuous path."""


@dataclass(frozen=True, slots=True)
class PolygonCoverageConfig:
    lane_spacing_m: float = 75.0
    boundary_inset_m: float = 0.5
    no_fly_clearance_m: float = 10.0

    def __post_init__(self) -> None:
        for name in ("lane_spacing_m", "no_fly_clearance_m"):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be a finite positive number")
        inset = self.boundary_inset_m
        if isinstance(inset, bool) or not math.isfinite(inset) or inset < 0.0:
            raise ValueError("boundary_inset_m must be finite and non-negative")


class PolygonCoveragePlanner:
    """Generate a continuous longest-axis zigzag within a free-space polygon."""

    _EPSILON = 1e-7

    def __init__(self, config: PolygonCoverageConfig | None = None) -> None:
        self.config = config or PolygonCoverageConfig()

    def plan(
        self,
        polygon: Sequence[Wgs84Point],
        *,
        no_fly_polygons: Sequence[Sequence[Wgs84Point]] = (),
        outward_from: Wgs84Point | None = None,
        start_from: Wgs84Point | None = None,
    ) -> tuple[Wgs84Point, ...]:
        """Return ordered geographic waypoints covering ``polygon``.

        Only geometry supplied through ``no_fly_polygons`` is subtracted;
        callers must deliberately omit IGNORE regions.

        ``start_from`` (the aircraft) picks, among the four orientations of
        the same sweep -- run forwards or backwards, first lane entered from
        either end -- the one that begins nearest to it. Every orientation
        flies exactly the same lanes, so coverage is unchanged; only the
        approach is. It supersedes ``outward_from``, which orders lanes from a
        fixed point (home) and ignores where the aircraft actually is.
        """
        if len(polygon) < 3:
            raise ValueError("coverage polygon must contain at least three points")

        anchor = _geographic_centroid(polygon)
        shell = [_to_local(point, anchor) for point in polygon]
        target = _validated_polygon(shell, "coverage polygon")
        if self.config.boundary_inset_m:
            target = target.buffer(-self.config.boundary_inset_m)
        if target.is_empty:
            raise CoveragePathUnavailable("coverage polygon is smaller than its boundary inset")

        obstacles: list[BaseGeometry] = []
        for index, no_fly_polygon in enumerate(no_fly_polygons):
            if len(no_fly_polygon) < 3:
                raise ValueError(f"no-fly polygon {index} must contain at least three points")
            obstacle = _validated_polygon(
                [_to_local(point, anchor) for point in no_fly_polygon],
                f"no-fly polygon {index}",
            )
            obstacles.append(
                obstacle.buffer(self.config.no_fly_clearance_m, quad_segs=2)
            )
        if obstacles:
            target = target.difference(unary_union(obstacles))
        if target.is_empty:
            raise CoveragePathUnavailable("no usable coverage area remains")
        if target.geom_type != "Polygon":
            raise CoveragePathUnavailable(
                "usable coverage area is disconnected; no continuous in-polygon path exists"
            )

        axis_angle = _longest_axis_angle(target)
        rotation_origin = target.centroid.coords[0]
        rotated = affinity.rotate(target, -axis_angle, origin=rotation_origin)
        min_x, min_y, max_x, max_y = rotated.bounds
        lane_ys = _lane_offsets(min_y, max_y, self.config.lane_spacing_m)

        if start_from is None and outward_from is not None and len(lane_ys) > 1:
            outward_local = Point(_to_local(outward_from, anchor))
            outward_rotated = affinity.rotate(
                outward_local, -axis_angle, origin=rotation_origin
            )
            if abs(lane_ys[-1] - outward_rotated.y) < abs(
                lane_ys[0] - outward_rotated.y
            ):
                lane_ys.reverse()

        rotated_path = _sweep(rotated, lane_ys, min_x, max_x, first_left_to_right=True)
        if start_from is not None and rotated_path:
            start_rotated = affinity.rotate(
                Point(_to_local(start_from, anchor)), -axis_angle, origin=rotation_origin
            ).coords[0]
            candidates: list[list[Point2]] = []
            for first_left_to_right in (True, False):
                path = (rotated_path if first_left_to_right else
                        _sweep(rotated, lane_ys, min_x, max_x, first_left_to_right=False))
                candidates.extend([path, path[::-1]])
            # Strict min keeps the default orientation on a tie.
            rotated_path = min(candidates, key=lambda path: math.dist(path[0], start_rotated))

        if not rotated_path:
            raise CoveragePathUnavailable("coverage polygon produced no usable lanes")

        local_path = [
            affinity.rotate(Point(point), axis_angle, origin=rotation_origin).coords[0]
            for point in rotated_path
        ]
        validation_area = target.buffer(self._EPSILON)
        for start, end in zip(local_path, local_path[1:]):
            if not validation_area.covers(LineString([start, end])):
                raise CoveragePathUnavailable("generated coverage segment leaves usable geometry")

        return tuple(
            local_meters_to_wgs84(LocalPosition(x=x, y=y, z=0.0), anchor)
            for x, y in local_path
        )


def _sweep(
    rotated: BaseGeometry, lane_ys: Sequence[float], min_x: float, max_x: float,
    *, first_left_to_right: bool,
) -> list[Point2]:
    """One boustrophedon pass over ``lane_ys``, alternating direction per lane."""
    rotated_path: list[Point2] = []
    for lane_index, lane_y in enumerate(lane_ys):
        scan = LineString(
            [(min_x - 1.0, lane_y), (max_x + 1.0, lane_y)]
        )
        segments = sorted(
            _line_segments(rotated.intersection(scan)),
            key=lambda segment: segment.bounds[0],
        )
        if not segments:
            continue
        left_to_right = (lane_index % 2 == 0) == first_left_to_right
        if not left_to_right:
            segments.reverse()
        for segment in segments:
            coordinates = list(segment.coords)
            start = coordinates[0] if left_to_right else coordinates[-1]
            end = coordinates[-1] if left_to_right else coordinates[0]
            if rotated_path:
                connector = _safe_connector(rotated_path[-1], start, rotated)
                _extend_unique(rotated_path, connector[1:])
            else:
                rotated_path.append(start)
            _extend_unique(rotated_path, [end])
    return rotated_path


def _geographic_centroid(points: Sequence[Wgs84Point]) -> Wgs84Point:
    return Wgs84Point(
        lat=sum(point.lat for point in points) / len(points),
        lng=sum(point.lng for point in points) / len(points),
    )


def _to_local(point: Wgs84Point, anchor: Wgs84Point) -> Point2:
    local = wgs84_to_local_meters(point, anchor)
    return (local.x, local.y)


def _validated_polygon(points: Sequence[Point2], label: str) -> Polygon:
    polygon = Polygon(points)
    if polygon.is_empty or polygon.area <= 1e-6:
        raise ValueError(f"{label} must have positive area")
    if not polygon.is_valid:
        raise ValueError(f"{label} must be a valid simple polygon")
    return polygon


def _longest_axis_angle(polygon: Polygon) -> float:
    coordinates = list(polygon.minimum_rotated_rectangle.exterior.coords)
    edges: list[tuple[float, float, float]] = []
    for start, end in zip(coordinates, coordinates[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        angle = math.degrees(math.atan2(dy, dx)) % 180.0
        edges.append((math.hypot(dx, dy), -angle, angle))
    return max(edges)[2]


def _lane_offsets(minimum: float, maximum: float, spacing: float) -> list[float]:
    height = maximum - minimum
    if height <= 1e-9:
        return [(minimum + maximum) / 2.0]
    count = max(1, math.ceil((height - 1e-7) / spacing))
    occupied = (count - 1) * spacing
    margin = (height - occupied) / 2.0
    return [minimum + margin + index * spacing for index in range(count)]


def _line_segments(geometry: BaseGeometry) -> list[LineString]:
    if isinstance(geometry, LineString):
        return [geometry] if geometry.length > 1e-7 else []
    if isinstance(geometry, MultiLineString):
        return [line for line in geometry.geoms if line.length > 1e-7]
    if isinstance(geometry, GeometryCollection):
        segments: list[LineString] = []
        for child in geometry.geoms:
            segments.extend(_line_segments(child))
        return segments
    return []


def _safe_connector(start: Point2, end: Point2, free_space: Polygon) -> list[Point2]:
    tolerant_space = free_space.buffer(1e-7)
    direct = LineString([start, end])
    if tolerant_space.covers(direct):
        return [start, end]

    boundary_points: list[Point2] = []
    rings = [free_space.exterior, *free_space.interiors]
    for ring in rings:
        boundary_points.extend(list(ring.coords)[:-1])
    nodes = _deduplicate([start, end, *boundary_points])
    start_index = nodes.index(start)
    end_index = nodes.index(end)
    distances = [math.inf] * len(nodes)
    previous: list[int | None] = [None] * len(nodes)
    distances[start_index] = 0.0
    queue: list[tuple[float, int]] = [(0.0, start_index)]

    while queue:
        distance, index = heapq.heappop(queue)
        if distance != distances[index]:
            continue
        if index == end_index:
            break
        for neighbor, point in enumerate(nodes):
            if neighbor == index:
                continue
            edge = LineString([nodes[index], point])
            if not tolerant_space.covers(edge):
                continue
            candidate = distance + edge.length
            if candidate + 1e-9 >= distances[neighbor]:
                continue
            distances[neighbor] = candidate
            previous[neighbor] = index
            heapq.heappush(queue, (candidate, neighbor))

    if not math.isfinite(distances[end_index]):
        raise CoveragePathUnavailable("coverage lanes cannot be connected inside usable geometry")
    path: list[Point2] = []
    cursor: int | None = end_index
    while cursor is not None:
        path.append(nodes[cursor])
        cursor = previous[cursor]
    path.reverse()
    return path


def _deduplicate(points: Sequence[Point2]) -> list[Point2]:
    unique: list[Point2] = []
    seen: set[tuple[float, float]] = set()
    for point in points:
        normalized = (float(point[0]), float(point[1]))
        key = (round(normalized[0], 7), round(normalized[1], 7))
        if key not in seen:
            seen.add(key)
            unique.append(normalized)
    return unique


def _extend_unique(path: list[Point2], points: Sequence[Point2]) -> None:
    for point in points:
        normalized = (float(point[0]), float(point[1]))
        if not path or math.dist(path[-1], normalized) > 1e-7:
            path.append(normalized)
