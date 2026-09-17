"""Shortest-path V1 routing around NO_FLY patrol cells."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

from services.patrol_regions import PatrolAreaType, PatrolRegion


Point2 = tuple[float, float]


class NoFlyRouteUnavailable(RuntimeError):
    """Raised when no route can be found without entering a NO_FLY cell."""


@dataclass(frozen=True, slots=True)
class NoFlyRoutingConfig:
    clearance_m: float = 10.0
    arc_step_deg: float = 7.5
    collision_sample_m: float = 3.0

    def __post_init__(self) -> None:
        for name in ("clearance_m", "arc_step_deg", "collision_sample_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a finite positive number")


@dataclass(frozen=True, slots=True)
class _PolarObstacle:
    inner_radius_m: float
    outer_radius_m: float
    start_bearing_deg: float
    sweep_deg: float


class NoFlyRouter:
    """Build a shortest visibility path beside expanded NO_FLY boundaries.

    Patrol cells are concentric annular sectors, so their buffered radial and
    angular boundaries provide a small, deterministic visibility graph. The
    resulting route contains straight flight waypoints and always includes
    the requested destination as its final point.
    """

    _BOUNDARY_OFFSET_M = 0.5

    def __init__(self, config: NoFlyRoutingConfig | None = None) -> None:
        self.config = config or NoFlyRoutingConfig()

    def route(
        self,
        start: Point2,
        destination: Point2,
        regions: tuple[PatrolRegion, ...] | list[PatrolRegion],
    ) -> list[Point2]:
        no_fly = [
            region for region in regions
            if region.area_type is PatrolAreaType.NO_FLY
        ]
        if not no_fly or not self.segment_crosses_no_fly(start, destination, regions):
            return [destination]

        max_radius_m = max(region.outer_radius_m for region in regions)
        obstacles = [self._expanded_obstacle(region, max_radius_m) for region in no_fly]
        candidates = self._candidate_points(obstacles, max_radius_m)
        nodes = self._deduplicate([start, destination, *candidates])
        nodes = [
            point for point in nodes
            if point in {start, destination}
            or not self._inside_any(point, obstacles)
        ]
        start_index = nodes.index(start)
        destination_index = nodes.index(destination)
        distances = [math.inf] * len(nodes)
        previous: list[int | None] = [None] * len(nodes)
        distances[start_index] = 0.0
        queue: list[tuple[float, int]] = [(0.0, start_index)]

        while queue:
            distance, index = heapq.heappop(queue)
            if distance != distances[index]:
                continue
            if index == destination_index:
                break
            for neighbor, point in enumerate(nodes):
                if neighbor == index:
                    continue
                edge_length = math.dist(nodes[index], point)
                candidate_distance = distance + edge_length
                if candidate_distance >= distances[neighbor]:
                    continue
                if self._segment_hits_obstacles(nodes[index], point, obstacles):
                    continue
                distances[neighbor] = candidate_distance
                previous[neighbor] = index
                heapq.heappush(queue, (candidate_distance, neighbor))

        if not math.isfinite(distances[destination_index]):
            raise NoFlyRouteUnavailable("No route around the active NO_FLY areas")

        path: list[Point2] = []
        cursor: int | None = destination_index
        while cursor is not None:
            path.append(nodes[cursor])
            cursor = previous[cursor]
        path.reverse()
        return self._simplify(path, obstacles)[1:]

    def egress_route(
        self,
        start: Point2,
        destination: Point2,
        regions: tuple[PatrolRegion, ...] | list[PatrolRegion],
    ) -> list[Point2]:
        """Leave containing NO_FLY cells, then route normally to destination.

        Only the initial segment may occupy cells containing ``start``. It may
        not enter another cell, and after it exits it may not re-enter.
        """
        no_fly = [region for region in regions if region.area_type is PatrolAreaType.NO_FLY]
        if not no_fly:
            return [destination]
        max_radius_m = max(region.outer_radius_m for region in regions)
        obstacles = [self._expanded_obstacle(region, max_radius_m) for region in no_fly]
        containing = [obstacle for obstacle in obstacles if self._inside(start, obstacle)]
        if not containing:
            return self.route(start, destination, regions)

        candidates = self._candidate_points(containing, max_radius_m)
        candidates.extend(
            self._direct_egress_candidates(start, destination, containing, max_radius_m)
        )
        best: tuple[float, list[Point2]] | None = None
        for candidate in self._deduplicate(candidates):
            if self._inside_any(candidate, obstacles):
                continue
            if not self._valid_egress_segment(start, candidate, containing, obstacles):
                continue
            try:
                onward = self.route(candidate, destination, regions)
            except NoFlyRouteUnavailable:
                continue
            path = [candidate, *onward]
            length = sum(
                math.dist(left, right)
                for left, right in zip([start, *path], path)
            )
            if best is None or length < best[0]:
                best = (length, path)
        if best is None:
            raise NoFlyRouteUnavailable("No egress route out of the active NO_FLY area")
        return best[1]

    def segment_crosses_no_fly(
        self,
        start: Point2,
        destination: Point2,
        regions: tuple[PatrolRegion, ...] | list[PatrolRegion],
    ) -> bool:
        if not regions:
            return False
        max_radius_m = max(region.outer_radius_m for region in regions)
        obstacles = [
            self._expanded_obstacle(region, max_radius_m)
            for region in regions
            if region.area_type is PatrolAreaType.NO_FLY
        ]
        return self._segment_hits_obstacles(start, destination, obstacles)

    def _expanded_obstacle(
        self,
        region: PatrolRegion,
        max_radius_m: float,
    ) -> _PolarObstacle:
        clearance = self.config.clearance_m
        angular_margin = math.degrees(clearance / max(region.outer_radius_m, clearance))
        sweep = (region.end_bearing_deg - region.start_bearing_deg) % 360.0
        return _PolarObstacle(
            inner_radius_m=max(0.0, region.inner_radius_m - clearance),
            outer_radius_m=min(max_radius_m, region.outer_radius_m + clearance),
            start_bearing_deg=(region.start_bearing_deg - angular_margin) % 360.0,
            sweep_deg=min(359.0, sweep + 2.0 * angular_margin),
        )

    def _candidate_points(
        self,
        obstacles: list[_PolarObstacle],
        max_radius_m: float,
    ) -> list[Point2]:
        points: list[Point2] = []
        offset = self._BOUNDARY_OFFSET_M
        for obstacle in obstacles:
            bearings = self._arc_bearings(
                obstacle.start_bearing_deg,
                obstacle.sweep_deg,
            )
            if obstacle.outer_radius_m < max_radius_m - offset:
                points.extend(
                    self._polar(obstacle.outer_radius_m + offset, bearing)
                    for bearing in bearings
                )
            if obstacle.inner_radius_m > offset:
                points.extend(
                    self._polar(obstacle.inner_radius_m - offset, bearing)
                    for bearing in bearings
                )
        return points

    def _arc_bearings(self, start: float, sweep: float) -> list[float]:
        steps = max(1, math.ceil(sweep / self.config.arc_step_deg))
        return [(start + sweep * index / steps) % 360.0 for index in range(steps + 1)]

    @staticmethod
    def _polar(radius_m: float, bearing_deg: float) -> Point2:
        bearing = math.radians(bearing_deg)
        return (radius_m * math.sin(bearing), radius_m * math.cos(bearing))

    def _segment_hits_obstacles(
        self,
        start: Point2,
        destination: Point2,
        obstacles: list[_PolarObstacle],
    ) -> bool:
        if not obstacles:
            return False
        steps = max(
            1,
            math.ceil(math.dist(start, destination) / self.config.collision_sample_m),
        )
        for index in range(steps + 1):
            fraction = index / steps
            point = (
                start[0] + (destination[0] - start[0]) * fraction,
                start[1] + (destination[1] - start[1]) * fraction,
            )
            if self._inside_any(point, obstacles):
                return True
        return False

    @staticmethod
    def _inside_any(point: Point2, obstacles: list[_PolarObstacle]) -> bool:
        radius = math.hypot(*point)
        bearing = math.degrees(math.atan2(point[0], point[1])) % 360.0
        for obstacle in obstacles:
            angular_offset = (bearing - obstacle.start_bearing_deg) % 360.0
            if not 1e-7 < angular_offset < obstacle.sweep_deg - 1e-7:
                continue
            if (
                obstacle.inner_radius_m + 1e-6
                < radius
                < obstacle.outer_radius_m - 1e-6
            ):
                return True
        return False

    @staticmethod
    def _inside(point: Point2, obstacle: _PolarObstacle) -> bool:
        radius = math.hypot(*point)
        bearing = math.degrees(math.atan2(point[0], point[1])) % 360.0
        angular_offset = (bearing - obstacle.start_bearing_deg) % 360.0
        return (
            1e-7 < angular_offset < obstacle.sweep_deg - 1e-7
            and obstacle.inner_radius_m + 1e-6 < radius < obstacle.outer_radius_m - 1e-6
        )

    def _direct_egress_candidates(
        self,
        start: Point2,
        destination: Point2,
        obstacles: list[_PolarObstacle],
        max_radius_m: float,
    ) -> list[Point2]:
        start_radius = math.hypot(*start)
        start_bearing = math.degrees(math.atan2(start[0], start[1])) % 360.0
        destination_bearing = math.degrees(math.atan2(destination[0], destination[1])) % 360.0
        offset = self._BOUNDARY_OFFSET_M
        candidates: list[Point2] = []
        for obstacle in obstacles:
            bearings = (start_bearing, destination_bearing)
            if obstacle.inner_radius_m > offset:
                candidates.extend(
                    self._polar(obstacle.inner_radius_m - offset, bearing)
                    for bearing in bearings
                )
            if obstacle.outer_radius_m < max_radius_m - offset:
                candidates.extend(
                    self._polar(obstacle.outer_radius_m + offset, bearing)
                    for bearing in bearings
                )
            radius = min(
                obstacle.outer_radius_m + offset,
                max(obstacle.inner_radius_m - offset, start_radius),
            )
            candidates.extend(
                (
                    self._polar(radius, obstacle.start_bearing_deg - 0.01),
                    self._polar(radius, obstacle.start_bearing_deg + obstacle.sweep_deg + 0.01),
                )
            )
        return candidates

    def _valid_egress_segment(
        self,
        start: Point2,
        destination: Point2,
        containing: list[_PolarObstacle],
        all_obstacles: list[_PolarObstacle],
    ) -> bool:
        other_obstacles = [item for item in all_obstacles if item not in containing]
        steps = max(1, math.ceil(math.dist(start, destination) / self.config.collision_sample_m))
        exited = False
        for index in range(1, steps + 1):
            fraction = index / steps
            point = (
                start[0] + (destination[0] - start[0]) * fraction,
                start[1] + (destination[1] - start[1]) * fraction,
            )
            if self._inside_any(point, other_obstacles):
                return False
            inside_source = self._inside_any(point, containing)
            if exited and inside_source:
                return False
            exited = exited or not inside_source
        return exited

    def _simplify(
        self,
        path: list[Point2],
        obstacles: list[_PolarObstacle],
    ) -> list[Point2]:
        simplified = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            next_index = len(path) - 1
            while (
                next_index > anchor + 1
                and self._segment_hits_obstacles(path[anchor], path[next_index], obstacles)
            ):
                next_index -= 1
            simplified.append(path[next_index])
            anchor = next_index
        return simplified

    @staticmethod
    def _deduplicate(points: list[Point2]) -> list[Point2]:
        unique: list[Point2] = []
        seen: set[tuple[float, float]] = set()
        for point in points:
            key = (round(point[0], 6), round(point[1], 6))
            if key not in seen:
                seen.add(key)
                unique.append(point)
        return unique
