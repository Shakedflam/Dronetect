"""Generate the static 8-direction x 4-band PATROL regions inside the Home circle."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum
import math
import time

from config.shared_config import shared_config
from services.gps import DEFAULT_HOME, Wgs84Point, destination_point


# Sourced from config/shared.json (see config/shared_config.py): the frontend
# generates its own copy of this same direction x band grid (for map display
# and click-to-rank/click-to-restrict) from that same file, and only region
# IDs (e.g. "SW-1") cross the wire -- never geometry. A mismatch here would
# make the frontend's polygons/priority badges point at different real-world
# places than the regions this store (and the geofence derived from it)
# actually flies against.
HOME_RADIUS_M = shared_config.home_radius_m
PATROL_BAND_COUNT = shared_config.patrol_band_count
SECTOR_HALF_WIDTH_DEG = 22.5

PATROL_DIRECTIONS: tuple[tuple[float, str], ...] = (
    (0.0, "N"),
    (45.0, "NE"),
    (90.0, "E"),
    (135.0, "SE"),
    (180.0, "S"),
    (225.0, "SW"),
    (270.0, "W"),
    (315.0, "NW"),
)
PATROL_REGION_IDS = frozenset(
    f"{label}-{band_index}"
    for _, label in PATROL_DIRECTIONS
    for band_index in range(1, PATROL_BAND_COUNT + 1)
)


class PatrolAreaType(StrEnum):
    PATROL = "PATROL"
    IGNORE = "IGNORE"
    NO_FLY = "NO_FLY"


@dataclass(frozen=True, slots=True)
class PatrolRegion:
    """One static direction x band cell of the Home circle.

    Geometry is fixed at generation time. Priority and observation time are
    backend-owned values managed by PatrolRegionStore.
    """

    id: str
    direction: str
    direction_deg: float
    band_index: int
    inner_radius_m: float
    outer_radius_m: float
    start_bearing_deg: float
    end_bearing_deg: float
    center: Wgs84Point
    priority: int | None = None
    last_observed: float | None = None
    area_type: PatrolAreaType = PatrolAreaType.PATROL

    @property
    def priority_rank(self) -> int | None:
        """Compatibility name matching the current UI patrol model."""
        return self.priority


def patrol_band_outer_radii_m(
    home_radius_m: float,
    band_count: int = PATROL_BAND_COUNT,
) -> list[float]:
    """Equal-area band outer radii: outer(k) = home_radius_m * sqrt(k / N).

    Ring area is proportional to outer^2 - inner^2, so this makes each of the
    N concentric rings cover exactly 1/N of the Home circle's area. Every
    radius here scales linearly with home_radius_m, so growing or shrinking
    the Home circle rescales every band boundary by the same factor and the
    equal-area split holds at any radius.
    """
    if not math.isfinite(home_radius_m) or home_radius_m <= 0:
        raise ValueError("home_radius_m must be a finite positive number")
    if isinstance(band_count, bool) or not isinstance(band_count, int) or band_count <= 0:
        raise ValueError("band_count must be a positive integer")

    return [
        home_radius_m * math.sqrt((k + 1) / band_count) for k in range(band_count)
    ]


def generate_patrol_regions(
    home: Wgs84Point = DEFAULT_HOME,
    home_radius_m: float = HOME_RADIUS_M,
) -> list[PatrolRegion]:
    """Build the 8 x PATROL_BAND_COUNT static patrol region grid around home."""
    outer_radii = patrol_band_outer_radii_m(home_radius_m)
    regions: list[PatrolRegion] = []

    for direction_deg, label in PATROL_DIRECTIONS:
        for band_index in range(1, PATROL_BAND_COUNT + 1):
            inner_radius_m = 0.0 if band_index == 1 else outer_radii[band_index - 2]
            outer_radius_m = outer_radii[band_index - 1]
            mid_radius_m = (inner_radius_m + outer_radius_m) / 2
            center = destination_point(home, mid_radius_m, direction_deg)

            regions.append(
                PatrolRegion(
                    id=f"{label}-{band_index}",
                    direction=label,
                    direction_deg=direction_deg,
                    band_index=band_index,
                    inner_radius_m=inner_radius_m,
                    outer_radius_m=outer_radius_m,
                    start_bearing_deg=(direction_deg - SECTOR_HALF_WIDTH_DEG) % 360,
                    end_bearing_deg=(direction_deg + SECTOR_HALF_WIDTH_DEG) % 360,
                    center=center,
                )
            )

    return regions


def patrol_region_polygon_points(
    region: PatrolRegion,
    home: Wgs84Point = DEFAULT_HOME,
    *,
    arc_step_deg: float = 2.5,
) -> tuple[Wgs84Point, ...]:
    """Return a geographic polygon tracing one annular patrol sector.

    The outer arc is walked from the region's start bearing to its end
    bearing, then the inner arc is walked in reverse. Chords are kept short
    by ``arc_step_deg`` so coverage and no-fly geometry closely follow the
    circular region boundaries.
    """
    if not math.isfinite(arc_step_deg) or arc_step_deg <= 0.0:
        raise ValueError("arc_step_deg must be a finite positive number")

    sweep_deg = (region.end_bearing_deg - region.start_bearing_deg) % 360.0
    steps = max(1, math.ceil(sweep_deg / arc_step_deg))
    bearings = [
        region.start_bearing_deg + sweep_deg * index / steps
        for index in range(steps + 1)
    ]
    outer = [
        destination_point(home, region.outer_radius_m, bearing)
        for bearing in bearings
    ]
    inner = [
        destination_point(home, region.inner_radius_m, bearing)
        for bearing in reversed(bearings)
    ]

    points: list[Wgs84Point] = []
    for point in [*outer, *inner]:
        if not points or point != points[-1]:
            points.append(point)
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    return tuple(points)


class PatrolRegionStore:
    """Own mutable patrol metadata while keeping region geometry immutable."""

    def __init__(
        self,
        home: Wgs84Point = DEFAULT_HOME,
        home_radius_m: float = HOME_RADIUS_M,
    ) -> None:
        self._regions = {
            region.id: region
            for region in generate_patrol_regions(home, home_radius_m)
        }

    @property
    def regions(self) -> tuple[PatrolRegion, ...]:
        return tuple(self._regions.values())

    def get(self, region_id: str) -> PatrolRegion:
        try:
            return self._regions[region_id]
        except KeyError as exc:
            raise KeyError(f"Unknown patrol region: {region_id}") from exc

    def set_priority(self, region_id: str, priority: int | None) -> PatrolRegion:
        if priority is not None and (
            isinstance(priority, bool) or not isinstance(priority, int) or priority < 1
        ):
            raise ValueError("priority must be a positive integer or None")
        updated = replace(self.get(region_id), priority=priority)
        self._regions[region_id] = updated
        return updated

    def set_area_type(
        self,
        region_id: str,
        area_type: PatrolAreaType | str,
    ) -> PatrolRegion:
        resolved = PatrolAreaType(area_type)
        updated = replace(self.get(region_id), area_type=resolved)
        self._regions[region_id] = updated
        if resolved is not PatrolAreaType.PATROL:
            self._regions[region_id] = replace(updated, priority=None)
            self._compact_priorities()
        return self._regions[region_id]

    def replace_area_types(
        self,
        area_types: list[tuple[str, PatrolAreaType | str]],
    ) -> tuple[PatrolRegion, ...]:
        """Replace all type overrides; omitted regions default to PATROL."""
        resolved: dict[str, PatrolAreaType] = {}
        for region_id, area_type in area_types:
            if region_id in resolved:
                raise ValueError("patrol area type region IDs must be unique")
            self.get(region_id)
            resolved[region_id] = PatrolAreaType(area_type)
        self._regions = {
            region_id: replace(
                region,
                area_type=resolved.get(region_id, PatrolAreaType.PATROL),
                priority=(
                    region.priority
                    if resolved.get(region_id, PatrolAreaType.PATROL)
                    is PatrolAreaType.PATROL
                    else None
                ),
            )
            for region_id, region in self._regions.items()
        }
        self._compact_priorities()
        return self.regions

    def _compact_priorities(self) -> None:
        ranked = sorted(
            (region for region in self._regions.values() if region.priority is not None),
            key=lambda region: region.priority,
        )
        compacted = {region.id: rank for rank, region in enumerate(ranked, start=1)}
        self._regions = {
            region_id: replace(region, priority=compacted.get(region_id))
            for region_id, region in self._regions.items()
        }

    def replace_priorities(
        self,
        priorities: list[tuple[str, int]],
    ) -> tuple[PatrolRegion, ...]:
        """Atomically replace the complete user priority ranking."""
        for region_id, rank in priorities:
            if not isinstance(region_id, str):
                raise ValueError("patrol priority region IDs must be strings")
            if isinstance(rank, bool) or not isinstance(rank, int) or rank < 1:
                raise ValueError("patrol priority ranks must be positive integers")
        region_ids = [region_id for region_id, _ in priorities]
        ranks = [rank for _, rank in priorities]
        if len(region_ids) != len(set(region_ids)):
            raise ValueError("patrol priority region IDs must be unique")
        if len(ranks) != len(set(ranks)):
            raise ValueError("patrol priority ranks must be unique")
        if set(ranks) != set(range(1, len(ranks) + 1)):
            raise ValueError("patrol priority ranks must be contiguous from 1")
        for region_id in region_ids:
            self.get(region_id)

        ranks_by_region = dict(priorities)
        replacements = {
            region_id: replace(region, priority=ranks_by_region.get(region_id))
            for region_id, region in self._regions.items()
        }
        self._regions = replacements
        return self.regions

    def record_observation(
        self,
        region_id: str,
        observed_at: float | None = None,
    ) -> PatrolRegion:
        if isinstance(observed_at, bool):
            raise ValueError("observed_at must be a finite non-negative timestamp")
        timestamp = time.time() if observed_at is None else float(observed_at)
        if not math.isfinite(timestamp) or timestamp < 0:
            raise ValueError("observed_at must be a finite non-negative timestamp")
        updated = replace(self.get(region_id), last_observed=timestamp)
        self._regions[region_id] = updated
        return updated
