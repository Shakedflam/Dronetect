"""Priority-weighted persistent coverage planner for normal PATROL."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import time

from services.gps import Wgs84Point, wgs84_distance_m
from services.patrol_regions import PatrolAreaType, PatrolRegion, PatrolRegionStore


@dataclass(frozen=True, slots=True)
class PatrolScoringConfig:
    age_weight: float = 1.0
    priority_boost: float = 3.0
    travel_weight: float = 1.0
    nominal_travel_speed_mps: float = 7.0

    def __post_init__(self) -> None:
        for name in ("age_weight", "priority_boost", "travel_weight"):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be a finite non-negative number")
        speed = self.nominal_travel_speed_mps
        if isinstance(speed, bool) or not math.isfinite(speed) or speed <= 0:
            raise ValueError(
                "nominal_travel_speed_mps must be a finite positive number"
            )


@dataclass(frozen=True, slots=True)
class PatrolScore:
    total: float
    age_seconds: float
    priority_multiplier: float
    distance_m: float
    travel_seconds: float


@dataclass(frozen=True, slots=True)
class PatrolDecision:
    region: PatrolRegion
    score: PatrolScore


class PatrolRegionScorer:
    """Calculate tunable AoI scores without choosing or mutating regions."""

    def __init__(self, config: PatrolScoringConfig | None = None) -> None:
        self.config = config or PatrolScoringConfig()

    def score(
        self,
        region: PatrolRegion,
        current_position: Wgs84Point,
        *,
        now: float,
        unobserved_since: float,
        travel_distance_m: float | None = None,
    ) -> PatrolScore:
        timestamp = _validated_timestamp(now, "now")
        baseline = _validated_timestamp(unobserved_since, "unobserved_since")
        observation_time = (
            baseline
            if region.last_observed is None
            else _validated_timestamp(region.last_observed, "last_observed")
        )
        age_seconds = max(0.0, timestamp - observation_time)
        priority_multiplier = self.priority_multiplier(region.priority)
        distance_m = (
            wgs84_distance_m(current_position, region.center)
            if travel_distance_m is None
            else _validated_distance(travel_distance_m)
        )
        travel_seconds = distance_m / self.config.nominal_travel_speed_mps
        total = (
            age_seconds * self.config.age_weight * priority_multiplier
            - travel_seconds * self.config.travel_weight
        )
        return PatrolScore(
            total=total,
            age_seconds=age_seconds,
            priority_multiplier=priority_multiplier,
            distance_m=distance_m,
            travel_seconds=travel_seconds,
        )

    def priority_multiplier(self, priority_rank: int | None) -> float:
        """Convert one UI/backend rank into the planner's scoring weight."""
        if priority_rank is None:
            return 1.0
        if (
            isinstance(priority_rank, bool)
            or not isinstance(priority_rank, int)
            or priority_rank < 1
        ):
            raise ValueError("priority rank must be a positive integer or None")
        return 1.0 + self.config.priority_boost / priority_rank


class PatrolPlanner:
    """Select the next normal PATROL region without issuing flight commands."""

    def __init__(
        self,
        store: PatrolRegionStore | None = None,
        scorer: PatrolRegionScorer | None = None,
        *,
        clock: Callable[[], float] = time.time,
        started_at: float | None = None,
    ) -> None:
        self.store = store or PatrolRegionStore()
        self.scorer = scorer or PatrolRegionScorer()
        self._clock = clock
        initial_time = clock() if started_at is None else started_at
        self.started_at = _validated_timestamp(initial_time, "started_at")

    def choose_next(
        self,
        current_position: Wgs84Point,
        now: float | None = None,
        *,
        route_distance_m: Callable[[PatrolRegion], float | None] | None = None,
    ) -> PatrolDecision:
        timestamp = self._resolve_time(now)
        decisions: list[PatrolDecision] = []
        for region in self.store.regions:
            if region.area_type is not PatrolAreaType.PATROL:
                continue
            travel_distance = (
                None if route_distance_m is None else route_distance_m(region)
            )
            if route_distance_m is not None and travel_distance is None:
                continue
            decisions.append(
                PatrolDecision(
                    region=region,
                    score=self.scorer.score(
                        region,
                        current_position,
                        now=timestamp,
                        unobserved_since=self.started_at,
                        travel_distance_m=travel_distance,
                    ),
                )
            )
        if not decisions:
            raise RuntimeError("Cannot plan patrol without reachable regions")

        return min(
            decisions,
            key=lambda decision: (
                -decision.score.total,
                -decision.score.age_seconds,
                decision.score.distance_m,
                decision.region.id,
            ),
        )

    @property
    def uptime_s(self) -> float:
        """Seconds since this planner's baseline -- the age every unobserved
        region starts from, and so how strongly ranks currently dominate."""
        return max(0.0, self._resolve_time(None) - self.started_at)

    def record_observation(
        self,
        region_id: str,
        observed_at: float | None = None,
    ) -> PatrolRegion:
        return self.store.record_observation(
            region_id,
            observed_at=self._resolve_time(observed_at),
        )

    def _resolve_time(self, value: float | None) -> float:
        timestamp = self._clock() if value is None else value
        return _validated_timestamp(timestamp, "timestamp")


def _validated_timestamp(value: float, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite non-negative timestamp")
    timestamp = float(value)
    if not math.isfinite(timestamp) or timestamp < 0:
        raise ValueError(f"{name} must be a finite non-negative timestamp")
    return timestamp


def _validated_distance(value: float) -> float:
    if isinstance(value, bool):
        raise ValueError("travel_distance_m must be a finite non-negative number")
    distance = float(value)
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError("travel_distance_m must be a finite non-negative number")
    return distance


def describe_patrol_plan(
    decision: PatrolDecision,
    *,
    source: str,
    position: tuple[float, float, float],
    first_waypoint: tuple[float, float, float],
    waypoint_count: int,
    patrol_radius_m: float,
    planner_uptime_s: float,
) -> str:
    """One log line per patrol plan, shared by the simulator and DJI bridge.

    Positions are local ENU metres from home (x=East, y=North); the bearing is
    a compass bearing from the aircraft to the first waypoint, so the line
    reads directly against the tablet map.
    """
    region = decision.region
    score = decision.score
    dx = first_waypoint[0] - position[0]
    dy = first_waypoint[1] - position[1]
    bearing_deg = math.degrees(math.atan2(dx, dy)) % 360.0
    return (
        f"patrol_plan source={source} region={region.id} "
        f"rank={region.priority if region.priority is not None else '-'} "
        f"score={score.total:.1f} ageS={score.age_seconds:.0f} "
        f"priorityX={score.priority_multiplier:.2f} travelM={score.distance_m:.0f} "
        f"radiusM={patrol_radius_m:.0f} "
        f"bandM={region.inner_radius_m:.0f}-{region.outer_radius_m:.0f} "
        f"sectorDeg={region.start_bearing_deg:.1f}-{region.end_bearing_deg:.1f} "
        f"pos=({position[0]:.1f},{position[1]:.1f},{position[2]:.1f}) "
        f"firstWp=({first_waypoint[0]:.1f},{first_waypoint[1]:.1f},{first_waypoint[2]:.1f}) "
        f"bearingDeg={bearing_deg:.0f} legM={math.hypot(dx, dy):.0f} "
        f"waypoints={waypoint_count} plannerUptimeS={planner_uptime_s:.0f}"
    )
