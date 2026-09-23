"""Pure decision logic for applying a new OperatingConfig mid-flight.

Deciding what to rebuild (patrol geometry, geofence, ROI clipping) and
whether a radius change requires flying to a rescaled position before
resuming patrol/ROI is kept independent of Webots so it can be unit tested
directly, the same way flight_logic.py and roi_patrol.py already are.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import TYPE_CHECKING

from config.shared_config import OperatingConfig
from services.gps import DEFAULT_HOME, Wgs84Point
from services.no_fly_router import NoFlyRouteUnavailable, NoFlyRouter
from services.patrol_planner import PatrolPlanner
from services.patrol_regions import PatrolAreaType, PatrolRegionStore

if TYPE_CHECKING:
    # Type-checking only: avoid importing flight_logic/pursuit/roi_patrol at
    # runtime. Webots loads those modules under a top-level name
    # ("flight_logic", not "controllers.autonomous_drone.flight_logic"); a
    # second, differently-named import here would create a *second* set of
    # FlightState/PursuitConfig/RoiPatrolConfig classes, and enum members
    # from the two copies compare unequal. Everything below therefore works
    # with plain state-name strings and pre-built instances instead.
    from controllers.autonomous_drone.pursuit import PursuitConfig
    from controllers.autonomous_drone.roi_patrol import RoiPatrolConfig


Position = tuple[float, float, float]

_RESUME_STATE_FOR_CURRENT_STATE = {
    "PATROLLING": "PATROLLING",
    "TRANSITING_TO_ROI": "ROI_PATROL",
    "ROI_PATROL": "ROI_PATROL",
}


@dataclass(frozen=True, slots=True)
class OperatingConfigTransition:
    """Everything to swap in, plus how (if at all) to reposition first."""

    store: PatrolRegionStore
    planner: PatrolPlanner
    pursuit_config: PursuitConfig
    roi_config: RoiPatrolConfig
    reposition_waypoints: tuple[Position, ...] = ()
    resume_state: str | None = None  # "PATROLLING" | "ROI_PATROL" | None
    discard_patrol_progress: bool = False


def plan_operating_config_transition(
    *,
    candidate: OperatingConfig,
    old_max_flight_radius_m: float,
    current_flight_state: str,
    position: Position,
    patrol_regions: PatrolRegionStore,
    patrol_planner: PatrolPlanner,
    pursuit_config: PursuitConfig,
    roi_config: RoiPatrolConfig,
    no_fly_router: NoFlyRouter,
    reposition_destination: Position | None = None,
    reposition_resume_state: str | None = None,
    home: Wgs84Point = DEFAULT_HOME,
) -> OperatingConfigTransition:
    """Decide what to swap in for `candidate`, and whether the radius change
    requires flying to a new position before resuming patrol/ROI.

    `home` is the geographic origin the rebuilt PatrolRegionStore's polygons
    are generated around -- callers flying a real aircraft away from
    DEFAULT_HOME (services/gps.py) must pass their session's confirmed home
    here, or the rebuilt regions silently drift back to it on every
    operating-config change.

    While patrolling the open grid, that position is the current local
    (x, y) multiplied by `candidate.max_flight_radius_m /
    old_max_flight_radius_m` -- home is the local origin, and the patrol
    regions themselves are radial bands whose outer edge *is* the flight
    radius, so rescaling keeps the aircraft at the same bearing and the same
    proportion of the way to the (new) boundary. An ROI's center/radius are
    independent of the flight radius -- they don't move or resize when it
    does -- so while ROI patrolling, no repositioning by that ratio happens;
    the aircraft resumes from where it already is, with only the ROI's clip
    against the new home range refreshed.
    """
    priorities = [
        (region.id, region.priority)
        for region in patrol_regions.regions
        if region.priority is not None
    ]
    area_types = [
        (region.id, region.area_type)
        for region in patrol_regions.regions
        if region.area_type is not PatrolAreaType.PATROL
    ]
    store = PatrolRegionStore(home=home, home_radius_m=candidate.max_flight_radius_m)
    store.replace_area_types(area_types)
    store.replace_priorities(priorities)
    planner = PatrolPlanner(
        store=store,
        scorer=patrol_planner.scorer,
        started_at=patrol_planner.started_at,
    )
    new_pursuit_config = replace(
        pursuit_config, geofence_radius_m=candidate.max_flight_radius_m,
    )
    new_roi_config = replace(
        roi_config, max_flight_radius_m=candidate.max_flight_radius_m,
    )
    radius_changed = not math.isclose(
        old_max_flight_radius_m, candidate.max_flight_radius_m, rel_tol=1e-9,
    )
    unchanged = OperatingConfigTransition(
        store=store,
        planner=planner,
        pursuit_config=new_pursuit_config,
        roi_config=new_roi_config,
        # Any patrol coverage already planned -- the live route, a paused
        # checkpoint, or one queued behind a takeoff that has not finished
        # climbing -- was laid out over the old radius' polygons. Flying it
        # now covers the wrong ground, and after a shrink it leaves the
        # aircraft working its way around a perimeter outside the new
        # boundary. This holds in every state, not just the ones that
        # reposition below: request_patrol() plans its whole first coverage
        # route up front, so a patrol ordered seconds before the change is
        # already holding old-radius waypoints while still on the climb.
        discard_patrol_progress=radius_changed,
    )

    if current_flight_state == "REPOSITIONING":
        # A radius change while already flying a reposition leg must retarget
        # that leg now, not wait for it to finish. Waiting let the aircraft
        # continue to a destination chosen for the *previous* radius -- across
        # the new boundary when shrinking -- and only then replan, which also
        # re-scored every region from scratch instead of continuing toward the
        # one it was heading for.
        resume_state = reposition_resume_state
        rescale_from = reposition_destination
    else:
        resume_state = _RESUME_STATE_FOR_CURRENT_STATE.get(current_flight_state)
        rescale_from = position
    if not radius_changed or resume_state is None or rescale_from is None:
        return unchanged

    if resume_state == "PATROLLING":
        ratio = candidate.max_flight_radius_m / old_max_flight_radius_m
        destination = (rescale_from[0] * ratio, rescale_from[1] * ratio)
    else:
        # ROI_PATROL: the ROI itself hasn't moved or resized, so there is no
        # boundary for the aircraft's position to stay proportional to --
        # rescaling it here would fly the aircraft away from the ROI instead
        # of back toward it. A same-position "reposition" (immediately
        # satisfied) still routes through start_reposition()'s resume path,
        # which is what re-clips the ROI against the new home range.
        destination = (position[0], position[1])
    try:
        exit_point = no_fly_router.nearest_exit(
            (position[0], position[1]), store.regions,
        )
        leg = no_fly_router.route(
            exit_point if exit_point is not None else (position[0], position[1]),
            destination,
            store.regions,
        )
    except NoFlyRouteUnavailable:
        # No safe route to the rescaled position exists (e.g. NO_FLY sectors
        # now box it in) -- apply the new config in place rather than leaving
        # the drone stuck trying to get somewhere unreachable.
        return unchanged

    points = [exit_point, *leg] if exit_point is not None else leg
    waypoints = _deduplicate_waypoints(
        [(x, y, candidate.cruise_altitude_m) for x, y in points]
    )
    if not waypoints:
        return unchanged

    return OperatingConfigTransition(
        store=store,
        planner=planner,
        pursuit_config=new_pursuit_config,
        roi_config=new_roi_config,
        reposition_waypoints=tuple(waypoints),
        resume_state=resume_state,
        discard_patrol_progress=True,
    )


def _deduplicate_waypoints(waypoints: list[Position]) -> list[Position]:
    deduplicated: list[Position] = []
    for waypoint in waypoints:
        if not deduplicated or math.dist(deduplicated[-1], waypoint) > 1e-7:
            deduplicated.append(waypoint)
    return deduplicated
