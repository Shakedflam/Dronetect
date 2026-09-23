from unittest.mock import Mock

import pytest

from config.shared_config import OperatingConfig
from controllers.autonomous_drone.pursuit import PursuitConfig
from controllers.autonomous_drone.roi_patrol import RoiPatrolConfig
from services.gps import DEFAULT_HOME, Wgs84Point, wgs84_distance_m
from services.no_fly_router import NoFlyRouteUnavailable, NoFlyRouter
from services.operating_config_apply import plan_operating_config_transition
from services.patrol_planner import PatrolPlanner
from services.patrol_regions import PatrolRegionStore


POSITION = (100.0, 0.0, 12.0)


def _plan(
    *,
    current_flight_state,
    old_radius=300.0,
    new_radius=300.0,
    new_altitude=12.0,
    position=POSITION,
    no_fly_router=None,
    reposition_destination=None,
    reposition_resume_state=None,
    home=DEFAULT_HOME,
):
    patrol_regions = PatrolRegionStore(home=home, home_radius_m=old_radius)
    return plan_operating_config_transition(
        reposition_destination=reposition_destination,
        reposition_resume_state=reposition_resume_state,
        candidate=OperatingConfig(
            max_flight_radius_m=new_radius, cruise_altitude_m=new_altitude,
        ),
        old_max_flight_radius_m=old_radius,
        current_flight_state=current_flight_state,
        position=position,
        patrol_regions=patrol_regions,
        patrol_planner=PatrolPlanner(store=patrol_regions),
        pursuit_config=PursuitConfig(geofence_radius_m=old_radius),
        roi_config=RoiPatrolConfig(max_flight_radius_m=old_radius),
        no_fly_router=no_fly_router or NoFlyRouter(),
        home=home,
    )


def test_home_defaults_to_default_home_unchanged_for_simulator_callers():
    """autonomous_drone.py (the simulator caller) never passes `home` --
    confirms the new parameter is purely additive and simulator geometry is
    unaffected."""
    transition = _plan(current_flight_state="PATROLLING", old_radius=300.0, new_radius=150.0)
    assert transition.store.regions[0].center == PatrolRegionStore(home_radius_m=150.0).regions[0].center


def test_explicit_home_rehomes_the_rebuilt_store():
    """A real-DJI caller (adapters/dji/flight_bridge.py) passes its session's
    confirmed home; the rebuilt store's region geometry must be generated
    around *that* home, not silently fall back to DEFAULT_HOME."""
    far_away_home = Wgs84Point(lat=35.6762, lng=139.6503)  # Tokyo
    transition = _plan(current_flight_state="PATROLLING", home=far_away_home)
    region = transition.store.regions[0]
    assert wgs84_distance_m(region.center, far_away_home) < wgs84_distance_m(region.center, DEFAULT_HOME)


def test_radius_change_while_patrolling_triggers_reposition_and_discards_progress():
    transition = _plan(current_flight_state="PATROLLING", old_radius=300.0, new_radius=150.0)

    assert transition.resume_state == "PATROLLING"
    assert transition.discard_patrol_progress is True
    assert transition.reposition_waypoints
    ratio = 150.0 / 300.0
    expected = (POSITION[0] * ratio, POSITION[1] * ratio, 12.0)
    assert transition.reposition_waypoints[-1] == pytest.approx(expected)
    assert transition.pursuit_config.geofence_radius_m == 150.0
    assert transition.roi_config.max_flight_radius_m == 150.0
    assert max(region.outer_radius_m for region in transition.store.regions) == 150.0


@pytest.mark.parametrize("current_flight_state", ["TRANSITING_TO_ROI", "ROI_PATROL"])
def test_radius_change_while_roi_active_triggers_reposition_toward_roi_resume(current_flight_state):
    transition = _plan(current_flight_state=current_flight_state, old_radius=300.0, new_radius=150.0)

    assert transition.resume_state == "ROI_PATROL"
    # The ROI itself is unaffected by the radius, but a patrol checkpoint
    # paused when the ROI began was planned over the old polygons and must
    # not be resumed once the ROI is cleared.
    assert transition.discard_patrol_progress is True
    assert transition.reposition_waypoints


def test_radius_change_mid_reposition_retargets_that_leg_proportionally():
    """A second radius change while a reposition leg is still flying must
    retarget it now. It rescales the leg's *destination*, not the aircraft's
    current position: mid-leg the aircraft sits at no meaningful proportion
    of the boundary, so rescaling where it happens to be would compound.
    """
    destination = (240.0, 0.0, 12.0)

    transition = _plan(
        current_flight_state="REPOSITIONING",
        old_radius=300.0,
        new_radius=150.0,
        reposition_destination=destination,
        reposition_resume_state="PATROLLING",
    )

    assert transition.resume_state == "PATROLLING"
    assert transition.reposition_waypoints[-1] == pytest.approx((120.0, 0.0, 12.0))


def test_radius_change_mid_reposition_keeps_the_leg_s_own_resume_mission():
    transition = _plan(
        current_flight_state="REPOSITIONING",
        old_radius=300.0,
        new_radius=150.0,
        reposition_destination=(240.0, 0.0, 12.0),
        reposition_resume_state="ROI_PATROL",
    )

    assert transition.resume_state == "ROI_PATROL"


@pytest.mark.parametrize("current_flight_state", ["TRANSITING_TO_ROI", "ROI_PATROL"])
def test_radius_change_while_roi_active_does_not_rescale_position(current_flight_state):
    # An ROI's center/radius don't move or resize when the flight radius
    # does, so the aircraft must resume from where it already is -- not from
    # POSITION scaled by the old/new radius ratio, which would fly it away
    # from the (unchanged) ROI. Regression check for that exact bug.
    transition = _plan(current_flight_state=current_flight_state, old_radius=300.0, new_radius=3009.0)

    assert transition.resume_state == "ROI_PATROL"
    assert transition.reposition_waypoints[-1] == pytest.approx((POSITION[0], POSITION[1], 12.0))


def test_altitude_only_change_does_not_trigger_reposition():
    transition = _plan(
        current_flight_state="PATROLLING",
        old_radius=300.0,
        new_radius=300.0,
        new_altitude=20.0,
    )

    assert transition.resume_state is None
    assert transition.reposition_waypoints == ()


@pytest.mark.parametrize("current_flight_state", ["HOLDING", "LANDED", "IDLE"])
def test_non_coverage_states_apply_immediately_without_reposition(current_flight_state):
    transition = _plan(current_flight_state=current_flight_state, old_radius=300.0, new_radius=150.0)

    assert transition.resume_state is None
    assert transition.reposition_waypoints == ()
    # The rebuilt geometry is still there for whenever patrol/ROI is next requested.
    assert max(region.outer_radius_m for region in transition.store.regions) == 150.0


@pytest.mark.parametrize(
    "current_flight_state", ["TAKING_OFF", "HOLDING", "TARGET_PURSUIT", "LANDED"],
)
def test_radius_change_discards_old_geometry_coverage_in_every_state(current_flight_state):
    """A patrol ordered just before the change already holds a fully planned
    coverage route -- request_patrol() plans it up front, before the climb
    even starts. States that don't reposition must still drop it, or the
    aircraft flies the old radius' perimeter once airborne."""
    transition = _plan(current_flight_state=current_flight_state, old_radius=300.0, new_radius=150.0)

    assert transition.discard_patrol_progress is True


@pytest.mark.parametrize("current_flight_state", ["PATROLLING", "TAKING_OFF", "HOLDING"])
def test_altitude_only_change_keeps_patrol_progress(current_flight_state):
    transition = _plan(
        current_flight_state=current_flight_state,
        old_radius=300.0,
        new_radius=300.0,
        new_altitude=20.0,
    )

    assert transition.discard_patrol_progress is False


def test_unreachable_reposition_falls_back_to_no_reposition():
    unreachable_router = Mock(spec=NoFlyRouter)
    unreachable_router.nearest_exit.side_effect = NoFlyRouteUnavailable("boxed in")

    transition = _plan(
        current_flight_state="PATROLLING",
        old_radius=300.0,
        new_radius=150.0,
        no_fly_router=unreachable_router,
    )

    assert transition.resume_state is None
    assert transition.reposition_waypoints == ()
    assert transition.pursuit_config.geofence_radius_m == 150.0
    # Still stale under the new radius even though no reposition is possible.
    assert transition.discard_patrol_progress is True


def test_existing_priorities_and_area_types_are_replayed_onto_new_store():
    old_store = PatrolRegionStore(home_radius_m=300.0)
    region_id = old_store.regions[0].id
    old_store.set_priority(region_id, 1)
    old_store.set_area_type(old_store.regions[1].id, "NO_FLY")

    transition = plan_operating_config_transition(
        candidate=OperatingConfig(max_flight_radius_m=150.0, cruise_altitude_m=12.0),
        old_max_flight_radius_m=300.0,
        current_flight_state="HOLDING",
        position=POSITION,
        patrol_regions=old_store,
        patrol_planner=PatrolPlanner(store=old_store),
        pursuit_config=PursuitConfig(geofence_radius_m=300.0),
        roi_config=RoiPatrolConfig(max_flight_radius_m=300.0),
        no_fly_router=NoFlyRouter(),
    )

    assert transition.store.get(region_id).priority == 1
    assert transition.store.get(old_store.regions[1].id).area_type.value == "NO_FLY"
