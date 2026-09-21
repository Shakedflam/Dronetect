import math
import random

import pytest

from controllers.autonomous_drone.roi_patrol import RoiPatrolConfig
from controllers.autonomous_drone.flight_logic import (
    advance_kinematic_flight,
    boost_forward_pitch,
    FlightConfig,
    FlightLogic,
    FlightState,
    KinematicFlightConfig,
    navigation_disturbances,
    ramp_command,
)


HOME = (0.0, 0.0, 0.1)


def make_logic(**config_overrides):
    config = FlightConfig(**config_overrides)
    logic = FlightLogic(config=config, rng=random.Random(7))
    logic.initialize_home(HOME)
    return logic


def finish_takeoff(logic):
    logic.request_mission_takeoff(HOME)
    logic.update((HOME[0], HOME[1], logic.config.takeoff_altitude), 0.1)


def test_initializes_idle_at_captured_home():
    logic = make_logic()

    assert logic.state is FlightState.IDLE
    assert logic.home == HOME
    assert logic.target == HOME


def test_stays_idle_until_a_mission_is_requested():
    logic = make_logic(idle_duration=1.0)

    logic.update(HOME, 60.0)
    assert logic.state is FlightState.IDLE
    assert logic.target == HOME


def test_startup_updates_never_cause_takeoff_or_horizontal_movement():
    logic = make_logic(takeoff_altitude=12.0)

    for _ in range(100):
        logic.update(HOME, 1.0)

    assert logic.state is FlightState.IDLE
    assert logic.target == HOME


def test_mission_takeoff_instruction_starts_takeoff_from_idle():
    logic = make_logic()

    logic.request_mission_takeoff(HOME)

    assert logic.state is FlightState.TAKING_OFF
    assert logic.target == (HOME[0], HOME[1], logic.config.takeoff_altitude)


def test_reaching_takeoff_altitude_holds():
    logic = make_logic()

    finish_takeoff(logic)

    assert logic.state is FlightState.HOLDING
    assert logic.target is not None


def test_patrol_command_takes_off_to_configured_altitude_then_holds():
    logic = make_logic(takeoff_altitude=12.0)

    logic.request_mission_takeoff(HOME)
    assert logic.state is FlightState.TAKING_OFF
    assert logic.target == (HOME[0], HOME[1], 12.0)

    logic.update((HOME[0], HOME[1], 12.0), 0.1)
    assert logic.state is FlightState.HOLDING
    assert logic.target == (HOME[0], HOME[1], 12.0)

    logic.update((HOME[0], HOME[1], 12.0), 10.0)
    assert logic.state is FlightState.HOLDING
    assert logic.target == (HOME[0], HOME[1], 12.0)


def test_normal_patrol_takes_off_and_continuously_requests_waypoints():
    waypoints = iter(((20.0, 0.0, 12.0), (0.0, 20.0, 12.0)))
    calls = []

    def next_waypoint(position, completed):
        calls.append((position, completed))
        return next(waypoints)

    logic = FlightLogic(
        config=FlightConfig(takeoff_altitude=12.0),
        patrol_waypoint_provider=next_waypoint,
    )
    logic.initialize_home(HOME)

    first = logic.request_patrol(HOME)
    logic.update((HOME[0], HOME[1], 12.0), 0.1)
    assert logic.state is FlightState.PATROLLING
    assert logic.target == first

    logic.update(first, 0.1)

    assert logic.state is FlightState.PATROLLING
    assert logic.target == (0.0, 20.0, 12.0)
    assert calls == [(HOME, False), (first, True)]


def test_takeoff_completion_accepts_altitude_above_cruise_target():
    logic = make_logic(takeoff_altitude=12.0, vertical_tolerance=0.25)
    logic.request_mission_takeoff(HOME)

    logic.update((HOME[0], HOME[1], 15.0), 0.1)

    assert logic.state is FlightState.HOLDING


def test_queued_roi_mission_refreshes_stale_geometry_when_marked_stale():
    logic = make_logic(takeoff_altitude=12.0)
    logic.request_roi(HOME, center=(100.0, 0.0), radius_m=20.0)
    assert logic.state is FlightState.TAKING_OFF
    assert logic._queued_waypoint == (100.0, 0.0, 12.0)  # computed under the old altitude

    # Simulate an operating-config change arriving mid-climb: the config
    # itself is swapped by the caller (autonomous_drone.py), which then
    # flags the queued mission stale.
    logic.config = FlightConfig(takeoff_altitude=18.0)
    logic.mark_queued_mission_config_stale()

    logic.update((HOME[0], HOME[1], 18.0), 0.1)

    assert logic.state is FlightState.TRANSITING_TO_ROI
    assert logic.target == (100.0, 0.0, 18.0)  # fresh, not the stale 12.0 waypoint


def test_queued_patrol_mission_is_not_recomputed_when_not_marked_stale():
    calls = []

    def next_waypoint(position, completed):
        calls.append((position, completed))
        return (20.0, 0.0, 12.0)

    logic = FlightLogic(
        config=FlightConfig(takeoff_altitude=12.0),
        patrol_waypoint_provider=next_waypoint,
    )
    logic.initialize_home(HOME)
    first = logic.request_patrol(HOME)
    assert calls == [(HOME, False)]

    logic.update((HOME[0], HOME[1], 12.0), 0.1)

    assert logic.state is FlightState.PATROLLING
    assert logic.target == first
    # No config change happened during the climb -- the provider must not
    # be called a second time just because takeoff completed.
    assert calls == [(HOME, False)]


def test_reposition_flies_to_target_then_resumes_patrolling():
    calls = []

    def next_waypoint(position, completed):
        calls.append((position, completed))
        return (5.0, 0.0, 12.0)

    logic = FlightLogic(
        config=FlightConfig(takeoff_altitude=12.0),
        patrol_waypoint_provider=next_waypoint,
    )
    logic.initialize_home(HOME)
    reposition_target = (50.0, 0.0, 12.0)

    logic.start_reposition([reposition_target], FlightState.PATROLLING)

    assert logic.state is FlightState.REPOSITIONING
    assert logic.target == reposition_target

    logic.update(reposition_target, 0.1)

    assert logic.state is FlightState.PATROLLING
    assert logic.target == (5.0, 0.0, 12.0)
    assert calls == [(reposition_target, False)]


def test_reposition_with_multiple_legs_visits_each_in_order():
    logic = FlightLogic(
        config=FlightConfig(takeoff_altitude=12.0),
        patrol_waypoint_provider=lambda position, completed: (0.0, 0.0, 12.0),
    )
    logic.initialize_home(HOME)
    legs = [(10.0, 0.0, 12.0), (10.0, 10.0, 12.0), (0.0, 10.0, 12.0)]

    logic.start_reposition(legs, FlightState.PATROLLING)
    assert logic.target == legs[0]

    logic.update(legs[0], 0.1)
    assert logic.state is FlightState.REPOSITIONING
    assert logic.target == legs[1]

    logic.update(legs[1], 0.1)
    assert logic.state is FlightState.REPOSITIONING
    assert logic.target == legs[2]

    logic.update(legs[2], 0.1)
    assert logic.state is FlightState.PATROLLING


class _SpyRoiPatrolPlanner:
    """Minimal RoiPatrolPlan double that records reset() calls."""

    def __init__(self, first_waypoint):
        self._first_waypoint = first_waypoint
        self.reset_calls = []
        self.config = RoiPatrolConfig()

    def reset(self, center, radius_m, altitude_m, current_position):
        self.reset_calls.append((center, radius_m, altitude_m, current_position))
        return self._first_waypoint

    def next_waypoint(self):
        return self._first_waypoint

    def entry_target(self, center, altitude_m):
        return self._first_waypoint

    def contains(self, position):
        return True

    def closest_waypoint(self, position):
        return self._first_waypoint

    def retarget_altitude(self, new_altitude_m):
        pass

    def clear(self):
        pass

    def snapshot(self):
        raise NotImplementedError

    def restore(self, progress):
        raise NotImplementedError


def test_reposition_resumes_roi_patrol_with_fresh_clip():
    spy = _SpyRoiPatrolPlanner(first_waypoint=(12.0, 0.0, 12.0))
    logic = FlightLogic(
        config=FlightConfig(takeoff_altitude=12.0),
        roi_patrol_planner=spy,
    )
    logic.initialize_home(HOME)
    logic.request_roi(HOME, center=(10.0, 0.0), radius_m=20.0)
    spy.reset_calls.clear()
    reposition_target = (5.0, 0.0, 12.0)

    logic.start_reposition([reposition_target], FlightState.ROI_PATROL)
    logic.update(reposition_target, 0.1)

    assert logic.state is FlightState.ROI_PATROL
    assert logic.target == (12.0, 0.0, 12.0)
    assert spy.reset_calls == [((10.0, 0.0), 20.0, 12.0, reposition_target)]


def test_reposition_resume_falls_back_to_patrolling_when_roi_no_longer_overlaps():
    calls = []

    def next_waypoint(position, completed):
        calls.append((position, completed))
        return (5.0, 0.0, 12.0)

    logic = FlightLogic(
        config=FlightConfig(takeoff_altitude=12.0),
        patrol_waypoint_provider=next_waypoint,
    )
    logic.initialize_home(HOME)
    logic.request_roi(HOME, center=(280.0, 0.0), radius_m=10.0)
    # Simulate a radius shrink applied while the ROI was set aside for the
    # reposition leg: once the drone arrives, resuming ROI patrol under this
    # much smaller flight radius would find zero overlap.
    logic.roi_patrol_planner.config = type(logic.roi_patrol_planner.config)(
        max_flight_radius_m=50.0
    )
    reposition_target = (15.0, 0.0, 12.0)

    logic.start_reposition([reposition_target], FlightState.ROI_PATROL)
    logic.update(reposition_target, 0.1)

    assert logic.state is FlightState.PATROLLING
    assert logic.target == (5.0, 0.0, 12.0)
    assert calls == [(reposition_target, False)]


def test_clear_roi_cancels_an_in_progress_roi_reposition():
    logic = make_logic(takeoff_altitude=12.0)
    logic.request_roi(HOME, center=(10.0, 0.0), radius_m=20.0)
    reposition_target = (5.0, 0.0, 12.0)
    logic.start_reposition([reposition_target], FlightState.ROI_PATROL)
    current = (2.0, 0.0, 12.0)

    logic.clear_roi(current)

    assert logic.state is FlightState.HOLDING
    assert logic.target == current


def test_start_reposition_rejects_empty_waypoints_and_bad_resume_state():
    logic = make_logic(takeoff_altitude=12.0)

    with pytest.raises(ValueError, match="at least one waypoint"):
        logic.start_reposition([], FlightState.PATROLLING)
    with pytest.raises(ValueError, match="PATROLLING or ROI_PATROL"):
        logic.start_reposition([(1.0, 0.0, 12.0)], FlightState.HOLDING)


@pytest.mark.parametrize(
    "state,expected",
    [
        (FlightState.TARGET_ACQUISITION, True),
        (FlightState.TARGET_FOLLOW, True),
        (FlightState.TARGET_PURSUIT, True),
        (FlightState.PATROLLING, False),
        (FlightState.REPOSITIONING, False),
        (FlightState.HOLDING, False),
    ],
)
def test_is_target_tracking_property(state, expected):
    logic = make_logic()
    logic.state = state

    assert logic.is_target_tracking is expected


def test_manual_return_home_can_be_replaced_by_patrol_while_airborne():
    logic = FlightLogic(
        config=FlightConfig(takeoff_altitude=12.0),
        patrol_waypoint_provider=lambda position, completed: (20.0, 0.0, 12.0),
    )
    logic.initialize_home(HOME)
    airborne = (4.0, 2.0, 12.0)
    logic.request_patrol(airborne)
    assert logic.request_return_home() is True

    waypoint = logic.request_patrol(airborne)

    assert logic.state is FlightState.PATROLLING
    assert logic.target == waypoint == (20.0, 0.0, 12.0)


def test_manual_return_home_can_be_replaced_by_roi_while_airborne():
    logic = make_logic(takeoff_altitude=12.0)
    airborne = (4.0, 2.0, 12.0)
    logic.request_mission_takeoff(airborne)
    assert logic.request_return_home() is True

    logic.request_roi(airborne, center=(100.0, 50.0), radius_m=20.0)

    assert logic.state is FlightState.TRANSITING_TO_ROI
    assert logic.target == (100.0, 50.0, 12.0)

def test_mission_command_while_airborne_skips_takeoff_and_holds_position():
    logic = make_logic(takeoff_altitude=12.0)
    airborne = (3.0, -2.0, 12.0)

    logic.request_mission_takeoff(airborne)

    assert logic.state is FlightState.HOLDING
    assert logic.target == airborne


def test_roi_from_ground_takes_off_then_transits_toward_center():
    logic = make_logic(takeoff_altitude=12.0, roi_arrival_tolerance=5.0)

    logic.request_roi(HOME, center=(100.0, 0.0), radius_m=20.0)
    assert logic.state is FlightState.TAKING_OFF

    airborne = (HOME[0], HOME[1], 12.0)
    logic.update(airborne, 0.1)

    assert logic.state is FlightState.TRANSITING_TO_ROI
    assert logic.target == (100.0, 0.0, 12.0)


def test_airborne_roi_command_skips_takeoff_and_starts_patrol_inside_boundary():
    logic = make_logic(takeoff_altitude=12.0, roi_arrival_tolerance=5.0)
    airborne = (0.0, 0.0, 12.0)

    logic.request_roi(airborne, center=(100.0, 0.0), radius_m=20.0)
    assert logic.state is FlightState.TRANSITING_TO_ROI

    outside = (74.9, 0.0, 12.0)
    logic.update(outside, 0.1)
    assert logic.state is FlightState.TRANSITING_TO_ROI

    tolerance_halo = (75.0, 0.0, 12.0)
    logic.update(tolerance_halo, 0.1)
    assert logic.state is FlightState.TRANSITING_TO_ROI

    boundary = (80.0, 0.0, 12.0)
    logic.update(boundary, 0.1)
    assert logic.state is FlightState.ROI_PATROL
    assert logic.target is not None
    assert math.hypot(logic.target[0] - 100.0, logic.target[1]) <= 20.0


def test_airborne_roi_command_already_inside_transitions_through_transit():
    logic = make_logic(takeoff_altitude=12.0, roi_arrival_tolerance=5.0)
    airborne = (90.0, 0.0, 12.0)

    logic.request_roi(airborne, center=(100.0, 0.0), radius_m=20.0)

    assert logic.state is FlightState.TRANSITING_TO_ROI
    assert logic.target == (100.0, 0.0, 12.0)

    logic.update(airborne, 0.1)

    assert logic.state is FlightState.ROI_PATROL


def test_roi_patrol_continuously_selects_another_in_bounds_waypoint():
    logic = make_logic(takeoff_altitude=12.0)
    airborne = (100.0, 0.0, 12.0)
    logic.request_roi(airborne, center=(100.0, 0.0), radius_m=20.0)
    logic.update(airborne, 0.1)
    first_target = logic.target

    logic.update(first_target, 0.1)

    assert logic.state is FlightState.ROI_PATROL
    assert logic.target != first_target
    assert math.hypot(logic.target[0] - 100.0, logic.target[1]) <= 20.0


def test_new_roi_during_roi_patrol_returns_through_transit():
    logic = make_logic(takeoff_altitude=12.0)
    airborne = (10.0, 0.0, 12.0)
    logic.request_roi(airborne, center=(10.0, 0.0), radius_m=20.0)
    logic.update(airborne, 0.1)
    assert logic.state is FlightState.ROI_PATROL

    logic.request_roi(airborne, center=(200.0, 0.0), radius_m=30.0)

    assert logic.state is FlightState.TRANSITING_TO_ROI
    assert logic.target == (200.0, 0.0, 12.0)


def test_new_roi_during_roi_transit_immediately_replaces_destination():
    logic = make_logic(takeoff_altitude=12.0)
    airborne = (0.0, 0.0, 12.0)
    logic.request_roi(airborne, center=(100.0, 0.0), radius_m=20.0)

    logic.request_roi((25.0, 0.0, 12.0), center=(200.0, 50.0), radius_m=30.0)

    assert logic.state is FlightState.TRANSITING_TO_ROI
    assert logic.target == (200.0, 50.0, 12.0)


def test_start_patrol_interrupts_roi_patrol_and_holds():
    logic = make_logic(takeoff_altitude=12.0)
    airborne = (10.0, 0.0, 12.0)
    logic.request_roi(airborne, center=(10.0, 0.0), radius_m=20.0)
    logic.update(airborne, 0.1)

    logic.request_mission_takeoff(airborne)

    assert logic.state is FlightState.HOLDING
    assert logic.target == airborne


def test_roi_patrol_recovers_toward_center_after_boundary_excursion():
    logic = make_logic(takeoff_altitude=12.0)
    center = (100.0, 0.0)
    logic.request_roi((100.0, 0.0, 12.0), center=center, radius_m=20.0)
    logic.update((100.0, 0.0, 12.0), 0.1)

    logic.update((121.0, 0.0, 12.0), 0.1)

    assert logic.state is FlightState.ROI_PATROL
    assert logic.target == (100.0, 0.0, 12.0)


def test_return_home_interrupts_roi_patrol_for_rth_condition():
    logic = make_logic(takeoff_altitude=12.0)
    airborne = (10.0, 0.0, 12.0)
    logic.request_roi(airborne, center=(10.0, 0.0), radius_m=20.0)
    logic.update(airborne, 0.1)

    assert logic.request_return_home() is True
    assert logic.state is FlightState.RETURNING_HOME
    assert logic.target == (HOME[0], HOME[1], 12.0)


def test_clear_roi_on_ground_does_not_start_takeoff():
    logic = make_logic(takeoff_altitude=12.0)

    logic.clear_roi(HOME)

    assert logic.state is FlightState.IDLE
    assert logic.target == HOME


def test_clear_roi_during_transit_holds_current_position():
    logic = make_logic(takeoff_altitude=12.0)
    airborne = (0.0, 0.0, 12.0)
    logic.request_roi(airborne, center=(100.0, 0.0), radius_m=20.0)
    current = (25.0, 0.0, 12.0)

    logic.clear_roi(current)

    assert logic.state is FlightState.HOLDING
    assert logic.target == current


def test_clear_roi_during_roi_patrol_holds_current_position():
    logic = make_logic(takeoff_altitude=12.0)
    current = (10.0, 0.0, 12.0)
    logic.request_roi(current, center=(10.0, 0.0), radius_m=20.0)
    logic.update(current, 0.1)

    logic.clear_roi(current)

    assert logic.state is FlightState.HOLDING
    assert logic.target == current


def test_clear_roi_during_takeoff_keeps_climb_but_cancels_transit():
    logic = make_logic(takeoff_altitude=12.0)
    logic.request_roi(HOME, center=(100.0, 0.0), radius_m=20.0)

    logic.clear_roi(HOME)
    logic.update((HOME[0], HOME[1], 12.0), 0.1)

    assert logic.state is FlightState.HOLDING
    assert logic.target == (HOME[0], HOME[1], 12.0)


def test_random_waypoints_stay_inside_all_configured_bounds():
    logic = make_logic()

    for _ in range(50):
        x, y, z = logic.choose_random_waypoint()
        assert logic.config.x_bounds[0] <= x <= logic.config.x_bounds[1]
        assert logic.config.y_bounds[0] <= y <= logic.config.y_bounds[1]
        assert logic.config.z_bounds[0] <= z <= logic.config.z_bounds[1]


@pytest.mark.parametrize(
    "starting_state",
    [
        FlightState.TAKING_OFF,
        FlightState.PATROLLING,
        FlightState.REPOSITIONING,
    ],
)
def test_return_home_replaces_active_target(starting_state):
    logic = make_logic()
    logic.state = starting_state
    logic.target = (6.0, -4.0, 4.0)

    accepted = logic.request_return_home()

    assert accepted is True
    assert logic.state is FlightState.RETURNING_HOME
    assert logic.target == (HOME[0], HOME[1], logic.config.takeoff_altitude)


def test_returning_home_never_resumes_the_previous_mission():
    logic = make_logic()
    finish_takeoff(logic)
    logic.request_return_home()

    logic.update((HOME[0], HOME[1], logic.config.takeoff_altitude), 0.1)

    assert logic.state is FlightState.LANDING
    assert logic.target[0:2] == HOME[0:2]


def test_landing_reduces_target_altitude_over_time():
    logic = make_logic(landing_rate=0.5)
    finish_takeoff(logic)
    logic.request_return_home()
    logic.update((HOME[0], HOME[1], logic.config.takeoff_altitude), 0.1)
    initial_target_altitude = logic.target[2]

    logic.update((HOME[0], HOME[1], 2.0), 1.0)

    assert logic.state is FlightState.LANDING
    assert logic.target[2] == pytest.approx(initial_target_altitude - 0.5)


def test_ground_level_arrival_lands_and_prevents_further_commands():
    logic = make_logic()
    finish_takeoff(logic)
    logic.request_return_home()
    logic.update((HOME[0], HOME[1], logic.config.takeoff_altitude), 0.1)

    logic.update((HOME[0], HOME[1], HOME[2] + 0.05), 0.1)

    assert logic.state is FlightState.LANDED
    assert logic.target == HOME
    assert logic.request_return_home() is False


def test_navigation_moves_forward_when_facing_a_distant_target():
    yaw_command, pitch_command = navigation_disturbances(
        position=(0.0, 0.0, 2.5),
        target=(5.0, 0.0, 2.5),
        yaw=0.0,
        horizontal_tolerance=0.5,
    )

    assert yaw_command == pytest.approx(0.0)
    assert pitch_command == pytest.approx(-1.0)


def test_boost_forward_pitch_uses_higher_forward_speed_gain():
    normal_pitch = -1.0
    faster_pitch = boost_forward_pitch(normal_pitch, forward_speed_gain=3.2)

    assert faster_pitch < normal_pitch
    assert faster_pitch == pytest.approx(-3.2)


def test_ramp_command_limits_single_step_change():
    assert ramp_command(0.0, -6.0, 0.5) == pytest.approx(-0.5)
    assert ramp_command(-5.8, -6.0, 0.5) == pytest.approx(-6.0)
    assert ramp_command(-1.0, 1.0, 0.25) == pytest.approx(-0.75)


def test_kinematic_flight_accelerates_toward_fast_horizontal_speed():
    step = advance_kinematic_flight(
        position=(0.0, 0.0, 15.0),
        target=(100.0, 0.0, 15.0),
        yaw=0.0,
        horizontal_speed_mps=0.0,
        dt=1.0,
    )

    assert step.position == pytest.approx((6.0, 0.0, 15.0))
    assert step.horizontal_speed_mps == pytest.approx(6.0)
    assert step.speed_mps == pytest.approx(6.0)


def test_kinematic_flight_caps_horizontal_speed():
    step = advance_kinematic_flight(
        position=(0.0, 0.0, 15.0),
        target=(100.0, 0.0, 15.0),
        yaw=0.0,
        horizontal_speed_mps=14.0,
        dt=1.0,
    )

    assert step.position == pytest.approx((14.0, 0.0, 15.0))
    assert step.horizontal_speed_mps == pytest.approx(14.0)


def test_kinematic_flight_uses_dji_like_ascent_speed():
    step = advance_kinematic_flight(
        position=(0.0, 0.0, 0.0),
        target=(0.0, 0.0, 15.0),
        yaw=1.25,
        horizontal_speed_mps=0.0,
        dt=1.0,
    )

    assert step.position == pytest.approx((0.0, 0.0, 5.0))
    assert step.yaw == pytest.approx(1.25)
    assert step.speed_mps == pytest.approx(5.0)
    assert step.vertical_speed_mps == pytest.approx(5.0)


def test_kinematic_flight_holds_yaw_for_tiny_horizontal_error():
    step = advance_kinematic_flight(
        position=(0.0, 0.0, 1.0),
        target=(0.1, -0.1, 10.0),
        yaw=1.25,
        horizontal_speed_mps=0.0,
        dt=1.0,
    )

    assert step.position == pytest.approx((0.0, 0.0, 6.0))
    assert step.yaw == pytest.approx(1.25)
    assert step.horizontal_speed_mps == pytest.approx(0.0)


def test_kinematic_flight_holds_yaw_within_a_few_centimeters_even_without_position_deadzone():
    # TARGET_RETURN uses yaw_hold_horizontal_distance_m=0.0 so position can
    # converge exactly. At sub-centimeter range atan2(dy, dx) is dominated by
    # floating-point noise and can swing by ~180 degrees between ticks; yaw
    # must still hold steady there instead of oscillating indefinitely.
    config = KinematicFlightConfig(yaw_hold_horizontal_distance_m=0.0, max_yaw_rate_rad_s=math.radians(150.0))

    step = advance_kinematic_flight(
        position=(0.0, 0.0, 10.0),
        target=(-1e-4, 1e-5, 10.0),
        yaw=0.0,
        horizontal_speed_mps=0.0,
        dt=0.03,
        config=config,
    )

    assert abs(step.yaw) < math.radians(1.0)


def test_kinematic_flight_turns_with_configured_yaw_rate():
    config = KinematicFlightConfig(max_yaw_rate_rad_s=1.0)

    step = advance_kinematic_flight(
        position=(0.0, 0.0, 15.0),
        target=(0.0, 100.0, 15.0),
        yaw=0.0,
        horizontal_speed_mps=0.0,
        dt=0.25,
        config=config,
    )

    assert step.yaw == pytest.approx(0.25)


def test_navigation_turns_toward_a_target_to_the_left():
    yaw_command, pitch_command = navigation_disturbances(
        position=(0.0, 0.0, 2.5),
        target=(0.0, 5.0, 2.5),
        yaw=0.0,
        horizontal_tolerance=0.5,
    )

    assert yaw_command > 0.0
    assert pitch_command > -1.0


def test_navigation_yaw_gain_makes_turns_more_decisive():
    gentle_yaw, _ = navigation_disturbances(
        position=(0.0, 0.0, 2.5),
        target=(0.0, 5.0, 2.5),
        yaw=0.0,
        horizontal_tolerance=0.5,
        yaw_gain=0.4,
    )
    stronger_yaw, _ = navigation_disturbances(
        position=(0.0, 0.0, 2.5),
        target=(0.0, 5.0, 2.5),
        yaw=0.0,
        horizontal_tolerance=0.5,
        yaw_gain=1.2,
    )

    assert stronger_yaw == pytest.approx(gentle_yaw * 3.0)


def test_navigation_stops_forward_motion_inside_target_tolerance():
    yaw_command, pitch_command = navigation_disturbances(
        position=(0.0, 0.0, 2.5),
        target=(0.2, 0.2, 2.5),
        yaw=1.0,
        horizontal_tolerance=0.5,
    )

    assert yaw_command == 0.0
    assert pitch_command == 0.0


