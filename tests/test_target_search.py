"""Getting onto the target's six, and what the aircraft does when it loses it."""

import math

import pytest

from controllers.autonomous_drone.flight_logic import (
    FlightConfig,
    FlightLogic,
    FlightState,
)
from controllers.autonomous_drone.pursuit import (
    PursuitConfig,
    PursuitController,
    TargetState,
)


HOME = (0.0, 0.0, 0.0)
POSITION = (10.0, 20.0, 15.0)
WAYPOINT = (100.0, 50.0, 15.0)


def tracking_logic():
    logic = FlightLogic(
        FlightConfig(
            target_observation_s=0.0,
            takeoff_altitude=15,
            target_standoff_box_area_ratio=0.01,
            target_standoff_deadband_fraction=0.1,
            target_reference_distance_m=2.0,
            target_reference_area_ratio=0.01,
        ),
        pursuit_config=PursuitConfig(stable_duration_s=0.3),
        patrol_waypoint_provider=lambda position, completed: WAYPOINT,
    )
    logic.initialize_home(HOME)
    logic.request_patrol(POSITION)
    return logic


def see_target(logic, position):
    size = 1000 * math.sqrt(logic._standoff_area_ratio())
    logic.observe_detections(
        [{
            "trackId": "one", "trackState": "confirmed", "confidence": 0.9,
            "box": [500 - size / 2, 500 - size / 2, 500 + size / 2, 500 + size / 2],
            "velocityPxPerSec": {"x": 0, "y": 0},
        }],
        frame_width=1000, frame_height=1000, position=position, yaw=0.0,
    )


@pytest.fixture
def reacquire():
    """Put a confirmed target back in frame at the aircraft's current pose."""
    def _reacquire(logic, position):
        for _ in range(6):
            see_target(logic, position)
            logic.update(position, 0.1)
    return _reacquire


@pytest.fixture
def lost_logic(reacquire):
    """A pursuit that has just lost sight of its target."""
    logic = tracking_logic()
    reacquire(logic, POSITION)
    assert logic.state is FlightState.TARGET_PURSUIT
    logic.observe_detections(
        [], frame_width=1000, frame_height=1000, position=POSITION, yaw=0.0
    )
    return logic, POSITION


def station_keeping_config(**overrides):
    overrides.setdefault("max_speed_mps", 10.0)
    return PursuitConfig(
        acceleration_mps2=5.0,
        smoothing_time_s=0.2,
        standoff_distance_m=5.0,
        altitude_offset_m=3.0,
        **overrides,
    )


def fly_to_station(config, *, position, target_state, ticks=40, dt=0.1):
    """Run the stern chase and report where the aircraft ends up."""
    pursuit = PursuitController(config)
    here = position
    for _ in range(ticks):
        velocity = pursuit.update(
            position=here,
            home=HOME,
            yaw=0.0,
            image_velocity=(0.0, 0.0),
            area=0.01,
            growth=0.0,
            desired_area=0.01,
            area_deadzone=0.001,
            collision_side=0.0,
            lateral_gain=0.0,
            dt=dt,
            estimated_horizontal_range_m=math.dist(
                here[:2], target_state.position[:2]
            ),
            target_state=target_state,
        )
        here = (
            here[0] + velocity[0] * dt,
            here[1] + velocity[1] * dt,
            here[2] + velocity[2] * dt,
        )
    return here


def station_error(position, target_state, config):
    """How far the aircraft is from the six o'clock station point."""
    vx, vy = target_state.velocity[0], target_state.velocity[1]
    heading = math.hypot(vx, vy)
    hx, hy = (vx / heading, vy / heading) if heading > 1e-9 else (1.0, 0.0)
    station = (
        target_state.position[0] - config.standoff_distance_m * hx,
        target_state.position[1] - config.standoff_distance_m * hy,
    )
    return math.dist(position[:2], station)


def moving_target(*, position=(30.0, 12.0, 20.0), speed=3.0):
    return TargetState(
        position=position,
        velocity=(speed, 0.0, 0.0),
        range_m=30.0,
        speed_mps=speed,
        closure_mps=0.0,
    )


def test_a_stronger_station_gain_gets_onto_the_six_sooner():
    """Swinging in behind is the slow part of a stern chase.

    Closing the gap is capped separately by the approach time to contact, so
    this gain is what governs how long the aircraft spends off to one side of
    a target it is otherwise keeping pace with.
    """
    target = moving_target()
    start = (10.0, 30.0, 20.0)

    slow = station_keeping_config(station_keeping_gain_per_s=0.8)
    fast = station_keeping_config(station_keeping_gain_per_s=1.6)

    slow_error = station_error(
        fly_to_station(slow, position=start, target_state=target), target, slow
    )
    fast_error = station_error(
        fly_to_station(fast, position=start, target_state=target), target, fast
    )

    assert fast_error < slow_error


def test_the_stern_chase_still_settles_rather_than_overshooting():
    """Quicker must not mean hunting around the station point."""
    target = moving_target()
    config = station_keeping_config()

    settled = fly_to_station(config, position=(10.0, 30.0, 20.0), target_state=target, ticks=120)
    assert station_error(settled, target, config) < 2.0


def test_the_search_climbs_to_see_over_what_the_target_went_behind(lost_logic):
    """A target does not vanish; something got between it and the camera."""
    logic, position = lost_logic
    start_altitude = position[2]

    for _ in range(12):
        logic.update(position, 0.1)
        position = (position[0], position[1], position[2] + logic.pursuit.velocity[2] * 0.1)

    gained = position[2] - start_altitude
    assert gained == pytest.approx(logic.config.target_lost_search_climb_m, abs=0.2)


def test_the_search_climb_stops_once_the_extra_height_is_gained(lost_logic):
    """It is a step up, not a climb that runs for the whole search."""
    logic, position = lost_logic

    for _ in range(25):
        logic.update(position, 0.1)
        position = (position[0], position[1], position[2] + logic.pursuit.velocity[2] * 0.1)

    assert logic.pursuit.velocity[2] == pytest.approx(0.0, abs=0.05)


def test_the_search_never_descends(lost_logic):
    """Dropping during a search puts the aircraft behind the same obstruction."""
    logic, position = lost_logic

    for _ in range(25):
        logic.update(position, 0.1)
        assert logic.pursuit.velocity[2] >= 0.0
        position = (position[0], position[1], position[2] + logic.pursuit.velocity[2] * 0.1)


def test_repeated_losses_do_not_ratchet_the_aircraft_upward(lost_logic, reacquire):
    """Each loss measures its climb from where that loss happened.

    Measuring from the original tracking altitude instead would stack a metre
    on every occlusion, and a target behind a row of trees would walk the
    aircraft into the ceiling.
    """
    logic, position = lost_logic

    for _ in range(25):
        logic.update(position, 0.1)
        position = (position[0], position[1], position[2] + logic.pursuit.velocity[2] * 0.1)
    after_first = position[2]

    reacquire(logic, position)
    logic.observe_detections(
        [], frame_width=1000, frame_height=1000, position=position, yaw=0.0
    )
    for _ in range(25):
        logic.update(position, 0.1)
        position = (position[0], position[1], position[2] + logic.pursuit.velocity[2] * 0.1)

    gained = position[2] - after_first
    assert gained == pytest.approx(logic.config.target_lost_search_climb_m, abs=0.3)


def test_the_search_outlasts_the_window_that_used_to_end_it(lost_logic):
    """An occlusion lasts longer than it feels like from the video feed.

    Two seconds ended the search while a target was still behind a roofline,
    which cost the track and sent the aircraft back to its patrol. The extra
    second is the difference between a brief occlusion and a lost target.
    """
    logic, position = lost_logic
    tracking = {FlightState.TARGET_FOLLOW, FlightState.TARGET_PURSUIT}

    # Still looking well past the old two-second window.
    for _ in range(22):
        logic.update(position, 0.1)
    assert logic.state in tracking

    # And still gives up rather than searching forever.
    for _ in range(15):
        logic.update(position, 0.1)
    assert logic.state not in tracking


def chase_moving_target(config, *, ticks=300, dt=0.1, target_speed=6.0):
    """Fly a full stern chase against a target that is actually running.

    Returns the horizontal gap to the target at each tick.
    """
    pursuit = PursuitController(config)
    here = (0.0, 0.0, 15.0)
    target = [30.0, 8.0, 12.0]
    gaps = []
    for _ in range(ticks):
        target[0] += target_speed * dt
        state = TargetState(
            position=tuple(target),
            velocity=(target_speed, 0.0, 0.0),
            range_m=math.dist(here, target),
            speed_mps=target_speed,
            closure_mps=0.0,
        )
        velocity = pursuit.update(
            position=here, home=HOME, yaw=0.0, image_velocity=(0.0, 0.0),
            area=0.01, growth=0.0, desired_area=0.01, area_deadzone=0.001,
            collision_side=0.0, lateral_gain=0.0, dt=dt,
            estimated_horizontal_range_m=math.dist(here[:2], target[:2]),
            target_state=state,
        )
        here = (
            here[0] + velocity[0] * dt,
            here[1] + velocity[1] * dt,
            here[2] + velocity[2] * dt,
        )
        gaps.append(math.dist(here[:2], target[:2]))
    return gaps


def test_matching_the_targets_top_speed_never_closes_the_gap():
    """Parity is not a chase.

    With no speed margin the gap can only hold or grow, whatever the station
    law does -- so an aircraft capped at the traffic drones' own 6 m/s cruise
    sits at whatever distance it started from, indefinitely.
    """
    gaps = chase_moving_target(station_keeping_config(max_speed_mps=6.0))
    assert gaps[-1] > 20.0
    # Not even slowly closing: it is simply stuck.
    assert gaps[-1] > gaps[49] - 1.0


def test_a_speed_margin_gets_onto_the_tail_and_holds_the_standoff():
    """Close the gap, settle at the standoff, and stay there."""
    config = station_keeping_config(max_speed_mps=10.0)
    gaps = chase_moving_target(config)

    settled = gaps[-1]
    assert settled == pytest.approx(config.standoff_distance_m, abs=1.5)
    # Held, not merely passed through on the way to a collision.
    assert min(gaps[150:]) > 1.0
    assert max(gaps[150:]) - min(gaps[150:]) < 1.0


def lost_at(center_x):
    """A pursuit whose target was last seen at that horizontal frame position."""
    logic = tracking_logic()
    logic._target_last_center = (center_x, 0.5)
    return logic


def test_a_target_lost_at_the_edge_is_swept_after_faster():
    """Where it vanished says which way it went, and how far it already is.

    Centre frame means it went behind something and will reappear near where
    it was. The edge means it left sideways and is already somewhere the
    camera is not pointing, so the sweep has to catch up rather than wait.
    """
    centre = lost_at(0.5).search_sweep_rate()
    edge = lost_at(1.0).search_sweep_rate()
    assert edge > centre * 2.5


def test_the_sweep_grows_with_how_near_the_edge_it_went():
    rates = [lost_at(0.5 + offset).search_sweep_rate() for offset in (0.0, 0.2, 0.4, 0.5)]
    assert rates == sorted(rates)
    assert rates[0] < rates[-1]


def test_the_sweep_does_not_care_which_side_it_left_by():
    left = lost_at(0.05).search_sweep_rate()
    right = lost_at(0.95).search_sweep_rate()
    assert left == pytest.approx(right)


def test_the_sweep_is_not_throttled_by_a_stale_yaw_limit(lost_logic):
    """The limit is only refreshed while the target is visible.

    Left alone it would still hold whatever the last frame that saw the
    target asked for, which for a target that drifted out slowly is the
    floor -- and the widened sweep would never actually be flown.
    """
    logic, position = lost_logic
    logic._target_last_center = (1.0, 0.5)
    logic._target_yaw_rate_limit = logic.config.target_min_yaw_rate_rad_s

    logic.update(position, 0.1)

    assert logic.target_yaw_rate_limit >= logic.search_sweep_rate() * 0.99
    assert logic.target_yaw_rate_limit <= logic.config.target_max_yaw_rate_rad_s
