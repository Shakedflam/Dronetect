import random

import pytest

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
    logic.update(HOME, logic.config.idle_duration)
    logic.update((HOME[0], HOME[1], logic.config.takeoff_altitude), 0.1)


def test_initializes_idle_at_captured_home():
    logic = make_logic()

    assert logic.state is FlightState.IDLE
    assert logic.home == HOME
    assert logic.target == HOME


def test_transitions_from_idle_to_takeoff_automatically():
    logic = make_logic(idle_duration=1.0)

    logic.update(HOME, 0.5)
    assert logic.state is FlightState.IDLE

    logic.update(HOME, 0.5)
    assert logic.state is FlightState.TAKING_OFF
    assert logic.target == (HOME[0], HOME[1], logic.config.takeoff_altitude)


def test_reaching_takeoff_altitude_starts_random_flight():
    logic = make_logic()

    finish_takeoff(logic)

    assert logic.state is FlightState.FLYING_RANDOMLY
    assert logic.target is not None


def test_random_waypoints_stay_inside_all_configured_bounds():
    logic = make_logic()
    finish_takeoff(logic)

    for _ in range(50):
        x, y, z = logic.choose_random_waypoint()
        assert logic.config.x_bounds[0] <= x <= logic.config.x_bounds[1]
        assert logic.config.y_bounds[0] <= y <= logic.config.y_bounds[1]
        assert logic.config.z_bounds[0] <= z <= logic.config.z_bounds[1]


def test_reaching_mission_waypoint_returns_to_home_automatically():
    logic = make_logic()
    finish_takeoff(logic)
    mission_target = logic.target

    logic.update(mission_target, 0.1)

    assert logic.state is FlightState.RETURNING_HOME
    assert logic.target == (HOME[0], HOME[1], logic.config.takeoff_altitude)


@pytest.mark.parametrize(
    "starting_state",
    [FlightState.TAKING_OFF, FlightState.FLYING_RANDOMLY],
)
def test_return_home_replaces_active_target(starting_state):
    logic = make_logic()
    logic.state = starting_state
    logic.target = (6.0, -4.0, 4.0)

    accepted = logic.request_return_home()

    assert accepted is True
    assert logic.state is FlightState.RETURNING_HOME
    assert logic.target == (HOME[0], HOME[1], logic.config.takeoff_altitude)


def test_returning_home_never_resumes_random_flight():
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


def test_new_target_from_landed_preserves_home_and_starts_takeoff():
    logic = make_logic()
    logic.state = FlightState.LANDED

    waypoint = logic.request_new_target(HOME)

    assert logic.home == HOME
    assert logic.state is FlightState.TAKING_OFF
    assert logic.target == (HOME[0], HOME[1], logic.config.takeoff_altitude)
    assert waypoint != logic.target


def test_new_target_during_flight_replaces_target_immediately():
    logic = make_logic()
    finish_takeoff(logic)
    previous_target = logic.target
    airborne = (0.0, 0.0, logic.config.takeoff_altitude)

    waypoint = logic.request_new_target(airborne)

    assert logic.home == HOME
    assert logic.state is FlightState.FLYING_RANDOMLY
    assert logic.target == waypoint
    assert logic.target != previous_target


def test_new_target_cancels_return_home_when_altitude_is_safe():
    logic = make_logic()
    finish_takeoff(logic)
    logic.request_return_home()
    airborne = (2.0, 1.0, logic.config.takeoff_altitude)

    waypoint = logic.request_new_target(airborne)

    assert logic.state is FlightState.FLYING_RANDOMLY
    assert logic.target == waypoint
    assert logic.home == HOME


def test_new_target_cancels_low_landing_and_climbs_before_departing():
    logic = make_logic()
    logic.state = FlightState.LANDING
    low_position = (0.2, -0.1, 0.5)

    waypoint = logic.request_new_target(low_position)

    assert logic.state is FlightState.TAKING_OFF
    assert logic.target == (
        low_position[0],
        low_position[1],
        logic.config.takeoff_altitude,
    )

    logic.update(logic.target, 0.1)

    assert logic.state is FlightState.FLYING_RANDOMLY
    assert logic.target == waypoint
    assert logic.home == HOME


def test_new_target_while_taking_off_replaces_the_queued_waypoint():
    logic = make_logic()
    first = logic.request_new_target(HOME)

    second = logic.request_new_target((0.0, 0.0, 1.0))
    logic.update((0.0, 0.0, logic.config.takeoff_altitude), 0.1)

    assert second != first
    assert logic.target == second
