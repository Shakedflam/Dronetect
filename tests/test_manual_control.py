"""Operator takeover: who is flying, what they may fly, and what happens after."""

import math
import random

import pytest

from controllers.autonomous_drone.flight_logic import (
    FlightConfig,
    FlightLogic,
    FlightState,
    ManualFlightConfig,
    ManualStick,
    manual_velocity_command,
)
from controllers.autonomous_drone.manual_input import (
    MANUAL_INPUT_RC,
    MANUAL_INPUT_TABLET,
    ManualInputRouter,
)


HOME = (0.0, 0.0, 0.1)
CRUISE = 10.0
MANUAL_CONFIG = ManualFlightConfig()


def make_logic(**config_overrides):
    config = FlightConfig(takeoff_altitude=CRUISE, **config_overrides)
    logic = FlightLogic(config=config, rng=random.Random(7))
    logic.initialize_home(HOME)
    return logic


def airborne(logic):
    return (0.0, 0.0, logic.config.takeoff_altitude)


def start_patrol(logic):
    logic.request_patrol(airborne(logic))
    return airborne(logic)


# --- stick to velocity -------------------------------------------------------


def velocity(stick, *, yaw=0.0, position=(0.0, 0.0, CRUISE), radius=300.0, dt=0.1):
    return manual_velocity_command(
        stick=stick,
        yaw=yaw,
        position=position,
        home=HOME,
        max_flight_radius_m=radius,
        max_altitude_m=48.0,
        dt=dt,
        config=MANUAL_CONFIG,
    )


def test_a_centered_stick_is_a_hover():
    """Letting go of every button must stop the aircraft, not coast it."""
    assert velocity(ManualStick()) == (0.0, 0.0, 0.0)


def test_forward_follows_the_nose_not_the_map():
    """The operator flies from the camera, so forward is wherever it points."""
    vx, vy, _ = velocity(ManualStick(forward=1.0), yaw=math.radians(90.0))
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(MANUAL_CONFIG.max_horizontal_speed_mps)


def test_right_stick_is_right_of_the_nose():
    vx, vy, _ = velocity(ManualStick(right=1.0), yaw=math.radians(90.0))
    assert vx == pytest.approx(MANUAL_CONFIG.max_horizontal_speed_mps)
    assert vy == pytest.approx(0.0, abs=1e-9)


def test_a_diagonal_is_no_faster_than_a_straight_line():
    """Holding two sticks must not buy 41% more speed than holding one."""
    vx, vy, _ = velocity(ManualStick(forward=1.0, right=1.0))
    assert math.hypot(vx, vy) == pytest.approx(MANUAL_CONFIG.max_horizontal_speed_mps)


def test_the_geofence_stops_outward_motion_only():
    """A boundary should feel like a wall, not like a failed control.

    Zeroing the whole stick at the fence leaves the operator pushing a dead
    button; zeroing only the outward component lets them fly along it.
    """
    at_edge = (299.9, 0.0, CRUISE)
    vx, vy, _ = velocity(ManualStick(forward=1.0, right=1.0), position=at_edge, radius=300.0)
    assert vx == pytest.approx(0.0, abs=1e-6)
    # The crossways component survives untouched.
    assert vy == pytest.approx(-MANUAL_CONFIG.max_horizontal_speed_mps / math.sqrt(2.0))


def test_flying_back_inside_the_geofence_is_always_allowed():
    outside = (400.0, 0.0, CRUISE)
    vx, _, _ = velocity(ManualStick(forward=-1.0), position=outside, radius=300.0)
    assert vx == pytest.approx(-MANUAL_CONFIG.max_horizontal_speed_mps)


def test_manual_descent_stops_above_the_ground():
    """Manual may descend, but landing stays a computer job."""
    low = (0.0, 0.0, MANUAL_CONFIG.min_altitude_m + 0.05)
    _, _, vz = velocity(ManualStick(up=-1.0), position=low)
    assert vz == pytest.approx(-0.5)  # exactly the remaining 0.05 m over dt=0.1

    at_floor = (0.0, 0.0, MANUAL_CONFIG.min_altitude_m)
    assert velocity(ManualStick(up=-1.0), position=at_floor)[2] == pytest.approx(0.0)


def test_manual_climb_stops_at_the_legal_ceiling():
    at_ceiling = (0.0, 0.0, 48.0)
    assert velocity(ManualStick(up=1.0), position=at_ceiling)[2] == pytest.approx(0.0)


# --- taking and giving back control ------------------------------------------


def test_manual_can_be_taken_while_patrolling_and_hovers_where_it_was_taken():
    """Engaging must not move the aircraft; it hands it over exactly as it is."""
    logic = make_logic()
    position = start_patrol(logic)

    assert logic.engage_manual_control(position) is True
    assert logic.state is FlightState.MANUAL
    assert logic.manual_control is True

    flown_to = (25.0, 5.0, CRUISE)
    logic.update(flown_to, 0.1)
    assert logic.target == flown_to


def test_manual_is_refused_on_the_ground_and_during_takeoff_and_landing():
    """There is nothing to fly and no mission to hand back to."""
    logic = make_logic()
    assert logic.engage_manual_control(HOME) is False

    logic.request_mission_takeoff(HOME)
    assert logic.state is FlightState.TAKING_OFF
    assert logic.engage_manual_control(HOME) is False

    logic.state = FlightState.LANDING
    assert logic.engage_manual_control(HOME) is False


def test_taking_control_twice_is_a_no_op():
    logic = make_logic()
    position = start_patrol(logic)
    assert logic.engage_manual_control(position) is True
    assert logic.engage_manual_control(position) is False


def test_releasing_from_a_patrol_asks_for_a_patrol():
    logic = make_logic()
    position = start_patrol(logic)
    logic.engage_manual_control(position)

    resume = logic.release_manual_control()
    assert resume.state is FlightState.PATROLLING
    assert resume.roi is None
    assert logic.manual_control is False


def test_releasing_from_an_roi_asks_for_that_same_roi_back():
    """The ROI the operator interrupted is the one they expect to come back to."""
    logic = make_logic()
    position = airborne(logic)
    logic.request_roi(position, (60.0, 20.0), 40.0)
    logic.engage_manual_control(position)

    resume = logic.release_manual_control()
    assert resume.state is FlightState.ROI_PATROL
    assert resume.roi == ((60.0, 20.0), 40.0)


def test_releasing_from_an_operator_rth_keeps_flying_home():
    logic = make_logic()
    start_patrol(logic)
    logic.request_return_home()
    logic.engage_manual_control(airborne(logic))

    resume = logic.release_manual_control()
    assert resume.state is FlightState.RETURNING_HOME


def test_releasing_when_nobody_took_control_asks_for_nothing():
    logic = make_logic()
    start_patrol(logic)
    assert logic.release_manual_control() is None


def test_a_safety_rth_can_take_the_aircraft_off_the_operator():
    """Home outranks the operator, and it must not resume anything on the way."""
    logic = make_logic()
    position = airborne(logic)
    logic.request_roi(position, (60.0, 20.0), 40.0)
    logic.engage_manual_control(position)

    assert logic.abort_manual_control() is True
    assert logic.request_return_home() is True
    assert logic.state is FlightState.RETURNING_HOME
    assert logic.manual_control is False
    # Nothing left over to resume: the aircraft is going home.
    assert logic.release_manual_control() is None


def test_a_stick_from_before_the_takeover_is_never_flown():
    """A takeover always starts from a hover, whatever was held last time."""
    logic = make_logic()
    position = start_patrol(logic)
    logic.engage_manual_control(position)
    logic.set_manual_stick(ManualStick(forward=1.0))
    logic.release_manual_control()

    logic.request_patrol(position)
    logic.engage_manual_control(position)
    assert logic.manual_stick.is_centered


def test_sticks_sent_while_the_computer_is_flying_are_ignored():
    logic = make_logic()
    position = start_patrol(logic)
    logic.set_manual_stick(ManualStick(forward=1.0))
    assert logic.manual_stick.is_centered
    assert logic.state is FlightState.PATROLLING


# --- which stick wins --------------------------------------------------------


class FakeRemoteController:
    def __init__(self, *, connected, stick=None):
        self.connected = connected
        self._stick = stick

    def read_stick(self):
        return self._stick


def test_the_tablet_flies_when_no_remote_controller_is_bound():
    router = ManualInputRouter(command_timeout_s=1.0)
    router.accept_tablet_stick(ManualStick(forward=1.0), now=10.0)
    assert router.source == MANUAL_INPUT_TABLET
    assert router.stick(10.05).forward == 1.0


def test_a_bound_remote_controller_outranks_the_tablet_pad():
    """Two operators must never be flying the same airframe at once.

    Whoever is holding a physical transmitter has the aircraft in sight; the
    tablet pad yields to them rather than summing into a third input.
    """
    router = ManualInputRouter(
        command_timeout_s=1.0,
        remote_controller=FakeRemoteController(
            connected=True, stick=ManualStick(right=1.0)
        ),
    )
    router.accept_tablet_stick(ManualStick(forward=1.0), now=10.0)
    assert router.source == MANUAL_INPUT_RC
    assert router.stick(10.05) == ManualStick(right=1.0)


def test_a_quiet_remote_controller_hovers_rather_than_falling_back_to_the_tablet():
    router = ManualInputRouter(
        command_timeout_s=1.0,
        remote_controller=FakeRemoteController(connected=True, stick=None),
    )
    router.accept_tablet_stick(ManualStick(forward=1.0), now=10.0)
    assert router.stick(10.05).is_centered


def test_a_tablet_that_stops_sending_hovers():
    """The pad resends while a button is down, so silence means let go."""
    router = ManualInputRouter(command_timeout_s=1.0)
    router.accept_tablet_stick(ManualStick(forward=1.0), now=10.0)
    assert router.stick(11.5).is_centered


def test_nothing_held_yet_is_a_hover():
    assert ManualInputRouter(command_timeout_s=1.0).stick(10.0).is_centered


# --- the latch must never outlive the takeover -------------------------------


def test_flying_home_from_the_cockpit_key_ends_the_takeover():
    """The H key hands the aircraft back exactly as the RTH button does.

    A latch left engaged over an aircraft that is flying itself home is the
    worst version of this bug: the pad stays on screen over an aircraft nobody
    is flying, detections stay suppressed, and pressing MANUAL to clear it
    would pull the aircraft off the return and into a patrol.
    """
    logic = make_logic()
    position = start_patrol(logic)
    logic.engage_manual_control(position)

    assert logic.request_return_home() is True
    assert logic.state is FlightState.RETURNING_HOME
    assert logic.manual_control is False
    assert logic.release_manual_control() is None


@pytest.mark.parametrize("start_a_mission", [
    lambda logic, position: logic.request_patrol(position),
    lambda logic, position: logic.request_roi(position, (60.0, 20.0), 40.0),
    lambda logic, position: logic.request_mission_takeoff(position),
])
def test_a_mission_command_during_a_takeover_takes_the_aircraft_back(
    start_a_mission,
):
    """Whoever is flying has to be whoever the aircraft says is flying.

    A mission command is the operator asking the computer to fly again. If the
    latch survived it, the computer would be flying while every readout still
    reported manual -- and target acquisition would stay suppressed for the
    rest of the sortie.
    """
    logic = make_logic()
    position = start_patrol(logic)
    logic.engage_manual_control(position)

    start_a_mission(logic, position)

    assert logic.state is not FlightState.MANUAL
    assert logic.manual_control is False


def test_clearing_the_roi_during_a_takeover_hands_back_a_patrol_instead():
    """Releasing must not fly to an ROI the operator has since cleared."""
    logic = make_logic()
    position = airborne(logic)
    logic.request_roi(position, (60.0, 20.0), 40.0)
    logic.engage_manual_control(position)

    logic.clear_roi(position)

    assert logic.state is FlightState.MANUAL
    assert logic.manual_control is True
    resume = logic.release_manual_control()
    assert resume.state is FlightState.PATROLLING
    assert resume.roi is None
