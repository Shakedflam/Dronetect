"""DJI-native takeoff and landing around Dronetect's own autonomy.

DJI flies only the lift-off and the touchdown. FlightLogic keeps the climb,
patrol/ROI, pursuit and the custom RETURNING_HOME, and its LANDING approach
holds low over home until DJI takes the last metres. These drive the real
DjiFlightBridge.run_tick_once() and read what actually went to Android.
"""

import asyncio
import math

import pytest

from adapters.dji.flight_bridge import (
    LANDING_APPROACH_HEIGHT_M,
    LANDING_CONFIRM_SECONDS,
    NATIVE_LANDING_INTERRUPT_SECONDS,
    NATIVE_LANDING_RELEASE_SETTLE_SECONDS,
    NATIVE_LANDING_SETTLE_SECONDS,
    NATIVE_LANDING_START_TIMEOUT_SECONDS,
    NATIVE_TAKEOFF_STABLE_SECONDS,
    NATIVE_TAKEOFF_TIMEOUT_SECONDS,
    DjiFlightBridge,
    TickOutcome,
)
from controllers.autonomous_drone.flight_logic import FlightConfig, FlightLogic, FlightState
from services.commands import CommandName
from services.gps import DEFAULT_HOME, LocalPosition, local_meters_to_wgs84
from tests.test_dji_flight_bridge import (
    TEST_OPERATING_CONFIG,
    _bridge_on_ground_with_home,
    _confident_detection_message,
    _feed_ground_rc,
    _feed_telemetry,
    _queue_command,
    _SetpointRecorder,
)

DT = 0.05
CRUISE = TEST_OPERATING_CONFIG.cruise_altitude_m
ROI_CENTER = local_meters_to_wgs84(LocalPosition(x=60.0, y=60.0, z=0.0))


class Flight:
    """One bridge plus the Android side of the link, ticked at 20 Hz."""

    def __init__(self, bridge: DjiFlightBridge, now: list[float]):
        self.bridge = bridge
        self.now = now
        self.recorder = _SetpointRecorder()
        self.outcome: TickOutcome | None = None

    def tick(self, *, is_flying: bool = True, alt: float = CRUISE, mode: str = "GPS_NORMAL",
             east: float = 0.0, north: float = 0.0, speed: float = 0.0,
             detection: dict | None = None, **sticks) -> TickOutcome:
        self.now[0] += DT
        gps = local_meters_to_wgs84(LocalPosition(x=east, y=north, z=alt))
        _feed_telemetry(self.bridge, self.now, is_flying=is_flying, altitude_m=alt,
                        flight_mode=mode, lat=gps.lat, lng=gps.lng, speed_mps=speed)
        _feed_ground_rc(self.bridge, self.now, is_flying=is_flying, **sticks)
        if detection is not None:
            self.bridge.realtime_link._detections.put(detection)
        self.bridge.android_link._connection = self.recorder
        self.outcome = asyncio.run(self.bridge.run_tick_once())
        return self.outcome

    def hold(self, seconds: float, **telemetry) -> TickOutcome:
        outcome = None
        for _ in range(max(1, round(seconds / DT))):
            outcome = self.tick(**telemetry)
        assert outcome is not None
        return outcome

    def follow(self, ticks: int, *, mode: str = "VIRTUAL_STICK", until=None) -> TickOutcome:
        """Closed loop: each tick's telemetry reports where the last one flew to."""
        for _ in range(ticks):
            assert self.outcome is not None and self.outcome.position is not None
            x, y, z = self.outcome.position
            self.tick(alt=z, east=x, north=y, mode=mode, speed=self.outcome.speed_mps)
            if until is not None and until(self):
                break
        assert self.outcome is not None
        return self.outcome

    def sent(self, kind: str) -> list[dict]:
        return [m for m in self.recorder.messages if m.get("type") == kind]

    def types(self) -> list[str]:
        return [m["type"] for m in self.recorder.messages if m["type"] != "dji_gimbal_command"]

    def clear(self) -> None:
        self.recorder.messages.clear()


def _on_ground(now: list[float] | None = None) -> Flight:
    now = now if now is not None else [0.0]
    flight = Flight(_bridge_on_ground_with_home(now), now)
    flight.tick(is_flying=False, alt=0.0)
    return flight


def _start(flight: Flight, mission: str) -> None:
    if mission == "roi":
        _queue_command(flight.bridge, CommandName.SET_ROI.value,
                       {"center": {"lat": ROI_CENTER.lat, "lon": ROI_CENTER.lng}, "radiusM": 20.0},
                       command_id="roi")
    else:
        _queue_command(flight.bridge, CommandName.START_PATROL.value, command_id="patrol")


def _airborne_after_native_takeoff(mission: str = "patrol") -> Flight:
    flight = _on_ground()
    _start(flight, mission)
    flight.tick(is_flying=False, alt=0.0)
    flight.hold(NATIVE_TAKEOFF_STABLE_SECONDS + 0.2, alt=1.2)
    assert flight.bridge._native_takeoff is None
    return flight


def _cruising(mission: str = "patrol") -> Flight:
    """Airborne at cruise over home, mission running on Virtual Stick."""
    now = [0.0]
    flight = Flight(_bridge_on_ground_with_home(now), now)
    _start(flight, mission)
    flight.tick(alt=CRUISE, mode="VIRTUAL_STICK")
    return flight


def _landing_approach() -> Flight:
    """Custom RTH flown home, now in LANDING above home."""
    flight = _cruising()
    _queue_command(flight.bridge, CommandName.RETURN_HOME.value, command_id="rth")
    flight.tick(alt=CRUISE, mode="VIRTUAL_STICK")
    assert flight.bridge.logic.state is FlightState.LANDING
    return flight


def _at_hold(flight: Flight, **telemetry) -> TickOutcome:
    params = dict(alt=LANDING_APPROACH_HEIGHT_M, mode="VIRTUAL_STICK")
    params.update(telemetry)
    return flight.tick(**params)


def _handed_over() -> Flight:
    flight = _landing_approach()
    flight.clear()
    for _ in range(round((NATIVE_LANDING_SETTLE_SECONDS + 0.2) / DT)):
        _at_hold(flight)
    assert flight.bridge._native_landing is not None
    return flight


def _dji_landing() -> Flight:
    """Handed over, Virtual Stick released, DJI landing requested and running."""
    flight = _handed_over()
    flight.hold(NATIVE_LANDING_RELEASE_SETTLE_SECONDS + 0.2, alt=LANDING_APPROACH_HEIGHT_M)
    assert flight.sent("dji_land")
    flight.tick(alt=2.5, mode="AUTO_LANDING")
    assert flight.bridge._native_landing.descending
    return flight


# -- native takeoff ---------------------------------------------------------------


@pytest.mark.parametrize("mission", ["roi", "patrol"])
def test_mission_started_on_the_ground_requests_one_native_takeoff(mission, capsys):
    flight = _on_ground()
    _start(flight, mission)
    flight.tick(is_flying=False, alt=0.0)
    assert flight.bridge.logic.state is FlightState.TAKING_OFF
    assert len(flight.sent("dji_takeoff")) == 1
    assert flight.sent("dji_takeoff")[0]["controlEpoch"] == flight.bridge._control_epoch
    flight.hold(2.0, is_flying=False, alt=0.0)
    assert len(flight.sent("dji_takeoff")) == 1, "never re-sent"
    out = capsys.readouterr().out
    assert f"takeoff=native_requested mission={'set_roi' if mission == 'roi' else 'start_patrol'}" in out


def test_no_setpoints_while_dji_flies_the_takeoff():
    flight = _on_ground()
    _start(flight, "roi")
    flight.tick(is_flying=False, alt=0.0)
    flight.hold(2.0, is_flying=False, alt=0.0, mode="MOTOR_START")
    for alt in (0.3, 0.6, 0.9, 1.1, 1.2):  # DJI's own climb-out
        flight.hold(0.5, alt=alt, mode="AUTO_TAKE_OFF")
    assert flight.sent("dji_setpoint") == []
    assert flight.bridge.logic.state is FlightState.TAKING_OFF
    # Hovering, but not yet stable for the full window: still nothing.
    flight.hold(NATIVE_TAKEOFF_STABLE_SECONDS - 0.3, alt=1.2)
    assert flight.sent("dji_setpoint") == []


@pytest.mark.parametrize("mission, final_state", [
    ("roi", FlightState.TRANSITING_TO_ROI), ("patrol", FlightState.PATROLLING)])
def test_confirmed_takeoff_hands_the_climb_to_virtual_stick(mission, final_state, capsys):
    flight = _on_ground()
    _start(flight, mission)
    flight.tick(is_flying=False, alt=0.0)
    epoch = flight.bridge._control_epoch
    flight.hold(NATIVE_TAKEOFF_STABLE_SECONDS + 0.2, alt=1.2)

    setpoints = flight.sent("dji_setpoint")
    assert setpoints, "autonomy takes over once DJI's hover is stable"
    assert setpoints[-1]["velocity"]["z"] > 0.0
    assert {s["controlEpoch"] for s in setpoints} == {epoch}
    assert "takeoff=confirmed" in capsys.readouterr().out

    # The existing climb continues to cruise, then the queued mission.
    flight.follow(2000, until=lambda f: f.bridge.logic.state is not FlightState.TAKING_OFF)
    assert flight.bridge.logic.state is final_state
    assert flight.outcome.position[2] == pytest.approx(CRUISE, abs=1.0)


def test_mission_started_airborne_skips_native_takeoff(capsys):
    now = [0.0]
    flight = Flight(_bridge_on_ground_with_home(now), now)
    flight.tick(alt=5.0)  # the pilot already flew it up
    _start(flight, "roi")
    flight.tick(alt=5.0)
    assert flight.sent("dji_takeoff") == []
    assert flight.bridge._native_takeoff is None
    assert flight.bridge.logic.state is FlightState.TAKING_OFF
    assert flight.sent("dji_setpoint")[-1]["velocity"]["z"] > 0.0
    assert "takeoff=native_skipped reason=already_airborne" in capsys.readouterr().out


def test_native_takeoff_that_never_leaves_the_ground_aborts_to_idle(capsys):
    flight = _on_ground()
    _start(flight, "roi")
    flight.hold(NATIVE_TAKEOFF_TIMEOUT_SECONDS + 0.5, is_flying=False, alt=0.0)
    assert flight.bridge.logic.state is FlightState.IDLE
    assert flight.bridge._native_takeoff is None
    assert flight.bridge._active_roi_id is None
    assert flight.bridge._pilot_control is None
    assert flight.sent("dji_setpoint") == []
    assert len(flight.sent("dji_takeoff")) == 1
    assert "takeoff=native_failed reason=not_airborne" in capsys.readouterr().out
    # Nothing silently retries afterwards.
    flight.hold(5.0, is_flying=False, alt=0.0)
    assert len(flight.sent("dji_takeoff")) == 1


def test_native_takeoff_stuck_in_an_unflyable_mode_goes_to_the_pilot(capsys):
    flight = _on_ground()
    _start(flight, "patrol")
    flight.tick(is_flying=False, alt=0.0)
    flight.hold(NATIVE_TAKEOFF_TIMEOUT_SECONDS + 0.5, alt=1.2, mode="ATTI")
    assert flight.bridge.logic.state is FlightState.IDLE
    assert flight.bridge._pilot_control == "native_takeoff_failed"
    assert flight.sent("dji_setpoint") == []
    assert [m["reason"] for m in flight.sent("dji_release")] == ["native_takeoff_failed"]
    assert "takeoff=native_failed reason=unstable_mode_ATTI" in capsys.readouterr().out
    # Even once the mode is flyable again, nothing re-arms.
    flight.hold(3.0, alt=1.2)
    assert flight.sent("dji_setpoint") == []


def test_rc_takeover_during_native_takeoff_wins_immediately(capsys):
    flight = _on_ground()
    _start(flight, "roi")
    flight.tick(is_flying=False, alt=0.0)
    flight.tick(alt=0.8, mode="AUTO_TAKE_OFF")
    flight.tick(alt=0.9, mode="AUTO_TAKE_OFF", forward=0.5)
    assert flight.bridge.logic.state is FlightState.IDLE
    assert flight.bridge._native_takeoff is None
    assert flight.bridge._pilot_control is not None
    assert len(flight.sent("dji_release")) == 1
    assert "takeoff=native_abandoned reason=pilot_control_rc_stick_forward" in capsys.readouterr().out
    flight.hold(3.0, alt=1.2)
    assert flight.sent("dji_setpoint") == []
    assert len(flight.sent("dji_takeoff")) == 1


def test_detections_cannot_start_a_chase_during_native_takeoff():
    flight = _on_ground()
    _start(flight, "patrol")
    flight.tick(is_flying=False, alt=0.0)
    for _ in range(60):
        flight.tick(alt=1.2, mode="AUTO_TAKE_OFF", detection=_confident_detection_message())
    assert flight.bridge.logic.state is FlightState.TAKING_OFF
    assert not flight.bridge.logic.is_target_tracking


# -- custom RTH and landing approach -------------------------------------------------


def test_operator_rth_uses_the_custom_return_home_not_djis():
    flight = _cruising()
    flight.follow(200)  # patrol away from home for a while
    away = flight.outcome.position
    assert math.hypot(away[0], away[1]) > 5.0
    flight.clear()
    _queue_command(flight.bridge, CommandName.RETURN_HOME.value, command_id="rth")
    flight.follow(1)
    assert flight.bridge.logic.state is FlightState.RETURNING_HOME
    assert flight.bridge.logic.target == pytest.approx((0.0, 0.0, CRUISE))
    flight.follow(3000, until=lambda f: f.bridge.logic.state is not FlightState.RETURNING_HOME)
    assert flight.bridge.logic.state is FlightState.LANDING
    assert flight.sent("dji_rth") == []
    assert flight.sent("dji_land") == []
    assert flight.sent("dji_setpoint"), "flown home on Virtual Stick"


def test_rth_reaches_home_and_descends_to_the_hold_then_hands_to_dji(capsys):
    flight = _landing_approach()
    flight.follow(3000, until=lambda f: f.bridge._native_landing is not None)
    position = flight.outcome.position
    assert position[2] == pytest.approx(LANDING_APPROACH_HEIGHT_M, abs=0.6)
    assert math.hypot(position[0], position[1]) <= 2.0
    out = capsys.readouterr().out
    assert "landing=approach_waiting reason=height_" in out
    assert "landing=native_handover" in out


def test_simulator_landing_still_flies_all_the_way_down():
    logic = FlightLogic(FlightConfig(takeoff_altitude=10.0, landing_rate=1.0))
    logic.initialize_home((0.0, 0.0, 0.0))
    logic.request_patrol((0.0, 0.0, 10.0))
    logic.request_return_home()
    logic.update((0.0, 0.0, 10.0), 0.1)
    assert logic.state is FlightState.LANDING
    for _ in range(200):
        logic.update((0.0, 0.0, 10.0), 0.1)
    assert logic.target[2] == pytest.approx(0.0)


def test_real_landing_approach_holds_at_the_handover_height():
    flight = _landing_approach()
    for _ in range(400):
        flight.bridge.logic.update((0.0, 0.0, 10.0), 0.1)
    assert flight.bridge.logic.target[2] == pytest.approx(LANDING_APPROACH_HEIGHT_M)


@pytest.mark.parametrize("telemetry, reason", [
    (dict(east=3.0), "horizontal_3.0m"),
    (dict(alt=6.0), "height_6.0m"),
    (dict(speed=2.5), "speed_2.5mps"),
    (dict(mode="ATTI"), "flight_mode_ATTI"),
])
def test_native_landing_waits_for_valid_home_altitude_and_flight_state(telemetry, reason, capsys):
    flight = _landing_approach()
    flight.clear()
    for _ in range(round(10.0 / DT)):
        _at_hold(flight, **telemetry)
    assert flight.bridge._native_landing is None
    assert flight.sent("dji_land") == [] and flight.sent("dji_release") == []
    assert f"landing=approach_waiting reason={reason}" in capsys.readouterr().out


def test_native_landing_waits_for_fresh_telemetry():
    flight = _landing_approach()
    for _ in range(round(NATIVE_LANDING_SETTLE_SECONDS / DT) - 2):
        _at_hold(flight)
    flight.now[0] += 5.0  # telemetry stops arriving
    flight.bridge.tick(DT, now=flight.now[0])
    assert flight.bridge._native_landing is None


def test_a_safety_rth_also_ends_in_djis_native_landing():
    """Dronetect's RTH owns a safety RTH too; before, the handover was skipped
    and the aircraft hovered at the 3 m hold until DJI's battery protection."""
    flight = _landing_approach()
    flight.bridge._safety_rth_active = True
    flight.bridge._safety_rth_reason = "low_battery"
    flight.hold(NATIVE_LANDING_SETTLE_SECONDS + 0.2, alt=LANDING_APPROACH_HEIGHT_M, mode="VIRTUAL_STICK")
    assert flight.bridge._native_landing is not None
    flight.hold(1.0, alt=LANDING_APPROACH_HEIGHT_M, mode="GPS_NORMAL")
    assert len(flight.sent("dji_land")) == 1
    assert flight.sent("dji_rth") == [], "Virtual Stick is ours: no DJI go-home"


def test_when_dji_flies_its_own_go_home_dronetect_stands_back(capsys):
    flight = _cruising()
    flight.follow(20)
    flight.clear()
    flight.bridge._safety_rth_active = True
    flight.bridge._safety_rth_reason = "aircraft_safety_rth"
    flight.bridge.logic.request_return_home()
    flight.hold(5.0, alt=CRUISE, mode="GO_HOME")
    assert flight.sent("dji_setpoint") == []
    assert flight.sent("dji_land") == [] and flight.bridge._native_landing is None
    assert capsys.readouterr().out.count("setpoints=suppressed reason=dji_go_home") == 1


# -- native landing ------------------------------------------------------------------


def test_virtual_stick_is_released_and_seen_released_before_dji_lands(capsys):
    flight = _handed_over()  # the release goes out on the handover tick itself
    assert flight.types()[-1] == "dji_release"
    assert flight.sent("dji_release")[-1]["reason"] == "native_landing"
    assert "dji_land" not in flight.types()
    flight.clear()
    # Android still reports VIRTUAL_STICK, so no land yet -- and nothing else either.
    flight.hold(2.0, alt=LANDING_APPROACH_HEIGHT_M, mode="VIRTUAL_STICK")
    assert flight.types() == []
    # Virtual Stick drops out: DJI is asked to land, once.
    flight.hold(1.0, alt=LANDING_APPROACH_HEIGHT_M, mode="GPS_NORMAL")
    assert flight.types() == ["dji_land"]
    flight.hold(2.0, alt=2.0, mode="AUTO_LANDING")
    assert flight.types() == ["dji_land"]
    out = capsys.readouterr().out
    assert out.index("virtual_stick=release_requested reason=native_landing") < out.index(
        "landing=native_requested")


def test_no_setpoints_from_handover_through_dji_landing():
    flight = _handed_over()
    flight.clear()
    flight.hold(1.0, alt=LANDING_APPROACH_HEIGHT_M, mode="GPS_NORMAL")
    for alt in (2.5, 2.0, 1.5, 1.0, 0.5, 0.2):
        flight.hold(0.5, alt=alt, mode="AUTO_LANDING")
    assert flight.sent("dji_setpoint") == []


def test_touchdown_for_two_seconds_resets_to_idle_and_a_new_mission_takes_off_again(capsys):
    flight = _dji_landing()
    config = flight.bridge.operating_config
    mission_epoch = flight.bridge._control_epoch
    flight.hold(LANDING_CONFIRM_SECONDS - 0.5, is_flying=False, alt=0.0, mode="GPS_NORMAL")
    assert flight.bridge.logic.state is not FlightState.IDLE, "not before 2 s"
    flight.hold(1.0, is_flying=False, alt=0.0, mode="GPS_NORMAL")
    assert flight.bridge.logic.state is FlightState.IDLE
    assert flight.bridge._native_landing is None
    assert flight.bridge.operating_config is config
    assert flight.bridge.session_home is not None
    assert "landing=confirmed stateBefore=" in capsys.readouterr().out
    flight.clear()
    flight.hold(2.0, is_flying=False, alt=0.0)
    assert flight.sent("dji_setpoint") == []

    _start(flight, "roi")
    flight.tick(is_flying=False, alt=0.0)
    assert len(flight.sent("dji_takeoff")) == 1
    flight.hold(NATIVE_TAKEOFF_STABLE_SECONDS + 0.2, alt=1.2)
    assert flight.sent("dji_setpoint")[-1]["controlEpoch"] > mission_epoch


def test_dji_refusing_to_land_leaves_the_pilot_in_charge(capsys):
    flight = _handed_over()
    flight.hold(1.0, alt=LANDING_APPROACH_HEIGHT_M, mode="GPS_NORMAL")
    assert flight.sent("dji_land")
    flight.clear()
    flight.hold(NATIVE_LANDING_START_TIMEOUT_SECONDS + 0.5, alt=LANDING_APPROACH_HEIGHT_M,
                mode="GPS_NORMAL")
    assert "landing=native_failed reason=not_started_mode_GPS_NORMAL" in capsys.readouterr().out
    assert flight.bridge._native_landing is None
    assert flight.bridge._pilot_control == "native_landing_failed"
    assert flight.bridge.logic.state is FlightState.IDLE
    flight.hold(10.0, alt=LANDING_APPROACH_HEIGHT_M, mode="GPS_NORMAL")
    assert flight.sent("dji_setpoint") == [], "no silent Virtual Stick re-arm"
    assert len(flight.sent("dji_land")) == 0, "no silent landing retry"


def test_dji_abandoning_its_landing_midway_is_a_failure_not_a_resume(capsys):
    flight = _dji_landing()
    flight.clear()
    flight.hold(NATIVE_LANDING_INTERRUPT_SECONDS + 0.3, alt=1.8, mode="GPS_NORMAL")
    assert "landing=native_failed reason=interrupted_mode_GPS_NORMAL" in capsys.readouterr().out
    assert flight.bridge._pilot_control == "native_landing_failed"
    flight.hold(5.0, alt=1.8, mode="GPS_NORMAL")
    assert flight.sent("dji_setpoint") == []


def test_new_missions_are_refused_while_dji_is_landing(capsys):
    flight = _dji_landing()
    flight.clear()
    _start(flight, "roi")
    flight.tick(alt=2.0, mode="AUTO_LANDING")
    assert flight.bridge.logic.state in {FlightState.LANDING, FlightState.LANDED}
    assert "command=set_roi ignored reason=native_landing_in_progress" in capsys.readouterr().out
    assert flight.sent("dji_setpoint") == []


# -- manual takeover around the native phases -----------------------------------


def test_rc_takeover_during_landing_approach_releases_and_never_reacquires():
    flight = _landing_approach()
    flight.follow(30)  # Virtual Stick descent toward the hold
    assert flight.sent("dji_setpoint")
    flight.clear()
    x, y, z = flight.outcome.position
    flight.tick(alt=z, east=x, north=y, mode="VIRTUAL_STICK", up=0.4)
    assert flight.bridge.logic.state is FlightState.IDLE
    assert flight.bridge._pilot_control is not None
    assert len(flight.sent("dji_release")) == 1
    for _ in range(200):  # pilot centres the sticks and hovers right over home
        _at_hold(flight, mode="GPS_NORMAL")
    assert flight.sent("dji_setpoint") == []
    assert flight.sent("dji_land") == []


def test_rc_takeover_during_dji_landing_stops_us_waiting_but_touchdown_still_resets(capsys):
    flight = _dji_landing()
    flight.clear()
    flight.tick(alt=1.5, mode="AUTO_LANDING", up=0.6)
    assert flight.bridge._native_landing is None
    assert flight.bridge._pilot_control is not None
    assert "landing=native_abandoned reason=pilot_control_rc_stick_up" in capsys.readouterr().out
    flight.hold(3.0, alt=1.5, mode="GPS_NORMAL")
    assert flight.sent("dji_setpoint") == []
    assert flight.sent("dji_land") == []
    flight.hold(LANDING_CONFIRM_SECONDS + 0.3, is_flying=False, alt=0.0)
    assert flight.bridge.logic.state is FlightState.IDLE
    assert flight.bridge._pilot_control is None


def test_after_a_takeover_only_a_new_mission_regains_control():
    flight = _cruising()
    flight.follow(20)
    epoch = flight.sent("dji_setpoint")[-1]["controlEpoch"]
    flight.tick(alt=CRUISE, mode="VIRTUAL_STICK", right=-0.5)
    flight.clear()
    flight.hold(5.0, alt=CRUISE)
    assert flight.sent("dji_setpoint") == []
    _start(flight, "patrol")
    flight.tick(alt=CRUISE)
    assert flight.sent("dji_takeoff") == [], "airborne: no takeoff"
    assert flight.sent("dji_setpoint")[-1]["controlEpoch"] > epoch


# -- unchanged behaviour ---------------------------------------------------------------


def test_pursuit_hold_survives_the_whole_native_lifecycle():
    flight = _on_ground()
    _queue_command(flight.bridge, CommandName.SET_PURSUIT_BLOCK.value, {"blocked": True}, command_id="hold")
    flight.tick(is_flying=False, alt=0.0)
    _start(flight, "patrol")
    flight.tick(is_flying=False, alt=0.0)
    flight.hold(NATIVE_TAKEOFF_STABLE_SECONDS + 0.2, alt=1.2)
    flight.follow(100, until=lambda f: f.bridge.logic.state is FlightState.PATROLLING)
    for _ in range(40):
        x, y, z = flight.outcome.position
        flight.tick(alt=z, east=x, north=y, mode="VIRTUAL_STICK", detection=_confident_detection_message())
    assert not flight.bridge.logic.is_target_tracking
    assert flight.bridge.logic.pursuit_blocked is True
