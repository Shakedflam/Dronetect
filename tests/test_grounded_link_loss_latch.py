"""Link-loss safety RTH vs. an aircraft that never left the ground.

Field failure (2026-10-13): three takeoffs were refused on the tablet, each
leaving the bridge in TAKING_OFF on the ground for 20 s; Wi-Fi drops during
those waits latched an android_link_lost safety RTH on a grounded aircraft.
Nothing could clear it without a real landing, so the bridge ignored every
new mission and realtime_server re-latched RTH on each one (stage IDLE,
mission RTH). These pin the fix and the cases it must not weaken.
"""

from adapters.dji.flight_bridge import LANDING_CONFIRM_SECONDS, NATIVE_TAKEOFF_STABLE_SECONDS
from controllers.autonomous_drone.flight_logic import FlightState
from realtime_server.server import GROUNDED_LATCH_CLEAR_AFTER_S
from services.commands import CommandName
from services.mission_state import MissionState
from tests.test_dji_flight_bridge import _feed_ground_rc
from tests.test_dji_native_takeoff_landing import CRUISE, DT, _on_ground, _start
from tests.test_midair_transitions import _run_server_scenario

ROI_AWAY = {"center": {"lat": 32.0858, "lon": 34.7818}, "radiusM": 20}


def _android_outage(flight, seconds: float):
    """The tablet<->bridge link is down: no telemetry, no RC state."""
    link = flight.bridge.android_link
    link._connection = None
    link._mark_disconnected()
    for _ in range(round(seconds / DT)):
        flight.now[0] += DT
        flight.bridge.tick(DT, now=flight.now[0])


def _latched(flight):
    return flight.bridge._safety_rth_active


# -- the field failure, end to end ----------------------------------------------------


def test_refused_takeoff_then_link_loss_then_reconnect_accepts_a_new_roi(capsys):
    flight = _on_ground()
    _start(flight, "roi")
    flight.tick(is_flying=False, alt=0.0)
    assert len(flight.sent("dji_takeoff")) == 1          # the tablet refused it
    flight.hold(2.0, is_flying=False, alt=0.0)
    _android_outage(flight, 25.0)                         # Wi-Fi drop mid-wait
    # A takeoff was in flight: the conservative branch still latches.
    assert _latched(flight) and flight.bridge._safety_rth_reason == "android_link_lost"

    # Reconnect, still on the ground: the stale takeoff ends, then the latch.
    outcome = flight.hold(LANDING_CONFIRM_SECONDS + 0.5, is_flying=False, alt=0.0)
    assert not _latched(flight)
    assert flight.bridge.logic.state is FlightState.IDLE and outcome.state == "IDLE"
    assert outcome.safety_rth_active is False
    assert "safety_rth=cleared reason=confirmed_grounded latch=android_link_lost" in capsys.readouterr().out

    # The next ROI is accepted and takes off normally.
    flight.clear()
    _start(flight, "roi")
    flight.tick(is_flying=False, alt=0.0)
    assert flight.bridge.logic.state is FlightState.TAKING_OFF
    assert len(flight.sent("dji_takeoff")) == 1
    flight.hold(NATIVE_TAKEOFF_STABLE_SECONDS + 0.2, alt=1.2)
    assert flight.sent("dji_setpoint")


def test_link_loss_before_the_takeoff_is_sent_cancels_it_without_a_latch(capsys):
    flight = _on_ground()
    _start(flight, "roi")
    flight.bridge.tick(DT, now=flight.now[0])             # accepted, not yet sent
    assert flight.bridge._native_takeoff is not None and not flight.bridge._native_takeoff.sent
    _android_outage(flight, 4.0)
    assert not _latched(flight)
    assert flight.bridge.logic.state is FlightState.IDLE
    assert flight.bridge._native_takeoff is None
    assert "failsafe=link_lost_on_ground reason=android_link_lost action=mission_cancelled" in capsys.readouterr().out
    flight.hold(1.0, is_flying=False, alt=0.0)
    assert flight.sent("dji_takeoff") == []               # never sent after reconnecting


def test_server_follows_the_bridge_out_of_the_stuck_rth():
    """realtime_server side of the same sequence, with the bridge's reports."""
    clock = [0.0]
    grounded = ("IDLE", 0.0, {})
    latched = ("IDLE", 0.0, {"aircraftSafetyRthActive": True, "aircraftSafetyRthReason": "android_link_lost"})
    results = _run_server_scenario([
        ("tele", grounded), ("set_roi", ROI_AWAY), ("tele", ("TAKING_OFF", 0.0, {})),
        ("tele", latched),                       # takeoff failed, bridge latched on the ground
        ("set_roi", ROI_AWAY), ("tele", latched),     # the field symptom: stage IDLE, mission RTH
        ("set_roi", ROI_AWAY),                   # refused while latched
        ("tele", grounded), ("tick", 1.0), ("tele", grounded),   # bridge cleared; 1 s: not yet
        ("tick", GROUNDED_LATCH_CLEAR_AFTER_S), ("tele", grounded),
        ("set_roi", ROI_AWAY),
    ], clock=clock)
    by_step = [(kind, mission, safety, errors) for kind, mission, safety, errors in results]
    assert by_step[5][1] is MissionState.RTH and by_step[5][2]          # stage IDLE, mission RTH
    assert by_step[6][3] and by_step[6][3][0]["code"] == "invalid_mission_transition"
    assert by_step[8][2] is True                                        # 1 s grounded: kept
    assert by_step[9][1] is MissionState.ON_GROUND and not by_step[9][2]
    assert by_step[10][1] is MissionState.TAKING_OFF and not by_step[10][3]


# -- must not weaken: airborne cases ---------------------------------------------------


def _climbing():
    flight = _on_ground()
    _start(flight, "patrol")
    flight.tick(is_flying=False, alt=0.0)
    flight.hold(NATIVE_TAKEOFF_STABLE_SECONDS + 0.2, alt=1.2)
    flight.follow(40)
    assert flight.bridge.logic.state is FlightState.TAKING_OFF
    assert flight.outcome.position[2] > 1.5
    return flight


def test_link_loss_during_an_actual_climb_latches_and_stays_latched_in_the_air():
    flight = _climbing()
    _android_outage(flight, 4.0)
    assert _latched(flight) and flight.bridge.logic.state is FlightState.RETURNING_HOME
    flight.hold(10.0, alt=8.0, mode="VIRTUAL_STICK")      # reconnected, still flying
    assert _latched(flight)


def test_gps_dropout_then_link_loss_still_latches_from_rc_flight_state():
    """Telemetry needs a GPS fix; RC state does not. Flying with no GPS, the
    last isFlying comes from RC state -- and it says airborne."""
    flight = _climbing()
    for _ in range(round(2.0 / DT)):                      # GPS lost: RC state only
        flight.now[0] += DT
        _feed_ground_rc(flight.bridge, flight.now, is_flying=True)
        flight.bridge.tick(DT, now=flight.now[0])
    _android_outage(flight, 4.0)
    assert _latched(flight)


def test_unknown_flight_state_is_never_taken_for_grounded():
    flight = _on_ground()
    _start(flight, "patrol")
    flight.tick(is_flying=False, alt=0.0)
    flight.hold(NATIVE_TAKEOFF_STABLE_SECONDS + 0.2, alt=1.2)
    flight.follow(20)
    flight.tick(is_flying=None, alt=5.0)                  # last sample: isFlying unknown
    _android_outage(flight, 4.0)
    assert _latched(flight)


def test_a_genuine_low_battery_rth_is_not_cleared_in_the_air():
    flight = _climbing()
    flight.bridge.realtime_link._commands.put({
        "type": "command", "commandId": "low", "command": CommandName.RETURN_HOME.value,
        "params": {}, "safetyReason": "low_battery"})
    flight.follow(400)                                    # flying home, 20 s
    assert _latched(flight) and flight.bridge._safety_rth_reason == "low_battery"
    # One glitched grounded sample mid-air does not clear it either.
    x, y, z = flight.outcome.position
    flight.tick(is_flying=False, alt=z, east=x, north=y)
    flight.follow(40)
    assert _latched(flight)


def test_a_grounded_latch_needs_unbroken_fresh_grounded_samples():
    flight = _on_ground()
    _start(flight, "roi")
    flight.tick(is_flying=False, alt=0.0)
    _android_outage(flight, 25.0)
    assert _latched(flight)
    for _ in range(round(4.0 / DT)):                      # flickering unknown state
        flight.tick(is_flying=None, alt=0.0)
        flight.tick(is_flying=False, alt=0.0)
    assert _latched(flight)
    for _ in range(4):                                    # stale gaps between samples
        flight.tick(is_flying=False, alt=0.0)
        flight.now[0] += 1.5
        flight.bridge.tick(DT, now=flight.now[0])
    assert _latched(flight)
    flight.hold(LANDING_CONFIRM_SECONDS + 0.2, is_flying=False, alt=0.0)
    assert not _latched(flight)


def test_server_keeps_a_latch_while_the_aircraft_reports_flying_or_its_own_rth():
    clock = [0.0]
    low = {"batteryPercent": 15.0}
    results = _run_server_scenario([
        ("tele", ("IDLE", 0.0, {})), ("start_patrol", None),
        ("tele", ("PATROLLING", CRUISE, {})),
        ("tele", ("PATROLLING", CRUISE, low)),            # low battery: latched
        ("tick", 5.0), ("tele", ("RETURNING_HOME", CRUISE, low)),
        ("tick", 5.0), ("tele", ("MANUAL", 4.0, low)),    # pilot flying it home
        ("tick", 5.0), ("tele", ("MANUAL", 4.0, {**low, "aircraftSafetyRthActive": True})),
    ], clock=clock)
    assert all(safety for _, _, safety, _ in results[3:])
