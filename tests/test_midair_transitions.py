"""Mid-air mission/state changes: the fixes from the mid-air audit.

Bridge side: tablet manual everywhere airborne, CLEAR_ROI after a completed
ROI cycle, a chase from HOLDING, an airborne IDLE never reported as IDLE, and
who flies a safety RTH. Server side: no false touchdown, and the mission
following the aircraft when a forwarded command was refused.
"""

import asyncio
import json

from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from adapters.dji.flight_bridge import DJI_RTH_REQUEST_INTERVAL_SECONDS, NATIVE_TAKEOFF_STABLE_SECONDS
from controllers.autonomous_drone.flight_logic import FlightState
from realtime_server.protocol import encode_message
from realtime_server.server import MISSION_RESYNC_AFTER_S, RealtimeServer
from services.commands import CommandName
from services.gps import DEFAULT_HOME
from services.mission_state import MissionState
from tests.test_dji_flight_bridge import _confident_detection_message, _queue_command
from tests.test_dji_native_takeoff_landing import (
    CRUISE,
    LANDING_APPROACH_HEIGHT_M,
    _cruising,
    _landing_approach,
    _on_ground,
    _start,
)
from tests.test_realtime_server import _dji_telemetry


def _manual(flight, engaged: bool, **telemetry):
    _queue_command(flight.bridge, CommandName.SET_MANUAL_CONTROL.value, {"engaged": engaged},
                   command_id=f"manual-{engaged}")
    return flight.tick(**telemetry)


# -- tablet manual takeover in every airborne state -------------------------------


def test_tablet_manual_during_the_climb_hands_the_aircraft_to_the_pilot():
    flight = _on_ground()
    _start(flight, "patrol")
    flight.tick(is_flying=False, alt=0.0)
    flight.hold(NATIVE_TAKEOFF_STABLE_SECONDS + 0.2, alt=1.2)
    flight.follow(20)
    assert flight.bridge.logic.state is FlightState.TAKING_OFF
    flight.clear()
    x, y, z = flight.outcome.position
    outcome = _manual(flight, True, alt=z, east=x, north=y)
    assert flight.bridge._pilot_control == "tablet_manual"
    assert outcome.state == FlightState.MANUAL.value and outcome.manual_control_active
    assert len(flight.sent("dji_release")) == 1
    flight.hold(3.0, alt=z)
    assert flight.sent("dji_setpoint") == []


def test_tablet_manual_during_the_landing_approach_hands_the_aircraft_to_the_pilot():
    flight = _landing_approach()
    flight.follow(20)
    flight.clear()
    x, y, z = flight.outcome.position
    _manual(flight, True, alt=z, east=x, north=y, mode="VIRTUAL_STICK")
    assert flight.bridge._pilot_control == "tablet_manual"
    flight.hold(5.0, alt=LANDING_APPROACH_HEIGHT_M, mode="GPS_NORMAL")
    assert flight.sent("dji_setpoint") == [] and flight.sent("dji_land") == []


def test_tablet_manual_on_the_ground_does_nothing():
    flight = _on_ground()
    _manual(flight, True, is_flying=False, alt=0.0)
    assert flight.bridge._pilot_control is None


# -- reported state ----------------------------------------------------------------


def test_an_airborne_idle_is_reported_manual_and_a_grounded_one_idle():
    flight = _on_ground()
    assert flight.tick(is_flying=False, alt=0.0).state == FlightState.IDLE.value
    assert flight.tick(alt=8.0).state == FlightState.MANUAL.value  # flown by hand, no mission
    assert flight.bridge.logic.state is FlightState.IDLE
    assert flight.tick(is_flying=False, alt=0.0).state == FlightState.IDLE.value


# -- HOLDING after a completed ROI cycle ---------------------------------------------


def _holding_after_roi():
    flight = _cruising("roi")
    flight.follow(5)
    flight.bridge.logic._complete_roi_coverage(flight.outcome.position)
    assert flight.bridge.logic.state is FlightState.HOLDING
    assert flight.bridge._active_roi_id is not None
    return flight


def test_clear_roi_while_holding_after_the_roi_moves_on_to_patrol():
    flight = _holding_after_roi()
    _queue_command(flight.bridge, CommandName.CLEAR_ROI.value, command_id="clear")
    flight.follow(1)
    assert flight.bridge.logic.state is FlightState.PATROLLING
    assert flight.bridge._active_roi_id is None


def test_a_target_seen_from_the_hold_is_chased_and_the_hold_resumes():
    flight = _holding_after_roi()
    hold = flight.bridge.logic.target
    for _ in range(60):
        x, y, z = flight.outcome.position
        flight.tick(alt=z, east=x, north=y, mode="VIRTUAL_STICK", detection=_confident_detection_message())
    assert flight.bridge.logic.is_target_tracking
    flight.bridge.logic.complete_target_follow(flight.outcome.position)
    flight.follow(2000, until=lambda f: f.bridge.logic.state is FlightState.HOLDING)
    assert flight.bridge.logic.state is FlightState.HOLDING
    assert flight.bridge.logic.target[:2] == hold[:2]


def test_pursuit_hold_still_suppresses_a_chase_from_the_hold():
    flight = _holding_after_roi()
    _queue_command(flight.bridge, CommandName.SET_PURSUIT_BLOCK.value, {"blocked": True}, command_id="hold")
    for _ in range(60):
        x, y, z = flight.outcome.position
        flight.tick(alt=z, east=x, north=y, mode="VIRTUAL_STICK", detection=_confident_detection_message())
    assert flight.bridge.logic.state is FlightState.HOLDING


# -- safety RTH ownership -------------------------------------------------------------


def test_safety_rth_under_pilot_control_asks_dji_at_most_every_five_seconds():
    flight = _cruising()
    flight.follow(5)
    flight.tick(alt=CRUISE, right=-0.5)  # RC takeover
    assert flight.bridge._pilot_control is not None
    flight.clear()
    flight.bridge.realtime_link._commands.put({
        "type": "command", "commandId": "safety", "command": CommandName.RETURN_HOME.value,
        "params": {}, "safetyReason": "low_battery"})
    flight.hold(DJI_RTH_REQUEST_INTERVAL_SECONDS * 2 + 0.5, alt=CRUISE)
    assert len(flight.sent("dji_rth")) == 3
    assert flight.sent("dji_setpoint") == []


def test_a_safety_rth_with_virtual_stick_never_asks_dji_to_go_home():
    flight = _cruising()
    flight.follow(5)
    flight.clear()
    flight.bridge.realtime_link._commands.put({
        "type": "command", "commandId": "safety", "command": CommandName.RETURN_HOME.value,
        "params": {}, "safetyReason": "low_battery"})
    flight.follow(200)
    assert flight.bridge.logic.state in {FlightState.RETURNING_HOME, FlightState.LANDING}
    assert flight.sent("dji_rth") == []
    assert flight.sent("dji_setpoint")


# -- realtime server --------------------------------------------------------------------


async def _drain(ws, timeout=0.15):
    messages = []
    try:
        while True:
            messages.append(json.loads(await asyncio.wait_for(ws.recv(), timeout)))
    except asyncio.TimeoutError:
        return messages


def _run_server_scenario(steps, *, clock=None):
    async def scenario():
        server_state = RealtimeServer()
        if clock is not None:
            server_state._clock = lambda: clock[0]
        async with serve(server_state.handler, "127.0.0.1", 0) as server:
            base = f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}/ws"
            async with connect(f"{base}?role=drone") as drone, connect(f"{base}?role=tablet") as tablet:
                await _drain(tablet)
                results = []
                for kind, arg in steps:
                    if kind == "tick":
                        clock[0] += arg
                        continue
                    if kind == "tele":
                        state, altitude, extra = arg
                        await drone.send(encode_message(_dji_telemetry(
                            lat=DEFAULT_HOME.lat, lng=DEFAULT_HOME.lng, state=state, altitudeM=altitude, **extra)))
                    else:
                        await tablet.send(encode_message({"type": "command", "commandId": kind,
                                                          "command": kind, "params": arg or {}}))
                    replies = await _drain(tablet)
                    await _drain(drone, 0.05)
                    results.append((kind, server_state.mission.state, server_state.mission.safety_rth_active,
                                    [r for r in replies if r.get("type") == "error"]))
                return results
    return asyncio.run(scenario())


def test_a_pilot_flying_after_the_computer_stood_down_is_not_a_touchdown():
    results = _run_server_scenario([
        ("tele", ("IDLE", 0.0, {})), ("start_patrol", None),
        ("tele", ("PATROLLING", 30.0, {})),
        ("tele", ("PATROLLING", 30.0, {"batteryPercent": 15.0})),   # safety RTH latches
        ("tele", ("LANDING", 4.0, {"batteryPercent": 15.0})),
        ("tele", ("MANUAL", 4.0, {"batteryPercent": 15.0, "manualControlActive": True})),
        ("set_roi", {"center": {"lat": DEFAULT_HOME.lat + 0.0005, "lon": DEFAULT_HOME.lng}, "radiusM": 20}),
    ], clock=[0.0])
    _, mission, safety, _ = results[-2]
    assert mission is MissionState.LANDING and safety, "still airborne, latch kept"
    _, _, _, errors = results[-1]
    assert errors and errors[0]["code"] == "invalid_mission_transition"


def test_the_server_follows_the_aircraft_when_it_refuses_a_command():
    clock = [0.0]
    results = _run_server_scenario([
        ("tele", ("IDLE", 0.0, {})), ("start_patrol", None),
        ("tele", ("PATROLLING", 30.0, {})), ("return_home", None),
        ("tele", ("LANDING", 3.0, {})),
        ("start_patrol", None),              # accepted: the approach can be aborted...
        ("tele", ("LANDING", 2.0, {})),      # ...but DJI owned the touchdown
        ("tick", MISSION_RESYNC_AFTER_S + 0.1),
        ("tele", ("LANDING", 1.5, {})),
    ], clock=clock)
    states = [mission for kind, mission, _, _ in results]
    assert MissionState.PATROL in states
    assert states[-1] is MissionState.LANDING


def test_a_just_forwarded_command_is_not_undone_by_older_telemetry():
    clock = [0.0]
    results = _run_server_scenario([
        ("tele", ("IDLE", 0.0, {})), ("start_patrol", None),
        ("tele", ("PATROLLING", 30.0, {})),
        ("set_roi", {"center": {"lat": DEFAULT_HOME.lat + 0.0005, "lon": DEFAULT_HOME.lng}, "radiusM": 20}),
        ("tele", ("PATROLLING", 30.0, {})),  # sample from before the bridge saw the ROI
        ("tick", 0.3),
        ("tele", ("TRANSITING_TO_ROI", 30.0, {})),
    ], clock=clock)
    assert all(mission is MissionState.TRANSIT_TO_ROI for _, mission, _, _ in results[3:])
