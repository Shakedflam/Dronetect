"""Tests for DjiFlightBridge: command dispatch, tick gating, and fail-safes.

These drive DjiFlightBridge.tick() directly with a fake clock and manually
-populated AndroidLink/realtime_link state, the same style
tests/test_flight_logic.py and tests/test_manual_control.py already use for
the underlying pure autonomy modules -- no real sockets are needed to
exercise the bridge's own logic.
"""

import asyncio
import json
import math

import pytest

from adapters.dji.android_link import AndroidLink
from adapters.dji.flight_bridge import BridgeConfig, DjiFlightBridge, TickOutcome
from adapters.dji.local_protocol import AndroidRcState, AndroidTelemetry, build_android_rc_state_message, build_android_telemetry_message
from adapters.simulator.client import SimulatorRealtimeClient
from config.shared_config import OperatingConfig
from controllers.autonomous_drone.flight_logic import FlightState
from realtime_server.protocol import encode_message
from realtime_server.protocol import validate_telemetry_message
from services.commands import CommandName
from services.gps import DEFAULT_HOME, LocalPosition, Wgs84Point, destination_point, local_meters_to_wgs84, wgs84_distance_m

# Real-world-scale, matching config/shared_config.py's own default_operating_
# config (radius) with the altitude already used throughout this file's
# telemetry fixtures. Deliberately NOT FlightConfig()'s simulator-scale
# defaults -- these tests exist specifically to prove the bridge behaves
# under real-flight-shaped configuration, not the 8x8 m test-arena box.
TEST_OPERATING_CONFIG = OperatingConfig(max_flight_radius_m=300.0, cruise_altitude_m=15.0)


def _make_bridge(now: list[float], *, operating_config: OperatingConfig | None = TEST_OPERATING_CONFIG) -> DjiFlightBridge:
    android_link = AndroidLink(clock=lambda: now[0])
    realtime_link = SimulatorRealtimeClient(clock=lambda: now[0])
    return DjiFlightBridge(
        realtime_link=realtime_link,
        android_link=android_link,
        operating_config=operating_config,
        bridge_config=BridgeConfig(
            android_telemetry_timeout_s=1.0,
            android_connection_loss_rth_s=3.0,
            realtime_connection_loss_rth_s=3.0,
        ),
        clock=lambda: now[0],
    )


def _feed_telemetry(bridge: DjiFlightBridge, now: list[float], **overrides) -> None:
    # is_flying=False by default: session_home (and therefore any position at
    # all -- see DjiFlightBridge._maybe_establish_session_home) is only ever
    # captured from a telemetry sample explicitly reporting the aircraft as
    # grounded, so most fixtures here need "grounded, complete telemetry" as
    # their baseline. Tests exercising airborne/unknown telemetry override it.
    defaults = dict(
        timestamp=now[0],
        lat=DEFAULT_HOME.lat,
        lng=DEFAULT_HOME.lng,
        altitude_m=15.0,
        heading_deg=0.0,
        speed_mps=0.0,
        battery_percent=90.0,
        gimbal_pitch_deg=0.0,
        flight_mode="GPS_ATTI",
        is_flying=False,
    )
    defaults.update(overrides)
    bridge.android_link._connection = object()
    bridge.android_link._mark_connected()
    bridge.android_link._handle_message(
        json.dumps(build_android_telemetry_message(AndroidTelemetry(**defaults)))
    )
    bridge.android_link._telemetry_received_monotonic = now[0]


def _feed_ground_rc(bridge: DjiFlightBridge, now: list[float], **overrides) -> None:
    defaults = dict(timestamp=now[0], connected=True, forward=0.0, right=0.0,
                    up=0.0, yaw_rate=0.0, aircraft_connected=True, is_flying=False)
    defaults.update(overrides)
    bridge.android_link._handle_message(json.dumps(
        build_android_rc_state_message(AndroidRcState(**defaults))))


def _queue_command(bridge: DjiFlightBridge, command: str, params: dict | None = None, command_id: str = "cmd-1") -> None:
    bridge.realtime_link._commands.put(
        {"type": "command", "commandId": command_id, "command": command, "params": params or {}}
    )


def test_phase1_uses_measured_dji_state_and_never_sends_aircraft_commands(monkeypatch):
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "1")
    now = [100.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now, heading_deg=97.8, speed_mps=0.3,
                    battery_percent=64.0, gimbal_pitch_deg=-15.0, is_flying=False)
    _queue_command(bridge, CommandName.START_PATROL.value)
    outcome = asyncio.run(bridge.run_tick_once())
    message = bridge.build_telemetry_message(outcome)
    assert validate_telemetry_message(message)["source"] == "dji"
    assert outcome.has_position
    assert bridge.logic.state is FlightState.IDLE
    assert message["data"]["gps"] == {"lat": DEFAULT_HOME.lat, "lng": DEFAULT_HOME.lng}
    assert message["data"]["headingDeg"] == 97.8
    assert message["data"]["speedMps"] == 0.3
    assert message["data"]["state"] == "LANDED"
    assert math.isclose(bridge._last_yaw, math.radians(90 - 97.8))
    assert math.isclose(bridge.camera_gimbal.pitch, math.radians(15))
    assert not bridge.realtime_link.drain_commands()


# -- phase1_telemetry_only: read-only targetFollowDiagnostics display -------
# See DjiFlightBridge._update_phase1_detection_diagnostics/_build_phase1_
# target_follow_diagnostics. These tests use _confident_detection_message
# (defined further below, alongside the legacy-path mission-interruption
# tests that also use it).


def test_phase1_no_detections_drained_keeps_target_follow_diagnostics_none(monkeypatch):
    """No regression: with nothing drained this tick, the telemetry field
    stays exactly None, as it always has."""
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "1")
    now = [100.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now, is_flying=False)

    outcome = asyncio.run(bridge.run_tick_once())
    message = bridge.build_telemetry_message(outcome)

    assert message["data"]["targetFollowDiagnostics"] is None
    assert validate_telemetry_message(message)["data"]["targetFollowDiagnostics"] is None


def test_phase1_confirmed_detection_populates_valid_target_follow_diagnostics(monkeypatch):
    """A confirmed detection drained this tick produces a wire-valid
    targetFollowDiagnostics object, with the right trackerId, and with
    pursuit/targetDecision omitted (both optional in the schema) rather
    than fabricated."""
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "1")
    now = [100.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now, is_flying=False)
    bridge.realtime_link._detections.put(_confident_detection_message())

    outcome = asyncio.run(bridge.run_tick_once())
    message = bridge.build_telemetry_message(outcome)
    validated = validate_telemetry_message(message)

    assert message["data"]["state"] == "LANDED"
    diagnostics = validated["data"]["targetFollowDiagnostics"]
    assert diagnostics is not None
    assert diagnostics["trackerId"] == "one"
    assert diagnostics["detectionConfidence"] == 0.9
    assert diagnostics["flightState"] == "LANDED"
    assert "pursuit" not in diagnostics
    assert "targetDecision" not in diagnostics
    assert diagnostics["patrolInterrupted"] is False
    assert diagnostics["patrolResumeState"] is None
    assert diagnostics["patrolResumeStatus"] == "inactive"
    assert diagnostics["altitudeMatchingActive"] is False
    assert diagnostics["confirmation"] == {"hits": 1, "requiredHits": 1, "progress": 1.0}
    assert math.isclose(diagnostics["boundingBoxAreaRatio"], 0.05, rel_tol=1e-6)
    assert math.isclose(diagnostics["boundingBoxScale"], math.sqrt(0.05), rel_tol=1e-6)
    assert diagnostics["imageCenterError"] == {"x": 0.0, "y": 0.0}  # box is frame-centered
    assert diagnostics["estimatedImageVelocity"] == {
        "x": 0.0, "y": 0.0, "magnitude": 0.0, "directionDeg": None,
    }


def test_phase1_detection_diagnostics_expire_after_ttl(monkeypatch):
    """Freshness rule: once nothing is drained, the last detection keeps
    being reported briefly (surviving the tick/publish-throttle gap), then
    falls back to None once config.target_follow_diagnostics_ttl_s elapses."""
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "1")
    now = [100.0]
    bridge = _make_bridge(now)
    bridge.config = BridgeConfig(
        android_telemetry_timeout_s=bridge.config.android_telemetry_timeout_s,
        android_connection_loss_rth_s=bridge.config.android_connection_loss_rth_s,
        realtime_connection_loss_rth_s=bridge.config.realtime_connection_loss_rth_s,
        target_follow_diagnostics_ttl_s=0.2,
    )
    _feed_telemetry(bridge, now, is_flying=False)
    bridge.realtime_link._detections.put(_confident_detection_message())
    outcome = asyncio.run(bridge.run_tick_once())
    assert bridge.build_telemetry_message(outcome)["data"]["targetFollowDiagnostics"] is not None

    now[0] += 0.1  # still within the TTL, nothing new drained
    _feed_telemetry(bridge, now, is_flying=False)
    outcome = asyncio.run(bridge.run_tick_once())
    assert bridge.build_telemetry_message(outcome)["data"]["targetFollowDiagnostics"] is not None

    now[0] += 0.3  # now past the TTL
    _feed_telemetry(bridge, now, is_flying=False)
    outcome = asyncio.run(bridge.run_tick_once())
    assert bridge.build_telemetry_message(outcome)["data"]["targetFollowDiagnostics"] is None


def test_phase1_detection_diagnostics_do_not_affect_flight_control(monkeypatch):
    """The critical read-only claim: feeding a confirmed detection through
    phase1's tick()/run_tick_once() must leave FlightLogic.state/target,
    patrol bookkeeping, and everything sent to android_link byte-for-byte
    identical to a scenario where no detection was ever fed -- proving the
    new targetFollowDiagnostics bookkeeping is a pure display computation
    with zero coupling to flight control."""
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "1")
    monkeypatch.setenv("DRONTECT_DJI_PHASE2_ACTIONS", "1")

    async def _fail_if_called(*args, **kwargs):
        raise AssertionError("send_setpoint must never be called in phase1_telemetry_only mode")

    monkeypatch.setattr(AndroidLink, "send_setpoint", _fail_if_called)

    class RecordingConnection:
        def __init__(self):
            self.messages = []

        async def send(self, raw):
            self.messages.append(json.loads(raw))

    def run_scenario(feed_detection: bool):
        now = [100.0]
        bridge = _make_bridge(now)
        bridge.realtime_link._mark_connected()
        connection = RecordingConnection()
        for _ in range(5):
            if feed_detection:
                bridge.realtime_link._detections.put(_confident_detection_message())
            now[0] += 0.05
            _feed_telemetry(bridge, now, is_flying=False)
            bridge.android_link._connection = connection
            asyncio.run(bridge.run_tick_once())
        return bridge, connection

    baseline_bridge, baseline_connection = run_scenario(feed_detection=False)
    detected_bridge, detected_connection = run_scenario(feed_detection=True)

    # The detection was in fact seen and recorded for display -- otherwise
    # the comparison below would be vacuous.
    assert detected_bridge._phase1_detection_snapshot is not None
    assert baseline_bridge._phase1_detection_snapshot is None

    assert detected_bridge.logic.state is baseline_bridge.logic.state
    assert detected_bridge.logic.target == baseline_bridge.logic.target
    assert detected_bridge._target_patrol_region_id == baseline_bridge._target_patrol_region_id
    assert detected_bridge._paused_patrol_coverage == baseline_bridge._paused_patrol_coverage
    assert detected_bridge._active_roi_id == baseline_bridge._active_roi_id
    assert detected_bridge._mission_interruption == baseline_bridge._mission_interruption
    assert detected_connection.messages == baseline_connection.messages == []


def test_phase2_sends_only_one_gimbal_or_rth_action_and_never_setpoints(monkeypatch):
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "1")
    monkeypatch.setenv("DRONTECT_DJI_PHASE2_ACTIONS", "1")
    now = [100.0]
    bridge = _make_bridge(now)
    bridge.realtime_link._mark_connected()

    class RecordingConnection:
        def __init__(self):
            self.messages = []

        async def send(self, raw):
            self.messages.append(json.loads(raw))

    connection = RecordingConnection()
    _feed_telemetry(bridge, now, gimbal_pitch_deg=-15.0, is_flying=False)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())  # establishes the hardware session

    _queue_command(bridge, CommandName.CAMERA_TILT_UP.value)
    now[0] += 0.05
    _feed_telemetry(bridge, now, gimbal_pitch_deg=-15.0, is_flying=False)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())
    assert [message["type"] for message in connection.messages] == ["dji_gimbal_command"]
    assert connection.messages[0]["ttlMs"] == bridge.config.gimbal_command_ttl_ms

    _queue_command(bridge, CommandName.RETURN_HOME.value)
    now[0] += 0.05
    _feed_telemetry(bridge, now, gimbal_pitch_deg=-15.0, is_flying=False)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())
    assert [message["type"] for message in connection.messages] == ["dji_gimbal_command", "dji_rth"]
    assert connection.messages[1]["seq"] > connection.messages[0]["seq"]

    _queue_command(bridge, CommandName.START_PATROL.value)
    now[0] += 0.05
    _feed_telemetry(bridge, now, gimbal_pitch_deg=-15.0, is_flying=False)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())
    assert len(connection.messages) == 2


def test_phase3_emits_only_zero_setpoints_while_grounded_configured_and_connected(monkeypatch):
    monkeypatch.setenv("DRONTECT_DJI_PHASE3_ZERO_STICK", "1")
    now = [100.0]
    bridge = _make_bridge(now)
    bridge.realtime_link._mark_connected()

    class RecordingConnection:
        def __init__(self):
            self.messages = []

        async def send(self, raw):
            self.messages.append(json.loads(raw))

    connection = RecordingConnection()
    _feed_telemetry(bridge, now, is_flying=False)
    _feed_ground_rc(bridge, now)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())
    assert bridge.phase1_telemetry_only
    assert [m["type"] for m in connection.messages] == ["dji_setpoint"]
    command = connection.messages[0]
    assert command["velocity"] == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert command["sessionId"] == bridge.android_link._session_id
    _queue_command(bridge, CommandName.START_PATROL.value)
    now[0] += 0.05
    _feed_telemetry(bridge, now, is_flying=False)
    _feed_ground_rc(bridge, now)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())
    assert bridge.logic.state is FlightState.IDLE
    assert all(m["velocity"] == {"x": 0.0, "y": 0.0, "z": 0.0} for m in connection.messages)
    now[0] += 0.05
    _feed_telemetry(bridge, now, is_flying=True)
    _feed_ground_rc(bridge, now, is_flying=True)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())
    assert len(connection.messages) == 2
    bridge.realtime_link._mark_disconnected()
    now[0] += 0.05
    _feed_telemetry(bridge, now, is_flying=False)
    _feed_ground_rc(bridge, now)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())
    assert len(connection.messages) == 2


def test_phase3_zero_heartbeat_survives_gps_gap_but_not_ground_safety_gap(monkeypatch):
    monkeypatch.setenv("DRONTECT_DJI_PHASE3_ZERO_STICK", "1")
    now = [100.0]
    bridge = _make_bridge(now)
    bridge.realtime_link._mark_connected()

    class RecordingConnection:
        def __init__(self):
            self.messages = []

        async def send(self, raw):
            self.messages.append(json.loads(raw))

    connection = RecordingConnection()
    _feed_telemetry(bridge, now)
    _feed_ground_rc(bridge, now)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())
    assert len(connection.messages) == 1

    now[0] += 1.5  # GPS/position is stale, but grounded RC safety is current.
    _feed_ground_rc(bridge, now)
    outcome = asyncio.run(bridge.run_tick_once())
    assert not outcome.has_position
    assert len(connection.messages) == 2
    assert all(message["velocity"] == {"x": 0.0, "y": 0.0, "z": 0.0}
               for message in connection.messages)

    now[0] += 0.05
    _feed_ground_rc(bridge, now, right=0.1)
    asyncio.run(bridge.run_tick_once())
    assert len(connection.messages) == 2

    now[0] += 0.05
    _feed_ground_rc(bridge, now, is_flying=True)
    asyncio.run(bridge.run_tick_once())
    assert len(connection.messages) == 2

    now[0] += 1.1  # Even a last-known safe state cannot keep control alive.
    asyncio.run(bridge.run_tick_once())
    assert len(connection.messages) == 2


def test_phase2_applies_ui_operating_config_before_hardware_gps(monkeypatch):
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "1")
    monkeypatch.setenv("DRONTECT_DJI_PHASE2_ACTIONS", "1")
    now = [100.0]
    bridge = _make_bridge(now, operating_config=None)
    bridge.realtime_link._mark_connected()
    _queue_command(bridge, CommandName.SET_OPERATING_CONFIG.value,
        {"maxFlightRadiusM": 20.0, "cruiseAltitudeM": 1.5}, command_id="ui-config")

    outcome = asyncio.run(bridge.run_tick_once())

    assert not outcome.has_position
    assert bridge.is_configured_for_flight
    assert bridge.operating_config == OperatingConfig(20.0, 1.5)
    assert bridge.logic.config.takeoff_altitude == 1.5
    result = bridge.realtime_link._control_outgoing.get_nowait()
    assert result["commandId"] == "ui-config"
    assert result["accepted"] is True
    assert result["pending"] is False


def test_phase2_flag_cannot_select_legacy_setpoint_path(monkeypatch):
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "0")
    monkeypatch.setenv("DRONTECT_DJI_PHASE2_ACTIONS", "1")
    bridge = _make_bridge([100.0], operating_config=None)
    assert bridge.phase1_telemetry_only
    assert bridge.phase2_actions


def test_phase2_hardware_reconnect_drops_actions_but_preserves_config_sync(monkeypatch):
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "1")
    monkeypatch.setenv("DRONTECT_DJI_PHASE2_ACTIONS", "1")
    now = [100.0]
    bridge = _make_bridge(now, operating_config=None)
    bridge.realtime_link._mark_connected()
    _feed_telemetry(bridge, now, gimbal_pitch_deg=-15.0)
    _queue_command(bridge, CommandName.CAMERA_TILT_UP.value, command_id="old-action")
    _queue_command(bridge, CommandName.SET_OPERATING_CONFIG.value,
        {"maxFlightRadiusM": 20.0, "cruiseAltitudeM": 1.5}, command_id="sync")

    bridge.tick(0.05, now=now[0])

    assert bridge.is_configured_for_flight
    assert bridge.realtime_link._control_outgoing.get_nowait()["commandId"] == "sync"
    assert bridge._pending_phase2_actions == []


def _make_airborne(bridge: DjiFlightBridge, now: list[float]) -> None:
    """Get FlightLogic out of IDLE (request_return_home/engage_manual_control
    both refuse a parked aircraft) via a plain START_PATROL. With telemetry
    fed above FlightConfig's default takeoff altitude, this lands directly in
    PATROLLING with no TAKING_OFF phase to wait out."""
    _queue_command(bridge, CommandName.START_PATROL.value, command_id="airborne-setup")
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    outcome = bridge.tick(0.05, now=now[0])
    assert outcome.state == FlightState.PATROLLING.value


def _fly_ticks(bridge: DjiFlightBridge, now: list[float], count: int, *, dt: float = 0.05) -> TickOutcome:
    """Tick the bridge `count` times with *closed-loop* synthetic telemetry:
    each tick's Android telemetry reports wherever the previous tick's
    TickOutcome actually flew to, rather than a fixed static position. A
    fixed-position feed (as a plain repeated _feed_telemetry call gives) lets
    velocity/yaw evolve correctly but never lets simulated position
    accumulate, so any test that needs the aircraft to actually *arrive*
    somewhere (a "reached target" transition) must use this instead."""
    outcome: TickOutcome | None = None
    for _ in range(count):
        now[0] += dt
        if outcome is not None and outcome.has_position:
            assert outcome.position is not None and outcome.yaw_rad is not None
            local = LocalPosition(x=outcome.position[0], y=outcome.position[1], z=outcome.position[2])
            gps = local_meters_to_wgs84(local)
            heading_deg = (math.degrees(outcome.yaw_rad) + 360.0) % 360.0
            _feed_telemetry(bridge, now, lat=gps.lat, lng=gps.lng, altitude_m=outcome.position[2], heading_deg=heading_deg)
        else:
            _feed_telemetry(bridge, now)
        outcome = bridge.tick(dt, now=now[0])
    assert outcome is not None
    return outcome


# -- fail-safe: never advance flight logic on stale/absent telemetry --------


def test_tick_without_any_telemetry_holds_and_does_not_touch_flight_logic():
    now = [0.0]
    bridge = _make_bridge(now)

    outcome = bridge.tick(0.05, now=now[0])

    assert outcome.has_position is False
    assert bridge.logic.home is None


def test_tick_freezes_once_telemetry_goes_stale_even_with_a_prior_fix():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)

    outcome = bridge.tick(0.05, now=now[0])
    assert outcome.has_position is True
    position_after_first_tick = bridge.logic.target

    now[0] = 5.0  # telemetry is now far stale (timeout is 1.0s)
    outcome = bridge.tick(0.05, now=now[0])

    assert outcome.has_position is False
    # FlightLogic must not have advanced against the stale position.
    assert bridge.logic.target == position_after_first_tick


def test_setpoint_ttl_and_gate_are_independent_of_flight_logic_state():
    """AndroidLink itself refuses a stale send regardless of what the bridge
    computed -- covered directly in test_dji_android_link.py. This test just
    confirms the bridge's TickOutcome carries has_position=False through to
    run_tick_once territory (i.e. there's nothing to send) rather than a
    velocity computed from stale data."""
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    now[0] = 10.0
    outcome = bridge.tick(0.05, now=now[0])

    assert outcome.has_position is False
    assert outcome.velocity_x_mps == 0.0
    assert outcome.velocity_y_mps == 0.0
    assert outcome.velocity_z_mps == 0.0


# -- command dispatch ---------------------------------------------------------


def test_set_roi_transitions_flight_state_toward_the_roi():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now, altitude_m=15.0)
    bridge.tick(0.05, now=now[0])
    assert bridge.logic.home is not None

    roi_point = destination_point(DEFAULT_HOME, 100.0, 0.0)
    _queue_command(
        bridge,
        CommandName.SET_ROI.value,
        {"center": {"lat": roi_point.lat, "lon": roi_point.lng}, "radiusM": 20.0},
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now, altitude_m=15.0)
    outcome = bridge.tick(0.05, now=now[0])

    assert outcome.state in {FlightState.TRANSITING_TO_ROI.value, FlightState.TAKING_OFF.value}


def test_return_home_sets_safety_rth_when_safety_reason_present():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _make_airborne(bridge, now)

    _queue_command(bridge, CommandName.RETURN_HOME.value, command_id="rth-1")
    bridge.realtime_link._commands.queue[-1]["safetyReason"] = "low_battery"
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    outcome = bridge.tick(0.05, now=now[0])

    assert bridge._safety_rth_active is True
    assert bridge._safety_rth_reason == "low_battery"
    # Home is close and cruise altitude shallow in this fixture, so a single
    # 0.05s tick can carry the aircraft from RETURNING_HOME straight into
    # LANDING -- both are evidence request_return_home() actually engaged.
    assert outcome.state in {FlightState.RETURNING_HOME.value, FlightState.LANDING.value}


def test_manual_control_engage_and_release_round_trip():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _make_airborne(bridge, now)

    _queue_command(bridge, CommandName.SET_MANUAL_CONTROL.value, {"engaged": True}, command_id="mc-1")
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    outcome = bridge.tick(0.05, now=now[0])
    assert outcome.state == FlightState.MANUAL.value
    assert outcome.manual_control_active is True

    _queue_command(bridge, CommandName.SET_MANUAL_CONTROL.value, {"engaged": False}, command_id="mc-2")
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    outcome = bridge.tick(0.05, now=now[0])
    assert outcome.manual_control_active is False


def test_manual_stick_command_is_forwarded_while_manual_and_rc_not_connected():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _make_airborne(bridge, now)
    _queue_command(bridge, CommandName.SET_MANUAL_CONTROL.value, {"engaged": True}, command_id="mc-1")
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    _queue_command(
        bridge,
        CommandName.SET_MANUAL_COMMAND.value,
        {"forward": 0.5, "right": 0.0, "up": 0.0, "yawRate": 0.0},
        command_id="stick-1",
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    assert bridge.logic.manual_stick.forward == 0.5


def test_rc_stick_outranks_tablet_stick_when_rc_connected():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _make_airborne(bridge, now)
    _queue_command(bridge, CommandName.SET_MANUAL_CONTROL.value, {"engaged": True}, command_id="mc-1")
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    # Tablet asks for full-forward...
    _queue_command(
        bridge, CommandName.SET_MANUAL_COMMAND.value, {"forward": 1.0, "right": 0.0, "up": 0.0, "yawRate": 0.0}
    )
    # ...but the RC-N3 is connected and holding a centered stick.
    bridge.android_link._latest_rc_state = AndroidRcState(
        timestamp=now[0], connected=True, forward=0.0, right=0.0, up=0.0, yaw_rate=0.0
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    assert bridge.manual_input.source == "rc"
    assert bridge.logic.manual_stick.forward == 0.0


def test_camera_tilt_commands_move_the_gimbal():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    initial_pitch = bridge.camera_gimbal.pitch

    _queue_command(bridge, CommandName.CAMERA_TILT_DOWN.value)
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    assert bridge.camera_gimbal.pitch > initial_pitch


def test_unknown_command_params_do_not_crash_the_bridge():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    # Malformed params (missing required key) for a command that reads them
    # directly -- _safe_dispatch_command must swallow this, not raise.
    _queue_command(bridge, CommandName.SET_ROI.value, {"center": {"lat": 0.0}})  # missing lon/radiusM
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    outcome = bridge.tick(0.05, now=now[0])

    assert outcome.has_position is True  # bridge kept running


# -- fail-safes: connection loss triggers RTH --------------------------------


def test_android_link_loss_triggers_rth_while_airborne():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _queue_command(bridge, CommandName.START_PATROL.value)
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    assert bridge.logic.state is not FlightState.IDLE

    # Simulate the Android link dropping: disconnect, then let enough time
    # pass for the connection-loss threshold.
    bridge.android_link._connection = None
    bridge.android_link._mark_disconnected()
    now[0] += 4.0  # past android_connection_loss_rth_s=3.0

    outcome = bridge.tick(0.05, now=now[0])

    assert bridge._safety_rth_active is True
    assert bridge._safety_rth_reason == "android_link_lost"


def test_realtime_link_loss_triggers_rth_while_airborne():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _queue_command(bridge, CommandName.START_PATROL.value)
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    bridge.realtime_link._mark_connected()
    bridge.realtime_link._mark_disconnected()
    now[0] += 4.0  # past realtime_connection_loss_rth_s=3.0
    _feed_telemetry(bridge, now)

    outcome = bridge.tick(0.05, now=now[0])

    assert bridge._safety_rth_active is True
    assert bridge._safety_rth_reason == "connection_lost"


def test_no_failsafe_trigger_while_still_on_the_ground():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    assert bridge.logic.state is FlightState.IDLE

    bridge.android_link._connection = None
    bridge.android_link._mark_disconnected()
    now[0] += 10.0

    bridge.tick(0.05, now=now[0])

    assert bridge._safety_rth_active is False


# -- real-flight configuration gate ------------------------------------------


def test_unconfigured_bridge_refuses_all_movement_commands():
    now = [0.0]
    bridge = _make_bridge(now, operating_config=None)
    assert bridge.is_configured_for_flight is False
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    _queue_command(bridge, CommandName.START_PATROL.value)
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    outcome = bridge.tick(0.05, now=now[0])

    # No FlightLogic advancement at all -- has_position stays False even
    # though telemetry is fresh, because the bridge is not configured.
    assert outcome.has_position is False
    assert bridge.logic.state is FlightState.IDLE


def test_set_operating_config_bootstraps_an_unconfigured_bridge():
    now = [0.0]
    bridge = _make_bridge(now, operating_config=None)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    _queue_command(
        bridge,
        CommandName.SET_OPERATING_CONFIG.value,
        {"maxFlightRadiusM": 300.0, "cruiseAltitudeM": 20.0},
        command_id="config-1",
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    assert bridge.is_configured_for_flight is True
    assert bridge.operating_config.max_flight_radius_m == 300.0
    assert bridge.operating_config.cruise_altitude_m == 20.0
    assert bridge.logic.config.takeoff_altitude == 20.0

    # Now that it's configured, a mission command actually moves the state.
    _queue_command(bridge, CommandName.START_PATROL.value, command_id="patrol-1")
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    outcome = bridge.tick(0.05, now=now[0])
    assert outcome.has_position is True
    assert outcome.state != FlightState.IDLE.value


def test_set_operating_config_rejects_params_over_the_legal_maximum():
    now = [0.0]
    bridge = _make_bridge(now, operating_config=None)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    _queue_command(
        bridge,
        CommandName.SET_OPERATING_CONFIG.value,
        {"maxFlightRadiusM": 999_999.0, "cruiseAltitudeM": 20.0},  # exceeds shared_config's legal maximum
        command_id="bad-config",
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    # OperatingConfig's own validation (config/shared_config.py, reused
    # unmodified) rejected it -- the bridge stays unconfigured rather than
    # silently accepting an out-of-bounds value.
    assert bridge.is_configured_for_flight is False


# -- region-based PATROL: waypoint provider parity ---------------------------


def test_start_patrol_selects_a_real_region_and_a_multi_waypoint_sweep():
    """Closes the gap the previous foundation left open: a bare START_PATROL
    must fly a real lawn-mower coverage sweep, not hover at the current
    position (patrol_waypoint_provider was previously unwired)."""
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    _make_airborne(bridge, now)  # issues START_PATROL and ticks once

    assert bridge._target_patrol_region_id is not None
    first_target = bridge.logic.target
    assert first_target is not None
    # The aircraft is not simply holding at its own position (the previous
    # foundation's fallback behavior) -- it picked a real coverage waypoint.
    assert first_target[:2] != (0.0, 0.0)
    # A lawn-mower sweep of a whole region queues more than one leg.
    assert len(bridge._patrol_route_waypoints) > 0


def test_patrol_priority_edit_replans_the_active_sweep():
    """Mirrors autonomous_drone.py's _replan_active_patrol: an operator's
    mid-patrol priority edit must take effect immediately, not only once the
    aircraft happens to finish whatever region it already committed to."""
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _make_airborne(bridge, now)
    original_region = bridge._target_patrol_region_id
    assert original_region is not None

    other_region_id = next(
        region.id for region in bridge.patrol_regions.regions if region.id != original_region
    )
    _queue_command(
        bridge,
        CommandName.SET_PATROL_PRIORITIES.value,
        {"priorities": [{"regionId": other_region_id, "rank": 1}]},
        command_id="priorities-1",
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    assert bridge._target_patrol_region_id == other_region_id
    # The original region's unfinished coverage was checkpointed, not
    # discarded, by _pause_patrol_coverage inside _replan_active_patrol.
    assert bridge._paused_patrol_coverage is not None
    assert bridge._paused_patrol_coverage.region_id == original_region


# -- mission interruption / resume-after-target ------------------------------


def _confident_detection_message() -> dict:
    import math as _math

    size = 1000 * _math.sqrt(0.05)
    return {
        "detections": [
            {
                "trackId": "one",
                "trackState": "confirmed",
                "confidence": 0.9,
                "box": [500 - size / 2, 500 - size / 2, 500 + size / 2, 500 + size / 2],
                "velocityPxPerSec": {"x": 0.0, "y": 0.0},
            }
        ],
        "frameWidth": 1000,
        "frameHeight": 1000,
    }


def test_target_detection_checkpoints_patrol_then_resumes_the_same_region():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _make_airborne(bridge, now)

    region_before = bridge._target_patrol_region_id
    route_before = list(bridge._patrol_route_waypoints)
    assert region_before is not None

    detection = _confident_detection_message()
    for _ in range(5):
        now[0] += 0.05
        _feed_telemetry(bridge, now)
        bridge.realtime_link._detections.put(detection)
        outcome = bridge.tick(0.05, now=now[0])

    # Acquisition/follow interrupted the patrol and checkpointed it.
    assert outcome.state in {
        FlightState.TARGET_ACQUISITION.value,
        FlightState.TARGET_FOLLOW.value,
        FlightState.TARGET_PURSUIT.value,
    }
    assert bridge._mission_interruption is not None
    assert bridge._mission_interruption.source_type.value == "PATROL"
    assert bridge._mission_interruption.active_id == region_before
    assert bridge._mission_interruption.progress.remaining_waypoints == tuple(route_before)

    # Stop sending detections -- the target is lost, and the mission should
    # resume the exact same region and the exact same remaining waypoints.
    for _ in range(200):
        now[0] += 0.05
        _feed_telemetry(bridge, now)
        outcome = bridge.tick(0.05, now=now[0])
        if bridge.logic.state is FlightState.PATROLLING and bridge._mission_interruption is None:
            break

    assert bridge.logic.state is FlightState.PATROLLING
    assert bridge._mission_interruption is None
    assert bridge._target_patrol_region_id == region_before
    assert list(bridge._patrol_route_waypoints) == route_before


def test_target_detection_during_roi_checkpoints_and_resumes_roi():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    roi_point = destination_point(DEFAULT_HOME, 50.0, 0.0)
    _queue_command(
        bridge,
        CommandName.SET_ROI.value,
        {"center": {"lat": roi_point.lat, "lon": roi_point.lng}, "radiusM": 20.0},
        command_id="roi-1",
    )
    # Reaching ROI_PATROL requires the aircraft to actually arrive at the
    # ROI's entry waypoint -- needs closed-loop telemetry (see _fly_ticks),
    # not a fixed-position feed.
    outcome = _fly_ticks(bridge, now, 200)
    assert outcome.state == FlightState.ROI_PATROL.value
    active_roi_id_before = bridge._active_roi_id
    assert active_roi_id_before is not None

    def _feed_at_last_position() -> None:
        assert outcome.position is not None and outcome.yaw_rad is not None
        local = LocalPosition(x=outcome.position[0], y=outcome.position[1], z=outcome.position[2])
        gps = local_meters_to_wgs84(local)
        heading_deg = (math.degrees(outcome.yaw_rad) + 360.0) % 360.0
        _feed_telemetry(bridge, now, lat=gps.lat, lng=gps.lng, altitude_m=outcome.position[2], heading_deg=heading_deg)

    detection = _confident_detection_message()
    for _ in range(5):
        now[0] += 0.05
        _feed_at_last_position()
        bridge.realtime_link._detections.put(detection)
        outcome = bridge.tick(0.05, now=now[0])

    assert bridge._mission_interruption is not None
    assert bridge._mission_interruption.source_type.value == "ROI"
    assert bridge._mission_interruption.active_id == active_roi_id_before

    for _ in range(200):
        now[0] += 0.05
        if outcome.has_position:
            _feed_at_last_position()
        else:
            _feed_telemetry(bridge, now)
        outcome = bridge.tick(0.05, now=now[0])
        if bridge.logic.state is FlightState.ROI_PATROL and bridge._mission_interruption is None:
            break

    assert bridge.logic.state is FlightState.ROI_PATROL
    assert bridge._mission_interruption is None
    assert bridge._active_roi_id == active_roi_id_before


# -- SET_OPERATING_CONFIG parity ---------------------------------------------


def test_set_operating_config_mid_flight_retargets_altitude():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _make_airborne(bridge, now)
    assert bridge.logic.target[2] == 15.0

    _queue_command(
        bridge,
        CommandName.SET_OPERATING_CONFIG.value,
        {"maxFlightRadiusM": 300.0, "cruiseAltitudeM": 30.0},
        command_id="config-alt",
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])

    assert bridge.operating_config.cruise_altitude_m == 30.0
    assert bridge.logic.config.takeoff_altitude == 30.0
    # Every already-planned waypoint was retargeted to the new altitude, not
    # just future ones -- mirrors autonomous_drone.py's
    # _retarget_stored_altitudes.
    assert bridge.logic.target[2] == 30.0
    assert all(waypoint[2] == 30.0 for waypoint in bridge._patrol_route_waypoints)


def test_set_operating_config_is_deferred_while_target_tracking():
    """Mirrors autonomous_drone.py: a config change during pursuit/follow
    must not interrupt it -- it's queued and applied once tracking ends."""
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now)
    bridge.tick(0.05, now=now[0])
    _make_airborne(bridge, now)

    detection = _confident_detection_message()
    for _ in range(3):
        now[0] += 0.05
        _feed_telemetry(bridge, now)
        bridge.realtime_link._detections.put(detection)
        outcome = bridge.tick(0.05, now=now[0])
    assert bridge.logic.is_target_tracking is True

    _queue_command(
        bridge,
        CommandName.SET_OPERATING_CONFIG.value,
        {"maxFlightRadiusM": 300.0, "cruiseAltitudeM": 25.0},
        command_id="config-deferred",
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now)
    bridge.realtime_link._detections.put(detection)
    bridge.tick(0.05, now=now[0])

    # Not applied yet -- still tracking, and the geometry underneath it must
    # not change mid-pursuit.
    assert bridge.operating_config.cruise_altitude_m == 15.0
    assert bridge._pending_operating_config is not None
    assert bridge._pending_operating_config.cruise_altitude_m == 25.0


# -- real-DJI session-home lifecycle -----------------------------------------
# The aircraft-session lifecycle this bridge uses: one DjiFlightBridge process
# lifetime is one session. session_home is captured exactly once, from the
# first Android telemetry sample that is both a complete GPS fix and
# explicitly grounded (is_flying is False, never on None/unknown or True); it
# never resets on an AndroidLink reconnect (a hardware-link auth/replay
# -protection concept -- see android_link.py's session_generation -- that is
# deliberately unrelated to where home is), and it never persists across a
# process restart, so a restart mid-flight cannot silently adopt an airborne
# position as home. See DjiFlightBridge._maybe_establish_session_home.

# Deliberately far from DEFAULT_HOME (Tel Aviv) so any test that accidentally
# still computes geometry against DEFAULT_HOME fails loudly.
FAR_AWAY_FIX = Wgs84Point(lat=35.6762, lng=139.6503)  # Tokyo


def test_session_home_captured_from_first_grounded_fix_and_used_for_position():
    now = [0.0]
    bridge = _make_bridge(now)
    assert bridge.session_home is None

    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=False)
    outcome = bridge.tick(0.05, now=now[0])

    assert bridge.session_home == FAR_AWAY_FIX
    # Reported relative to the far-away fix (near the local origin), not the
    # thousands of kilometers away DEFAULT_HOME would put it at.
    assert outcome.has_position is True
    assert abs(outcome.position[0]) < 5.0
    assert abs(outcome.position[1]) < 5.0


def test_session_home_fails_closed_on_airborne_or_unknown_first_fix():
    now = [0.0]
    bridge = _make_bridge(now)

    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=True)
    outcome = bridge.tick(0.05, now=now[0])
    assert bridge.session_home is None
    assert outcome.has_position is False

    now[0] += 0.05
    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=None)
    outcome = bridge.tick(0.05, now=now[0])
    assert bridge.session_home is None
    assert outcome.has_position is False

    # Only an explicitly-grounded sample establishes it.
    now[0] += 0.05
    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=False)
    outcome = bridge.tick(0.05, now=now[0])
    assert bridge.session_home == FAR_AWAY_FIX
    assert outcome.has_position is True


def test_session_home_survives_a_transient_android_link_reconnect():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=False)
    bridge.tick(0.05, now=now[0])
    assert bridge.session_home == FAR_AWAY_FIX

    # A network blip: AndroidLink disconnects and reconnects (bumping
    # session_generation, which resets local-frame kinematic state) without
    # the bridge process restarting.
    bridge.android_link._connection = None
    bridge.android_link._mark_disconnected()
    now[0] += 0.1
    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=False)
    bridge.tick(0.05, now=now[0])

    assert bridge.session_home == FAR_AWAY_FIX


def test_new_bridge_process_starts_a_genuinely_new_session():
    """A new DjiFlightBridge instance -- what a process restart is -- has no
    session_home and no leftover patrol priorities."""
    now = [0.0]
    first = _make_bridge(now)
    _feed_telemetry(first, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=False)
    first.tick(0.05, now=now[0])
    first.patrol_regions.set_priority(first.patrol_regions.regions[0].id, 1)
    assert first.session_home == FAR_AWAY_FIX

    second = _make_bridge(now)
    assert second.session_home is None
    assert all(region.priority is None for region in second.patrol_regions.regions)


def test_patrol_and_roi_geometry_use_session_home_not_default_home():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=False)
    bridge.tick(0.05, now=now[0])
    assert bridge.session_home == FAR_AWAY_FIX

    region = bridge.patrol_regions.regions[0]
    assert wgs84_distance_m(region.center, FAR_AWAY_FIX) < wgs84_distance_m(region.center, DEFAULT_HOME)

    _make_airborne(bridge, now)
    assert bridge._target_patrol_region_id is not None
    target = bridge.logic.target
    assert target is not None
    # A small local-meters displacement from the (0,0)-ish local origin at
    # the far-away home -- not thousands of km off, which is what using
    # DEFAULT_HOME's frame here would produce.
    assert math.hypot(target[0], target[1]) < 10_000.0

    roi_point = destination_point(FAR_AWAY_FIX, 100.0, 0.0)
    _queue_command(
        bridge, CommandName.SET_ROI.value,
        {"center": {"lat": roi_point.lat, "lon": roi_point.lng}, "radiusM": 20.0},
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng)
    outcome = bridge.tick(0.05, now=now[0])
    assert outcome.state in {FlightState.TRANSITING_TO_ROI.value, FlightState.TAKING_OFF.value}


def test_operating_config_change_keeps_patrol_geometry_anchored_to_session_home():
    now = [0.0]
    bridge = _make_bridge(now)
    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=False)
    bridge.tick(0.05, now=now[0])
    assert bridge.session_home == FAR_AWAY_FIX

    _queue_command(
        bridge, CommandName.SET_OPERATING_CONFIG.value,
        {"maxFlightRadiusM": 500.0, "cruiseAltitudeM": 25.0}, command_id="config-far",
    )
    now[0] += 0.05
    _feed_telemetry(bridge, now, lat=FAR_AWAY_FIX.lat, lng=FAR_AWAY_FIX.lng, is_flying=False)
    bridge.tick(0.05, now=now[0])

    assert bridge.operating_config.max_flight_radius_m == 500.0
    region = bridge.patrol_regions.regions[0]
    assert wgs84_distance_m(region.center, FAR_AWAY_FIX) < wgs84_distance_m(region.center, DEFAULT_HOME)


def test_phase2_rth_still_reachable_before_session_home_is_established(monkeypatch):
    """send_rth must stay reachable even while session_home is unset (e.g. a
    bridge restart that comes back up mid-flight, so the first telemetry
    sample reports is_flying=True) -- a degraded/unconfirmed state is exactly
    when a go-home request matters most."""
    monkeypatch.setenv("DRONTECT_DJI_PHASE1_TELEMETRY_ONLY", "1")
    monkeypatch.setenv("DRONTECT_DJI_PHASE2_ACTIONS", "1")
    now = [100.0]
    bridge = _make_bridge(now)
    bridge.realtime_link._mark_connected()

    class RecordingConnection:
        def __init__(self):
            self.messages = []

        async def send(self, raw):
            self.messages.append(json.loads(raw))

    connection = RecordingConnection()
    _feed_telemetry(bridge, now, is_flying=True)  # airborne on first contact
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())
    assert bridge.session_home is None

    _queue_command(bridge, CommandName.RETURN_HOME.value)
    now[0] += 0.05
    _feed_telemetry(bridge, now, is_flying=True)
    bridge.android_link._connection = connection
    asyncio.run(bridge.run_tick_once())

    assert [m["type"] for m in connection.messages] == ["dji_rth"]
