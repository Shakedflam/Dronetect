"""Tests for DjiFlightBridge: command dispatch, tick gating, and fail-safes.

These drive DjiFlightBridge.tick() directly with a fake clock and manually
-populated AndroidLink/realtime_link state, the same style
tests/test_flight_logic.py and tests/test_manual_control.py already use for
the underlying pure autonomy modules -- no real sockets are needed to
exercise the bridge's own logic.
"""

import json
import math

import pytest

from adapters.dji.android_link import AndroidLink
from adapters.dji.flight_bridge import BridgeConfig, DjiFlightBridge, TickOutcome
from adapters.dji.local_protocol import AndroidRcState, AndroidTelemetry, build_android_telemetry_message
from adapters.simulator.client import SimulatorRealtimeClient
from config.shared_config import OperatingConfig
from controllers.autonomous_drone.flight_logic import FlightState
from realtime_server.protocol import encode_message
from services.commands import CommandName
from services.gps import DEFAULT_HOME, LocalPosition, destination_point, local_meters_to_wgs84

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
    )
    defaults.update(overrides)
    bridge.android_link._connection = object()
    bridge.android_link._mark_connected()
    bridge.android_link._handle_message(
        json.dumps(build_android_telemetry_message(AndroidTelemetry(**defaults)))
    )
    bridge.android_link._telemetry_received_monotonic = now[0]


def _queue_command(bridge: DjiFlightBridge, command: str, params: dict | None = None, command_id: str = "cmd-1") -> None:
    bridge.realtime_link._commands.put(
        {"type": "command", "commandId": command_id, "command": command, "params": params or {}}
    )


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
