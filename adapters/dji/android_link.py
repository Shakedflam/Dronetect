"""Hardware link between the AWS DJI bridge and the Android DJI layer.

Runs a small WebSocket server that the Android app dials into -- the
mirror image of how tablet/simulator clients dial into realtime_server.
Exactly one Android connection is expected at a time (there is exactly one
aircraft); a second concurrent connection is refused rather than silently
replacing the first.

Safety property this module exists to provide: normal `send_setpoint` needs
fresh position telemetry. The Phase 3 zero-only ground-test heartbeat instead
needs fresh grounded/connected/centered RC state; it cannot construct motion.
`send_rth` is best-effort even when observations are stale.
"""

from __future__ import annotations

import asyncio
import hmac
import secrets
import time
from typing import Callable

from websockets.asyncio.server import Server, ServerConnection, serve
from websockets.exceptions import ConnectionClosed

from adapters.dji.local_protocol import (
    AndroidGimbalState,
    AndroidRcState,
    AndroidTelemetry,
    GimbalCommand,
    LocalProtocolError,
    MESSAGE_TYPE_GIMBAL_STATE,
    MESSAGE_TYPE_RC_STATE,
    MESSAGE_TYPE_TELEMETRY,
    RthCommand,
    VelocitySetpoint,
    build_gimbal_command_message,
    build_rth_message,
    build_setpoint_message,
    encode_message,
    parse_json_message,
    validate_android_gimbal_state_message,
    validate_android_rc_state_message,
    validate_android_telemetry_message,
)

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8790
# Only one Android connection is meaningful (there is one aircraft); refusing
# a second rather than replacing the first avoids two processes racing to
# actuate the same drone.
CONNECTION_REFUSED_CODE = 1013
CONNECTION_REFUSED_REASON = "Only one Android connection is supported."


class AndroidLink:
    """Server-side half of the bridge<->Android hardware protocol."""

    def __init__(
        self,
        *,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        clock: Callable[[], float] = time.monotonic,
        auth_token: str | None = None,
    ) -> None:
        self.host = host
        self.port = port
        self._clock = clock
        self._auth_token = auth_token
        self._session_id: str | None = None
        self._last_sequence = -1
        self._outbound_sequence = 0
        self.session_generation = 0
        self._connection: ServerConnection | None = None
        self._server: Server | None = None

        self._latest_telemetry: AndroidTelemetry | None = None
        self._telemetry_received_monotonic: float | None = None
        self._latest_rc_state: AndroidRcState | None = None
        self._rc_received_monotonic: float | None = None
        self._latest_gimbal_state: AndroidGimbalState | None = None

        self._ever_connected = False
        self._disconnected_at: float | None = None
        self._last_setpoint_sent_at: float | None = None
        self._last_setpoint_issued_at: float | None = None
        self._last_setpoint_report_at = 0.0

    # -- connection lifecycle -------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._connection is not None

    async def serve_forever(self) -> None:
        """Run the local server until cancelled."""
        async with serve(self._handle_connection, self.host, self.port) as server:
            self._server = server
            try:
                await asyncio.Future()
            finally:
                self._server = None

    async def _handle_connection(self, websocket: ServerConnection) -> None:
        if self._auth_token is not None:
            supplied = websocket.request.headers.get("Authorization", "")
            if not hmac.compare_digest(supplied, f"Bearer {self._auth_token}") or websocket.request.path != "/dji/bridge":
                print("DJI bridge: hardware connection rejected")
                await websocket.close(code=1008, reason="Unauthorized hardware link")
                return
        if self._connection is not None:
            await websocket.close(code=CONNECTION_REFUSED_CODE, reason=CONNECTION_REFUSED_REASON)
            return
        self._connection = websocket
        self._mark_connected()
        print("DJI bridge: hardware connected")
        if self._auth_token is not None:
            await websocket.send(encode_message({"type": "dji_hello", "sessionId": self._session_id}))
        try:
            async for raw in websocket:
                response = self._handle_message(str(raw))
                if response is not None:
                    await websocket.send(encode_message(response))
        except ConnectionClosed:
            pass
        finally:
            self._connection = None
            self._mark_disconnected()
            print("DJI bridge: hardware disconnected; observations cleared")

    def _mark_connected(self) -> None:
        if not self._ever_connected or self._disconnected_at is not None:
            self._session_id = secrets.token_hex(16)
            self._last_sequence = -1
            self._outbound_sequence = 0
            self.session_generation += 1
            self._clear_observations()
            self._last_setpoint_sent_at = None
            self._last_setpoint_issued_at = None
        self._ever_connected = True
        self._disconnected_at = None

    def _mark_disconnected(self) -> None:
        self._session_id = None
        self._clear_observations()
        self._last_setpoint_sent_at = None
        self._last_setpoint_issued_at = None
        if not self._ever_connected:
            return
        self._disconnected_at = self._clock()

    def _clear_observations(self) -> None:
        self._latest_telemetry = None
        self._telemetry_received_monotonic = None
        self._latest_rc_state = None
        self._rc_received_monotonic = None
        self._latest_gimbal_state = None

    def connection_lost_for(self, timeout_s: float) -> bool:
        """Return true once a previously-established Android link has been down this long.

        Mirrors adapters.simulator.client.SimulatorRealtimeClient.connection_lost_for
        exactly, so the same failsafe pattern applies to both links the bridge
        depends on.
        """
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if self.connected or not self._ever_connected:
            return False
        return (
            self._disconnected_at is not None
            and self._clock() - self._disconnected_at >= timeout_s
        )

    # -- inbound (Android -> bridge) ------------------------------------------

    def _handle_message(self, raw: str) -> dict | None:
        try:
            message = parse_json_message(raw)
        except LocalProtocolError:
            return
        except Exception:
            return
        message_type = message.get("type")
        if self._auth_token is not None:
            sequence = message.get("seq")
            timestamp = message.get("timestamp")
            if (message.get("sessionId") != self._session_id or
                not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= self._last_sequence or
                not isinstance(timestamp, (int, float)) or isinstance(timestamp, bool) or
                abs(time.time() - timestamp) > 3.0):
                return
        try:
            if message_type == "dji_clock_probe" and self._auth_token is not None:
                # The tablet-send / server-receive pair gives Android a
                # conservative lower bound on its clock offset from AWS.
                self._last_sequence = sequence
                return {"type": "dji_clock_sync", "sessionId": self._session_id,
                        "probeTimestamp": timestamp, "serverReceivedAt": time.time()}
            if message_type == MESSAGE_TYPE_TELEMETRY:
                self._latest_telemetry = validate_android_telemetry_message(message)
                self._telemetry_received_monotonic = self._clock()
            elif message_type == MESSAGE_TYPE_RC_STATE:
                self._latest_rc_state = validate_android_rc_state_message(message)
                self._rc_received_monotonic = self._clock()
            elif message_type == MESSAGE_TYPE_GIMBAL_STATE:
                self._latest_gimbal_state = validate_android_gimbal_state_message(message)
            else:
                return
            if self._auth_token is not None:
                self._last_sequence = sequence
            # Unknown message types are dropped rather than raised: a forward
            # -compatible Android build sending a message type this bridge
            # doesn't know yet should not take the link down.
        except LocalProtocolError:
            # Malformed message from Android. Drop it and keep the link open
            # rather than crashing the bridge on bad input.
            return

    @property
    def latest_telemetry(self) -> AndroidTelemetry | None:
        return self._latest_telemetry

    @property
    def latest_rc_state(self) -> AndroidRcState | None:
        return self._latest_rc_state

    @property
    def latest_gimbal_state(self) -> AndroidGimbalState | None:
        return self._latest_gimbal_state

    def telemetry_age_s(self, now: float | None = None) -> float | None:
        if self._telemetry_received_monotonic is None:
            return None
        return (now if now is not None else self._clock()) - self._telemetry_received_monotonic

    def is_telemetry_fresh(self, timeout_s: float, now: float | None = None) -> bool:
        """Whether there is a connected Android link with recent GPS telemetry."""
        if not self.connected:
            return False
        age = self.telemetry_age_s(now)
        return age is not None and age <= timeout_s

    def is_zero_ground_safety_fresh(self, timeout_s: float, now: float | None = None) -> bool:
        """Phase 3 only: fresh grounded/connected/centered RC facts, independent of GPS."""
        state = self._latest_rc_state
        if not self.connected or state is None or self._rc_received_monotonic is None:
            return False
        age = (self._clock() if now is None else now) - self._rc_received_monotonic
        return (0.0 <= age <= timeout_s and state.connected and
                state.aircraft_connected is True and state.is_flying is False and
                all(abs(axis) <= 33 / 660 for axis in
                    (state.forward, state.right, state.up, state.yaw_rate)))

    def is_phase4_airborne_safety_fresh(self, timeout_s: float, now: float | None = None) -> bool:
        """Strict bridge-side gate for the isolated Phase 4 climb pulse."""
        return self.phase4_airborne_safety_rejection(timeout_s, now) is None

    def phase4_airborne_safety_rejection(
        self, timeout_s: float, now: float | None = None,
    ) -> str | None:
        """Return the exact Phase 4 guard failure, or None when safe.

        This is diagnostic detail for the existing gate, not a second policy:
        is_phase4_airborne_safety_fresh delegates here so the two cannot drift.
        """
        telemetry = self._latest_telemetry
        rc = self._latest_rc_state
        if not self.connected:
            return "hardware_disconnected"
        if telemetry is None or self._telemetry_received_monotonic is None:
            return "telemetry_unavailable"
        if rc is None or self._rc_received_monotonic is None:
            return "rc_state_unavailable"
        current = self._clock() if now is None else now
        telemetry_age = current - self._telemetry_received_monotonic
        rc_age = current - self._rc_received_monotonic
        if not 0.0 <= telemetry_age <= timeout_s:
            return "telemetry_stale"
        if not 0.0 <= rc_age <= timeout_s:
            return "rc_state_stale"
        if telemetry.is_flying is not True:
            return "telemetry_not_airborne"
        if rc.is_flying is not True:
            return "rc_state_not_airborne"
        if not rc.connected:
            return "rc_disconnected"
        if rc.aircraft_connected is not True:
            return "aircraft_disconnected"
        if telemetry.battery_percent < 30.0:
            return "battery_below_30"
        if not 2.0 <= telemetry.altitude_m <= 7.0:
            return "altitude_outside_2_to_7_m"
        if telemetry.speed_mps > 1.0:
            return "speed_above_1_mps"
        if telemetry.flight_mode not in {"GPS_NORMAL", "VIRTUAL_STICK"}:
            return f"flight_mode_{telemetry.flight_mode or 'unavailable'}"
        axes = {
            "forward": rc.forward,
            "right": rc.right,
            "up": rc.up,
            "yaw_rate": rc.yaw_rate,
        }
        moved = next(((name, value) for name, value in axes.items()
                      if abs(value) > 33 / 660), None)
        if moved is not None:
            return f"rc_stick_moved_{moved[0]}_{moved[1]:.3f}"
        return None

    # -- outbound (bridge -> Android) -----------------------------------------

    async def send_setpoint(
        self,
        *,
        velocity_x_mps: float,
        velocity_y_mps: float,
        velocity_z_mps: float,
        yaw_deg: float,
        ttl_ms: int,
        telemetry_timeout_s: float,
    ) -> bool:
        """Send a velocity/yaw setpoint. Returns False, sending nothing, if
        Android telemetry is stale or absent -- see the module docstring."""
        if not self.is_telemetry_fresh(telemetry_timeout_s):
            return False
        return await self._send_setpoint(
            velocity_x_mps=velocity_x_mps, velocity_y_mps=velocity_y_mps,
            velocity_z_mps=velocity_z_mps, yaw_deg=yaw_deg, ttl_ms=ttl_ms)

    async def send_zero_ground_test_setpoint(self, *, ttl_ms: int, safety_timeout_s: float) -> bool:
        if not self.is_zero_ground_safety_fresh(safety_timeout_s):
            return False
        # Android's zero-only actuator ignores yawDeg and sends angular velocity 0.
        return await self._send_setpoint(
            velocity_x_mps=0.0, velocity_y_mps=0.0,
            velocity_z_mps=0.0, yaw_deg=0.0, ttl_ms=ttl_ms)

    async def send_phase4_vertical_pulse_setpoint(
        self, *, vertical_mps: float, ttl_ms: int, safety_timeout_s: float,
    ) -> bool:
        if vertical_mps not in {0.0, 0.30}:
            return False
        if not self.is_phase4_airborne_safety_fresh(safety_timeout_s):
            return False
        return await self._send_setpoint(
            velocity_x_mps=0.0, velocity_y_mps=0.0,
            velocity_z_mps=vertical_mps, yaw_deg=0.0, ttl_ms=ttl_ms)

    async def _send_setpoint(
        self, *, velocity_x_mps: float, velocity_y_mps: float,
        velocity_z_mps: float, yaw_deg: float, ttl_ms: int,
    ) -> bool:
        setpoint = VelocitySetpoint(
            issued_at=time.time(),
            ttl_ms=ttl_ms,
            velocity_x_mps=velocity_x_mps,
            velocity_y_mps=velocity_y_mps,
            velocity_z_mps=velocity_z_mps,
            yaw_deg=yaw_deg,
        )
        message = self._session_command(build_setpoint_message(setpoint))
        send_started = time.monotonic()
        await self._send(message)
        sent_at = time.monotonic()
        cadence_ms = None if self._last_setpoint_sent_at is None else (
            sent_at - self._last_setpoint_sent_at) * 1000.0
        issue_gap_ms = None if self._last_setpoint_issued_at is None else (
            setpoint.issued_at - self._last_setpoint_issued_at) * 1000.0
        send_ms = (sent_at - send_started) * 1000.0
        self._last_setpoint_sent_at = sent_at
        self._last_setpoint_issued_at = setpoint.issued_at
        if (send_ms > ttl_ms or (cadence_ms is not None and cadence_ms > ttl_ms) or
                (issue_gap_ms is not None and cadence_ms is not None and
                 abs(issue_gap_ms - cadence_ms) > ttl_ms) or
                sent_at - self._last_setpoint_report_at >= 5.0):
            self._last_setpoint_report_at = sent_at
            cadence = "first" if cadence_ms is None else f"{cadence_ms:.0f}"
            issue_gap = "first" if issue_gap_ms is None else f"{issue_gap_ms:.0f}"
            transport = getattr(self._connection, "transport", None)
            buffer_bytes = transport.get_write_buffer_size() if transport is not None else 0
            print(f"DJI bridge: setpoint_tx seq={message['seq']} "
                  f"cadenceMs={cadence} issueGapMs={issue_gap} sendMs={send_ms:.0f} "
                  f"bufferBytes={buffer_bytes} ttlMs={ttl_ms}")
        return True

    async def send_gimbal_command(self, *, pitch_deg: float, ttl_ms: int) -> bool:
        if not self.connected:
            return False
        command = GimbalCommand(issued_at=time.time(), ttl_ms=ttl_ms, pitch_deg=pitch_deg)
        await self._send(self._session_command(build_gimbal_command_message(command)))
        return True

    async def send_rth(self, *, reason: str) -> bool:
        """Best-effort go-home request. Not gated on telemetry freshness."""
        if not self.connected:
            return False
        command = RthCommand(issued_at=time.time(), reason=reason)
        message = build_rth_message(command)
        message["ttlMs"] = 1500
        await self._send(self._session_command(message))
        return True

    def _session_command(self, message: dict) -> dict:
        self._outbound_sequence += 1
        return {**message, "sessionId": self._session_id, "seq": self._outbound_sequence}

    async def _send(self, message: dict) -> None:
        connection = self._connection
        if connection is None:
            return
        try:
            await connection.send(encode_message(message))
        except ConnectionClosed:
            pass
