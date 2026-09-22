"""Local link between the DJI flight bridge and the (not yet built) Android DJI layer.

Runs a small local WebSocket server that the Android app dials into -- the
mirror image of how tablet/simulator clients dial into realtime_server.
Exactly one Android connection is expected at a time (there is exactly one
aircraft); a second concurrent connection is refused rather than silently
replacing the first.

Safety property this module exists to provide: `send_setpoint` (and
`send_gimbal_command`) will not transmit anything unless Android telemetry
has been received recently. A network hiccup, a stalled Android process, or
a bridge that has fallen behind therefore degrades to "stop sending new
movement commands" rather than "keep re-sending the last one computed from
data that is no longer current." `send_rth` is the deliberate exception: it
is best-effort and never gated on freshness, since a degraded link is
exactly when a go-home request matters most.
"""

from __future__ import annotations

import asyncio
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
    """Server-side half of the bridge<->Android local protocol."""

    def __init__(
        self,
        *,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.host = host
        self.port = port
        self._clock = clock
        self._connection: ServerConnection | None = None
        self._server: Server | None = None

        self._latest_telemetry: AndroidTelemetry | None = None
        self._telemetry_received_monotonic: float | None = None
        self._latest_rc_state: AndroidRcState | None = None
        self._latest_gimbal_state: AndroidGimbalState | None = None

        self._ever_connected = False
        self._disconnected_at: float | None = None

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
        if self._connection is not None:
            await websocket.close(code=CONNECTION_REFUSED_CODE, reason=CONNECTION_REFUSED_REASON)
            return
        self._connection = websocket
        self._mark_connected()
        try:
            async for raw in websocket:
                self._handle_message(str(raw))
        except ConnectionClosed:
            pass
        finally:
            self._connection = None
            self._mark_disconnected()

    def _mark_connected(self) -> None:
        self._ever_connected = True
        self._disconnected_at = None

    def _mark_disconnected(self) -> None:
        if not self._ever_connected:
            return
        self._disconnected_at = self._clock()

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

    def _handle_message(self, raw: str) -> None:
        try:
            message = parse_json_message(raw)
        except LocalProtocolError:
            return
        except Exception:
            return
        message_type = message.get("type")
        try:
            if message_type == MESSAGE_TYPE_TELEMETRY:
                self._latest_telemetry = validate_android_telemetry_message(message)
                self._telemetry_received_monotonic = self._clock()
            elif message_type == MESSAGE_TYPE_RC_STATE:
                self._latest_rc_state = validate_android_rc_state_message(message)
            elif message_type == MESSAGE_TYPE_GIMBAL_STATE:
                self._latest_gimbal_state = validate_android_gimbal_state_message(message)
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
        """Whether there is a connected Android link with recent telemetry.

        This is the single gate `send_setpoint`/`send_gimbal_command` check
        before transmitting anything -- see the module docstring.
        """
        if not self.connected:
            return False
        age = self.telemetry_age_s(now)
        return age is not None and age <= timeout_s

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
        setpoint = VelocitySetpoint(
            issued_at=time.time(),
            ttl_ms=ttl_ms,
            velocity_x_mps=velocity_x_mps,
            velocity_y_mps=velocity_y_mps,
            velocity_z_mps=velocity_z_mps,
            yaw_deg=yaw_deg,
        )
        await self._send(build_setpoint_message(setpoint))
        return True

    async def send_gimbal_command(self, *, pitch_deg: float, ttl_ms: int) -> bool:
        if not self.connected:
            return False
        command = GimbalCommand(issued_at=time.time(), ttl_ms=ttl_ms, pitch_deg=pitch_deg)
        await self._send(build_gimbal_command_message(command))
        return True

    async def send_rth(self, *, reason: str) -> bool:
        """Best-effort go-home request. Not gated on telemetry freshness."""
        if not self.connected:
            return False
        command = RthCommand(issued_at=time.time(), reason=reason)
        await self._send(build_rth_message(command))
        return True

    async def _send(self, message: dict) -> None:
        connection = self._connection
        if connection is None:
            return
        try:
            await connection.send(encode_message(message))
        except ConnectionClosed:
            pass
