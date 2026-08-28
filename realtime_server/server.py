"""Local realtime WebSocket server."""

from __future__ import annotations

import asyncio
import os
from urllib.parse import parse_qs, urlsplit

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed

from realtime_server.drone_sdk import DroneSdkConnection, WebSocketDroneSdkAdapter
from realtime_server.protocol import (
    ROLE_DRONE,
    ROLE_TABLET,
    ProtocolError,
    command_ack,
    encode_message,
    parse_json_message,
    validate_command_message,
    validate_role,
)


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
WS_PATH = "/ws"


class RealtimeServer:
    """Routes telemetry from one drone source to any connected tablet clients."""

    def __init__(self) -> None:
        self.drone: DroneSdkConnection | None = None
        self.tablets: set[ServerConnection] = set()

    async def handler(self, websocket: ServerConnection) -> None:
        try:
            role = self._role_from_request(websocket)
        except ProtocolError as exc:
            await websocket.send(encode_message(exc.to_message()))
            await websocket.close(code=1008, reason=exc.code)
            return

        if role == ROLE_DRONE:
            await self._handle_drone(WebSocketDroneSdkAdapter(websocket))
        else:
            await self._handle_tablet(websocket)

    def _role_from_request(self, websocket: ServerConnection) -> str:
        request = websocket.request
        if request is None:
            raise ProtocolError(
                "missing_request",
                "WebSocket handshake request is unavailable.",
            )

        request_path = request.path
        parsed = urlsplit(request_path)
        if parsed.path != WS_PATH:
            raise ProtocolError(
                "invalid_path",
                f"WebSocket endpoint is {WS_PATH}.",
                details={"path": parsed.path},
            )
        query = parse_qs(parsed.query)
        role = query.get("role", [None])[0]
        return validate_role(role)

    async def _handle_drone(self, drone: DroneSdkConnection) -> None:
        if self.drone is not None and self.drone is not drone:
            await drone.send_error(
                ProtocolError(
                    "drone_already_connected",
                    "Only one drone connection is supported.",
                )
            )
            await drone.close(code=1008, reason="drone_already_connected")
            return

        self.drone = drone
        try:
            async for telemetry in drone.iter_telemetry():
                await self._route_drone_telemetry(telemetry)
        finally:
            if self.drone is drone:
                self.drone = None

    async def _handle_tablet(self, websocket: ServerConnection) -> None:
        self.tablets.add(websocket)
        try:
            async for raw in websocket:
                await self._route_tablet_message(websocket, str(raw))
        finally:
            self.tablets.discard(websocket)

    async def _route_drone_telemetry(self, telemetry: dict) -> None:
        await self._broadcast_to_tablets(encode_message(telemetry))

    async def _route_tablet_message(
        self,
        tablet: ServerConnection,
        raw: str,
    ) -> None:
        try:
            command = validate_command_message(parse_json_message(raw))
        except ProtocolError as exc:
            await tablet.send(encode_message(exc.to_message()))
            return

        if self.drone is None:
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "drone_unavailable",
                        "No drone is connected to receive commands.",
                        command_id=command["commandId"],
                    ).to_message()
                )
            )
            return

        try:
            await self.drone.send_command(command)
        except ConnectionClosed:
            self.drone = None
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "drone_unavailable",
                        "Drone disconnected before receiving the command.",
                        command_id=command["commandId"],
                    ).to_message()
                )
            )
            return

        await tablet.send(encode_message(command_ack(command["commandId"])))

    async def _broadcast_to_tablets(self, message: str) -> None:
        disconnected: set[ServerConnection] = set()
        for tablet in self.tablets:
            try:
                await tablet.send(message)
            except ConnectionClosed:
                disconnected.add(tablet)
        self.tablets.difference_update(disconnected)


async def run_realtime_server(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
    realtime_server = RealtimeServer()
    async with serve(realtime_server.handler, host, port):
        print(f"Drontect realtime server listening on ws://{host}:{port}{WS_PATH}")
        await asyncio.Future()


def main() -> None:
    host = os.getenv("DRONTECT_REALTIME_HOST", DEFAULT_HOST)
    port = int(os.getenv("DRONTECT_REALTIME_PORT", str(DEFAULT_PORT)))
    asyncio.run(run_realtime_server(host, port))


if __name__ == "__main__":
    main()
