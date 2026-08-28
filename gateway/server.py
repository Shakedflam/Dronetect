"""Local edge gateway WebSocket server."""

from __future__ import annotations

import asyncio
import os
from urllib.parse import parse_qs, urlsplit

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed

from gateway.protocol import (
    ROLE_SIMULATOR,
    ROLE_TABLET,
    ProtocolError,
    command_ack,
    encode_message,
    parse_json_message,
    validate_command_message,
    validate_role,
    validate_telemetry_message,
)


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
WS_PATH = "/ws"


class EdgeGateway:
    """Routes telemetry from one simulator to any connected tablet clients."""

    def __init__(self) -> None:
        self.simulator: ServerConnection | None = None
        self.tablets: set[ServerConnection] = set()

    async def handler(self, websocket: ServerConnection) -> None:
        try:
            role = self._role_from_request(websocket)
        except ProtocolError as exc:
            await websocket.send(encode_message(exc.to_message()))
            await websocket.close(code=1008, reason=exc.code)
            return

        if role == ROLE_SIMULATOR:
            await self._handle_simulator(websocket)
        else:
            await self._handle_tablet(websocket)

    def _role_from_request(self, websocket: ServerConnection) -> str:
        request_path = websocket.request.path
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

    async def _handle_simulator(self, websocket: ServerConnection) -> None:
        if self.simulator is not None and self.simulator is not websocket:
            await websocket.send(
                encode_message(
                    ProtocolError(
                        "simulator_already_connected",
                        "Only one simulator connection is supported.",
                    ).to_message()
                )
            )
            await websocket.close(code=1008, reason="simulator_already_connected")
            return

        self.simulator = websocket
        try:
            async for raw in websocket:
                await self._route_simulator_message(str(raw))
        finally:
            if self.simulator is websocket:
                self.simulator = None

    async def _handle_tablet(self, websocket: ServerConnection) -> None:
        self.tablets.add(websocket)
        try:
            async for raw in websocket:
                await self._route_tablet_message(websocket, str(raw))
        finally:
            self.tablets.discard(websocket)

    async def _route_simulator_message(self, raw: str) -> None:
        try:
            telemetry = validate_telemetry_message(parse_json_message(raw))
        except ProtocolError as exc:
            if self.simulator is not None:
                await self.simulator.send(encode_message(exc.to_message()))
            return

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

        if self.simulator is None:
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "simulator_unavailable",
                        "No simulator is connected to receive commands.",
                        command_id=command["commandId"],
                    ).to_message()
                )
            )
            return

        try:
            await self.simulator.send(encode_message(command))
        except ConnectionClosed:
            self.simulator = None
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "simulator_unavailable",
                        "Simulator disconnected before receiving the command.",
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


async def run_gateway(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
    gateway = EdgeGateway()
    async with serve(gateway.handler, host, port):
        print(f"Drontect edge gateway listening on ws://{host}:{port}{WS_PATH}")
        await asyncio.Future()


def main() -> None:
    host = os.getenv("DRONTECT_GATEWAY_HOST", DEFAULT_HOST)
    port = int(os.getenv("DRONTECT_GATEWAY_PORT", str(DEFAULT_PORT)))
    asyncio.run(run_gateway(host, port))


if __name__ == "__main__":
    main()
