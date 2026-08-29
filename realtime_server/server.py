"""Local realtime WebSocket server."""

from __future__ import annotations

import asyncio
import os
from urllib.parse import parse_qs, urlsplit

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed

from config.video import VideoConfig, load_video_config
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
from realtime_server.video_pipeline import (
    ROLE_VIEWER,
    VIDEO_INGEST_PATH,
    VIDEO_WEBRTC_PATH,
    VideoPipeline,
    validate_video_role,
)
from realtime_server.video_source import VideoSourceConnection, WebSocketVideoSourceAdapter
from services.video import VIDEO_MAX_PACKET_BYTES, VideoPacketError


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
WS_PATH = "/ws"


class RealtimeServer:
    """Routes telemetry from one drone source to any connected tablet clients."""

    def __init__(self, video_config: VideoConfig | None = None) -> None:
        self.drone: DroneSdkConnection | None = None
        self.video_source: VideoSourceConnection | None = None
        self.video_pipeline = VideoPipeline(video_config or load_video_config())
        self._video_pipeline_started = False
        self.tablets: set[ServerConnection] = set()

    async def handler(self, websocket: ServerConnection) -> None:
        try:
            request_path = self._request_path(websocket)
        except ProtocolError as exc:
            await websocket.send(encode_message(exc.to_message()))
            await websocket.close(code=1008, reason=exc.code)
            return

        parsed = urlsplit(request_path)

        if parsed.path == VIDEO_INGEST_PATH:
            await self._handle_video_ingest(websocket, parsed.query)
            return
        if parsed.path == VIDEO_WEBRTC_PATH:
            await self._handle_video_webrtc(websocket, parsed.query)
            return

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

    def start_background_tasks(self) -> None:
        if not self._video_pipeline_started:
            self.video_pipeline.start()
            self._video_pipeline_started = True

    def _request_path(self, websocket: ServerConnection) -> str:
        request = websocket.request
        if request is None:
            raise ProtocolError(
                "missing_request",
                "WebSocket handshake request is unavailable.",
            )
        return request.path

    def _role_from_request(self, websocket: ServerConnection) -> str:
        request_path = self._request_path(websocket)
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

    async def _handle_video_ingest(self, websocket: ServerConnection, query: str) -> None:
        try:
            role = validate_video_role(
                parse_qs(query).get("role", [None])[0],
                expected=ROLE_DRONE,
            )
        except ProtocolError as exc:
            await websocket.send(encode_message(exc.to_message()))
            await websocket.close(code=1008, reason=exc.code)
            return

        if role != ROLE_DRONE:
            await websocket.close(code=1008, reason="invalid_role")
            return

        self.start_background_tasks()
        await self._handle_video_source(WebSocketVideoSourceAdapter(websocket))

    async def _handle_video_webrtc(self, websocket: ServerConnection, query: str) -> None:
        try:
            validate_video_role(parse_qs(query).get("role", [None])[0], expected=ROLE_VIEWER)
        except ProtocolError as exc:
            await websocket.send(encode_message(exc.to_message()))
            await websocket.close(code=1008, reason=exc.code)
            return

        self.start_background_tasks()
        await self.video_pipeline.handle_webrtc_viewer(websocket)

    async def _handle_video_source(self, source: VideoSourceConnection) -> None:
        if self.video_source is not None and self.video_source is not source:
            await source.close(code=1008, reason="video_source_already_connected")
            return

        self.video_source = source
        try:
            async for frame in source.iter_frames():
                await self.video_pipeline.publish(frame)
        except VideoPacketError:
            await source.close(code=1008, reason="invalid_video_frame")
        finally:
            if self.video_source is source:
                self.video_source = None

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
    realtime_server.start_background_tasks()
    async with serve(realtime_server.handler, host, port, max_size=VIDEO_MAX_PACKET_BYTES):
        print(f"Drontect realtime server listening on ws://{host}:{port}{WS_PATH}")
        await asyncio.Future()


def main() -> None:
    host = os.getenv("DRONTECT_REALTIME_HOST", DEFAULT_HOST)
    port = int(os.getenv("DRONTECT_REALTIME_PORT", str(DEFAULT_PORT)))
    asyncio.run(run_realtime_server(host, port))


if __name__ == "__main__":
    main()
