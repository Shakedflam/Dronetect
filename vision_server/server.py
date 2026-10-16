"""Minimal local vision frame receiver and diagnostics server."""

from __future__ import annotations

import asyncio
import json
import os

from websockets.asyncio.server import ServerConnection, serve

from config.deployment import DeploymentConfig, load_vision_deployment_config
from config.video import VideoConfig, load_video_config
from realtime_server.protocol import build_detection_message, encode_message
from services.access_control import AccessControl
from services.video import VIDEO_MAX_PACKET_BYTES, VideoFrame, VideoPacketError, decode_video_frame
from vision_server.detector import Detection, DroneDetector, create_drone_detector
from vision_server.diagnostics import VisionDiagnostics
from vision_server.tracker import DroneTracker


DEFAULT_FRAME_HOST = "127.0.0.1"
DEFAULT_FRAME_PORT = 8870
FRAMES_PATH = "/frames"


class VisionServer:
    def __init__(
        self,
        config: VideoConfig | None = None,
        detector: DroneDetector | None = None,
        access: AccessControl | None = None,
    ) -> None:
        self.access = access or AccessControl.from_env()
        self.config = config or load_video_config()
        self.detector = detector or create_drone_detector()
        self.tracker = DroneTracker(
            lost_after_seconds=self.config.vision_tracker_lost_after_seconds,
            remove_after_seconds=self.config.vision_tracker_remove_after_seconds,
            max_velocity_px_per_sec=self.config.vision_tracker_max_velocity_px_per_sec,
            min_center_distance_frame_diagonals=(
                self.config.vision_tracker_min_center_distance_frame_diagonals
            ),
        )
        self.diagnostics = VisionDiagnostics()
        self._last_log_count = 0

    async def frame_handler(self, websocket: ServerConnection) -> None:
        peer = websocket.remote_address
        if not self.access.authorized(websocket.request, "vision", peer[0] if peer else ""):
            await websocket.close(code=1008, reason="Unauthorized")
            return
        request = websocket.request
        if request is None or request.path != FRAMES_PATH:
            await websocket.close(code=1008, reason="invalid_path")
            return

        async for packet in websocket:
            if isinstance(packet, str):
                continue
            try:
                frame = decode_video_frame(bytes(packet))
            except VideoPacketError:
                continue
            self.diagnostics.record(frame)
            detections = await self._detect(frame)
            tracked_detections = self.tracker.update(
                detections,
                timestamp=frame.timestamp,
                frame_width=frame.width,
                frame_height=frame.height,
            )
            await websocket.send(encode_message(build_detection_message(frame, tracked_detections)))
            self._log_detections(frame.frame_id, detections)
            self._log_progress()

    async def diagnostics_handler(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        try:
            request_line = await reader.readline()
            parts = request_line.decode("ascii", errors="ignore").split()
            path = parts[1] if len(parts) >= 2 else "/"
            if path == "/diagnostics":
                body_obj: dict = self.diagnostics.to_dict()
                status = "200 OK"
            elif path == "/health":
                body_obj = {"status": "ok"}
                status = "200 OK"
            else:
                body_obj = {"error": "not_found"}
                status = "404 Not Found"
            body = json.dumps(body_obj, separators=(",", ":")).encode("utf-8")
            writer.write(
                b"\r\n".join(
                    [
                        f"HTTP/1.1 {status}".encode("ascii"),
                        b"Content-Type: application/json",
                        f"Content-Length: {len(body)}".encode("ascii"),
                        b"Connection: close",
                        b"",
                        body,
                    ]
                )
            )
            await writer.drain()
        finally:
            writer.close()
            await writer.wait_closed()

    def _log_progress(self) -> None:
        if self.diagnostics.received_count - self._last_log_count < 30:
            return
        self._last_log_count = self.diagnostics.received_count
        print(
            "vision_server received "
            f"{self.diagnostics.received_count} frames; "
            f"latest frame_id={self.diagnostics.latest_frame_id}; "
            f"fps={self.diagnostics.fps:.1f}"
        )

    async def _detect(self, frame: VideoFrame) -> list[Detection]:
        try:
            return await asyncio.to_thread(self.detector.detect, frame)
        except Exception as exc:
            print(f"vision_server detection failed frame_id={frame.frame_id}: {exc}")
            return []

    def _log_detections(self, frame_id: int, detections: list[Detection]) -> None:
        if not detections:
            return

        details = " ".join(
            f"{detection.class_name} "
            f"conf={detection.confidence:.2f} "
            f"box=({detection.box[0]:.1f},{detection.box[1]:.1f},"
            f"{detection.box[2]:.1f},{detection.box[3]:.1f})"
            for detection in detections
        )
        print(
            f"vision_server detections frame_id={frame_id} "
            f"detections={len(detections)} {details}"
        )


async def run_vision_server(
    frame_host: str = DEFAULT_FRAME_HOST,
    frame_port: int = DEFAULT_FRAME_PORT,
    deployment: DeploymentConfig | None = None,
) -> None:
    deployment = deployment or DeploymentConfig()
    access = AccessControl.from_env()
    access.validate_listener(frame_host, ("vision",))
    vision_server = VisionServer(access=access)

    def admit(connection, request):
        peer = connection.remote_address
        if request.path != FRAMES_PATH or not access.authorized(request, "vision", peer[0] if peer else ""):
            return connection.respond(401, "Unauthorized\n")
        return None
    diagnostics_server = await asyncio.start_server(
        vision_server.diagnostics_handler,
        vision_server.config.vision_server_diagnostics_host,
        vision_server.config.vision_server_diagnostics_port,
    )
    async with (
        diagnostics_server,
        serve(
            vision_server.frame_handler,
            frame_host,
            frame_port,
            max_size=VIDEO_MAX_PACKET_BYTES,
            origins=deployment.origins,
            process_request=admit,
        ),
    ):
        print(f"Drontect vision server receiving frames on ws://{frame_host}:{frame_port}{FRAMES_PATH}")
        print(
            "Drontect vision diagnostics on "
            f"http://{vision_server.config.vision_server_diagnostics_host}:"
            f"{vision_server.config.vision_server_diagnostics_port}/diagnostics"
        )
        await asyncio.Future()


def main() -> None:
    frame_host = os.getenv("DRONTECT_VISION_HOST", DEFAULT_FRAME_HOST)
    frame_port = int(os.getenv("DRONTECT_VISION_PORT", str(DEFAULT_FRAME_PORT)))
    deployment = load_vision_deployment_config()
    asyncio.run(run_vision_server(frame_host, frame_port, deployment))


if __name__ == "__main__":
    main()
