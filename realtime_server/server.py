"""Local realtime WebSocket server."""

from __future__ import annotations

import asyncio
import os
import time
import uuid
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
    mission_status_message,
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
from services.commands import CommandName, MISSION_COMMANDS
from services.mission_commands import (
    InvalidMissionCommand,
    MissionCommandConfig,
    MissionCommandService,
)
from services.mission_state import MissionState
from services.gps import Wgs84Point
from services.flight_safety import (
    DefaultSafetyRthPolicy,
    FlightSafetySnapshot,
    SafetyRthPolicy,
)
from services.patrol_planner import PatrolPlanner
from services.patrol_regions import PatrolAreaType, PatrolRegionStore


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
WS_PATH = "/ws"


class RealtimeServer:
    """Routes telemetry from one drone source to one connected tablet client."""

    def __init__(
        self,
        video_config: VideoConfig | None = None,
        mission_config: MissionCommandConfig | None = None,
        safety_rth_policy: SafetyRthPolicy | None = None,
    ) -> None:
        self.drone: DroneSdkConnection | None = None
        self.video_source: VideoSourceConnection | None = None
        self.video_pipeline = VideoPipeline(
            video_config or load_video_config(),
            detection_handler=self._route_vision_detections,
        )
        self._video_pipeline_started = False
        self.tablet: ServerConnection | None = None
        self.mission = MissionCommandService(
            config=mission_config
            or MissionCommandConfig(
                takeoff_altitude_m=float(
                    os.getenv("DRONTECT_TAKEOFF_ALTITUDE_M", "15.0")
                ),
                roi_arrival_tolerance_m=float(
                    os.getenv("DRONTECT_ROI_ARRIVAL_TOLERANCE_M", "10.0")
                ),
            )
        )
        self.patrol_regions = PatrolRegionStore()
        self.patrol_planner = PatrolPlanner(store=self.patrol_regions)
        self.safety_rth_policy = safety_rth_policy or DefaultSafetyRthPolicy(
            low_battery_threshold_percent=float(
                os.getenv("DRONTECT_LOW_BATTERY_RTH_PERCENT", "20.0")
            )
        )
        self._battery_percent: float | None = None
        self._low_battery = False
        self._target_patrol_region_id: str | None = None

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
        await self._send_patrol_priorities_to_drone(drone)
        await self._send_patrol_area_types_to_drone(drone)
        await self._send_to_tablet(encode_message(self._mission_status()))
        try:
            async for telemetry in drone.iter_telemetry():
                await self._route_drone_telemetry(telemetry)
        finally:
            if self.drone is drone:
                self.drone = None
                await self._send_to_tablet(encode_message(self._mission_status()))

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
        if self.tablet is not None and self.tablet is not websocket:
            await websocket.send(
                encode_message(
                    ProtocolError(
                        "tablet_already_connected",
                        "Only one tablet connection is supported.",
                    ).to_message()
                )
            )
            await websocket.close(code=1008, reason="tablet_already_connected")
            return

        self.tablet = websocket
        try:
            await websocket.send(encode_message(self._mission_status()))
            async for raw in websocket:
                await self._route_tablet_message(websocket, str(raw))
        finally:
            if self.tablet is websocket:
                self.tablet = None

    async def _route_drone_telemetry(self, telemetry: dict) -> None:
        previous_state = self.mission.state
        data = telemetry["data"]
        safety_signal = self.safety_rth_policy.evaluate(
            FlightSafetySnapshot(
                battery_percent=data["batteryPercent"],
                aircraft_rth_active=data["aircraftSafetyRthActive"],
                aircraft_rth_reason=data["aircraftSafetyRthReason"],
            )
        )
        self._battery_percent = float(data["batteryPercent"])
        self._target_patrol_region_id = data["targetPatrolRegionId"]
        self._low_battery = safety_signal.reason == "low_battery"
        self.mission.observe_safety(safety_signal)
        safety_rth_started = (
            previous_state is not MissionState.RTH
            and self.mission.state is MissionState.RTH
            and self.mission.safety_rth_active
        )
        if safety_rth_started:
            await self._send_safety_rth_to_drone()

        self.mission.observe_altitude(data["altitudeM"])
        gps = data["gps"]
        self.mission.observe_position(Wgs84Point(lat=gps["lat"], lng=gps["lng"]))
        self.mission.observe_flight_state(data["state"])
        await self._send_to_tablet(encode_message(self._mission_status()))
        await self._send_to_tablet(encode_message(telemetry))

    async def _send_safety_rth_to_drone(self) -> None:
        drone = self.drone
        if drone is None:
            return
        command = {
            "type": "command",
            "commandId": f"safety-rth-{uuid.uuid4()}",
            "command": CommandName.RETURN_HOME.value,
            "params": {},
            "safetyReason": self.mission.safety_rth_reason or "safety_rth",
        }
        try:
            await drone.send_command(command)
        except ConnectionClosed:
            if self.drone is drone:
                self.drone = None

    async def _send_patrol_priorities_to_drone(
        self,
        drone: DroneSdkConnection,
    ) -> None:
        priorities = sorted(
            (
                {"regionId": region.id, "rank": region.priority}
                for region in self.patrol_regions.regions
                if region.priority is not None
            ),
            key=lambda entry: entry["rank"],
        )
        if not priorities:
            return
        command = {
            "type": "command",
            "commandId": f"patrol-priorities-{uuid.uuid4()}",
            "command": CommandName.SET_PATROL_PRIORITIES.value,
            "params": {"priorities": priorities},
        }
        try:
            await drone.send_command(command)
        except ConnectionClosed:
            if self.drone is drone:
                self.drone = None

    async def _send_patrol_area_types_to_drone(
        self,
        drone: DroneSdkConnection,
    ) -> None:
        area_types = [
            {"regionId": region.id, "areaType": region.area_type.value}
            for region in self.patrol_regions.regions
            if region.area_type is not PatrolAreaType.PATROL
        ]
        if not area_types:
            return
        command = {
            "type": "command",
            "commandId": f"patrol-area-types-{uuid.uuid4()}",
            "command": CommandName.SET_PATROL_AREA_TYPES.value,
            "params": {"areaTypes": area_types},
        }
        try:
            await drone.send_command(command)
        except ConnectionClosed:
            if self.drone is drone:
                self.drone = None

    async def _route_vision_detections(self, detections: dict) -> None:
        timestamp_ms = detections.get("timestampMs")
        detection_age_ms = (
            max(0, round(time.time() * 1000.0 - timestamp_ms))
            if isinstance(timestamp_ms, (int, float))
            else 0
        )
        routed_detections = {**detections, "detectionAgeMs": detection_age_ms}
        if detections.get("detections"):
            print(
                "realtime_server forwarding detections "
                f"frame_id={detections.get('frameId')} "
                f"count={len(detections['detections'])} "
                f"tablet_connected={self.tablet is not None}"
            )
        drone = self.drone
        if drone is not None:
            try:
                # The SDK transport can carry validated server-to-aircraft
                # messages as well as operator commands.
                await drone.send_command(routed_detections)
            except ConnectionClosed:
                if self.drone is drone:
                    self.drone = None
        await self._send_to_tablet(encode_message(routed_detections))

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

        if "safetyReason" in command:
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "invalid_params",
                        "safetyReason is reserved for backend safety commands.",
                        command_id=command["commandId"],
                    ).to_message()
                )
            )
            return

        command_name = CommandName(command["command"])
        if (
            self.mission.safety_rth_active
            and command_name is CommandName.NEW_MISSION
        ):
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "invalid_mission_transition",
                        f"Command {command_name.value} is invalid while in "
                        f"{self.mission.state.value}",
                        command_id=command["commandId"],
                        details={"state": self.mission.state.value},
                    ).to_message()
                )
            )
            return
        if command_name is CommandName.SET_PATROL_PRIORITIES:
            self.patrol_regions.replace_priorities(
                [
                    (entry["regionId"], entry["rank"])
                    for entry in command["params"]["priorities"]
                ]
            )
            drone = self.drone
            if drone is not None:
                try:
                    await drone.send_command(command)
                except ConnectionClosed:
                    if self.drone is drone:
                        self.drone = None
            await tablet.send(encode_message(command_ack(command["commandId"])))
            await tablet.send(encode_message(self._mission_status()))
            return
        if command_name is CommandName.SET_PATROL_AREA_TYPES:
            self.patrol_regions.replace_area_types(
                [
                    (entry["regionId"], entry["areaType"])
                    for entry in command["params"]["areaTypes"]
                ]
            )
            drone = self.drone
            if drone is not None:
                try:
                    await drone.send_command(command)
                except ConnectionClosed:
                    if self.drone is drone:
                        self.drone = None
            await tablet.send(encode_message(command_ack(command["commandId"])))
            await tablet.send(encode_message(self._mission_status()))
            return
        if command_name in MISSION_COMMANDS:
            await self._route_mission_command(tablet, command, command_name)
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

    async def _route_mission_command(
        self,
        tablet: ServerConnection,
        command: dict,
        command_name: CommandName,
    ) -> None:
        try:
            self.mission.validate(command_name)
        except InvalidMissionCommand as exc:
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "invalid_mission_transition",
                        str(exc),
                        command_id=command["commandId"],
                        details={"state": exc.state.value},
                    ).to_message()
                )
            )
            return

        drone = self.drone
        if drone is None:
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "drone_unavailable",
                        "No drone is connected to receive mission commands.",
                        command_id=command["commandId"],
                    ).to_message()
                )
            )
            return

        try:
            await drone.send_command(command)
        except ConnectionClosed:
            if self.drone is drone:
                self.drone = None
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "drone_unavailable",
                        "Drone disconnected before receiving the mission command.",
                        command_id=command["commandId"],
                    ).to_message()
                )
            )
            return

        self.mission.apply(command_name, command["params"])

        await tablet.send(encode_message(command_ack(command["commandId"])))
        await tablet.send(encode_message(self._mission_status()))

    def _mission_status(self) -> dict:
        roi = self.mission.roi
        active_roi = (
            None
            if roi is None
            else {
                "center": {"lat": roi.lat, "lon": roi.lon},
                "radiusM": roi.radius_m,
            }
        )
        priorities = sorted(
            (
                {"regionId": region.id, "rank": region.priority}
                for region in self.patrol_regions.regions
                if region.priority is not None
            ),
            key=lambda entry: entry["rank"],
        )
        area_types = [
            {"regionId": region.id, "areaType": region.area_type.value}
            for region in self.patrol_regions.regions
            if region.area_type is not PatrolAreaType.PATROL
        ]
        return mission_status_message(
            state=self.mission.state,
            active_roi=active_roi,
            patrol_priorities=priorities,
            current_patrol_region_id=None,
            target_patrol_region_id=self._target_patrol_region_id,
            battery_percent=self._battery_percent,
            low_battery=self._low_battery,
            safety_rth_active=self.mission.safety_rth_active,
            rth_reason=self.mission.safety_rth_reason,
            drone_connected=self.drone is not None,
            patrol_area_types=area_types,
        )

    async def _send_to_tablet(self, message: str) -> None:
        tablet = self.tablet
        if tablet is None:
            return
        try:
            await tablet.send(message)
        except ConnectionClosed:
            if self.tablet is tablet:
                self.tablet = None


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
