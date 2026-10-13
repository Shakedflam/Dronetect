"""Local realtime WebSocket server."""

from __future__ import annotations

import asyncio
from dataclasses import replace
import os
import time
import uuid
from urllib.parse import parse_qs, urlsplit

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response

from config.deployment import DeploymentConfig, build_ssl_context, load_realtime_deployment_config
from config.video import VideoConfig, load_video_config
from config.shared_config import OperatingConfig, default_operating_config
from realtime_server.drone_sdk import DroneSdkConnection, WebSocketDroneSdkAdapter
from realtime_server.protocol import (
    MANUAL_INPUT_SOURCES,
    MANUAL_INPUT_TABLET,
    ROLE_DRONE,
    MESSAGE_TYPE_OPERATING_CONFIG_RESULT,
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
from services.gps import DEFAULT_HOME, Wgs84Point, wgs84_distance_m
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
HEALTH_PATH = "/healthz"

# Matches adapters/dji/flight_bridge.py's DJI_TELEMETRY_SOURCE and Android's
# DroneWireProtocol.kt -- the only two things that ever report this source.
# The simulator always reports "simulator" (services/telemetry.py's
# SIMULATOR_SOURCE), so gating session-home capture on this string is what
# keeps simulator/browser mode on DEFAULT_HOME unconditionally.
DJI_TELEMETRY_SOURCE = "dji"
# DjiTelemetryMapper.kt (Android) and flight_bridge.py's own phase1 telemetry
# both report this state string exactly when the aircraft is grounded -- the
# one grounded/airborne signal common to *every* path telemetry can reach
# this server through (the AWS bridge, or a tablet connecting directly as
# role=drone), so it's what session-home capture keys off here.
# Both of FlightLogic's on-ground states. IDLE is the one a fresh session
# actually reports: LANDED is only reached after an *autonomous* landing, so a
# manually flown session never leaves IDLE and would never confirm a session
# home. flight_bridge.py's kinematic step already treats the pair identically.
DJI_GROUNDED_STATES = frozenset({"IDLE", "LANDED"})

# Aircraft flight states that name a mission unambiguously. Target chases,
# manual, holding and repositioning do not: the mission underneath is
# unchanged, so they never re-synchronise the server.
AIRCRAFT_MISSION_STATES: dict[str, MissionState] = {
    "TAKING_OFF": MissionState.TAKING_OFF,
    "PATROLLING": MissionState.PATROL,
    "TRANSITING_TO_ROI": MissionState.TRANSIT_TO_ROI,
    "ROI_PATROL": MissionState.ROI_PATROL,
    "RETURNING_HOME": MissionState.RTH,
    "LANDING": MissionState.LANDING,
}
# Longer than a command's round trip and a telemetry sample in flight, so a
# just-forwarded command is never undone by telemetry that predates it.
MISSION_RESYNC_AFTER_S = 1.0


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
                    os.getenv(
                        "DRONTECT_TAKEOFF_ALTITUDE_M",
                        str(default_operating_config.cruise_altitude_m),
                    )
                ),
                roi_arrival_tolerance_m=float(
                    os.getenv("DRONTECT_ROI_ARRIVAL_TOLERANCE_M", "10.0")
                ),
            )
        )
        self.operating_config = OperatingConfig(
            max_flight_radius_m=default_operating_config.max_flight_radius_m,
            cruise_altitude_m=self.mission.config.takeoff_altitude_m,
        )
        self.patrol_regions = PatrolRegionStore(
            home_radius_m=self.operating_config.max_flight_radius_m
        )
        self.patrol_planner = PatrolPlanner(store=self.patrol_regions)
        # Real-DJI session home, captured once from the first grounded DJI
        # telemetry sample (see _route_drone_telemetry) and never moved again
        # for this process's lifetime -- the same "one process lifetime is
        # one aircraft session" rule adapters/dji/flight_bridge.py uses, kept
        # independent of it since telemetry can reach this server either via
        # that bridge or directly from a tablet (DroneWireProtocol.kt).
        # Stays None (and every home-dependent computation below keeps using
        # DEFAULT_HOME) for simulator/browser mode, which never reports
        # source="dji".
        self._session_home: Wgs84Point | None = None
        self._operating_config_revision = 0
        self._drone_config_ready = False
        self._pending_drone_config_results: dict[str, asyncio.Future[dict]] = {}
        self._operating_config_pending = False
        # Set only while the drone has accepted a config change but deferred
        # actually applying it (mid pursuit/reposition); swapped into the
        # active operating_config/patrol_regions/patrol_planner/mission.config
        # once telemetry reports operatingConfigPending flipping back to
        # False. Kept separate so the published config -- and anything
        # derived from it, like the map's boundary circle and patrol grid --
        # never claims a radius/altitude the drone is not actually enforcing
        # yet.
        self._pending_operating_config_target: tuple[
            OperatingConfig, PatrolRegionStore, PatrolPlanner, MissionCommandConfig
        ] | None = None
        self.safety_rth_policy = safety_rth_policy or DefaultSafetyRthPolicy(
            low_battery_threshold_percent=float(
                os.getenv("DRONTECT_LOW_BATTERY_RTH_PERCENT", "20.0")
            )
        )
        self._battery_percent: float | None = None
        self._low_battery = False
        # Last flight state the drone reported, for the airborne -> grounded
        # touchdown edge. None on a fresh drone connection.
        self._drone_reported_state: str | None = None
        # Mission/aircraft disagreement timer -- see _resync_mission_with_aircraft.
        self._mission_mismatch_since: float | None = None
        self._clock = time.monotonic
        # The operator hold. Held here so a reconnecting drone is told about
        # it -- a hold that silently lapsed into "pursuing" when the link
        # blinked would be the worst possible failure for this control.
        self.pursuit_blocked = False
        # Manual takeover. Unlike the pursuit hold this is NOT re-asserted on a
        # drone reconnect: the hold only ever makes the aircraft do less, while
        # manual makes it do more, and a controller that restarted has no idea
        # what the sticks were doing. The aircraft is the authority -- this is
        # refreshed from its telemetry and only mirrored here for the tablet.
        self.manual_control = False
        self.manual_input_source = MANUAL_INPUT_TABLET
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
        self._drone_config_ready = self._operating_config_revision == 0
        sync_task: asyncio.Task | None = None
        if self._drone_config_ready:
            await self._send_pursuit_block_to_drone(drone)
            await self._send_patrol_priorities_to_drone(drone)
            await self._send_patrol_area_types_to_drone(drone)
        else:
            sync_task = asyncio.create_task(self._synchronize_operating_config(drone))
        await self._send_to_tablet(encode_message(self._mission_status()))
        try:
            async for message in drone.iter_telemetry():
                if message.get("type") == MESSAGE_TYPE_OPERATING_CONFIG_RESULT:
                    self._resolve_operating_config_result(message)
                else:
                    await self._route_drone_telemetry(message)
        finally:
            if sync_task is not None:
                sync_task.cancel()
            if self.drone is drone:
                self.drone = None
                self._drone_reported_state = None
                self._drone_config_ready = False
                self._operating_config_pending = False
                # Discard rather than apply on reconnect: a fresh drone
                # connection gets synced with the currently-active config
                # (see _synchronize_operating_config), and this stashed
                # target reflects a maneuver on the connection that just
                # dropped, not necessarily anything the new one is doing.
                self._pending_operating_config_target = None
                self._reject_pending_drone_config_results(
                    "Drone disconnected during operating configuration"
                )
                await self._send_to_tablet(encode_message(self._mission_status()))

    async def _synchronize_operating_config(self, drone: DroneSdkConnection) -> None:
        command = self._operating_config_command(
            f"operating-config-sync-{uuid.uuid4()}"
        )
        try:
            result = await self._request_drone_operating_config(drone, command)
            if not result["accepted"] or self.drone is not drone:
                return
            self._drone_config_ready = True
            self._operating_config_pending = bool(result.get("pending", False))
            await self._send_pursuit_block_to_drone(drone)
            await self._send_patrol_priorities_to_drone(drone)
            await self._send_patrol_area_types_to_drone(drone)
            await self._send_to_tablet(encode_message(self._mission_status()))
        except (ConnectionClosed, ConnectionError, TimeoutError):
            if self.drone is drone:
                self._drone_config_ready = False

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
        reported_state = data["state"]
        if (
            self._drone_reported_state is not None
            and self._drone_reported_state not in DJI_GROUNDED_STATES | {"UNKNOWN"}
            and reported_state in DJI_GROUNDED_STATES
        ):
            # Before observe_safety, so a low battery on the landing sample
            # cannot re-latch an RTH onto a mission that has just ended.
            before = self.mission.state
            self.mission.observe_touchdown()
            print(f"realtime_server touchdown reported={self._drone_reported_state}->"
                  f"{reported_state} mission={before.value}->{self.mission.state.value}")
        self._drone_reported_state = reported_state
        reported_block = data.get("pursuitBlocked")
        if isinstance(reported_block, bool):
            self.pursuit_blocked = reported_block
        reported_manual = data.get("manualControlActive")
        if isinstance(reported_manual, bool):
            self.manual_control = reported_manual
        reported_manual_source = data.get("manualInputSource")
        if reported_manual_source in MANUAL_INPUT_SOURCES:
            self.manual_input_source = reported_manual_source
        was_pending = self._operating_config_pending
        self._operating_config_pending = bool(data.get("operatingConfigPending", False))
        if (
            was_pending
            and not self._operating_config_pending
            and self._pending_operating_config_target is not None
        ):
            (
                self.operating_config,
                self.patrol_regions,
                self.patrol_planner,
                self.mission.config,
            ) = self._pending_operating_config_target
            self._pending_operating_config_target = None
            self._operating_config_revision += 1
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
        self._resync_mission_with_aircraft(data["state"])
        if (
            self._session_home is None
            and telemetry.get("source") == DJI_TELEMETRY_SOURCE
            and data["state"] in DJI_GROUNDED_STATES
        ):
            # First grounded real-DJI fix, from whichever path it arrived
            # through (the AWS bridge or a tablet connecting directly) --
            # captured once, mirrored here independently of
            # flight_bridge.py's own session_home so this server's SET_ROI
            # validation, patrol_regions copy, and the home it reports to
            # tablets all agree with whatever geometry the aircraft itself is
            # actually flying against.
            self._session_home = Wgs84Point(lat=gps["lat"], lng=gps["lng"])
            store, planner, _ = self._prepare_operating_config(self.operating_config)
            self.patrol_regions, self.patrol_planner = store, planner
        await self._send_to_tablet(encode_message(self._mission_status()))
        await self._send_to_tablet(encode_message(telemetry))

    def _resync_mission_with_aircraft(self, reported_state: str) -> None:
        """Follow the aircraft's mission when it persistently disagrees."""
        observed = AIRCRAFT_MISSION_STATES.get(reported_state)
        if observed is None or observed is self.mission.state:
            self._mission_mismatch_since = None
            return
        now = self._clock()
        if self._mission_mismatch_since is None:
            self._mission_mismatch_since = now
            return
        if now - self._mission_mismatch_since < MISSION_RESYNC_AFTER_S:
            return
        before = self.mission.state
        after = self.mission.adopt_aircraft_state(observed)
        self._mission_mismatch_since = None
        if after is not before:
            print(f"realtime_server mission_resync aircraft={reported_state} "
                  f"mission={before.value}->{after.value}")

    async def _forward_to_drone(self, command: dict) -> None:
        """Pass a validated command through to the aircraft, if one is linked."""
        drone = self.drone
        if drone is None:
            return
        try:
            await drone.send_command(command)
        except ConnectionClosed:
            if self.drone is drone:
                self.drone = None

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

    async def _send_pursuit_block_to_drone(
        self,
        drone: DroneSdkConnection,
    ) -> None:
        """Re-assert the operator hold on a freshly connected drone.

        A controller starts unblocked, so only an engaged hold needs sending
        -- but it must be sent, or a controller that restarted mid-hold comes
        back pursuing because the command predated its connection.
        """
        if not self.pursuit_blocked:
            return
        command = {
            "type": "command",
            "commandId": f"pursuit-block-{uuid.uuid4()}",
            "command": CommandName.SET_PURSUIT_BLOCK.value,
            "params": {"blocked": self.pursuit_blocked},
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
        if command_name is CommandName.SET_MANUAL_CONTROL:
            engaged = bool(command["params"]["engaged"])
            if engaged and self.mission.safety_rth_active:
                # Taking the sticks away from a safety RTH is the one takeover
                # that is never allowed. Releasing always is.
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
            await self._forward_to_drone(command)
            await tablet.send(encode_message(command_ack(command["commandId"])))
            # The aircraft confirms the latch in its own telemetry; this only
            # keeps the tablet from waiting a frame to see the button change.
            self.manual_control = engaged
            await tablet.send(encode_message(self._mission_status()))
            return
        if command_name is CommandName.SET_MANUAL_COMMAND:
            # Sticks are forwarded and forgotten. Nothing here may remember
            # them: a stale stick re-asserted later flies the aircraft.
            await self._forward_to_drone(command)
            await tablet.send(encode_message(command_ack(command["commandId"])))
            return
        if command_name is CommandName.SET_PURSUIT_BLOCK:
            self.pursuit_blocked = bool(command["params"]["blocked"])
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
        if command_name is CommandName.SET_OPERATING_CONFIG:
            await self._route_operating_config_command(tablet, command)
            return
        if (
            self.mission.safety_rth_active
            and command_name is CommandName.REJECT_TARGET
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

    async def _route_operating_config_command(
        self,
        tablet: ServerConnection,
        command: dict,
    ) -> None:
        # Whether this can apply right now (grounded), must reposition first
        # (patrol/ROI), or must defer (mid pursuit/follow) is entirely the
        # drone's call, based on its own FlightState -- the server's
        # MissionState has no equivalent granularity to reproduce that here.
        params = command["params"]
        try:
            candidate = OperatingConfig(
                max_flight_radius_m=params["maxFlightRadiusM"],
                cruise_altitude_m=params["cruiseAltitudeM"],
            )
        except ValueError as exc:
            # _validate_command_params already enforces the same bounds, so
            # this should be unreachable through any validated tablet
            # command -- kept as a defense-in-depth backstop.
            await tablet.send(
                encode_message(
                    ProtocolError(
                        "invalid_params",
                        str(exc),
                        command_id=command["commandId"],
                    ).to_message()
                )
            )
            return
        candidate_store, candidate_planner, candidate_mission_config = (
            self._prepare_operating_config(candidate)
        )

        drone = self.drone
        if drone is not None:
            try:
                result = await self._request_drone_operating_config(drone, command)
            except (ConnectionClosed, ConnectionError, TimeoutError):
                await tablet.send(
                    encode_message(
                        ProtocolError(
                            "drone_unavailable",
                            "Drone did not confirm the operating configuration.",
                            command_id=command["commandId"],
                        ).to_message()
                    )
                )
                return
            if not result["accepted"]:
                await tablet.send(
                    encode_message(
                        ProtocolError(
                            "invalid_mission_transition",
                            result.get("message", "Drone rejected the operating configuration."),
                            command_id=command["commandId"],
                        ).to_message()
                    )
                )
                return

        pending = bool(result.get("pending", False)) if drone is not None else False

        # Every fallible check and object construction has completed. If the
        # drone deferred actually applying this (mid pursuit/reposition),
        # stash it instead of publishing it now -- otherwise the map's
        # boundary circle and patrol grid would jump to the new radius while
        # the drone keeps enforcing the old one, until whatever it's mid-way
        # through settles (see _route_drone_telemetry, which swaps this in
        # once telemetry reports operatingConfigPending clearing). Publish
        # the complete pair and all derived backend state without an await in
        # between, so readers can never observe a half-applied configuration.
        if pending:
            self._pending_operating_config_target = (
                candidate, candidate_store, candidate_planner, candidate_mission_config,
            )
        else:
            self._pending_operating_config_target = None
            (
                self.operating_config,
                self.patrol_regions,
                self.patrol_planner,
                self.mission.config,
            ) = (
                candidate,
                candidate_store,
                candidate_planner,
                candidate_mission_config,
            )
            self._operating_config_revision += 1
        self._drone_config_ready = drone is not None
        self._operating_config_pending = pending
        await tablet.send(encode_message(command_ack(command["commandId"])))
        await tablet.send(encode_message(self._mission_status()))

    def _effective_home(self) -> Wgs84Point:
        return self._session_home if self._session_home is not None else DEFAULT_HOME

    def _prepare_operating_config(
        self,
        candidate: OperatingConfig,
    ) -> tuple[PatrolRegionStore, PatrolPlanner, MissionCommandConfig]:
        priorities = [
            (region.id, region.priority)
            for region in self.patrol_regions.regions
            if region.priority is not None
        ]
        area_types = [
            (region.id, region.area_type)
            for region in self.patrol_regions.regions
            if region.area_type is not PatrolAreaType.PATROL
        ]
        store = PatrolRegionStore(home=self._effective_home(), home_radius_m=candidate.max_flight_radius_m)
        store.replace_area_types(area_types)
        store.replace_priorities(priorities)
        planner = PatrolPlanner(
            store=store,
            scorer=self.patrol_planner.scorer,
            started_at=self.patrol_planner.started_at,
        )
        mission_config = replace(
            self.mission.config,
            takeoff_altitude_m=candidate.cruise_altitude_m,
        )
        return store, planner, mission_config

    def _operating_config_command(self, command_id: str) -> dict:
        return {
            "type": "command",
            "commandId": command_id,
            "command": CommandName.SET_OPERATING_CONFIG.value,
            "params": {
                "maxFlightRadiusM": self.operating_config.max_flight_radius_m,
                "cruiseAltitudeM": self.operating_config.cruise_altitude_m,
            },
        }

    async def _request_drone_operating_config(
        self,
        drone: DroneSdkConnection,
        command: dict,
    ) -> dict:
        command_id = command["commandId"]
        future = asyncio.get_running_loop().create_future()
        self._pending_drone_config_results[command_id] = future
        try:
            await drone.send_command(command)
            return await asyncio.wait_for(future, timeout=5.0)
        finally:
            self._pending_drone_config_results.pop(command_id, None)

    def _resolve_operating_config_result(self, result: dict) -> None:
        future = self._pending_drone_config_results.get(result["commandId"])
        if future is not None and not future.done():
            future.set_result(result)

    def _reject_pending_drone_config_results(self, message: str) -> None:
        for future in self._pending_drone_config_results.values():
            if not future.done():
                future.set_exception(ConnectionError(message))

    async def _route_mission_command(
        self,
        tablet: ServerConnection,
        command: dict,
        command_name: CommandName,
    ) -> None:
        if command_name is CommandName.SET_ROI:
            params = command["params"]
            center = params["center"]
            distance_from_home_m = wgs84_distance_m(
                Wgs84Point(lat=center["lat"], lng=center["lon"]),
                self._effective_home(),
            )
            if (
                distance_from_home_m
                >= self.operating_config.max_flight_radius_m + params["radiusM"]
            ):
                await tablet.send(
                    encode_message(
                        ProtocolError(
                            "invalid_params",
                            "ROI does not overlap the maximum flight radius.",
                            command_id=command["commandId"],
                            details={"field": "center"},
                        ).to_message()
                    )
                )
                return
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
            drone_connected=self.drone is not None and self._drone_config_ready,
            patrol_area_types=area_types,
            operating_config={
                "maxFlightRadiusM": self.operating_config.max_flight_radius_m,
                "cruiseAltitudeM": self.operating_config.cruise_altitude_m,
            },
            operating_config_pending=self._operating_config_pending,
            session_home=(
                {"lat": self._session_home.lat, "lng": self._session_home.lng}
                if self._session_home is not None
                else None
            ),
            pursuit_blocked=self.pursuit_blocked,
            manual_control_active=self.manual_control,
            manual_input_source=self.manual_input_source,
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


def _process_health_check(connection: ServerConnection, request: Request) -> Response | None:
    """Answer a plain HTTP GET to HEALTH_PATH without a WebSocket handshake,
    for container/load-balancer health probes. Any other request continues
    through the normal WebSocket upgrade unchanged."""
    if request.path == HEALTH_PATH:
        return connection.respond(200, "ok\n")
    return None


async def run_realtime_server(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    deployment: DeploymentConfig | None = None,
) -> None:
    deployment = deployment or DeploymentConfig()
    realtime_server = RealtimeServer()
    realtime_server.start_background_tasks()
    async with serve(
        realtime_server.handler,
        host,
        port,
        max_size=VIDEO_MAX_PACKET_BYTES,
        process_request=_process_health_check,
        origins=deployment.origins,
        ssl=build_ssl_context(deployment.tls),
    ):
        print(f"Drontect realtime server listening on ws://{host}:{port}{WS_PATH}")
        await asyncio.Future()


def main() -> None:
    host = os.getenv("DRONTECT_REALTIME_HOST", DEFAULT_HOST)
    port = int(os.getenv("DRONTECT_REALTIME_PORT", str(DEFAULT_PORT)))
    deployment = load_realtime_deployment_config()
    asyncio.run(run_realtime_server(host, port, deployment))


if __name__ == "__main__":
    main()
