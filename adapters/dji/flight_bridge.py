"""Local flight-bridge process for a real DJI aircraft.

Runs the existing pure-Python autonomy stack (flight_logic / pursuit /
roi_patrol / target_decisions / manual_input / patrol_planner /
polygon_coverage / no_fly_router / mission_interruption / patrol_progress /
operating_config_apply, all under controllers/autonomous_drone/ and
services/) against a real DJI Mini 4 Pro + RC-N3 instead of the Webots
simulator, without changing or duplicating any of that logic.

Architecture (see docs/deployment.md and the prior conversation's
architecture analysis for the full picture):

    tablet(s) <--wss--> realtime_server (cloud, unchanged)
                              ^
                              | role=drone, same protocol the Webots
                              | controller already speaks
                              v
                     DjiFlightBridge (this module -- runs locally,
                     e.g. on a companion laptop on the same LAN/hotspot
                     as the tablet, NOT in the cloud: the tight per-tick
                     control loop cannot tolerate cloud round-trip
                     latency the way event-driven mission commands can)
                              ^
                              | local-only protocol, see local_protocol.py
                              | (telemetry/RC/gimbal state up, velocity+yaw
                              | + gimbal setpoints + RTH down)
                              v
                       Android DJI app (NOT implemented yet -- out of
                       scope for this change) <--DJI Mobile SDK V5--> RC-N3
                              --OcuSync--> Mini 4 Pro

What this module does:
    - Connects to realtime_server as role=drone (via SimulatorRealtimeClient,
      reused unmodified -- its WS transport is already source-agnostic) and
      receives the existing mission commands.
    - Runs a local WebSocket server (AndroidLink) for the future Android app.
    - Drives flight_logic.FlightLogic's tick using position/yaw sourced from
      Android telemetry instead of Webots GPS/IMU devices, and sends the
      resulting velocity/yaw/gimbal setpoints to Android instead of
      teleporting a simulated aircraft body (Supervisor.translation, in
      autonomous_drone.py) -- that Supervisor call is the *only* thing
      "flight control" meant in the simulator, since the simulation is
      kinematic, not physics-driven: propeller motors there are cosmetic
      ("_set_visual_motor_velocities"), not something to port at all.
    - Wires region-based PATROL (patrol_planner region scoring +
      polygon_coverage lane-sweep waypoints + no_fly_router routing),
      mission-interruption/resume-coverage (detection -> acquisition/follow/
      pursuit -> lost/rejected -> resume PATROL/ROI, including paused
      -coverage checkpoints), and full SET_OPERATING_CONFIG parity
      (services/operating_config_apply.py's plan_operating_config_transition,
      in-flight repositioning, stored-waypoint altitude retargeting) -- all
      by calling the exact same pure functions/methods
      autonomous_drone.py calls, reproduced here only because that file
      cannot be imported without Webots installed (see "Why this is a port,
      not an import" below).

Real-flight configuration (see DjiFlightBridge.__init__ and
_apply_operating_config): FlightConfig()'s dataclass defaults are
SIMULATOR-SCALE TEST-ARENA VALUES (x_bounds/y_bounds fixed at an 8x8 m box;
takeoff_altitude/z_bounds only become real-world-scale once an
OperatingConfig is applied). DjiFlightBridge refuses to run flight_logic at
all -- no command dispatch beyond SET_OPERATING_CONFIG itself, no
FlightLogic.update(), no kinematic advancement -- until an OperatingConfig
(the existing config/shared_config.py structure, with its existing
legal-maximum validation) has been applied, either passed explicitly to the
constructor or delivered by a SET_OPERATING_CONFIG command at runtime. There
is no silent fallback to default_operating_config: "not yet configured" is
the same as "not yet safe to fly."

Why this is a port, not an import: autonomous_drone.py has a module-level
`from controller import Camera, GPS, InertialUnit, Motor, Supervisor`
(Webots' proprietary API), so nothing in that file -- including its
Webots-independent methods -- can be imported without Webots installed. Every
method ported here calls the exact same underlying pure functions/objects
(flight_logic.py, pursuit.py, roi_patrol.py, manual_input.py,
target_decisions.py, services/*) that autonomous_drone.py calls; none of
their algorithms are reimplemented.

Canonical tuning: FlightConfig/ManualFlightConfig/PursuitConfig and the
per-flight-state KinematicFlightConfig selection (patrol/search/manual/
target-tracking speed and acceleration profiles) all come from
controllers/autonomous_drone/flight_tuning.py -- the module
build_flight_config()/build_manual_flight_config()/build_pursuit_config()/
KINEMATIC_FLIGHT_CONFIG/SEARCH_KINEMATIC_FLIGHT_CONFIG/
MANUAL_KINEMATIC_FLIGHT_CONFIG/kinematic_config_for_state were extracted
into so both autonomous_drone.py and this bridge import the exact same
values, unduplicated. See that module's docstring for why it lives there and
how it stays import-safe under both Webots' flat module loading and this
bridge's normal dotted imports. The one field left as a genuine per-caller
override is `camera_horizontal_fov_rad` (a hardware property, not tuning --
see DjiFlightBridge's docstring).

Deliberately still deferred (see the parity-gap-closing conversation for the
full reasoning):
    - Android DJI Mobile SDK V5 integration itself. Nothing in this package
      makes a DJI SDK call; that stays entirely on the Android side, which
      does not exist yet.
"""

from __future__ import annotations

import asyncio
import math
import os
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Callable

from adapters.dji.android_link import DEFAULT_HOST, DEFAULT_PORT, AndroidLink
from adapters.simulator.client import SimulatorRealtimeClient
from config.shared_config import OperatingConfig, shared_config
from controllers.autonomous_drone.camera_control import CameraGimbal
from controllers.autonomous_drone.flight_logic import (
    FlightConfig,
    FlightLogic,
    FlightState,
    KinematicFlightStep,
    ManualFlightConfig,
    ManualStick,
    Position,
    advance_kinematic_flight,
    manual_velocity_command,
)
from controllers.autonomous_drone.flight_tuning import (
    build_flight_config,
    build_manual_flight_config,
    build_pursuit_config,
    kinematic_config_for_state,
    target_tracking_kinematic_config,
)
from controllers.autonomous_drone.manual_input import ManualInputRouter
from controllers.autonomous_drone.pursuit import PursuitConfig
from controllers.autonomous_drone.roi_patrol import LawnMowerRoiPatrolPlanner
from realtime_server.protocol import operating_config_result
from services.commands import CommandName
from services.flight_safety import CONNECTION_LOSS_REASON
from services.gps import DEFAULT_HOME, LocalPosition, Wgs84Point, local_meters_to_wgs84, wgs84_to_local_meters
from services.mission_interruption import InterruptedMissionType, MissionInterruptionContext
from services.no_fly_router import NoFlyRouteUnavailable, NoFlyRouter
from services.operating_config_apply import plan_operating_config_transition
from services.patrol_planner import PatrolPlanner
from services.patrol_progress import PausedPatrolCoverage
from services.patrol_regions import PatrolAreaType, PatrolRegion, PatrolRegionStore, patrol_region_polygon_points
from services.polygon_coverage import CoveragePathUnavailable, PolygonCoverageConfig, PolygonCoveragePlanner

DEFAULT_REALTIME_WS_URL = "ws://127.0.0.1:8765/ws?role=drone"
DJI_TELEMETRY_SOURCE = "dji"

ANDROID_LINK_LOSS_REASON = "android_link_lost"

# Matches autonomous_drone.py's PolygonCoverageConfig(lane_spacing_m=...)
# default (DRONTECT_PATROL_LANE_SPACING_M's fallback there).
DEFAULT_PATROL_LANE_SPACING_M = 75.0
# Matches autonomous_drone.py's DRONTECT_MISSION_RETURN_TIMEOUT_S fallback.
DEFAULT_MISSION_RETURN_TIMEOUT_S = 10.0

# States in which losing a command/telemetry link means "still in the air,
# bring it home" rather than "already on the ground, nothing to do". Mirrors
# autonomous_drone.py's _enforce_connection_loss_rth exactly.
_AIRBORNE_STATES = frozenset(
    {
        FlightState.TAKING_OFF,
        FlightState.HOLDING,
        FlightState.TRANSITING_TO_ROI,
        FlightState.ROI_PATROL,
        FlightState.PATROLLING,
        FlightState.REPOSITIONING,
        FlightState.TARGET_ACQUISITION,
        FlightState.TARGET_FOLLOW,
        FlightState.TARGET_PURSUIT,
        FlightState.TARGET_RETURN,
        FlightState.RETURNING_HOME,
        FlightState.MANUAL,
        FlightState.LANDING,
    }
)

# Mirrors autonomous_drone.py's module-level TARGET_TRACKING_STATES exactly.
TARGET_TRACKING_STATES = frozenset(
    {FlightState.TARGET_ACQUISITION, FlightState.TARGET_FOLLOW, FlightState.TARGET_PURSUIT}
)

_RESUME_PATROL_ELIGIBLE_STATES = frozenset(
    {FlightState.TAKING_OFF, FlightState.TRANSITING_TO_ROI, FlightState.ROI_PATROL}
)


class RealFlightConfigurationError(ValueError):
    """Raised when an OperatingConfig is required but missing or invalid."""


class AndroidRemoteController:
    """manual_input.RemoteController implementation backed by AndroidLink.

    manual_input.py's RemoteController Protocol documents itself as
    "Implemented by a real drone adapter (DJI and friends)" -- this is that
    implementation. ManualInputRouter needs no changes at all: it already
    prefers this controller's stick over the tablet's whenever `connected`
    is true, which is exactly the RC-N3-outranks-tablet behavior the wire
    protocol's `manualInputSource` field was built for.
    """

    def __init__(self, android_link: AndroidLink) -> None:
        self._android_link = android_link

    @property
    def connected(self) -> bool:
        state = self._android_link.latest_rc_state
        return state is not None and state.connected

    def read_stick(self) -> ManualStick | None:
        state = self._android_link.latest_rc_state
        if state is None or not state.connected:
            return None
        return ManualStick(forward=state.forward, right=state.right, up=state.up, yaw_rate=state.yaw_rate)


@dataclass(frozen=True, slots=True)
class BridgeConfig:
    """Bridge-specific tuning, kept separate from FlightConfig/ManualFlightConfig
    /PursuitConfig so tests can adjust just these without touching autonomy
    tuning."""

    android_telemetry_timeout_s: float = 1.0
    android_connection_loss_rth_s: float = 3.0
    realtime_connection_loss_rth_s: float = 5.0
    setpoint_ttl_ms: int = 250
    gimbal_command_ttl_ms: int = 1000
    manual_stick_command_timeout_s: float = 0.5
    mission_return_timeout_s: float = DEFAULT_MISSION_RETURN_TIMEOUT_S
    patrol_lane_spacing_m: float = DEFAULT_PATROL_LANE_SPACING_M
    tick_interval_s: float = 0.05  # 20 Hz


@dataclass(frozen=True, slots=True)
class TickOutcome:
    """What one DjiFlightBridge.tick() call decided, for the async caller to act on."""

    has_position: bool
    position: Position | None = None
    yaw_rad: float | None = None
    speed_mps: float = 0.0
    vertical_speed_mps: float = 0.0
    # Local ENU meters/second, reconstructed from the pre- and post-tick
    # positions rather than reached out of advance_kinematic_flight's
    # branches -- see its call site in tick() for why that's both simpler
    # and exact (it's a plain explicit-Euler integrator: next = pos + v*dt).
    velocity_x_mps: float = 0.0
    velocity_y_mps: float = 0.0
    velocity_z_mps: float = 0.0
    gimbal_pitch_rad: float = 0.0
    battery_percent: float | None = None
    state: str = FlightState.IDLE.value
    safety_rth_active: bool = False
    safety_rth_reason: str | None = None
    manual_control_active: bool = False
    manual_input_source: str = "tablet"
    pursuit_blocked: bool = False
    target_patrol_region_id: str | None = None


class DjiFlightBridge:
    """Wires the existing autonomy stack to a real DJI aircraft via AndroidLink.

    IMPORTANT: no autonomous movement happens until an OperatingConfig has
    been applied -- either passed as `operating_config` here, or delivered
    later by a SET_OPERATING_CONFIG command. See the module docstring's
    "Real-flight configuration" section.

    `flight_config`/`manual_flight_config`/`pursuit_config` default to
    flight_tuning.build_flight_config()/build_manual_flight_config()/
    build_pursuit_config() -- the exact same canonical, tuned values
    autonomous_drone.py flies with -- not bare FlightConfig()/
    ManualFlightConfig()/PursuitConfig() dataclass defaults. Per-flight-state
    kinematic tuning (patrol/search/manual/target-tracking speed and
    acceleration profiles) is likewise the shared
    flight_tuning.kinematic_config_for_state, not a single flat
    KinematicFlightConfig -- there is no bridge-specific override for either,
    since neither is a DJI hardware difference. Pass these explicitly only to
    override tuning for a test; production use should rely on the shared
    defaults.

    `camera_horizontal_fov_rad` is the one field this refactor deliberately
    still leaves as a distinct override point: it is a genuine hardware
    difference (Webots' simulated camera vs. the Mini 4 Pro's real lens), not
    tuning -- autonomous_drone.py sets it separately too
    (`self.camera.getFov()`), after build_pursuit_config() runs. Unset, it
    stays at PursuitConfig's bare dataclass default, which is very likely
    wrong for the real lens -- set it from the Mini 4 Pro's calibrated FOV
    before relying on pursuit/target-decision geometry.
    """

    def __init__(
        self,
        *,
        realtime_link: SimulatorRealtimeClient | None = None,
        android_link: AndroidLink | None = None,
        operating_config: OperatingConfig | None = None,
        flight_config: FlightConfig | None = None,
        manual_flight_config: ManualFlightConfig | None = None,
        pursuit_config: PursuitConfig | None = None,
        camera_horizontal_fov_rad: float | None = None,
        bridge_config: BridgeConfig | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.realtime_link = realtime_link or SimulatorRealtimeClient(
            url=os.getenv("DRONTECT_DJI_REALTIME_WS_URL", DEFAULT_REALTIME_WS_URL)
        )
        self.android_link = android_link or AndroidLink(
            host=os.getenv("DRONTECT_DJI_LOCAL_HOST", DEFAULT_HOST),
            port=int(os.getenv("DRONTECT_DJI_LOCAL_PORT", str(DEFAULT_PORT))),
        )
        self.config = bridge_config or BridgeConfig()
        self.manual_flight_config = manual_flight_config or build_manual_flight_config()
        self._clock = clock

        self.patrol_regions = PatrolRegionStore()
        self.patrol_planner = PatrolPlanner(store=self.patrol_regions)
        self.no_fly_router = NoFlyRouter()
        self.polygon_coverage_planner = PolygonCoveragePlanner(
            PolygonCoverageConfig(lane_spacing_m=self.config.patrol_lane_spacing_m)
        )
        self.camera_gimbal = CameraGimbal()
        self.remote_controller = AndroidRemoteController(self.android_link)
        self.manual_input = ManualInputRouter(
            command_timeout_s=self.config.manual_stick_command_timeout_s,
            remote_controller=self.remote_controller,
        )

        # Patrol coverage progress -- mirrors autonomous_drone.py's instance
        # fields of the same name exactly.
        self._patrol_route_waypoints: deque[Position] = deque()
        self._target_patrol_region_id: str | None = None
        self._paused_patrol_coverage: PausedPatrolCoverage | None = None
        self._active_roi_id: str | None = None
        self._mission_interruption: MissionInterruptionContext | None = None
        self._mission_return_prepared = False
        self._mission_return_waiting_since: float | None = None
        self._mission_return_timeout_s = self.config.mission_return_timeout_s
        self._pending_operating_config: OperatingConfig | None = None

        self.logic = FlightLogic(
            config=flight_config or build_flight_config(),
            pursuit_config=pursuit_config
            or build_pursuit_config(
                geofence_radius_m=max(region.outer_radius_m for region in self.patrol_regions.regions)
            ),
            roi_patrol_planner=LawnMowerRoiPatrolPlanner(),
            patrol_waypoint_provider=self._next_patrol_waypoint,
        )
        if camera_horizontal_fov_rad is not None:
            self.logic.pursuit.config = replace(
                self.logic.pursuit.config, horizontal_fov_rad=camera_horizontal_fov_rad
            )
        # Mirrors autonomous_drone.py's __init__ exactly -- built once here,
        # not recomputed on an operating-config change (target_max_yaw_rate_
        # rad_s does not change when altitude/radius do).
        self._target_tracking_flight_config = target_tracking_kinematic_config(
            self.logic.config.target_max_yaw_rate_rad_s
        )

        self._elapsed_time = 0.0
        self._kinematic_yaw: float | None = None
        self._horizontal_speed_mps = 0.0
        self._last_position: Position | None = None
        self._last_yaw = 0.0

        self._safety_rth_active = False
        self._safety_rth_reason: str | None = None
        self._android_connection_loss_rth_active = False
        self._realtime_connection_loss_rth_active = False

        # No autonomous movement is possible until this is not None -- see
        # the module docstring. _apply_operating_config sets it.
        self.operating_config: OperatingConfig | None = None
        if operating_config is not None:
            self._apply_operating_config(operating_config, (0.0, 0.0, 0.0))

    @property
    def is_configured_for_flight(self) -> bool:
        return self.operating_config is not None

    # -- one control cycle, synchronous and unit-testable ---------------------

    def tick(self, dt: float, *, now: float | None = None) -> TickOutcome:
        now = self._clock() if now is None else now
        dt = max(0.0, dt)
        self._elapsed_time += dt

        telemetry = self.android_link.latest_telemetry
        fresh = telemetry is not None and self.android_link.is_telemetry_fresh(
            self.config.android_telemetry_timeout_s, now
        )
        if fresh:
            assert telemetry is not None
            position, yaw = self._position_from_telemetry(telemetry)
            self._last_position = position
            self._last_yaw = yaw
            self.camera_gimbal.set_pitch(math.radians(telemetry.gimbal_pitch_deg))
            if self.logic.home is None:
                self.logic.initialize_home(position)

        self._enforce_realtime_connection_loss_failsafe()
        self._enforce_android_connection_loss_failsafe(now)

        if self._last_position is not None:
            # Mission-level intent (a command, a vision detection) is safe to
            # accept even without *current* telemetry -- it only becomes an
            # actual movement setpoint once telemetry is fresh again, gated
            # below and a second time at AndroidLink.send_setpoint itself.
            # SET_OPERATING_CONFIG is the one command allowed through even
            # while unconfigured -- it's the only way OUT of "unconfigured".
            for message in self.realtime_link.drain_commands():
                if self.operating_config is not None or message.get("command") == CommandName.SET_OPERATING_CONFIG.value:
                    self._safe_dispatch_command(message, self._last_position)
                else:
                    print(
                        f"DJI flight bridge: command {message.get('command')} ignored -- "
                        "no real-flight operating configuration has been applied yet."
                    )
            if self.operating_config is not None:
                for message in self.realtime_link.drain_detections():
                    self._handle_detection_message(message, self._last_position, self._last_yaw)

        if not fresh or self._last_position is None or self.operating_config is None:
            # No trustworthy *current* position, or no real-flight config yet:
            # freeze flight-logic advancement rather than compute a new
            # movement decision from stale data or simulator-scale bounds.
            # This is the internal half of "never continue flying a stale
            # movement command" / "refuse autonomous movement when
            # unconfigured" -- AndroidLink.send_setpoint (see android_link.py)
            # is the other half, at the network boundary.
            return TickOutcome(
                has_position=False,
                safety_rth_active=self._safety_rth_active,
                safety_rth_reason=self._safety_rth_reason,
            )

        position = self._last_position
        yaw = self._last_yaw
        state_before_detection = self.logic.state
        self._update_manual_stick()
        self.logic.update(position, dt, yaw=yaw, camera_pitch=self.camera_gimbal.pitch)
        self._coordinate_mission_resume(position, state_before_detection)
        self._consume_pending_operating_config(position)
        step = self._apply_kinematic_flight(position, yaw, dt)
        if dt > 0.0:
            velocity_x = (step.position[0] - position[0]) / dt
            velocity_y = (step.position[1] - position[1]) / dt
            velocity_z = (step.position[2] - position[2]) / dt
        else:
            velocity_x = velocity_y = velocity_z = 0.0

        battery_percent = telemetry.battery_percent if telemetry is not None else None
        return TickOutcome(
            has_position=True,
            position=step.position,
            yaw_rad=step.yaw,
            speed_mps=step.speed_mps,
            vertical_speed_mps=step.vertical_speed_mps,
            velocity_x_mps=velocity_x,
            velocity_y_mps=velocity_y,
            velocity_z_mps=velocity_z,
            gimbal_pitch_rad=self.camera_gimbal.pitch,
            battery_percent=battery_percent,
            state=self.logic.state.value,
            safety_rth_active=self._safety_rth_active,
            safety_rth_reason=self._safety_rth_reason,
            manual_control_active=self.logic.manual_control,
            manual_input_source=self.manual_input.source,
            pursuit_blocked=self.logic.pursuit_blocked,
            target_patrol_region_id=self._target_patrol_region_id,
        )

    def _position_from_telemetry(self, telemetry) -> tuple[Position, float]:
        local = wgs84_to_local_meters(
            Wgs84Point(lat=telemetry.lat, lng=telemetry.lng),
            DEFAULT_HOME,
            altitude_m=telemetry.altitude_m,
        )
        position: Position = (local.x, local.y, local.z)
        yaw = math.radians(telemetry.heading_deg)
        return position, yaw

    def _update_manual_stick(self) -> None:
        """Mirrors autonomous_drone.py's _update_manual_stick exactly."""
        if not self.logic.manual_control:
            self.manual_input.reset()
            return
        self.logic.set_manual_stick(self.manual_input.stick(self._elapsed_time))

    def _apply_kinematic_flight(self, position: Position, yaw: float, dt: float) -> KinematicFlightStep:
        """Mirrors autonomous_drone.py's _apply_kinematic_flight, minus the
        Webots Supervisor calls (translation/rotation/velocity) that applied
        the result to a simulated body -- that mutation is exactly the part
        this bridge does not need: the caller sends the resulting
        velocity/yaw to Android instead."""
        if self._kinematic_yaw is None:
            self._kinematic_yaw = yaw

        if self.logic.state in {FlightState.IDLE, FlightState.LANDED}:
            self._horizontal_speed_mps = 0.0
            return KinematicFlightStep(
                position=position, yaw=self._kinematic_yaw, horizontal_speed_mps=0.0, speed_mps=0.0, vertical_speed_mps=0.0
            )

        target = self.logic.target
        assert target is not None
        manual = self.logic.state is FlightState.MANUAL
        velocity_command = None
        desired_yaw = self.logic.target_yaw
        if manual:
            stick = self.logic.manual_stick
            velocity_command = manual_velocity_command(
                stick=stick,
                yaw=self._kinematic_yaw,
                position=position,
                home=self.logic.home or position,
                max_flight_radius_m=self.logic.pursuit.config.geofence_radius_m,
                max_altitude_m=shared_config.legal_max_cruise_altitude_m,
                dt=dt,
                config=self.manual_flight_config,
            )
            desired_yaw = (
                self._kinematic_yaw
                + stick.yaw_rate * self.manual_flight_config.max_yaw_rate_rad_s * dt
            )
        elif self.logic.state is FlightState.TARGET_PURSUIT:
            velocity_command = self.logic.pursuit.velocity

        step = advance_kinematic_flight(
            position=position,
            target=target,
            yaw=self._kinematic_yaw,
            horizontal_speed_mps=self._horizontal_speed_mps,
            dt=dt,
            config=kinematic_config_for_state(
                self.logic.state,
                target_tracking_flight_config=self._target_tracking_flight_config,
                target_yaw_rate_limit=self.logic.target_yaw_rate_limit,
            ),
            desired_yaw=desired_yaw,
            velocity_command=velocity_command,
        )
        self._kinematic_yaw = step.yaw
        self._horizontal_speed_mps = step.horizontal_speed_mps
        return step

    # -- realtime_server mission commands --------------------------------------

    def _safe_dispatch_command(self, message: dict[str, Any], position: Position) -> None:
        try:
            self._dispatch_command(message, position)
        except Exception as exc:  # noqa: BLE001 - never let one bad command kill the bridge
            print(f"DJI flight bridge: command {message.get('command')} failed: {exc}")

    def _dispatch_command(self, message: dict[str, Any], position: Position) -> None:
        """Mirrors autonomous_drone.py's _handle_realtime_commands (per
        -message body) exactly, including the mission-interruption/patrol
        -progress hooks (_active_roi_id, _clear_patrol_progress,
        _discard_mission_interruption, _pause_patrol_coverage,
        _resume_paused_patrol) and the post-dispatch mission-resume check."""
        command = CommandName(message["command"])
        if command is CommandName.SET_OPERATING_CONFIG:
            self._handle_operating_config_command(message, position)
            return

        if self._safety_rth_active and (
            command
            in {
                CommandName.START_PATROL,
                CommandName.SET_ROI,
                CommandName.CLEAR_ROI,
                CommandName.REJECT_TARGET,
            }
            or (command is CommandName.SET_MANUAL_CONTROL and bool(message["params"]["engaged"]))
        ):
            return

        tracking_before = self.logic.state

        if command == CommandName.RETURN_HOME:
            safety_reason = message.get("safetyReason")
            if isinstance(safety_reason, str):
                self._safety_rth_active = True
                self._safety_rth_reason = safety_reason
            if self.logic.abort_manual_control():
                self.manual_input.reset()
            if self.logic.request_return_home():
                self._clear_patrol_progress()
                self._active_roi_id = None
        elif command == CommandName.START_PATROL:
            self._clear_patrol_progress()
            self._active_roi_id = None
            self.logic.request_patrol(position)
        elif command == CommandName.SET_PATROL_PRIORITIES:
            priorities = message["params"]["priorities"]
            self.patrol_regions.replace_priorities([(entry["regionId"], entry["rank"]) for entry in priorities])
            self._replan_active_patrol(position)
        elif command == CommandName.SET_PATROL_AREA_TYPES:
            area_types = message["params"]["areaTypes"]
            self.patrol_regions.replace_area_types([(entry["regionId"], entry["areaType"]) for entry in area_types])
            self._replan_active_patrol(position)
        elif command == CommandName.SET_ROI:
            self._discard_mission_interruption()
            self._pause_patrol_coverage(position)
            params = message["params"]
            center = params["center"]
            local_center = wgs84_to_local_meters(
                Wgs84Point(lat=center["lat"], lng=center["lon"]),
                DEFAULT_HOME,
                altitude_m=self.logic.config.takeoff_altitude,
            )
            self.logic.request_roi(position, (local_center.x, local_center.y), params["radiusM"])
            self._active_roi_id = f"roi:{message['commandId']}"
        elif command == CommandName.CLEAR_ROI:
            self._discard_mission_interruption()
            resume_patrol = self.logic.state in _RESUME_PATROL_ELIGIBLE_STATES
            self.logic.clear_roi(position)
            self._active_roi_id = None
            if resume_patrol:
                had_checkpoint = self._paused_patrol_coverage is not None
                if had_checkpoint:
                    if self._resume_paused_patrol(position):
                        self.logic.request_patrol(position)
                else:
                    self._clear_patrol_progress()
                    self.logic.request_patrol(position)
        elif command == CommandName.CAMERA_TILT_UP:
            self.camera_gimbal.tilt_up()
        elif command == CommandName.CAMERA_TILT_DOWN:
            self.camera_gimbal.tilt_down()
        elif command == CommandName.SET_PURSUIT_BLOCK:
            self.logic.set_pursuit_blocked(bool(message["params"]["blocked"]), position)
        elif command == CommandName.SET_MANUAL_CONTROL:
            if bool(message["params"]["engaged"]):
                if self.logic.engage_manual_control(position):
                    self.manual_input.reset()
                    self._discard_mission_interruption()
            else:
                self._release_manual_control(position)
        elif command == CommandName.SET_MANUAL_COMMAND:
            params = message["params"]
            self.manual_input.accept_tablet_stick(
                ManualStick(forward=params["forward"], right=params["right"], up=params["up"], yaw_rate=params["yawRate"]),
                self._elapsed_time,
            )
        elif command == CommandName.REJECT_TARGET:
            self.logic.reject_target(message["params"]["trackId"], position)

        # Commands are dispatched before this tick's logic.update(). One that
        # ended target tracking has to bring the interrupted mission back
        # with it, or update() runs against a flight state whose backing
        # data -- the ROI sweep, cleared on capture -- has not been restored
        # yet. Mirrors autonomous_drone.py's per-command post-check exactly.
        if tracking_before is not self.logic.state and tracking_before in TARGET_TRACKING_STATES:
            self._coordinate_mission_resume(position, tracking_before)

    def _release_manual_control(self, position: Position) -> None:
        """Mirrors autonomous_drone.py's _release_manual_control exactly."""
        resume = self.logic.release_manual_control()
        if resume is None:
            return
        self.manual_input.reset()

        if resume.state is FlightState.RETURNING_HOME:
            self.logic.request_return_home()
            return

        if resume.roi is not None:
            center, radius_m = resume.roi
            try:
                self.logic.request_roi(position, center, radius_m)
            except ValueError:
                pass
            else:
                return

        self._clear_patrol_progress()
        self._active_roi_id = None
        self.logic.request_patrol(position)

    def _handle_detection_message(self, message: dict[str, Any], position: Position, yaw: float) -> None:
        """Mirrors autonomous_drone.py's _handle_vision_detections exactly:
        would_intercept() decides whether to checkpoint the current mission
        via _capture_mission_interruption() before observe_detections() (pure
        flight_logic.py, reused unmodified) advances target acquisition/
        follow/pursuit."""
        detections = message.get("detections")
        if detections is None:
            return
        allow_landing_intercept = self.logic.state is FlightState.LANDING and not self._safety_rth_active
        if self.logic.would_intercept(
            detections,
            allow_landing_intercept=allow_landing_intercept,
            frame_width=message["frameWidth"],
            frame_height=message["frameHeight"],
            position=position,
            yaw=yaw,
            camera_pitch=self.camera_gimbal.pitch,
        ):
            self._capture_mission_interruption(position)
        self.logic.observe_detections(
            detections,
            frame_width=message["frameWidth"],
            frame_height=message["frameHeight"],
            position=position,
            yaw=yaw,
            camera_pitch=self.camera_gimbal.pitch,
            allow_landing_intercept=allow_landing_intercept,
        )

    # -- region-based PATROL: waypoint provider ---------------------------------
    # Mirrors autonomous_drone.py's _next_patrol_waypoint/_coverage_route_
    # waypoints/_pause_patrol_coverage/_resume_paused_patrol/
    # _clear_patrol_progress/_replan_active_patrol/_deduplicate_waypoints
    # exactly -- pure orchestration over patrol_planner/polygon_coverage_
    # planner/no_fly_router/patrol_regions, none of it Webots-specific.

    def _next_patrol_waypoint(self, position: Position, completed: bool) -> Position:
        if self._patrol_route_waypoints:
            return self._patrol_route_waypoints.popleft()
        if completed and self._target_patrol_region_id is not None:
            self.patrol_planner.record_observation(self._target_patrol_region_id)

        try:
            exit_point = self.no_fly_router.nearest_exit((position[0], position[1]), self.patrol_regions.regions)
        except NoFlyRouteUnavailable:
            self._target_patrol_region_id = None
            self._patrol_route_waypoints.clear()
            return (position[0], position[1], self.logic.config.takeoff_altitude)
        if exit_point is not None:
            return (exit_point[0], exit_point[1], self.logic.config.takeoff_altitude)

        current = local_meters_to_wgs84(LocalPosition(x=position[0], y=position[1], z=position[2]), DEFAULT_HOME)
        no_fly_polygons = [
            patrol_region_polygon_points(region)
            for region in self.patrol_regions.regions
            if region.area_type is PatrolAreaType.NO_FLY
        ]
        candidate_waypoints: dict[str, list[Position]] = {}

        def safe_route_distance(region: PatrolRegion) -> float | None:
            try:
                ordered, transit_distance = self._coverage_route_waypoints(position, region, no_fly_polygons)
            except (CoveragePathUnavailable, NoFlyRouteUnavailable):
                return None
            candidate_waypoints[region.id] = ordered
            return transit_distance

        try:
            decision = self.patrol_planner.choose_next(current, route_distance_m=safe_route_distance)
        except RuntimeError:
            self._target_patrol_region_id = None
            self._patrol_route_waypoints.clear()
            return (position[0], position[1], self.logic.config.takeoff_altitude)

        paused = self._paused_patrol_coverage
        if paused is not None and paused.region_id == decision.region.id and self._resume_paused_patrol(position):
            return self._patrol_route_waypoints.popleft()

        self._target_patrol_region_id = decision.region.id
        ordered = candidate_waypoints[decision.region.id]
        self._patrol_route_waypoints = deque(ordered[1:])
        return ordered[0]

    def _coverage_route_waypoints(
        self, position: Position, region: PatrolRegion, no_fly_polygons: list[tuple[Wgs84Point, ...]]
    ) -> tuple[list[Position], float]:
        coverage = self.polygon_coverage_planner.plan(
            patrol_region_polygon_points(region), no_fly_polygons=no_fly_polygons, outward_from=DEFAULT_HOME
        )
        local_coverage = [
            wgs84_to_local_meters(waypoint, DEFAULT_HOME, altitude_m=self.logic.config.takeoff_altitude)
            for waypoint in coverage
        ]
        first_coverage = local_coverage[0]
        route = self.no_fly_router.route(
            (position[0], position[1]), (first_coverage.x, first_coverage.y), self.patrol_regions.regions
        )
        route_waypoints: list[Position] = [(x, y, self.logic.config.takeoff_altitude) for x, y in route]
        transit_points = [(position[0], position[1]), *((wp[0], wp[1]) for wp in route_waypoints)]
        transit_distance = sum(math.dist(start, end) for start, end in zip(transit_points, transit_points[1:]))
        coverage_waypoints: list[Position] = [(wp.x, wp.y, wp.z) for wp in local_coverage[1:]]
        return self._deduplicate_waypoints([*route_waypoints, *coverage_waypoints]), transit_distance

    def _pause_patrol_coverage(self, position: Position) -> None:
        if self.logic.state is not FlightState.PATROLLING or self._target_patrol_region_id is None or self.logic.target is None:
            return
        self._paused_patrol_coverage = PausedPatrolCoverage(
            region_id=self._target_patrol_region_id,
            interruption_position=(position[0], position[1], self.logic.config.takeoff_altitude),
            active_waypoint=self.logic.target,
            remaining_waypoints=tuple(self._patrol_route_waypoints),
        )
        self._patrol_route_waypoints.clear()
        self._target_patrol_region_id = None

    def _resume_paused_patrol(self, position: Position) -> bool:
        paused = self._paused_patrol_coverage
        if paused is None:
            return False
        try:
            exit_point = self.no_fly_router.nearest_exit((position[0], position[1]), self.patrol_regions.regions)
            route = self.no_fly_router.route(
                exit_point or (position[0], position[1]),
                (paused.interruption_position[0], paused.interruption_position[1]),
                self.patrol_regions.regions,
            )
        except NoFlyRouteUnavailable:
            return False
        if exit_point is not None:
            route = [exit_point, *route]
        return_route: list[Position] = [(x, y, self.logic.config.takeoff_altitude) for x, y in route]
        self._patrol_route_waypoints = deque(paused.resumed_waypoints(return_route))
        self._target_patrol_region_id = paused.region_id
        self._paused_patrol_coverage = None
        return True

    def _clear_patrol_progress(self) -> None:
        self._discard_mission_interruption()
        self._patrol_route_waypoints.clear()
        self._target_patrol_region_id = None
        self._paused_patrol_coverage = None

    def _replan_active_patrol(self, position: Position) -> None:
        if self.logic.state is not FlightState.PATROLLING:
            return
        self._pause_patrol_coverage(position)
        self.logic.target = self._next_patrol_waypoint(position, completed=False)

    @staticmethod
    def _deduplicate_waypoints(waypoints: list[Position]) -> list[Position]:
        deduplicated: list[Position] = []
        for waypoint in waypoints:
            if not deduplicated or math.dist(deduplicated[-1], waypoint) > 1e-7:
                deduplicated.append(waypoint)
        return deduplicated

    # -- mission interruption / resume -------------------------------------
    # Mirrors autonomous_drone.py's _coordinate_mission_resume/
    # _prepare_mission_return/_capture_mission_interruption/
    # _restore_mission_interruption/_discard_mission_interruption exactly.

    def _coordinate_mission_resume(self, position: Position, state_before_detection: FlightState) -> None:
        context = self._mission_interruption
        if context is None:
            return
        if self.logic.state is FlightState.TARGET_RETURN:
            if not self._mission_return_prepared or not self.logic.target_return_ready:
                self._prepare_mission_return(position, context)
            return
        target_states = {
            FlightState.TARGET_ACQUISITION,
            FlightState.TARGET_FOLLOW,
            FlightState.TARGET_PURSUIT,
            FlightState.TARGET_RETURN,
        }
        if (
            self.logic.tracked_target_id is None
            and self.logic.state not in target_states
            and (state_before_detection in target_states or self.logic.state.value == context.source_state)
        ):
            self._restore_mission_interruption(context)

    def _prepare_mission_return(self, position: Position, context: MissionInterruptionContext) -> None:
        try:
            route = self.no_fly_router.egress_route(
                (position[0], position[1]),
                (context.interruption_position[0], context.interruption_position[1]),
                self.patrol_regions.regions,
            )
        except NoFlyRouteUnavailable:
            if self._mission_return_waiting_since is None:
                self._mission_return_waiting_since = self._elapsed_time
            waited = self._elapsed_time - self._mission_return_waiting_since
            if waited >= self._mission_return_timeout_s:
                self._restore_mission_interruption(context)
                self._mission_return_waiting_since = None
                self.logic.abandon_target_return(position)
                return
            self.logic.hold_target_return(position)
            self._mission_return_prepared = False
            return
        altitude = self.logic.config.takeoff_altitude
        waypoints: list[Position] = [(x, y, altitude) for x, y in route]
        waypoints[-1] = context.interruption_position
        self.logic.set_target_return_route(self._deduplicate_waypoints(waypoints))
        self._mission_return_prepared = True
        self._mission_return_waiting_since = None

    def _capture_mission_interruption(self, position: Position) -> None:
        if self._mission_interruption is not None:
            return
        state = self.logic.state
        pending_state = self.logic.pending_mission_state
        effective_state = pending_state if state is FlightState.TAKING_OFF else state
        exact_position = (float(position[0]), float(position[1]), float(position[2]))

        if effective_state is FlightState.PATROLLING:
            active_waypoint = self.logic.pending_mission_waypoint if state is FlightState.TAKING_OFF else self.logic.target
            progress = None
            if self._target_patrol_region_id is not None and active_waypoint is not None:
                progress = PausedPatrolCoverage(
                    region_id=self._target_patrol_region_id,
                    interruption_position=exact_position,
                    active_waypoint=active_waypoint,
                    remaining_waypoints=tuple(self._patrol_route_waypoints),
                )
            context = MissionInterruptionContext(
                source_type=InterruptedMissionType.PATROL,
                source_state=state.value,
                interruption_position=exact_position,
                active_id=self._target_patrol_region_id,
                active_waypoint=active_waypoint,
                remaining_path=tuple(self._patrol_route_waypoints),
                progress=progress,
            )
            self._patrol_route_waypoints.clear()
            self._target_patrol_region_id = None
        elif effective_state in {FlightState.TRANSITING_TO_ROI, FlightState.ROI_PATROL}:
            roi_progress = self.logic.roi_patrol_planner.snapshot()
            active_waypoint = self.logic.pending_mission_waypoint if state is FlightState.TAKING_OFF else self.logic.target
            context = MissionInterruptionContext(
                source_type=InterruptedMissionType.ROI,
                source_state=state.value,
                interruption_position=exact_position,
                active_id=self._active_roi_id,
                active_waypoint=active_waypoint,
                remaining_path=roi_progress.remaining_path,
                progress=roi_progress,
            )
            self.logic.roi_patrol_planner.clear()
        else:
            context = MissionInterruptionContext(
                source_type=InterruptedMissionType.FLIGHT_STATE,
                source_state=state.value,
                interruption_position=exact_position,
                active_id=None,
                active_waypoint=self.logic.target,
                remaining_path=(),
            )
        self._mission_interruption = context
        self._mission_return_prepared = False
        self._mission_return_waiting_since = None

    def _restore_mission_interruption(self, context: MissionInterruptionContext) -> None:
        if context.source_type is InterruptedMissionType.PATROL:
            progress = context.progress
            if isinstance(progress, PausedPatrolCoverage):
                self._target_patrol_region_id = progress.region_id
                self._patrol_route_waypoints = deque(progress.remaining_waypoints)
                if self.logic.state is not FlightState.TAKING_OFF:
                    self.logic.target = progress.active_waypoint
        elif context.source_type is InterruptedMissionType.ROI:
            progress = context.progress
            if progress is not None:
                self.logic.roi_patrol_planner.restore(progress)
            self._active_roi_id = context.active_id
            if context.active_waypoint is not None and self.logic.state is not FlightState.TAKING_OFF:
                self.logic.target = context.active_waypoint
        self._mission_interruption = None
        self._mission_return_prepared = False
        self._mission_return_waiting_since = None

    def _discard_mission_interruption(self) -> None:
        self._mission_interruption = None
        self._mission_return_prepared = False
        self._mission_return_waiting_since = None

    # -- SET_OPERATING_CONFIG parity -----------------------------------------
    # Mirrors autonomous_drone.py's _handle_operating_config_command/
    # _apply_operating_config/_retarget_stored_altitudes/
    # _consume_pending_operating_config exactly, reusing
    # services/operating_config_apply.py's plan_operating_config_transition
    # (pure, already portable) for the no-fly-aware repositioning decision.
    # This is also the bootstrapping path for real-flight configuration: see
    # __init__ and the module docstring.

    def _handle_operating_config_command(self, message: dict[str, Any], position: Position) -> None:
        command_id = message["commandId"]
        if self.logic.state in {FlightState.LANDING, FlightState.RETURNING_HOME}:
            self.realtime_link.publish_control_message(
                operating_config_result(
                    command_id,
                    accepted=False,
                    message="Operating configuration cannot change during landing or return-to-home.",
                )
            )
            return

        try:
            params = message["params"]
            candidate = OperatingConfig(
                max_flight_radius_m=params["maxFlightRadiusM"], cruise_altitude_m=params["cruiseAltitudeM"]
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.realtime_link.publish_control_message(
                operating_config_result(command_id, accepted=False, message=str(exc))
            )
            return

        if self.logic.is_target_tracking or self.logic.state is FlightState.TARGET_RETURN:
            # Don't interrupt pursuit/follow, nor a return-to-checkpoint;
            # apply once settled (_consume_pending_operating_config, called
            # every tick). REPOSITIONING is not deferred: the planner
            # retargets it in place.
            self._pending_operating_config = candidate
            self.realtime_link.publish_control_message(operating_config_result(command_id, accepted=True, pending=True))
            return

        self._apply_operating_config(candidate, position)
        self.realtime_link.publish_control_message(operating_config_result(command_id, accepted=True, pending=False))

    def _apply_operating_config(self, candidate: OperatingConfig, position: Position) -> None:
        old_max_flight_radius_m = self.logic.pursuit.config.geofence_radius_m
        current_state = self.logic.state
        transition = plan_operating_config_transition(
            candidate=candidate,
            old_max_flight_radius_m=old_max_flight_radius_m,
            current_flight_state=current_state.value,
            position=position,
            patrol_regions=self.patrol_regions,
            patrol_planner=self.patrol_planner,
            pursuit_config=self.logic.pursuit.config,
            roi_config=self.logic.roi_patrol_planner.config,
            no_fly_router=self.no_fly_router,
            reposition_destination=self.logic.reposition_destination,
            reposition_resume_state=(
                self.logic.reposition_resume_state.value if self.logic.reposition_resume_state is not None else None
            ),
        )
        altitude_changed = not math.isclose(self.logic.config.takeoff_altitude, candidate.cruise_altitude_m, rel_tol=1e-9)
        radius_changed = not math.isclose(old_max_flight_radius_m, candidate.max_flight_radius_m, rel_tol=1e-9)

        (
            self.patrol_regions,
            self.patrol_planner,
            self.logic.config,
            self.logic.target_decisions.config,
            self.logic.pursuit.config,
            self.logic.roi_patrol_planner.config,
        ) = (
            transition.store,
            transition.planner,
            replace(
                self.logic.config, takeoff_altitude=candidate.cruise_altitude_m,
                z_bounds=(candidate.cruise_altitude_m, candidate.cruise_altitude_m),
            ),
            replace(
                self.logic.config, takeoff_altitude=candidate.cruise_altitude_m,
                z_bounds=(candidate.cruise_altitude_m, candidate.cruise_altitude_m),
            ),
            transition.pursuit_config,
            transition.roi_config,
        )

        if transition.discard_patrol_progress:
            self._clear_patrol_progress()

        if altitude_changed:
            self._retarget_stored_altitudes(candidate.cruise_altitude_m)

        if transition.resume_state is not None:
            self.logic.start_reposition(list(transition.reposition_waypoints), FlightState(transition.resume_state))

        if current_state is FlightState.TAKING_OFF and (radius_changed or altitude_changed):
            self.logic.mark_queued_mission_config_stale()

        self.operating_config = candidate

    def _retarget_stored_altitudes(self, altitude_m: float) -> None:
        def moved(waypoint: Position | None) -> Position | None:
            if waypoint is None:
                return None
            return (waypoint[0], waypoint[1], altitude_m)

        def moved_all(waypoints) -> tuple[Position, ...]:
            return tuple((x, y, altitude_m) for x, y, _ in waypoints)

        target = moved(self.logic.target)
        if target is not None:
            self.logic.target = target
        self._patrol_route_waypoints = deque(moved_all(self._patrol_route_waypoints))
        self.logic.roi_patrol_planner.retarget_altitude(altitude_m)

        if self._paused_patrol_coverage is not None:
            self._paused_patrol_coverage = replace(
                self._paused_patrol_coverage,
                active_waypoint=moved(self._paused_patrol_coverage.active_waypoint),
                remaining_waypoints=moved_all(self._paused_patrol_coverage.remaining_waypoints),
            )

        context = self._mission_interruption
        if context is None:
            return
        progress = context.progress
        if isinstance(progress, PausedPatrolCoverage):
            progress = replace(
                progress,
                active_waypoint=moved(progress.active_waypoint),
                remaining_waypoints=moved_all(progress.remaining_waypoints),
            )
        elif progress is not None and hasattr(progress, "waypoints"):
            progress = replace(progress, waypoints=moved_all(progress.waypoints))
        self._mission_interruption = replace(
            context,
            active_waypoint=moved(context.active_waypoint),
            remaining_path=moved_all(context.remaining_path),
            progress=progress,
        )

    def _consume_pending_operating_config(self, position: Position) -> None:
        if (
            self._pending_operating_config is not None
            and not self.logic.is_target_tracking
            and self.logic.state is not FlightState.TARGET_RETURN
        ):
            candidate = self._pending_operating_config
            self._pending_operating_config = None
            self._apply_operating_config(candidate, position)

    # -- fail-safes -------------------------------------------------------------

    def _enforce_realtime_connection_loss_failsafe(self) -> None:
        """Mirrors autonomous_drone.py's _enforce_connection_loss_rth for the
        realtime_server link."""
        if self.logic.state is FlightState.LANDED:
            self._realtime_connection_loss_rth_active = False
            if self._safety_rth_reason == CONNECTION_LOSS_REASON:
                self._safety_rth_active = False
                self._safety_rth_reason = None
            return
        if self._realtime_connection_loss_rth_active:
            return
        if not self.realtime_link.connection_lost_for(self.config.realtime_connection_loss_rth_s):
            return
        if self.logic.state not in _AIRBORNE_STATES:
            return
        self._realtime_connection_loss_rth_active = True
        self._safety_rth_active = True
        self._safety_rth_reason = CONNECTION_LOSS_REASON
        self._clear_patrol_progress()
        if self.logic.abort_manual_control():
            self.manual_input.reset()
        if self.logic.state not in {FlightState.RETURNING_HOME, FlightState.LANDING}:
            self.logic.request_return_home()

    def _enforce_android_connection_loss_failsafe(self, now: float) -> None:
        """The equivalent fail-safe for the *local* Android link. There is no
        autonomous_drone.py precedent for this -- the simulator has no
        second link to lose -- so this is new, matching the same pattern."""
        if self.logic.state is FlightState.LANDED:
            self._android_connection_loss_rth_active = False
            if self._safety_rth_reason == ANDROID_LINK_LOSS_REASON:
                self._safety_rth_active = False
                self._safety_rth_reason = None
            return
        if self._android_connection_loss_rth_active:
            return
        if not self.android_link.connection_lost_for(self.config.android_connection_loss_rth_s):
            return
        if self.logic.state not in _AIRBORNE_STATES:
            return
        self._android_connection_loss_rth_active = True
        self._safety_rth_active = True
        self._safety_rth_reason = ANDROID_LINK_LOSS_REASON
        self._clear_patrol_progress()
        if self.logic.abort_manual_control():
            self.manual_input.reset()
        if self.logic.state not in {FlightState.RETURNING_HOME, FlightState.LANDING}:
            self.logic.request_return_home()

    # -- outbound telemetry -------------------------------------------------

    def build_telemetry_message(self, outcome: TickOutcome) -> dict[str, Any]:
        assert outcome.has_position and outcome.position is not None and outcome.yaw_rad is not None
        local = LocalPosition(x=outcome.position[0], y=outcome.position[1], z=outcome.position[2])
        gps = local_meters_to_wgs84(local)
        heading_deg = (math.degrees(outcome.yaw_rad) + 360.0) % 360.0
        return {
            "type": "telemetry",
            "timestamp": time.time(),
            "source": DJI_TELEMETRY_SOURCE,
            "data": {
                "gps": {"lat": gps.lat, "lng": gps.lng},
                "altitudeM": outcome.position[2],
                "headingDeg": heading_deg,
                "speedMps": outcome.speed_mps,
                "batteryPercent": outcome.battery_percent if outcome.battery_percent is not None else 0.0,
                "state": outcome.state,
                "targetPatrolRegionId": outcome.target_patrol_region_id,
                "aircraftSafetyRthActive": outcome.safety_rth_active,
                "aircraftSafetyRthReason": outcome.safety_rth_reason,
                "targetFollowDiagnostics": None,
                "operatingConfigPending": self._pending_operating_config is not None,
                "pursuitBlocked": outcome.pursuit_blocked,
                "manualControlActive": outcome.manual_control_active,
                "manualInputSource": outcome.manual_input_source,
            },
        }

    # -- async orchestration --------------------------------------------------

    async def run_tick_once(self) -> TickOutcome:
        """One tick, including the I/O a plain tick() call can't do itself:
        sending the resulting setpoint/gimbal command to Android (subject to
        AndroidLink's own freshness gate -- see android_link.py) and
        publishing telemetry to realtime_server."""
        outcome = self.tick(self.config.tick_interval_s)
        if not outcome.has_position:
            return outcome

        assert outcome.position is not None and outcome.yaw_rad is not None
        yaw_deg = (math.degrees(outcome.yaw_rad) + 360.0) % 360.0
        await self.android_link.send_setpoint(
            velocity_x_mps=outcome.velocity_x_mps,
            velocity_y_mps=outcome.velocity_y_mps,
            velocity_z_mps=outcome.velocity_z_mps,
            yaw_deg=yaw_deg,
            ttl_ms=self.config.setpoint_ttl_ms,
            telemetry_timeout_s=self.config.android_telemetry_timeout_s,
        )
        await self.android_link.send_gimbal_command(
            pitch_deg=math.degrees(outcome.gimbal_pitch_rad), ttl_ms=self.config.gimbal_command_ttl_ms
        )
        if outcome.safety_rth_active and outcome.state in {FlightState.RETURNING_HOME.value, FlightState.LANDING.value}:
            await self.android_link.send_rth(reason=outcome.safety_rth_reason or "safety_rth")

        self.realtime_link.publish_telemetry(self.build_telemetry_message(outcome))
        return outcome

    async def run(self) -> None:
        self.realtime_link.start()
        server_task = asyncio.create_task(self.android_link.serve_forever())
        try:
            while True:
                await self.run_tick_once()
                await asyncio.sleep(self.config.tick_interval_s)
        finally:
            server_task.cancel()
            self.realtime_link.stop()


def load_real_drone_operating_config() -> OperatingConfig | None:
    """Read DRONTECT_DJI_MAX_FLIGHT_RADIUS_M/DRONTECT_DJI_CRUISE_ALTITUDE_M.

    Returns None (not default_operating_config, and not FlightConfig()'s
    simulator-scale defaults) when neither is set -- the bridge then starts
    unconfigured and refuses autonomous movement until a SET_OPERATING_CONFIG
    command arrives from a tablet, exactly like a bare aircraft with no
    mission loaded. If either variable IS set, both must parse and pass
    OperatingConfig's own legal-maximum validation, or this raises rather
    than silently falling back -- an operator-set env var that fails
    validation is a configuration error, not an "unconfigured" state.
    """
    raw_radius = os.getenv("DRONTECT_DJI_MAX_FLIGHT_RADIUS_M")
    raw_altitude = os.getenv("DRONTECT_DJI_CRUISE_ALTITUDE_M")
    if raw_radius is None and raw_altitude is None:
        return None
    try:
        radius = float(raw_radius) if raw_radius is not None else shared_config.home_radius_m
        altitude = float(raw_altitude) if raw_altitude is not None else shared_config.cruise_altitude_m
        return OperatingConfig(max_flight_radius_m=radius, cruise_altitude_m=altitude)
    except (TypeError, ValueError) as exc:
        raise RealFlightConfigurationError(
            "DRONTECT_DJI_MAX_FLIGHT_RADIUS_M/DRONTECT_DJI_CRUISE_ALTITUDE_M are invalid: " f"{exc}"
        ) from exc


def main() -> None:
    operating_config = load_real_drone_operating_config()
    bridge = DjiFlightBridge(operating_config=operating_config)
    if operating_config is None:
        print(
            "DJI flight bridge starting UNCONFIGURED: no DRONTECT_DJI_MAX_FLIGHT_RADIUS_M/"
            "DRONTECT_DJI_CRUISE_ALTITUDE_M set. Autonomous movement is refused until a "
            "SET_OPERATING_CONFIG command is received. Do not expect the aircraft to fly "
            "any mission command until then."
        )
    else:
        print(
            "DJI flight bridge starting with operating config: "
            f"radius={operating_config.max_flight_radius_m:.1f} m "
            f"cruise_altitude={operating_config.cruise_altitude_m:.1f} m"
        )
    asyncio.run(bridge.run())


if __name__ == "__main__":
    main()
