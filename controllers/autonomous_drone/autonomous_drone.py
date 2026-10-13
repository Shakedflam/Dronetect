# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# This controller adapts the stabilization approach and empirical constants
# from Cyberbotics' Webots R2025a mavic2pro_patrol.py example.

"""Webots adapter for the autonomous generic-quadcopter proof of concept."""

from collections import deque
from dataclasses import replace
import json
import math
import os
import traceback
from pathlib import Path
import sys
from typing import cast

from controller import Camera, GPS, InertialUnit, Motor, Supervisor

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from adapters.simulator.client import SimulatorRealtimeClient
from adapters.simulator.video import SimulatorVideoClient
from camera_control import CameraGimbal
from pursuit import PursuitConfig
from pursuit import calibrate_reference_area_ratio
from flight_logic import (
    advance_kinematic_flight,
    FlightConfig,
    FlightLogic,
    FlightState,
    KinematicFlightConfig,
    ManualFlightConfig,
    ManualStick,
    manual_velocity_command,
    Position,
)
from manual_input import ManualInputRouter
from flight_tuning import (
    build_flight_config,
    build_manual_flight_config,
    build_pursuit_config,
    kinematic_config_for_state,
    target_tracking_kinematic_config,
    KINEMATIC_FLIGHT_CONFIG,
    SEARCH_KINEMATIC_FLIGHT_CONFIG,
    MANUAL_KINEMATIC_FLIGHT_CONFIG,
)
from hud import (
    HUD_STYLE,
    TelemetrySnapshot,
    distance_3d,
    format_telemetry_hud,
    signed_angle_to_target_degrees,
)
from services.commands import CommandName
from config.shared_config import OperatingConfig, default_operating_config, shared_config
from realtime_server.protocol import operating_config_result
from services.gps import (
    DEFAULT_HOME,
    LocalPosition,
    Wgs84Point,
    local_meters_to_wgs84,
    wgs84_to_local_meters,
)
from services.flight_safety import CONNECTION_LOSS_REASON
from services.mission_interruption import (
    InterruptedMissionType,
    MissionInterruptionContext,
)
from services.no_fly_router import NoFlyRouteUnavailable, NoFlyRouter
from services.polygon_coverage import (
    CoveragePathUnavailable,
    PolygonCoverageConfig,
    PolygonCoveragePlanner,
)
from services.operating_config_apply import plan_operating_config_transition
from services.patrol_planner import PatrolPlanner, describe_patrol_plan
from services.patrol_progress import PausedPatrolCoverage
from services.patrol_regions import (
    PatrolAreaType,
    PatrolRegion,
    PatrolRegionStore,
    patrol_region_polygon_points,
)
from services.telemetry import build_simulator_telemetry
from world_markers import WorldMarkers


def clamp(value: float, minimum: float, maximum: float) -> float:
    #The drone will not react too aggressively
    return min(max(value, minimum), maximum)


# Marks a safety RTH caused by the flight logic itself raising.
TICK_FAILURE_REASON = "flight_logic_error"

TARGET_TRACKING_STATES = frozenset({
    FlightState.TARGET_ACQUISITION,
    FlightState.TARGET_FOLLOW,
    FlightState.TARGET_PURSUIT,
})


class AutonomousDrone(Supervisor):
    K_VISUAL_PROPELLER_SPEED = 68.5
    # Past this many failed ticks the logic is not going to recover, so stop
    # commanding the aircraft rather than flying on whatever state is left.
    MAX_TICK_FAILURES = 20
    TAKEOFF_CAMERA_PITCH = -0.2
    # Keep a small amount of ground in view during horizontal flight.
    HORIZONTAL_CAMERA_PITCH = 0.16
    # Canonical tuned values, from flight_tuning.py -- shared with
    # adapters/dji/flight_bridge.py. Bound as class attributes here (rather
    # than referenced only at module level) because tests/test_target_
    # pursuit.py's `simulator` fixture reads
    # module.AutonomousDrone.SEARCH_KINEMATIC_FLIGHT_CONFIG directly.
    KINEMATIC_FLIGHT_CONFIG = KINEMATIC_FLIGHT_CONFIG
    SEARCH_KINEMATIC_FLIGHT_CONFIG = SEARCH_KINEMATIC_FLIGHT_CONFIG
    MANUAL_KINEMATIC_FLIGHT_CONFIG = MANUAL_KINEMATIC_FLIGHT_CONFIG

    def __init__(self) -> None:
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.patrol_regions = PatrolRegionStore()
        self.patrol_planner = PatrolPlanner(store=self.patrol_regions)
        self.no_fly_router = NoFlyRouter()
        self.polygon_coverage_planner = PolygonCoveragePlanner(
            PolygonCoverageConfig(
                lane_spacing_m=float(
                    os.getenv("DRONTECT_PATROL_LANE_SPACING_M", "75.0")
                )
            )
        )
        self._patrol_route_waypoints: deque[Position] = deque()
        self._target_patrol_region_id: str | None = None
        self._paused_patrol_coverage: PausedPatrolCoverage | None = None
        self._active_roi_id: str | None = None
        self._mission_interruption: MissionInterruptionContext | None = None
        self._mission_return_prepared = False
        self._mission_return_waiting_since: float | None = None
        self._pending_operating_config: OperatingConfig | None = None
        self.logic = FlightLogic(
            build_flight_config(),
            patrol_waypoint_provider=self._next_patrol_waypoint,
            pursuit_config=build_pursuit_config(
                geofence_radius_m=max(
                    region.outer_radius_m for region in self.patrol_regions.regions
                ),
            ),
        )
        self._target_tracking_flight_config = target_tracking_kinematic_config(
            self.logic.config.target_max_yaw_rate_rad_s,
        )

        self.markers = WorldMarkers(self)
        self.self_node = self.getSelf()
        self.translation = self.self_node.getField("translation")
        self.rotation = self.self_node.getField("rotation")
        if self.translation is None or self.rotation is None:
            raise RuntimeError("Autonomous drone requires translation and rotation fields.")

        self.camera = cast(Camera, self.getDevice("camera"))
        self.logic.pursuit.config = replace(
            self.logic.pursuit.config, horizontal_fov_rad=self.camera.getFov(),
        )
        #orientation: roll, pitch, yaw
        self.imu = cast(InertialUnit, self.getDevice("inertial unit"))
        self.gps = cast(GPS, self.getDevice("gps"))
        for sensor in (self.camera, self.imu, self.gps):
            sensor.enable(self.time_step)

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.time_step)

        self.camera_gimbal = CameraGimbal()
        self._target_gimbal_max_rate_rad_s = float(
            os.getenv("DRONTECT_TARGET_GIMBAL_MAX_RATE_RAD_S", "0.20")
        )
        self.camera_pitch_motor = cast(Motor, self.getDevice("camera pitch"))
        self.camera_pitch_motor.setPosition(self.camera_gimbal.pitch)
        self._camera_flight_state: FlightState | None = None
        self._altitude_smoke_enabled = (
            os.getenv("DRONTECT_WEBOTS_ALTITUDE_SMOKE", "0") == "1"
        )
        self._altitude_smoke_target = (
            self.getFromDef("TRAFFIC_DRONE_1") if self._altitude_smoke_enabled else None
        )
        self._altitude_smoke_started_at: float | None = None
        self._altitude_smoke_pursuit_seen = False
        self._altitude_smoke_direction_seen = False
        self._altitude_smoke_within_since: float | None = None
        self._altitude_smoke_converged_at: float | None = None
        self._altitude_smoke_lost_since: float | None = None

        self.motors: tuple[Motor, Motor, Motor, Motor] = (
            cast(Motor, self.getDevice("front left propeller")),
            cast(Motor, self.getDevice("front right propeller")),
            cast(Motor, self.getDevice("rear left propeller")),
            cast(Motor, self.getDevice("rear right propeller")),
        )
        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        self._announced_state = None
        self._announced_diagnostic_state: str | None = None
        self._elapsed_time = 0.0
        self._last_hud_update_time = -1.0
        self._horizontal_speed_mps = 0.0
        self._kinematic_yaw: float | None = None
        self._simulated_battery_percent = float(
            os.getenv("DRONTECT_SIMULATED_BATTERY_PERCENT", "100.0")
        )
        self._connection_loss_timeout_s = float(
            os.getenv("DRONTECT_CONNECTION_LOSS_RTH_SECONDS", "3.0")
        )
        if (
            not math.isfinite(self._connection_loss_timeout_s)
            or self._connection_loss_timeout_s <= 0.0
        ):
            raise ValueError(
                "DRONTECT_CONNECTION_LOSS_RTH_SECONDS must be a finite positive number"
            )
        self._mission_return_timeout_s = float(
            os.getenv("DRONTECT_MISSION_RETURN_TIMEOUT_S", "10.0")
        )
        self._tick_failures = 0
        self._connection_loss_rth_active = False
        self._safety_rth_active = False
        self._safety_rth_reason: str | None = None
        # Simulator-only range-calibration aid. Set to the DEF name of the
        # target node to have each tracked frame print the reference ratio
        # that its true range implies, so the calibration is measured off a
        # real run instead of assumed from airframe drawings.
        self._range_calibration_def = os.getenv(
            "DRONTECT_TARGET_RANGE_CALIBRATION", ""
        ).strip()
        self._last_range_calibration_at = -1.0
        self.manual_flight_config = build_manual_flight_config()
        # No physical transmitter in the simulator; a real drone adapter passes
        # one in and it takes precedence over the tablet pad.
        self.manual_input = ManualInputRouter(
            command_timeout_s=self.manual_flight_config.command_timeout_s,
        )
        self.realtime_client = SimulatorRealtimeClient()
        self.realtime_client.start()
        self.video_client = SimulatorVideoClient()
        self.video_client.start()

    def run(self) -> None:
        print(
            "Autonomous drone ready. Press H to return home or N for a new target."
        )
        dt = self.time_step / 1000.0

        while self.step(self.time_step) != -1:
            try:
                self._tick(dt)
            except Exception:
                # A controller that raises stops calling step(), and Webots
                # waits on it: one exception here freezes the whole world,
                # every other robot included, with the cause buried. Keep
                # stepping so the traceback is visible and the rest of the
                # simulation carries on, and send the aircraft home, because
                # continuing to fly on logic that just failed is worse than
                # landing.
                traceback.print_exc()
                self._tick_failures += 1
                if self._tick_failures == 1:
                    self._safety_rth_active = True
                    self._safety_rth_reason = TICK_FAILURE_REASON
                    self.logic.abort_manual_control()
                    try:
                        self.logic.request_return_home()
                    except Exception:
                        traceback.print_exc()
                if self._tick_failures >= self.MAX_TICK_FAILURES:
                    print(
                        "Flight logic has failed "
                        f"{self._tick_failures} times; holding motors and "
                        "stepping only, to keep the simulation alive."
                    )
                    self._stop_motors()

    def _tick(self, dt: float) -> None:
        """One simulation step of sensing, deciding and commanding."""
        self._elapsed_time += dt
        gps_values = self.gps.getValues()
        position: Position = (
            float(gps_values[0]),
            float(gps_values[1]),
            float(gps_values[2]),
        )
        roll, pitch, yaw = self.imu.getRollPitchYaw()

        if self.logic.home is None:
            self.logic.initialize_home(position)
            print(f"Home captured at {self._format_position(position)}")

        self._enforce_connection_loss_rth()
        self._handle_keyboard()
        self._handle_realtime_commands(position)
        self._update_manual_stick()
        state_before_detection = self.logic.state
        self._handle_vision_detections(position, yaw)
        self.logic.update(position, dt, yaw=yaw, camera_pitch=self.camera_gimbal.pitch)
        self._coordinate_mission_resume(position, state_before_detection)
        self._consume_pending_operating_config(position)
        position, yaw, speed_mps, vertical_speed_mps = self._apply_kinematic_flight(
            position,
            yaw,
            dt,
        )
        self._update_altitude_smoke(position)
        self._update_camera_for_flight_state()
        self._update_markers()
        self._announce_changes()
        self._update_hud(
            position,
            speed_mps,
            vertical_speed_mps,
            roll,
            pitch,
            yaw,
        )
        self._log_range_calibration(position)
        self._publish_realtime_telemetry(
            position,
            speed_mps,
            yaw,
        )
        self.video_client.publish_camera_frame(self.camera)
        if self.logic.state in {FlightState.IDLE, FlightState.LANDED}:
            self._stop_motors()
        else:
            self._set_visual_motor_velocities()

    def _handle_keyboard(self) -> None:
        key = self.keyboard.getKey()
        while key != -1:
            normalized_key = key & 0xFFFF
            if normalized_key in (ord("H"), ord("h")):
                if self.logic.request_return_home():
                    self._clear_patrol_progress()
                    print("Return Home requested.")
                else:
                    print(f"Return Home ignored while {self.logic.state.value}.")
            elif normalized_key in (ord("U"), ord("u")):
                pitch = self.camera_gimbal.tilt_up()
                self.camera_pitch_motor.setPosition(pitch)
                print(f"Camera pitch up: {pitch:.2f}")
            elif normalized_key in (ord("J"), ord("j")):
                pitch = self.camera_gimbal.tilt_down()
                self.camera_pitch_motor.setPosition(pitch)
                print(f"Camera pitch down: {pitch:.2f}")
            key = self.keyboard.getKey()

    def _handle_realtime_commands(self, position: Position) -> None:
        for message in self.realtime_client.drain_commands():
            command = CommandName(message["command"])
            if command is CommandName.SET_OPERATING_CONFIG:
                self._handle_operating_config_command(message, position)
                continue
            if self._safety_rth_active and (
                command in {
                    CommandName.START_PATROL,
                    CommandName.SET_ROI,
                    CommandName.CLEAR_ROI,
                    CommandName.REJECT_TARGET,
                }
                # Releasing manual is always allowed; taking the aircraft off a
                # safety RTH is not. The server refuses this too, but a
                # connection-loss RTH is latched here before it ever knows.
                or (
                    command is CommandName.SET_MANUAL_CONTROL
                    and bool(message["params"]["engaged"])
                )
            ):
                print(
                    f"Realtime command {command.value} ignored during "
                    "safety RTH."
                )
                continue
            tracking_before = self.logic.state
            if command == CommandName.RETURN_HOME:
                safety_reason = message.get("safetyReason")
                if isinstance(safety_reason, str):
                    self._safety_rth_active = True
                    self._safety_rth_reason = safety_reason
                # Home outranks the operator. Abort rather than release: there
                # is no mission to hand back to, the aircraft is flying home.
                if self.logic.abort_manual_control():
                    self.manual_input.reset()
                    print("Manual control released: returning home.")
                if self.logic.request_return_home():
                    self._clear_patrol_progress()
                    self._active_roi_id = None
                    print("Realtime command return_home accepted.")
                else:
                    print(f"Realtime command return_home ignored while {self.logic.state.value}.")
            elif command == CommandName.START_PATROL:
                self._clear_patrol_progress()
                self._active_roi_id = None
                self.logic.request_patrol(position)
                print(f"Realtime command {command.value} accepted.")
            elif command == CommandName.SET_PATROL_PRIORITIES:
                priorities = message["params"]["priorities"]
                self.patrol_regions.replace_priorities(
                    [(entry["regionId"], entry["rank"]) for entry in priorities]
                )
                self._replan_active_patrol(position)
                print("Realtime patrol priorities updated.")
            elif command == CommandName.SET_PATROL_AREA_TYPES:
                area_types = message["params"]["areaTypes"]
                self.patrol_regions.replace_area_types(
                    [(entry["regionId"], entry["areaType"]) for entry in area_types]
                )
                self._replan_active_patrol(position)
                print("Realtime patrol area types updated.")
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
                self.logic.request_roi(
                    position,
                    (local_center.x, local_center.y),
                    params["radiusM"],
                )
                self._active_roi_id = f"roi:{message['commandId']}"
                # ROI missions intentionally override NO_FLY patrol routing;
                # the restriction remains stored and resumes with PATROL.
                print("Realtime command set_roi accepted.")
            elif command == CommandName.CLEAR_ROI:
                self._discard_mission_interruption()
                resume_patrol = self.logic.state in {
                    FlightState.TAKING_OFF,
                    FlightState.TRANSITING_TO_ROI,
                    FlightState.ROI_PATROL,
                } or (
                    # Holding after a completed single-cycle ROI.
                    self.logic.state is FlightState.HOLDING
                    and self._active_roi_id is not None
                )
                self.logic.clear_roi(position)
                self._active_roi_id = None
                if resume_patrol:
                    had_checkpoint = self._paused_patrol_coverage is not None
                    if had_checkpoint:
                        if self._resume_paused_patrol(position):
                            self.logic.request_patrol(position)
                        else:
                            print(
                                "Paused patrol cannot currently be resumed safely; "
                                "holding position."
                            )
                    else:
                        self._clear_patrol_progress()
                        self.logic.request_patrol(position)
                print("Realtime command clear_roi accepted.")
            elif command == CommandName.CAMERA_TILT_UP:
                pitch = self.camera_gimbal.tilt_up()
                self.camera_pitch_motor.setPosition(pitch)
                print(f"Realtime command camera_tilt_up: {pitch:.2f}")
            elif command == CommandName.SET_PURSUIT_BLOCK:
                blocked = bool(message["params"]["blocked"])
                if self.logic.set_pursuit_blocked(blocked, position):
                    print(
                        "Realtime command set_pursuit_block: pursuit "
                        + ("held" if blocked else "released")
                        + "."
                    )
            elif command == CommandName.SET_MANUAL_CONTROL:
                if bool(message["params"]["engaged"]):
                    if self.logic.engage_manual_control(position):
                        self.manual_input.reset()
                        self._discard_mission_interruption()
                        print("Realtime command set_manual_control: operator has control.")
                    else:
                        print(
                            "Realtime command set_manual_control ignored while "
                            f"{self.logic.state.value}."
                        )
                else:
                    self._release_manual_control(position)
            elif command == CommandName.SET_MANUAL_COMMAND:
                params = message["params"]
                self.manual_input.accept_tablet_stick(
                    ManualStick(
                        forward=params["forward"],
                        right=params["right"],
                        up=params["up"],
                        yaw_rate=params["yawRate"],
                    ),
                    self._elapsed_time,
                )
            elif command == CommandName.REJECT_TARGET:
                track_id = message["params"]["trackId"]
                if not self.logic.reject_target(track_id, position):
                    # Silence here is what makes an operator think the button
                    # is broken: the usual cause is that the tracker has
                    # already re-identified the object under a different id.
                    print(
                        f"Realtime command reject_target ignored: {track_id} is no "
                        f"longer a known candidate (known: "
                        f"{self.logic.target_decision_diagnostics()['rejectableTrackIds']})."
                    )
            elif command == CommandName.CAMERA_TILT_DOWN:
                pitch = self.camera_gimbal.tilt_down()
                self.camera_pitch_motor.setPosition(pitch)
                print(f"Realtime command camera_tilt_down: {pitch:.2f}")

            # Commands are dispatched before this tick's logic.update(). One
            # that ended target tracking has to bring the interrupted mission
            # back with it, or update() runs against a flight state whose
            # backing data -- the ROI sweep, cleared on capture -- has not
            # been restored yet, and the aircraft silently drops to area
            # patrol. Loss-driven resumes avoid this by flipping state inside
            # update(), where the normal resume call still follows.
            if (
                tracking_before is not self.logic.state
                and tracking_before in TARGET_TRACKING_STATES
            ):
                self._coordinate_mission_resume(position, tracking_before)

    def _release_manual_control(self, position: Position) -> None:
        """Give the aircraft back to the computer and restart its mission.

        Where the operator left it is the only position any of this can be
        planned from, so the ROI sweep and the patrol route are both rebuilt
        from here rather than resumed from a waypoint the aircraft may now be
        kilometres away from.
        """
        resume = self.logic.release_manual_control()
        if resume is None:
            return
        self.manual_input.reset()

        if resume.state is FlightState.RETURNING_HOME:
            self.logic.request_return_home()
            print("Manual control released: resuming return home.")
            return

        if resume.roi is not None:
            center, radius_m = resume.roi
            try:
                self.logic.request_roi(position, center, radius_m)
            except ValueError as exc:
                # The operator flew somewhere the ROI no longer reaches from.
                print(f"ROI unreachable after manual flight, falling back to patrol: {exc}")
            else:
                print("Manual control released: resuming ROI.")
                return

        self._clear_patrol_progress()
        self._active_roi_id = None
        self.logic.request_patrol(position)
        print("Manual control released: starting a fresh patrol.")

    def _update_manual_stick(self) -> None:
        """Hand the autopilot whichever stick is live, or a centered one.

        A takeover can end anywhere -- the RTH key, a mission command, a safety
        return -- so the held stick is dropped here rather than at each of
        those call sites, and no route out of manual can leave one behind.
        """
        if not self.logic.manual_control:
            self.manual_input.reset()
            return
        self.logic.set_manual_stick(self.manual_input.stick(self._elapsed_time))

    def _handle_operating_config_command(self, message: dict, position: Position) -> None:
        command_id = message["commandId"]
        # Takeoff is deliberately not blocked here: _apply_operating_config
        # already has dedicated handling for it (mark_queued_mission_config_stale
        # below, consumed in FlightLogic.update() once the climb completes --
        # see test_queued_roi_mission_refreshes_stale_geometry_when_marked_stale).
        # Landing and RTH have no such support -- plan_operating_config_transition
        # has no resume state for them -- so a mid-maneuver change there is
        # rejected rather than left half-applied.
        if self.logic.state in {FlightState.LANDING, FlightState.RETURNING_HOME}:
            self.realtime_client.publish_control_message(
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
                max_flight_radius_m=params["maxFlightRadiusM"],
                cruise_altitude_m=params["cruiseAltitudeM"],
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.realtime_client.publish_control_message(
                operating_config_result(command_id, accepted=False, message=str(exc))
            )
            return

        if self.logic.is_target_tracking or self.logic.state is FlightState.TARGET_RETURN:
            # Don't interrupt pursuit/follow, nor a return-to-checkpoint:
            # TARGET_RETURN is not a resume state
            # plan_operating_config_transition recognizes, so applying here
            # would only partially land -- a radius change would patch
            # altitude but silently drop the radius until whatever this
            # settles into next. Apply once settled instead (see
            # _consume_pending_operating_config, called every tick from
            # run()). REPOSITIONING is deliberately *not* deferred: it is a
            # reposition leg, and the planner retargets it in place.
            self._pending_operating_config = candidate
            self.realtime_client.publish_control_message(
                operating_config_result(command_id, accepted=True, pending=True)
            )
            return

        self._apply_operating_config(candidate, position)
        self.realtime_client.publish_control_message(
            operating_config_result(command_id, accepted=True, pending=False)
        )

    def _apply_operating_config(
        self,
        candidate: OperatingConfig,
        position: Position,
    ) -> None:
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
                self.logic.reposition_resume_state.value
                if self.logic.reposition_resume_state is not None
                else None
            ),
        )
        altitude_changed = not math.isclose(
            self.logic.config.takeoff_altitude, candidate.cruise_altitude_m, rel_tol=1e-9,
        )
        radius_changed = not math.isclose(
            old_max_flight_radius_m, candidate.max_flight_radius_m, rel_tol=1e-9,
        )

        # All decisions/construction happened above (plan_operating_config_transition
        # is pure -- it mutated nothing). Swap the complete operating pair and its
        # derived objects together before acknowledging.
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
                self.logic.config,
                takeoff_altitude=candidate.cruise_altitude_m,
                z_bounds=(candidate.cruise_altitude_m, candidate.cruise_altitude_m),
            ),
            replace(
                self.logic.config,
                takeoff_altitude=candidate.cruise_altitude_m,
                z_bounds=(candidate.cruise_altitude_m, candidate.cruise_altitude_m),
            ),
            transition.pursuit_config,
            transition.roi_config,
        )

        if transition.discard_patrol_progress:
            self._clear_patrol_progress()

        if altitude_changed:
            # Patch every waypoint already computed under the old altitude so
            # the change reaches flight, not just future computations. This
            # is deliberately unconditional: stored waypoints outlive the
            # state that produced them. A pursuit resumes into whatever
            # mission it interrupted, and a reposition leg is followed by
            # replaying exactly these -- so gating the retarget on the
            # current state, or making it the alternative to repositioning,
            # left the aircraft climbing to the new altitude for the leg and
            # then descending back to the old one to fly the sweep.
            self._retarget_stored_altitudes(candidate.cruise_altitude_m)

        if transition.resume_state is not None:
            self.logic.start_reposition(
                list(transition.reposition_waypoints),
                FlightState(transition.resume_state),
            )

        if current_state is FlightState.TAKING_OFF and (radius_changed or altitude_changed):
            # A mission is queued behind this climb (request_roi()/
            # request_patrol() already computed its entry waypoint, and for
            # ROI its clipped geometry, under the config now being replaced)
            # -- recompute it fresh once takeoff completes instead of flying
            # to a stale target.
            self.logic.mark_queued_mission_config_stale()

        print(
            "Realtime operating configuration updated: "
            f"radius={candidate.max_flight_radius_m:.2f} m "
            f"cruise={candidate.cruise_altitude_m:.2f} m"
            + (" (repositioning before resuming)" if transition.resume_state is not None else "")
            + "."
        )

    def _retarget_stored_altitudes(self, altitude_m: float) -> None:
        """Rewrite the Z of every stored waypoint, preserving XY and order.

        Covers the active target, the patrol route, the ROI sweep cursor, and
        the progress stashed for a mission that a pursuit interrupted -- that
        last one matters because a config change during pursuit is applied
        while the interrupted mission is still parked.
        """
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
                remaining_waypoints=moved_all(
                    self._paused_patrol_coverage.remaining_waypoints
                ),
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
            # The ROI planner's own snapshot type, kept opaque here.
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

    def _handle_vision_detections(self, position: Position, yaw: float) -> None:
        allow_landing_intercept = (
            self.logic.state is FlightState.LANDING
            and not self._safety_rth_active
        )
        for message in self.realtime_client.drain_detections():
            detections = message["detections"]
            if self.logic.would_intercept(
                detections,
                allow_landing_intercept=allow_landing_intercept,
                frame_width=message["frameWidth"], frame_height=message["frameHeight"],
                position=position, yaw=yaw, camera_pitch=self.camera_gimbal.pitch,
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

    def _coordinate_mission_resume(
        self,
        position: Position,
        state_before_detection: FlightState,
    ) -> None:
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
            and (
                state_before_detection in target_states
                or self.logic.state.value == context.source_state
            )
        ):
            self._restore_mission_interruption(context)

    def _prepare_mission_return(
        self,
        position: Position,
        context: MissionInterruptionContext,
    ) -> None:
        try:
            route = self.no_fly_router.egress_route(
                (position[0], position[1]),
                (
                    context.interruption_position[0],
                    context.interruption_position[1],
                ),
                self.patrol_regions.regions,
            )
        except NoFlyRouteUnavailable as exc:
            if self._mission_return_waiting_since is None:
                self._mission_return_waiting_since = self._elapsed_time
                print(f"Mission resume waiting for a safe NO_FLY egress route: {exc}")
            waited = self._elapsed_time - self._mission_return_waiting_since
            if waited >= self._mission_return_timeout_s:
                # Hovering cannot change what the route depends on, so this
                # wait never ends on its own. Take the mission back here
                # rather than holding position until the battery decides.
                print(
                    f"No safe route back after {waited:.1f}s; resuming the "
                    "mission from the current position instead."
                )
                self._restore_mission_interruption(context)
                self._mission_return_waiting_since = None
                self.logic.abandon_target_return(position)
                return
            self.logic.hold_target_return(position)
            self._mission_return_prepared = False
            return
        altitude = self.logic.config.takeoff_altitude
        waypoints: list[Position] = [
            (x, y, altitude) for x, y in route
        ]
        waypoints[-1] = context.interruption_position
        self.logic.set_target_return_route(
            self._deduplicate_waypoints(waypoints)
        )
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
            active_waypoint = (
                self.logic.pending_mission_waypoint
                if state is FlightState.TAKING_OFF
                else self.logic.target
            )
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
        elif effective_state in {
            FlightState.TRANSITING_TO_ROI,
            FlightState.ROI_PATROL,
        }:
            roi_progress = self.logic.roi_patrol_planner.snapshot()
            active_waypoint = (
                self.logic.pending_mission_waypoint
                if state is FlightState.TAKING_OFF
                else self.logic.target
            )
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

    def _restore_mission_interruption(
        self,
        context: MissionInterruptionContext,
    ) -> None:
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
            if (
                context.active_waypoint is not None
                and self.logic.state is not FlightState.TAKING_OFF
            ):
                self.logic.target = context.active_waypoint
        self._mission_interruption = None
        self._mission_return_prepared = False
        self._mission_return_waiting_since = None

    def _discard_mission_interruption(self) -> None:
        self._mission_interruption = None
        self._mission_return_prepared = False
        self._mission_return_waiting_since = None

    def _update_markers(self) -> None:
        self.markers.update(
            self.logic.home,
            self.logic.target,
            self.logic.state,
        )

    def _update_camera_for_flight_state(self) -> None:
        state = self.logic.state
        if state is not self._camera_flight_state:
            self._camera_flight_state = state
            if state is FlightState.TAKING_OFF:
                self.camera_gimbal.set_pitch(self.TAKEOFF_CAMERA_PITCH)
            elif state in {
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
            }:
                if state not in {
                    FlightState.TARGET_ACQUISITION,
                    FlightState.TARGET_FOLLOW,
                    FlightState.TARGET_PURSUIT,
                }:
                    self.camera_gimbal.set_pitch(self.HORIZONTAL_CAMERA_PITCH)
            elif state in {FlightState.LANDING, FlightState.LANDED}:
                self.camera_gimbal.set_pitch(self.camera_gimbal.maximum_pitch)

        if state in {FlightState.TARGET_ACQUISITION, FlightState.TARGET_FOLLOW, FlightState.TARGET_PURSUIT}:
            desired_pitch = (
                self.logic.target_camera_pitch
                if self.logic.target_camera_pitch is not None
                else self.HORIZONTAL_CAMERA_PITCH
            )
            self.camera_gimbal.move_toward(
                desired_pitch,
                self._target_gimbal_max_rate_rad_s * self.time_step / 1000.0,
            )
        self.camera_pitch_motor.setPosition(self.camera_gimbal.pitch)

    def _announce_changes(self) -> None:
        diagnostic_state = self.logic.diagnostic_state
        previous_diagnostic_state = self._announced_diagnostic_state
        if diagnostic_state != previous_diagnostic_state:
            logged_transitions = {
                ("PATROL", "TARGET_ACQUISITION"),
                ("TARGET_ACQUISITION", "TARGET_FOLLOW"),
                ("TARGET_ACQUISITION", "PATROL"),
                ("TARGET_FOLLOW", "TARGET_LOST_SEARCH"),
                ("TARGET_LOST_SEARCH", "TARGET_FOLLOW"),
                ("TARGET_LOST_SEARCH", "PATROL"),
                ("TARGET_FOLLOW", "TARGET_PURSUIT"),
                ("TARGET_PURSUIT", "TARGET_LOST_SEARCH"),
                ("TARGET_LOST_SEARCH", "TARGET_PURSUIT"),
                ("TARGET_LOST_SEARCH", "TARGET_RETURN"),
                ("TARGET_PURSUIT", "TARGET_RETURN"),
                ("TARGET_RETURN", "PATROL"),
            }
            transition = (previous_diagnostic_state, diagnostic_state)
            if transition in logged_transitions:
                print(f"Flight transition: {transition[0]} -> {transition[1]}")
            self._announced_diagnostic_state = diagnostic_state

        if self.logic.state is not self._announced_state:
            print(f"State: {self.logic.state.value}")
            self._announced_state = self.logic.state

    def _update_altitude_smoke(self, position: Position) -> None:
        """Validate altitude matching against Webots truth in smoke mode only."""
        if not self._altitude_smoke_enabled:
            return
        now = self.getTime()
        if self._altitude_smoke_target is None:
            self._finish_altitude_smoke(False, "TRAFFIC_DRONE_1 was not found")
            return
        if self._altitude_smoke_started_at is None:
            if self.logic.state is FlightState.IDLE:
                return
            self._altitude_smoke_started_at = now
        if now - self._altitude_smoke_started_at > 120.0:
            self._finish_altitude_smoke(False, "120 simulated second timeout")
            return

        if self.logic.state is FlightState.TARGET_PURSUIT:
            self._altitude_smoke_pursuit_seen = True
        tracking_interrupted = (
            self.logic.diagnostic_state == "TARGET_LOST_SEARCH"
            or (
                self._altitude_smoke_pursuit_seen
                and self.logic.state is FlightState.TARGET_ACQUISITION
            )
        )
        if self._altitude_smoke_pursuit_seen and tracking_interrupted:
            self._altitude_smoke_within_since = None
            if self._altitude_smoke_lost_since is None:
                self._altitude_smoke_lost_since = now
            elif now - self._altitude_smoke_lost_since >= 1.5:
                self._finish_altitude_smoke(False, "target lost for 1.5 simulated seconds")
            return
        self._altitude_smoke_lost_since = None
        if (
            self._altitude_smoke_pursuit_seen
            and self.logic.state not in {
                FlightState.TARGET_ACQUISITION,
                FlightState.TARGET_FOLLOW,
                FlightState.TARGET_PURSUIT,
            }
        ):
            self._finish_altitude_smoke(False, "pursuit ended before altitude convergence")
            return
        if self.logic.state is not FlightState.TARGET_PURSUIT:
            return

        assert self.logic.home is not None
        floor = self.logic.home[2] + self.logic.pursuit.config.minimum_height_m
        ceiling = self.logic.home[2] + self.logic.pursuit.config.maximum_height_m
        if position[2] < floor - 0.05 or position[2] > ceiling + 0.05:
            self._finish_altitude_smoke(False, "pursuit height safety limit violated")
            return

        target_position = self._altitude_smoke_target.getPosition()
        true_error_m = float(target_position[2]) - position[2]
        vertical_command_mps = self.logic.pursuit.velocity[2]
        if abs(true_error_m) > self.logic.config.target_altitude_target_band_m:
            if vertical_command_mps * true_error_m > 0.0:
                self._altitude_smoke_direction_seen = True
            self._altitude_smoke_within_since = None
            return

        if self._altitude_smoke_within_since is None:
            self._altitude_smoke_within_since = now
            self._altitude_smoke_converged_at = now
        if now - self._altitude_smoke_within_since >= 3.0:
            if not self._altitude_smoke_direction_seen:
                self._finish_altitude_smoke(False, "no corrective command toward true altitude")
                return
            self._finish_altitude_smoke(
                True,
                "altitude target held for 3 simulated seconds",
                true_error_m=true_error_m,
            )

    def _finish_altitude_smoke(
        self,
        passed: bool,
        reason: str,
        *,
        true_error_m: float | None = None,
    ) -> None:
        result = {
            "passed": passed,
            "reason": reason,
            "simulationTimeS": self.getTime(),
            "convergenceTimeS": (
                None
                if self._altitude_smoke_converged_at is None
                or self._altitude_smoke_started_at is None
                else self._altitude_smoke_converged_at - self._altitude_smoke_started_at
            ),
            "finalTrueHeightDifferenceM": true_error_m,
            "directionObserved": self._altitude_smoke_direction_seen,
        }
        print(f"DRONTECT_ALTITUDE_SMOKE_RESULT {json.dumps(result, sort_keys=True)}")
        result_path = os.getenv("DRONTECT_ALTITUDE_SMOKE_RESULT_PATH")
        if result_path:
            Path(result_path).write_text(
                json.dumps(result, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        self._altitude_smoke_enabled = False
        self.simulationQuit(0 if passed else 1)

    def _stop_motors(self) -> None:
        self._set_motor_velocities(0.0, 0.0, 0.0, 0.0)

    def _set_visual_motor_velocities(self) -> None:
        speed = self.K_VISUAL_PROPELLER_SPEED
        self._set_motor_velocities(speed, -speed, -speed, speed)

    def _apply_kinematic_flight(
        self,
        position: Position,
        yaw: float,
        dt: float,
    ) -> tuple[Position, float, float, float]:
        if self._kinematic_yaw is None:
            self._kinematic_yaw = yaw

        if self.logic.state in {FlightState.IDLE, FlightState.LANDED}:
            self._horizontal_speed_mps = 0.0
            return (position, self._kinematic_yaw, 0.0, 0.0)

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
            # Yaw is commanded as a rate, so the stick asks for a heading one
            # tick ahead and the slew limit in the profile does the rest.
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
            config=self._kinematic_config_for_state(),
            desired_yaw=desired_yaw,
            velocity_command=velocity_command,
        )
        self.translation.setSFVec3f(list(step.position))
        self.rotation.setSFRotation([0.0, 0.0, 1.0, step.yaw])
        self.self_node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._kinematic_yaw = step.yaw
        self._horizontal_speed_mps = step.horizontal_speed_mps
        return (
            step.position,
            step.yaw,
            step.speed_mps,
            step.vertical_speed_mps,
        )

    def _kinematic_config_for_state(self) -> KinematicFlightConfig:
        # Slew fast enough for a close, fast target and no faster: the yaw
        # limit passed here is recomputed each pursuit tick from its speed
        # and range. See flight_tuning.kinematic_config_for_state, shared
        # with adapters/dji/flight_bridge.py.
        return kinematic_config_for_state(
            self.logic.state,
            target_tracking_flight_config=self._target_tracking_flight_config,
            target_yaw_rate_limit=self.logic.target_yaw_rate_limit,
        )

    def _update_hud(
        self,
        position: Position,
        speed_mps: float,
        vertical_speed_mps: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> None:
        if self._elapsed_time - self._last_hud_update_time < 0.1:
            return

        self._last_hud_update_time = self._elapsed_time
        home = self.logic.home
        target = self.logic.target
        height = position[2] - home[2] if home is not None else position[2]
        snapshot = TelemetrySnapshot(
            state=self.logic.state,
            current=position,
            home=home,
            target=target,
            speed_mps=speed_mps,
            vertical_speed_mps=vertical_speed_mps,
            height_m=height,
            distance_to_target_m=distance_3d(position, target)
            if target is not None
            else None,
            angle_to_target_degrees=signed_angle_to_target_degrees(
                position,
                target,
                yaw,
            ),
            roll_degrees=math.degrees(roll),
            pitch_degrees=math.degrees(pitch),
            yaw_degrees=math.degrees(yaw),
        )
        hud_text = format_telemetry_hud(snapshot)
        if HUD_STYLE.shadow_enabled:
            self.setLabel(
                0,
                hud_text,
                HUD_STYLE.x + HUD_STYLE.shadow_offset,
                HUD_STYLE.y + HUD_STYLE.shadow_offset,
                HUD_STYLE.font_size,
                HUD_STYLE.shadow_color,
                0,
                HUD_STYLE.font,
            )
        else:
            self.setLabel(0, "", 0.0, 0.0, 0.0, 0x000000, 0, HUD_STYLE.font)
        self.setLabel(
            1,
            hud_text,
            HUD_STYLE.x,
            HUD_STYLE.y,
            HUD_STYLE.font_size,
            HUD_STYLE.text_color,
            0,
            HUD_STYLE.font,
        )

    def _log_range_calibration(self, position: Position) -> None:
        """Print what the current box says the range calibration should be.

        Reads the target's true position straight out of the world, so one
        pass at any distance yields the ratio to configure. Throttled to once
        a second: the point is a number to read, not a flood to grep.
        """
        if not self._range_calibration_def or not self.logic.is_target_tracking:
            return
        if self._elapsed_time - self._last_range_calibration_at < 1.0:
            return

        node = self.getFromDef(self._range_calibration_def)
        if node is None:
            print(
                "Range calibration: no node named "
                f"{self._range_calibration_def}; set DRONTECT_TARGET_RANGE_CALIBRATION "
                "to a DEF in the world."
            )
            self._range_calibration_def = ""
            return

        diagnostics = self.logic.target_follow_diagnostics() or {}
        pursuit = diagnostics.get("pursuit") or {}
        area_ratio = pursuit.get("boundingBoxAreaRatio")
        if not isinstance(area_ratio, (int, float)) or area_ratio <= 0.0:
            return

        truth = node.getField("translation").getSFVec3f()
        true_range_m = distance_3d(position, (truth[0], truth[1], truth[2]))
        suggested = calibrate_reference_area_ratio(
            measured_area_ratio=float(area_ratio),
            measured_range_m=true_range_m,
            reference_distance_m=self.logic.config.target_reference_distance_m,
        )
        if suggested is None:
            return

        self._last_range_calibration_at = self._elapsed_time
        reported = pursuit.get("estimatedTargetDistanceM")
        print(
            "DRONTECT_TARGET_RANGE_CALIBRATION "
            + json.dumps(
                {
                    "node": self._range_calibration_def,
                    "boxAreaRatio": round(float(area_ratio), 8),
                    "trueRangeM": round(true_range_m, 2),
                    "reportedRangeM": (
                        round(float(reported), 2)
                        if isinstance(reported, (int, float)) else None
                    ),
                    "referenceDistanceM": self.logic.config.target_reference_distance_m,
                    "suggestedReferenceAreaRatio": round(suggested, 6),
                },
                sort_keys=True,
            )
        )

    def _publish_realtime_telemetry(
        self,
        position: Position,
        speed_mps: float,
        yaw: float,
    ) -> None:
        telemetry = build_simulator_telemetry(
            position_local=LocalPosition(
                x=position[0],
                y=position[1],
                z=position[2],
            ),
            heading_deg=(math.degrees(yaw) + 360.0) % 360.0,
            speed_mps=speed_mps,
            state=self.logic.state.value,
            battery_percent=self._simulated_battery_percent,
            target_patrol_region_id=self._target_patrol_region_id,
            aircraft_safety_rth_active=self._safety_rth_active,
            aircraft_safety_rth_reason=self._safety_rth_reason,
            target_follow_diagnostics=self.logic.target_follow_diagnostics(),
            operating_config_pending=self._pending_operating_config is not None,
            pursuit_blocked=self.logic.pursuit_blocked,
            manual_control_active=self.logic.manual_control,
            manual_input_source=self.manual_input.source,
        )
        self.realtime_client.publish_telemetry(telemetry)

    def _enforce_connection_loss_rth(self) -> None:
        if self.logic.state is FlightState.LANDED:
            self._connection_loss_rth_active = False
            self._safety_rth_active = False
            self._safety_rth_reason = None
            return
        if self._connection_loss_rth_active:
            return
        if not self.realtime_client.connection_lost_for(
            self._connection_loss_timeout_s
        ):
            return
        airborne_states = {
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
            # A link lost mid-takeover must still bring the aircraft home; the
            # operator's pad is on the far end of the link that just died.
            FlightState.MANUAL,
            FlightState.LANDING,
        }
        if self.logic.state not in airborne_states:
            return
        self._connection_loss_rth_active = True
        self._safety_rth_active = True
        self._safety_rth_reason = CONNECTION_LOSS_REASON
        self._clear_patrol_progress()
        if self.logic.abort_manual_control():
            self.manual_input.reset()
        if self.logic.state not in {
            FlightState.RETURNING_HOME,
            FlightState.LANDING,
        }:
            self.logic.request_return_home()
        print("Connection-loss safety RTH latched.")

    def _next_patrol_waypoint(
        self,
        position: Position,
        completed: bool,
    ) -> Position:
        if self._patrol_route_waypoints:
            return self._patrol_route_waypoints.popleft()
        if completed and self._target_patrol_region_id is not None:
            self.patrol_planner.record_observation(self._target_patrol_region_id)

        # If a NO_FLY cell currently contains the aircraft (e.g. an ROI that
        # overlapped one was just cleared), fly clear of it first and work
        # out which region to patrol -- and the full route to it -- only once
        # already outside. That scoring loop below calls route() once per
        # candidate region (there can be dozens), and every one of those
        # calls would otherwise need its own no-fly egress leg computed from
        # this same start point; deferring it here means the aircraft starts
        # moving immediately on a cheap, destination-agnostic lookup instead
        # of sitting still while the full plan is worked out. The deferred
        # planning happens on the next call, once _has_reached() the exit
        # point makes this method run again with completed=True.
        try:
            exit_point = self.no_fly_router.nearest_exit(
                (position[0], position[1]), self.patrol_regions.regions
            )
        except NoFlyRouteUnavailable as exc:
            self._target_patrol_region_id = None
            self._patrol_route_waypoints.clear()
            print(f"No safe patrol coverage is currently available; holding position: {exc}")
            return (
                position[0],
                position[1],
                self.logic.config.takeoff_altitude,
            )
        if exit_point is not None:
            print(f"Leaving NO_FLY area toward {exit_point} before planning patrol coverage.")
            return (exit_point[0], exit_point[1], self.logic.config.takeoff_altitude)

        current = local_meters_to_wgs84(
            LocalPosition(x=position[0], y=position[1], z=position[2]),
            DEFAULT_HOME,
        )
        no_fly_polygons = [
            patrol_region_polygon_points(region)
            for region in self.patrol_regions.regions
            if region.area_type is PatrolAreaType.NO_FLY
        ]
        candidate_waypoints: dict[str, list[Position]] = {}

        def safe_route_distance(region: PatrolRegion) -> float | None:
            try:
                ordered, transit_distance = self._coverage_route_waypoints(
                    position,
                    region,
                    no_fly_polygons,
                )
            except (CoveragePathUnavailable, NoFlyRouteUnavailable):
                return None
            candidate_waypoints[region.id] = ordered
            return transit_distance

        try:
            decision = self.patrol_planner.choose_next(
                current,
                route_distance_m=safe_route_distance,
            )
        except RuntimeError as exc:
            self._target_patrol_region_id = None
            self._patrol_route_waypoints.clear()
            print(f"No safe patrol coverage is currently available; holding position: {exc}")
            return (
                position[0],
                position[1],
                self.logic.config.takeoff_altitude,
            )

        paused = self._paused_patrol_coverage
        # Scoring picked the same region a previous interruption paused
        # mid-sweep: continue its exact remaining waypoints instead of
        # planning a brand-new coverage pass, so already-flown ground isn't
        # re-covered and the pass can still finish and be marked observed.
        # Falls through to a fresh plan below if no safe route back to the
        # interruption point exists any more (e.g. a new NO_FLY region).
        if (
            paused is not None
            and paused.region_id == decision.region.id
            and self._resume_paused_patrol(position)
        ):
            print(f"Resuming paused patrol region: {decision.region.id}")
            first = self._patrol_route_waypoints.popleft()
            print(describe_patrol_plan(
                decision, source="resumed_checkpoint", position=position, first_waypoint=first,
                waypoint_count=1 + len(self._patrol_route_waypoints),
                patrol_radius_m=self.logic.pursuit.config.geofence_radius_m,
                planner_uptime_s=self.patrol_planner.uptime_s,
            ))
            return first

        self._target_patrol_region_id = decision.region.id
        ordered = candidate_waypoints[decision.region.id]
        self._patrol_route_waypoints = deque(ordered[1:])
        print(
            f"Patrol target region: {decision.region.id} "
            f"(safe route {decision.score.distance_m:.1f} m)"
        )
        print(describe_patrol_plan(
            decision, source="new_coverage", position=position, first_waypoint=ordered[0],
            waypoint_count=len(ordered),
            patrol_radius_m=self.logic.pursuit.config.geofence_radius_m,
            planner_uptime_s=self.patrol_planner.uptime_s,
        ))
        return ordered[0]

    def _coverage_route_waypoints(
        self,
        position: Position,
        region: PatrolRegion,
        no_fly_polygons: list[tuple[Wgs84Point, ...]],
    ) -> tuple[list[Position], float]:
        coverage = self.polygon_coverage_planner.plan(
            patrol_region_polygon_points(region),
            no_fly_polygons=no_fly_polygons,
            # Same as the DJI bridge: enter the sweep nearest the aircraft.
            start_from=local_meters_to_wgs84(
                LocalPosition(x=position[0], y=position[1], z=0.0), DEFAULT_HOME),
        )
        local_coverage = [
            wgs84_to_local_meters(
                waypoint,
                DEFAULT_HOME,
                altitude_m=self.logic.config.takeoff_altitude,
            )
            for waypoint in coverage
        ]
        first_coverage = local_coverage[0]
        # _next_patrol_waypoint() only reaches this scoring loop once the
        # aircraft is already outside every NO_FLY cell (it flies clear of
        # any containing one first), so a plain route() is always valid here.
        route = self.no_fly_router.route(
            (position[0], position[1]),
            (first_coverage.x, first_coverage.y),
            self.patrol_regions.regions,
        )
        route_waypoints: list[Position] = [
            (x, y, self.logic.config.takeoff_altitude)
            for x, y in route
        ]
        transit_points = [
            (position[0], position[1]),
            *((waypoint[0], waypoint[1]) for waypoint in route_waypoints),
        ]
        transit_distance = sum(
            math.dist(start, end)
            for start, end in zip(transit_points, transit_points[1:])
        )
        coverage_waypoints: list[Position] = [
            (waypoint.x, waypoint.y, waypoint.z)
            for waypoint in local_coverage[1:]
        ]
        return (
            self._deduplicate_waypoints(
                [*route_waypoints, *coverage_waypoints]
            ),
            transit_distance,
        )

    def _pause_patrol_coverage(self, position: Position) -> None:
        if (
            self.logic.state is not FlightState.PATROLLING
            or self._target_patrol_region_id is None
            or self.logic.target is None
        ):
            return
        self._paused_patrol_coverage = PausedPatrolCoverage(
            region_id=self._target_patrol_region_id,
            interruption_position=(
                position[0],
                position[1],
                self.logic.config.takeoff_altitude,
            ),
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
            # Same reasoning as _coverage_route_waypoints: the resume point
            # may itself be inside a NO_FLY cell, so egress first.
            exit_point = self.no_fly_router.nearest_exit(
                (position[0], position[1]), self.patrol_regions.regions
            )
            route = self.no_fly_router.route(
                exit_point or (position[0], position[1]),
                (
                    paused.interruption_position[0],
                    paused.interruption_position[1],
                ),
                self.patrol_regions.regions,
            )
        except NoFlyRouteUnavailable:
            return False
        if exit_point is not None:
            route = [exit_point, *route]
        return_route: list[Position] = [
            (x, y, self.logic.config.takeoff_altitude)
            for x, y in route
        ]
        self._patrol_route_waypoints = deque(
            paused.resumed_waypoints(return_route)
        )
        self._target_patrol_region_id = paused.region_id
        self._paused_patrol_coverage = None
        return True

    def _clear_patrol_progress(self) -> None:
        self._discard_mission_interruption()
        self._patrol_route_waypoints.clear()
        self._target_patrol_region_id = None
        self._paused_patrol_coverage = None

    def _replan_active_patrol(self, position: Position) -> None:
        """Re-score patrol regions immediately, preserving in-progress coverage.

        Without this, an operator's mid-patrol priority or area-type edit only
        takes effect once the aircraft happens to finish covering whichever
        region it already committed to -- which, for a low-priority region
        picked before the edit, can be arbitrarily far in the future. Region
        selection itself already accounts for the operator's current
        priorities/area types (`PatrolRegionStore` is mutated before this
        runs), so forcing an immediate re-plan is enough to make both kinds of
        edit responsive.

        Simply discarding the in-progress coverage first (as this used to do)
        would also throw away real, already-flown ground and its eventual
        `record_observation()` credit every time an operator nudges a
        priority. So the active region's exact remaining waypoints are saved
        as a resumable checkpoint first -- the same `PausedPatrolCoverage`
        mechanism an ROI diversion already uses -- and `_next_patrol_waypoint`
        resumes that checkpoint instead of re-planning it from scratch if
        scoring picks the same region again, now or on a later edit. If a
        different region now scores higher, the checkpoint is simply left in
        place for whenever scoring returns to it.

        A checkpoint already paused for some other, currently-inactive reason
        (e.g. mid-ROI) is left untouched: there is nothing to fly differently
        right now, and its route is re-validated against current
        restrictions at the point it actually gets resumed.
        """
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

    def _set_motor_velocities(
        self,
        front_left: float,
        front_right: float,
        rear_left: float,
        rear_right: float,
    ) -> None:
        for motor, velocity in zip(
            self.motors,
            (front_left, front_right, rear_left, rear_right),
        ):
            motor.setVelocity(velocity)

    @staticmethod
    def _format_position(position) -> str:
        return f"({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"


if __name__ == "__main__":
    AutonomousDrone().run()
