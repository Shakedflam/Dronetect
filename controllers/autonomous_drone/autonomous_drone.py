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
import math
import os
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
from flight_logic import (
    advance_kinematic_flight,
    FlightConfig,
    FlightLogic,
    FlightState,
    KinematicFlightConfig,
    Position,
)
from hud import (
    HUD_STYLE,
    TelemetrySnapshot,
    distance_3d,
    format_telemetry_hud,
    signed_angle_to_target_degrees,
)
from services.commands import CommandName
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
from services.patrol_planner import PatrolPlanner
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


class AutonomousDrone(Supervisor):
    K_VISUAL_PROPELLER_SPEED = 68.5
    TAKEOFF_CAMERA_PITCH = -0.2
    # Keep a small amount of ground in view during horizontal flight.
    HORIZONTAL_CAMERA_PITCH = 0.16
    KINEMATIC_FLIGHT_CONFIG = KinematicFlightConfig(
        max_horizontal_speed_mps=12.0,
        horizontal_acceleration_mps2=3.0,
        max_ascent_speed_mps=5.0,
        max_descent_speed_mps=3.0,
        max_yaw_rate_rad_s=math.radians(150.0),
    )
    SEARCH_KINEMATIC_FLIGHT_CONFIG = KinematicFlightConfig(
        max_horizontal_speed_mps=12.0,
        horizontal_acceleration_mps2=1.5,
        max_ascent_speed_mps=5.0,
        max_descent_speed_mps=3.0,
        max_yaw_rate_rad_s=math.radians(150.0),
    )

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
        self.logic = FlightLogic(
            FlightConfig(
                takeoff_altitude=float(
                    os.getenv("DRONTECT_TAKEOFF_ALTITUDE_M", "15.0")
                ),
                x_bounds=(-60.0, -30.0),
                y_bounds=(5.0, 20.0),
                z_bounds=(13.0, 18.0),
                horizontal_tolerance=1.2,
                vertical_tolerance=0.5,
                landing_rate=1.0,
                roi_arrival_tolerance=float(
                    os.getenv("DRONTECT_ROI_ARRIVAL_TOLERANCE_M", "10.0")
                ),
                target_min_confidence=float(
                    os.getenv("DRONTECT_TARGET_MIN_CONFIDENCE", "0.45")
                ),
                target_confirmation_hits=int(
                    os.getenv("DRONTECT_TARGET_CONFIRMATION_HITS", "2")
                ),
                target_observation_s=float(os.getenv("DRONTECT_TARGET_OBSERVATION_S", "1.7")),
                target_observation_min_samples=int(os.getenv("DRONTECT_TARGET_OBSERVATION_MIN_SAMPLES", "3")),
                target_stationary_center_ratio=float(os.getenv("DRONTECT_TARGET_STATIONARY_CENTER_RATIO", "0.02")),
                target_stationary_observer_m=float(os.getenv("DRONTECT_TARGET_STATIONARY_OBSERVER_M", "0.25")),
                target_stationary_angle_rad=math.radians(float(os.getenv("DRONTECT_TARGET_STATIONARY_ANGLE_DEG", "2"))),
                target_low_confidence=float(os.getenv("DRONTECT_TARGET_LOW_CONFIDENCE", "0.60")),
                target_confidence_range=float(os.getenv("DRONTECT_TARGET_CONFIDENCE_RANGE", "0.20")),
                target_ignore_radius_m=float(os.getenv("DRONTECT_TARGET_IGNORE_RADIUS_M", "5")),
                target_ignore_timeout_s=float(os.getenv("DRONTECT_TARGET_IGNORE_TIMEOUT_S", "6")),
                target_reject_grace_s=float(os.getenv("DRONTECT_TARGET_REJECT_GRACE_S", "1.5")),
                target_projection_min_angle_rad=math.radians(float(os.getenv("DRONTECT_TARGET_PROJECTION_MIN_ANGLE_DEG", "5"))),
                target_projection_max_range_m=float(os.getenv("DRONTECT_TARGET_PROJECTION_MAX_RANGE_M", "250")),
                target_acquisition_timeout_s=float(
                    os.getenv("DRONTECT_TARGET_ACQUISITION_TIMEOUT_S", "2.5")
                ),
                target_lost_search_timeout_s=float(
                    os.getenv("DRONTECT_TARGET_LOST_SEARCH_TIMEOUT_S", "2.0")
                ),
                target_yaw_gain=float(
                    os.getenv("DRONTECT_TARGET_YAW_GAIN", "0.35")
                ),
                target_yaw_prediction_horizon_s=float(
                    os.getenv("DRONTECT_TARGET_YAW_PREDICTION_SECONDS", "0.5")
                ),
                target_yaw_deadband_ratio=float(
                    os.getenv("DRONTECT_TARGET_YAW_DEADBAND_RATIO", "0.04")
                ),
                target_velocity_smoothing=float(
                    os.getenv("DRONTECT_TARGET_VELOCITY_SMOOTHING", "0.35")
                ),
                target_max_yaw_rate_rad_s=math.radians(
                    float(os.getenv("DRONTECT_TARGET_MAX_YAW_RATE_DEG_S", "18.0"))
                ),
                target_search_yaw_rate_rad_s=math.radians(
                    float(os.getenv("DRONTECT_TARGET_SEARCH_YAW_RATE_DEG_S", "10.0"))
                ),
                target_gimbal_pitch_gain=float(
                    os.getenv("DRONTECT_TARGET_GIMBAL_PITCH_GAIN", "0.25")
                ),
                target_center_tolerance_x=float(
                    os.getenv("DRONTECT_TARGET_CENTER_TOLERANCE_X", "0.08")
                ),
                target_center_tolerance_y=float(
                    os.getenv("DRONTECT_TARGET_CENTER_TOLERANCE_Y", "0.12")
                ),
                target_follow_step_m=float(
                    os.getenv("DRONTECT_TARGET_FOLLOW_STEP_M", "6.0")
                ),
                target_standoff_box_area_ratio=float(
                    os.getenv("DRONTECT_TARGET_STANDOFF_BOX_AREA_RATIO", "0.025")
                ),
                target_collision_avoidance_step_m=float(
                    os.getenv("DRONTECT_TARGET_COLLISION_AVOIDANCE_STEP_M", "3.0")
                ),
                target_takeoff_pursuit_clearance_m=float(
                    os.getenv("DRONTECT_TARGET_TAKEOFF_PURSUIT_CLEARANCE_M", "2.0")
                ),
                target_takeoff_pursuit_ascent_mps=float(
                    os.getenv("DRONTECT_TARGET_TAKEOFF_PURSUIT_ASCENT_MPS", "2.0")
                ),
            ),
            patrol_waypoint_provider=self._next_patrol_waypoint,
            pursuit_config=PursuitConfig(
                stable_duration_s=float(os.getenv("DRONTECT_PURSUIT_STABLE_SECONDS", "0.8")),
                observation_timeout_s=float(os.getenv("DRONTECT_PURSUIT_OBSERVATION_TIMEOUT_SECONDS", "1.5")),
                max_speed_mps=float(os.getenv("DRONTECT_PURSUIT_MAX_SPEED_MPS", "5.0")),
                acceleration_mps2=float(os.getenv("DRONTECT_PURSUIT_ACCELERATION_MPS2", "2.0")),
                geofence_radius_m=max(region.outer_radius_m for region in self.patrol_regions.regions),
            ),
        )
        self._target_tracking_flight_config = KinematicFlightConfig(
            max_horizontal_speed_mps=self.SEARCH_KINEMATIC_FLIGHT_CONFIG.max_horizontal_speed_mps,
            horizontal_acceleration_mps2=self.SEARCH_KINEMATIC_FLIGHT_CONFIG.horizontal_acceleration_mps2,
            max_ascent_speed_mps=self.SEARCH_KINEMATIC_FLIGHT_CONFIG.max_ascent_speed_mps,
            max_descent_speed_mps=self.SEARCH_KINEMATIC_FLIGHT_CONFIG.max_descent_speed_mps,
            max_yaw_rate_rad_s=self.logic.config.target_max_yaw_rate_rad_s,
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
        self._announced_random_target = None
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
        self._connection_loss_rth_active = False
        self._safety_rth_active = False
        self._safety_rth_reason: str | None = None
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
            self._handle_keyboard(position)
            self._handle_realtime_commands(position)
            state_before_detection = self.logic.state
            self._handle_vision_detections(position, yaw)
            self.logic.update(position, dt, yaw=yaw, camera_pitch=self.camera_gimbal.pitch)
            self._coordinate_mission_resume(position, state_before_detection)
            position, yaw, speed_mps, vertical_speed_mps = self._apply_kinematic_flight(
                position,
                yaw,
                dt,
            )
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

    def _handle_keyboard(self, position: Position) -> None:
        key = self.keyboard.getKey()
        while key != -1:
            normalized_key = key & 0xFFFF
            if normalized_key in (ord("H"), ord("h")):
                if self.logic.request_return_home():
                    self._clear_patrol_progress()
                    print("Return Home requested: random flight cancelled.")
                else:
                    print(f"Return Home ignored while {self.logic.state.value}.")
            elif normalized_key in (ord("N"), ord("n")):
                if self._safety_rth_active:
                    print("New mission ignored during safety RTH.")
                    key = self.keyboard.getKey()
                    continue
                self._clear_patrol_progress()
                waypoint = self.logic.request_new_target(position)
                print(f"New mission target: {self._format_position(waypoint)}")
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
            if self._safety_rth_active and command in {
                CommandName.START_PATROL,
                CommandName.SET_ROI,
                CommandName.CLEAR_ROI,
                CommandName.NEW_MISSION,
                CommandName.APPROVE_TARGET,
                CommandName.REJECT_TARGET,
            }:
                print(
                    f"Realtime command {command.value} ignored during "
                    "safety RTH."
                )
                continue
            if command == CommandName.RETURN_HOME:
                safety_reason = message.get("safetyReason")
                if isinstance(safety_reason, str):
                    self._safety_rth_active = True
                    self._safety_rth_reason = safety_reason
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
                print("Realtime patrol priorities updated.")
            elif command == CommandName.SET_PATROL_AREA_TYPES:
                area_types = message["params"]["areaTypes"]
                self.patrol_regions.replace_area_types(
                    [(entry["regionId"], entry["areaType"]) for entry in area_types]
                )
                if (
                    self.logic.state is FlightState.PATROLLING
                    or self._paused_patrol_coverage is not None
                ):
                    was_patrolling = self.logic.state is FlightState.PATROLLING
                    self._clear_patrol_progress()
                    if was_patrolling:
                        self.logic.target = self._next_patrol_waypoint(
                            position,
                            completed=False,
                        )
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
                }
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
            elif command == CommandName.NEW_MISSION:
                self._clear_patrol_progress()
                self._active_roi_id = None
                waypoint = self.logic.request_new_target(position)
                print(f"Realtime command new_mission: {self._format_position(waypoint)}")
            elif command == CommandName.CAMERA_TILT_UP:
                pitch = self.camera_gimbal.tilt_up()
                self.camera_pitch_motor.setPosition(pitch)
                print(f"Realtime command camera_tilt_up: {pitch:.2f}")
            elif command == CommandName.APPROVE_TARGET:
                self.logic.approve_target(message["params"]["trackId"], position)
            elif command == CommandName.REJECT_TARGET:
                self.logic.reject_target(message["params"]["trackId"], position)
            elif command == CommandName.CAMERA_TILT_DOWN:
                pitch = self.camera_gimbal.tilt_down()
                self.camera_pitch_motor.setPosition(pitch)
                print(f"Realtime command camera_tilt_down: {pitch:.2f}")

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
            self.logic.hold_target_return(position)
            self._mission_return_prepared = False
            print(f"Mission resume waiting for a safe NO_FLY egress route: {exc}")
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

    def _discard_mission_interruption(self) -> None:
        self._mission_interruption = None
        self._mission_return_prepared = False

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
                FlightState.TARGET_ACQUISITION,
                FlightState.TARGET_FOLLOW,
                FlightState.TARGET_PURSUIT,
                FlightState.TARGET_RETURN,
                FlightState.FLYING_RANDOMLY,
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

        if (
            self.logic.state is FlightState.FLYING_RANDOMLY
            and self.logic.target != self._announced_random_target
        ):
            print(f"Random waypoint: {self._format_position(self.logic.target)}")
            self._announced_random_target = self.logic.target

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
        step = advance_kinematic_flight(
            position=position,
            target=target,
            yaw=self._kinematic_yaw,
            horizontal_speed_mps=self._horizontal_speed_mps,
            dt=dt,
            config=self._kinematic_config_for_state(),
            desired_yaw=self.logic.target_yaw,
            velocity_command=(self.logic.pursuit.velocity
                              if self.logic.state is FlightState.TARGET_PURSUIT else None),
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
        if self.logic.state is FlightState.TARGET_RETURN:
            return replace(self.SEARCH_KINEMATIC_FLIGHT_CONFIG, yaw_hold_horizontal_distance_m=0.0)
        if self.logic.state in {
            FlightState.TARGET_ACQUISITION,
            FlightState.TARGET_FOLLOW,
            FlightState.TARGET_PURSUIT,
        }:
            return self._target_tracking_flight_config
        if self.logic.state in {
            FlightState.FLYING_RANDOMLY,
            FlightState.ROI_PATROL,
            FlightState.PATROLLING,
        }:
            return self.SEARCH_KINEMATIC_FLIGHT_CONFIG
        return self.KINEMATIC_FLIGHT_CONFIG

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
            FlightState.TARGET_ACQUISITION,
            FlightState.TARGET_FOLLOW,
            FlightState.TARGET_PURSUIT,
            FlightState.TARGET_RETURN,
            FlightState.FLYING_RANDOMLY,
            FlightState.RETURNING_HOME,
            FlightState.LANDING,
        }
        if self.logic.state not in airborne_states:
            return
        self._connection_loss_rth_active = True
        self._safety_rth_active = True
        self._safety_rth_reason = CONNECTION_LOSS_REASON
        self._clear_patrol_progress()
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
        self._target_patrol_region_id = decision.region.id
        ordered = candidate_waypoints[decision.region.id]
        self._patrol_route_waypoints = deque(ordered[1:])
        print(
            f"Patrol target region: {decision.region.id} "
            f"(safe route {decision.score.distance_m:.1f} m)"
        )
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
            outward_from=DEFAULT_HOME,
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
            route = self.no_fly_router.route(
                (position[0], position[1]),
                (
                    paused.interruption_position[0],
                    paused.interruption_position[1],
                ),
                self.patrol_regions.regions,
            )
        except NoFlyRouteUnavailable:
            return False
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
