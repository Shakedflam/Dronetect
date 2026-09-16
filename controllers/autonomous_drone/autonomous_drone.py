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
from services.no_fly_router import NoFlyRouteUnavailable, NoFlyRouter
from services.patrol_planner import PatrolPlanner
from services.patrol_regions import PatrolRegionStore
from services.telemetry import build_simulator_telemetry
from world_markers import WorldMarkers


def clamp(value: float, minimum: float, maximum: float) -> float:
    #The drone will not react too aggressively
    return min(max(value, minimum), maximum)


class AutonomousDrone(Supervisor):
    K_VISUAL_PROPELLER_SPEED = 68.5
    TAKEOFF_CAMERA_PITCH = -0.2
    HORIZONTAL_CAMERA_PITCH = 0.0
    KINEMATIC_FLIGHT_CONFIG = KinematicFlightConfig(
        max_horizontal_speed_mps=7.0,
        horizontal_acceleration_mps2=3.0,
        max_ascent_speed_mps=5.0,
        max_descent_speed_mps=3.0,
        max_yaw_rate_rad_s=math.radians(150.0),
    )
    SEARCH_KINEMATIC_FLIGHT_CONFIG = KinematicFlightConfig(
        max_horizontal_speed_mps=3.5,
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
        self._patrol_route_waypoints: deque[Position] = deque()
        self._target_patrol_region_id: str | None = None
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
            ),
            patrol_waypoint_provider=self._next_patrol_waypoint,
        )

        self.markers = WorldMarkers(self)
        self.self_node = self.getSelf()
        self.translation = self.self_node.getField("translation")
        self.rotation = self.self_node.getField("rotation")
        if self.translation is None or self.rotation is None:
            raise RuntimeError("Autonomous drone requires translation and rotation fields.")

        self.camera = cast(Camera, self.getDevice("camera"))
        #orientation: roll, pitch, yaw
        self.imu = cast(InertialUnit, self.getDevice("inertial unit"))
        self.gps = cast(GPS, self.getDevice("gps"))
        for sensor in (self.camera, self.imu, self.gps):
            sensor.enable(self.time_step)

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.time_step)

        self.camera_gimbal = CameraGimbal()
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
        self._announced_random_target = None
        self._elapsed_time = 0.0
        self._last_hud_update_time = -1.0
        self._horizontal_speed_mps = 0.0
        self._kinematic_yaw: float | None = None
        self._simulated_battery_percent = float(
            os.getenv("DRONTECT_SIMULATED_BATTERY_PERCENT", "100.0")
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

            self._handle_keyboard(position)
            self._handle_realtime_commands(position)
            self.logic.update(position, dt)
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
                    self._patrol_route_waypoints.clear()
                    self._target_patrol_region_id = None
                    print("Return Home requested: random flight cancelled.")
                else:
                    print(f"Return Home ignored while {self.logic.state.value}.")
            elif normalized_key in (ord("N"), ord("n")):
                self._patrol_route_waypoints.clear()
                self._target_patrol_region_id = None
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
            if command == CommandName.RETURN_HOME:
                if self.logic.request_return_home():
                    self._patrol_route_waypoints.clear()
                    self._target_patrol_region_id = None
                    print("Realtime command return_home accepted.")
                else:
                    print(f"Realtime command return_home ignored while {self.logic.state.value}.")
            elif command == CommandName.START_PATROL:
                self._patrol_route_waypoints.clear()
                self._target_patrol_region_id = None
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
                if self.logic.state is FlightState.PATROLLING:
                    self._patrol_route_waypoints.clear()
                    self._target_patrol_region_id = None
                    self.logic.target = self._next_patrol_waypoint(
                        position,
                        completed=False,
                    )
                print("Realtime patrol area types updated.")
            elif command == CommandName.SET_ROI:
                self._patrol_route_waypoints.clear()
                self._target_patrol_region_id = None
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
                # ROI missions intentionally override NO_FLY patrol routing;
                # the restriction remains stored and resumes with PATROL.
                print("Realtime command set_roi accepted.")
            elif command == CommandName.CLEAR_ROI:
                resume_patrol = self.logic.state in {
                    FlightState.TAKING_OFF,
                    FlightState.TRANSITING_TO_ROI,
                    FlightState.ROI_PATROL,
                }
                self.logic.clear_roi(position)
                if resume_patrol:
                    self._patrol_route_waypoints.clear()
                    self._target_patrol_region_id = None
                    self.logic.request_patrol(position)
                print("Realtime command clear_roi accepted.")
            elif command == CommandName.NEW_MISSION:
                self._patrol_route_waypoints.clear()
                self._target_patrol_region_id = None
                waypoint = self.logic.request_new_target(position)
                print(f"Realtime command new_mission: {self._format_position(waypoint)}")
            elif command == CommandName.CAMERA_TILT_UP:
                pitch = self.camera_gimbal.tilt_up()
                self.camera_pitch_motor.setPosition(pitch)
                print(f"Realtime command camera_tilt_up: {pitch:.2f}")
            elif command == CommandName.CAMERA_TILT_DOWN:
                pitch = self.camera_gimbal.tilt_down()
                self.camera_pitch_motor.setPosition(pitch)
                print(f"Realtime command camera_tilt_down: {pitch:.2f}")

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
                FlightState.FLYING_RANDOMLY,
                FlightState.RETURNING_HOME,
            }:
                self.camera_gimbal.set_pitch(self.HORIZONTAL_CAMERA_PITCH)
            elif state in {FlightState.LANDING, FlightState.LANDED}:
                self.camera_gimbal.set_pitch(self.camera_gimbal.maximum_pitch)

        self.camera_pitch_motor.setPosition(self.camera_gimbal.pitch)

    def _announce_changes(self) -> None:
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
        )
        self.realtime_client.publish_telemetry(telemetry)

    def _next_patrol_waypoint(
        self,
        position: Position,
        completed: bool,
    ) -> Position:
        if completed and self._patrol_route_waypoints:
            return self._patrol_route_waypoints.popleft()
        if completed and self._target_patrol_region_id is not None:
            self.patrol_planner.record_observation(self._target_patrol_region_id)
        current = local_meters_to_wgs84(
            LocalPosition(x=position[0], y=position[1], z=position[2]),
            DEFAULT_HOME,
        )
        decision = self.patrol_planner.choose_next(current)
        self._target_patrol_region_id = decision.region.id
        local_target = wgs84_to_local_meters(
            decision.region.center,
            DEFAULT_HOME,
            altitude_m=self.logic.config.takeoff_altitude,
        )
        try:
            route = self.no_fly_router.route(
                (position[0], position[1]),
                (local_target.x, local_target.y),
                self.patrol_regions.regions,
            )
        except NoFlyRouteUnavailable:
            self._target_patrol_region_id = None
            self._patrol_route_waypoints.clear()
            print("No safe patrol route is currently available; holding position.")
            return (
                position[0],
                position[1],
                self.logic.config.takeoff_altitude,
            )
        route_waypoints = [
            (x, y, self.logic.config.takeoff_altitude)
            for x, y in route
        ]
        self._patrol_route_waypoints = deque(route_waypoints[1:])
        print(f"Patrol target region: {decision.region.id}")
        return route_waypoints[0]

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


AutonomousDrone().run()
