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

import math
from pathlib import Path
import sys
from typing import cast

from controller import Camera, GPS, InertialUnit, Motor, Supervisor

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from adapters.simulator.client import SimulatorRealtimeClient
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
from services.gps import LocalPosition
from services.telemetry import build_simulator_telemetry
from world_markers import WorldMarkers


def clamp(value: float, minimum: float, maximum: float) -> float:
    #The drone will not react too aggressively
    return min(max(value, minimum), maximum)


class AutonomousDrone(Supervisor):
    K_VISUAL_PROPELLER_SPEED = 68.5
    KINEMATIC_FLIGHT_CONFIG = KinematicFlightConfig(
        max_horizontal_speed_mps=14.0,
        horizontal_acceleration_mps2=6.0,
        max_ascent_speed_mps=5.0,
        max_descent_speed_mps=3.0,
        max_yaw_rate_rad_s=math.radians(150.0),
    )

    def __init__(self) -> None:
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.logic = FlightLogic(
            FlightConfig(
                takeoff_altitude=15.0,
                x_bounds=(-60.0, -30.0),
                y_bounds=(5.0, 20.0),
                z_bounds=(13.0, 18.0),
                horizontal_tolerance=1.2,
                vertical_tolerance=0.5,
                landing_rate=1.0,
            )
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
        self.realtime_client = SimulatorRealtimeClient()
        self.realtime_client.start()

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
            self._set_visual_motor_velocities()

    def _handle_keyboard(self, position: Position) -> None:
        key = self.keyboard.getKey()
        while key != -1:
            normalized_key = key & 0xFFFF
            if normalized_key in (ord("H"), ord("h")):
                if self.logic.request_return_home():
                    print("Return Home requested: random flight cancelled.")
                else:
                    print(f"Return Home ignored while {self.logic.state.value}.")
            elif normalized_key in (ord("N"), ord("n")):
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
            command = message["command"]
            if command == CommandName.RETURN_HOME:
                if self.logic.request_return_home():
                    print("Realtime command return_home accepted.")
                else:
                    print(f"Realtime command return_home ignored while {self.logic.state.value}.")
            elif command == CommandName.NEW_MISSION:
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
            config=self.KINEMATIC_FLIGHT_CONFIG,
        )
        self.translation.setSFVec3f(list(step.position))
        self.rotation.setSFRotation([0.0, 0.0, 1.0, step.yaw])
        self._kinematic_yaw = step.yaw
        self._horizontal_speed_mps = step.horizontal_speed_mps
        return (
            step.position,
            step.yaw,
            step.speed_mps,
            step.vertical_speed_mps,
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
        )
        self.realtime_client.publish_telemetry(telemetry)

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
