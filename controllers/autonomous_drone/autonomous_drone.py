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
from typing import cast

from controller import Camera, GPS, Gyro, InertialUnit, Motor, Supervisor

from camera_control import CameraGimbal
from flight_logic import (
    boost_forward_pitch,
    FlightConfig,
    FlightLogic,
    FlightState,
    Position,
    navigation_disturbances,
)
from hud import (
    HUD_STYLE,
    TelemetrySnapshot,
    distance_3d,
    format_telemetry_hud,
    signed_angle_to_target_degrees,
)
from world_markers import WorldMarkers


def clamp(value: float, minimum: float, maximum: float) -> float:
    #The drone will not react too aggressively
    return min(max(value, minimum), maximum)


class AutonomousDrone(Supervisor):
    #power needed to hover
    K_VERTICAL_THRUST = 68.5
    #adds small upward bias- drone does not sink easily
    K_VERTICAL_OFFSET = 0.6
    #power increase when the drone is too low or too high
    K_VERTICAL_P = 3.0
    #power increase to correct roll
    K_ROLL_P = 50.0
    #power increase to correct pitch (leaning forward or backward)
    K_PITCH_P = 30.0
    # Stronger forward pitch command during waypoint flight.
    K_FORWARD_SPEED_GAIN = 1.8

    def __init__(self) -> None:
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.logic = FlightLogic(
            FlightConfig(
                takeoff_altitude=15.0,
                x_bounds=(-60.0, -30.0),
                y_bounds=(5.0, 20.0),
                z_bounds=(13.0, 18.0),
                horizontal_tolerance=0.75,
                vertical_tolerance=0.5,
                landing_rate=1.0,
            )
        )

        self.markers = WorldMarkers(self)

        self.camera = cast(Camera, self.getDevice("camera"))
        #orientation: roll, pitch, yaw
        self.imu = cast(InertialUnit, self.getDevice("inertial unit"))
        self.gps = cast(GPS, self.getDevice("gps"))
        #angular velocity around roll, pitch, yaw axes
        self.gyro = cast(Gyro, self.getDevice("gyro"))
        for sensor in (self.camera, self.imu, self.gps, self.gyro):
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
        self._previous_position: Position | None = None
        self._elapsed_time = 0.0
        self._last_hud_update_time = -1.0

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
            roll_rate, pitch_rate, _ = self.gyro.getValues()
            speed_mps, vertical_speed_mps = self._update_motion_estimate(
                position,
                dt,
            )

            if self.logic.home is None:
                self.logic.initialize_home(position)
                print(f"Home captured at {self._format_position(position)}")

            self._handle_keyboard(position)
            self.logic.update(position, dt)
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

            if self.logic.state in {FlightState.IDLE, FlightState.LANDED}:
                self._stop_motors()
                continue

            target = self.logic.target
            assert target is not None
            yaw_input, pitch_disturbance = navigation_disturbances(
                position,
                target,
                yaw,
                self.logic.config.horizontal_tolerance,
            )
            pitch_disturbance = boost_forward_pitch(
                pitch_disturbance,
                self.K_FORWARD_SPEED_GAIN,
            )

            roll_input = self.K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_rate
            pitch_input = (
                self.K_PITCH_P * clamp(pitch, -1.0, 1.0)
                + pitch_rate
                + pitch_disturbance
            )
            #How far from target alltitude
            altitude_error = clamp(
                target[2] - position[2] + self.K_VERTICAL_OFFSET,
                -1.0,
                1.0,
            )
            vertical_input = self.K_VERTICAL_P * altitude_error**3

            front_left = (
                self.K_VERTICAL_THRUST
                + vertical_input
                - yaw_input
                + pitch_input
                - roll_input
            )
            front_right = (
                self.K_VERTICAL_THRUST
                + vertical_input
                + yaw_input
                + pitch_input
                + roll_input
            )
            rear_left = (
                self.K_VERTICAL_THRUST
                + vertical_input
                + yaw_input
                - pitch_input
                - roll_input
            )
            rear_right = (
                self.K_VERTICAL_THRUST
                + vertical_input
                - yaw_input
                - pitch_input
                + roll_input
            )
            self._set_motor_velocities(
                front_left,
                -front_right,
                -rear_left,
                rear_right,
            )

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

    def _update_motion_estimate(
        self,
        position: Position,
        dt: float,
    ) -> tuple[float, float]:
        previous = self._previous_position
        self._previous_position = position
        if previous is None or dt <= 0.0:
            return (0.0, 0.0)

        return (
            distance_3d(previous, position) / dt,
            (position[2] - previous[2]) / dt,
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
