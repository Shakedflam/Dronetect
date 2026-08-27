"""Reusable deterministic profiles for background simulation actors."""

from __future__ import annotations


def traffic_drone_profile(index: int) -> dict:
    behavior = ("patrol", "circle", "flythrough")[index % 3]
    group = index // 3
    altitude_offset = group * 1.6
    lateral_offset = group * 4.0

    if behavior == "patrol":
        waypoints = [
            [-10.0 - lateral_offset, -5.0, 9.0 + altitude_offset],
            [-20.0, 1.0 + lateral_offset, 10.5 + altitude_offset],
            [-32.0 + lateral_offset, 12.0, 9.8 + altitude_offset],
            [-18.0, 17.0 - lateral_offset, 11.2 + altitude_offset],
        ]
        return {
            "behavior": behavior,
            "start": waypoints[0],
            "yaw": 0.2,
            "waypoints": waypoints,
            "speed_mps": 4.0 + group * 0.4,
            "phase_s": group * 3.5,
        }

    if behavior == "circle":
        center = [-13.0 + lateral_offset, 7.0, 12.0 + altitude_offset]
        radius = 5.0 + group * 1.5
        return {
            "behavior": behavior,
            "start": [center[0] + radius, center[1], center[2]],
            "yaw": 1.5708,
            "center": center,
            "radius_m": radius,
            "altitude_amplitude_m": 0.8,
            "period_s": 24.0 + group * 4.0,
            "phase_rad": group * 0.7,
        }

    waypoints = [
        [-6.0, 18.0 + lateral_offset, 11.0 + altitude_offset],
        [-18.0, 10.0, 11.8 + altitude_offset],
        [-35.0, 3.0 - lateral_offset, 12.4 + altitude_offset],
        [-55.0, -5.0, 11.6 + altitude_offset],
    ]
    return {
        "behavior": behavior,
        "start": waypoints[0],
        "yaw": 0.55,
        "waypoints": waypoints,
        "speed_mps": 6.0 + group * 0.6,
        "phase_s": group * 2.0,
    }


def bird_profile(index: int) -> dict:
    group = index // 6
    slot = index % 6
    center_x = -50.0 + (slot % 3) * 8.0 + group * 3.0
    center_y = 4.0 + (slot // 3) * 14.0 - group * 2.0
    altitude = 12.0 + (slot % 3) * 1.4 + group
    phase = slot * 0.75 + group * 0.4
    radius_x = 5.0 + (slot % 2) * 2.0
    radius_y = 3.0 + ((slot + 1) % 2) * 2.0
    return {
        "start": [
            center_x + radius_x,
            center_y,
            altitude,
        ],
        "yaw": 1.5708,
        "center": [center_x, center_y, altitude],
        "radius_x_m": radius_x,
        "radius_y_m": radius_y,
        "altitude_amplitude_m": 0.35 + 0.1 * (slot % 3),
        "period_s": 10.0 + slot * 1.7 + group,
        "phase_rad": phase,
        "scale": 0.9 + 0.08 * (slot % 3),
    }
