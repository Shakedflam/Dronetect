"""Deterministic paths for background simulation actors."""

from __future__ import annotations

from dataclasses import dataclass
import math


Position = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class WaypointPath:
    waypoints: tuple[Position, ...]
    speed_mps: float
    phase_s: float = 0.0


@dataclass(frozen=True, slots=True)
class CircularPath:
    center: Position
    radius_m: float
    altitude_amplitude_m: float
    period_s: float
    phase_rad: float = 0.0


@dataclass(frozen=True, slots=True)
class BirdPath:
    center: Position
    radius_x_m: float
    radius_y_m: float
    altitude_amplitude_m: float
    period_s: float
    phase_rad: float = 0.0


def waypoint_position(path: WaypointPath, elapsed_s: float) -> Position:
    """Return a looping constant-speed position along waypoint segments."""
    if len(path.waypoints) == 0:
        return (0.0, 0.0, 0.0)
    if len(path.waypoints) == 1 or path.speed_mps <= 0.0:
        return path.waypoints[0]

    segment_lengths = [
        distance(path.waypoints[index], path.waypoints[(index + 1) % len(path.waypoints)])
        for index in range(len(path.waypoints))
    ]
    total_length = sum(segment_lengths)
    if total_length <= 0.0:
        return path.waypoints[0]

    traveled = ((max(0.0, elapsed_s) + path.phase_s) * path.speed_mps) % total_length
    for index, segment_length in enumerate(segment_lengths):
        if traveled <= segment_length:
            start = path.waypoints[index]
            end = path.waypoints[(index + 1) % len(path.waypoints)]
            ratio = 0.0 if segment_length <= 0.0 else traveled / segment_length
            return interpolate(start, end, ratio)
        traveled -= segment_length
    return path.waypoints[-1]


def circular_position(path: CircularPath, elapsed_s: float) -> Position:
    """Return a deterministic circular flight position."""
    period = max(path.period_s, 0.001)
    angle = path.phase_rad + (2.0 * math.pi * max(0.0, elapsed_s) / period)
    return (
        path.center[0] + path.radius_m * math.cos(angle),
        path.center[1] + path.radius_m * math.sin(angle),
        path.center[2] + path.altitude_amplitude_m * math.sin(angle * 2.0),
    )


def bird_position(path: BirdPath, elapsed_s: float) -> Position:
    """Return a looping bird distractor position."""
    period = max(path.period_s, 0.001)
    angle = path.phase_rad + (2.0 * math.pi * max(0.0, elapsed_s) / period)
    return (
        path.center[0] + path.radius_x_m * math.cos(angle),
        path.center[1] + path.radius_y_m * math.sin(angle),
        path.center[2] + path.altitude_amplitude_m * math.sin(angle * 3.0),
    )


def yaw_toward(previous: Position, current: Position, fallback: float = 0.0) -> float:
    dx = current[0] - previous[0]
    dy = current[1] - previous[1]
    if math.hypot(dx, dy) <= 1e-9:
        return fallback
    return math.atan2(dy, dx)


def interpolate(start: Position, end: Position, ratio: float) -> Position:
    bounded = min(max(ratio, 0.0), 1.0)
    return (
        start[0] + (end[0] - start[0]) * bounded,
        start[1] + (end[1] - start[1]) * bounded,
        start[2] + (end[2] - start[2]) * bounded,
    )


def distance(first: Position, second: Position) -> float:
    return math.sqrt(
        (second[0] - first[0]) ** 2
        + (second[1] - first[1]) ** 2
        + (second[2] - first[2]) ** 2
    )
