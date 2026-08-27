"""Telemetry helpers for the Webots on-screen HUD."""

from __future__ import annotations

from dataclasses import dataclass
import math

try:
    from .flight_logic import FlightState, Position
except ImportError:  # Webots runs controller files as top-level scripts.
    from flight_logic import FlightState, Position


@dataclass(frozen=True, slots=True)
class TelemetrySnapshot:
    state: FlightState
    current: Position
    home: Position | None
    target: Position | None
    speed_mps: float
    vertical_speed_mps: float
    height_m: float
    distance_to_target_m: float | None
    angle_to_target_degrees: float | None
    roll_degrees: float
    pitch_degrees: float
    yaw_degrees: float


@dataclass(frozen=True, slots=True)
class HudStyle:
    # Webots setLabel uses normalized screen coordinates in the [0, 1] range.
    x: float = 0.62
    y: float = 0.02
    shadow_offset: float = 0.003
    font_size: float = 0.035
    shadow_enabled: bool = False
    shadow_color: int = 0x000000
    text_color: int = 0x111111
    font: str = "Arial"


HUD_STYLE = HudStyle()


def distance_3d(first: Position, second: Position) -> float:
    """Return full 3D distance between two Webots GPS positions."""
    return math.sqrt(
        (second[0] - first[0]) ** 2
        + (second[1] - first[1]) ** 2
        + (second[2] - first[2]) ** 2
    )


def signed_angle_to_target_degrees(
    position: Position,
    target: Position | None,
    yaw: float,
) -> float | None:
    """Return signed heading error to target in degrees; left is positive."""
    if target is None:
        return None

    dx = target[0] - position[0]
    dy = target[1] - position[1]
    if math.hypot(dx, dy) <= 1e-9:
        return 0.0

    desired_yaw = math.atan2(dy, dx)
    angle = (desired_yaw - yaw + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(angle)


def format_telemetry_hud(snapshot: TelemetrySnapshot) -> str:
    """Format a compact Webots setLabel telemetry table."""
    target = _format_optional_position(snapshot.target)
    home = _format_optional_position(snapshot.home)
    distance = _format_optional_value(snapshot.distance_to_target_m, "m")
    angle = _format_optional_value(snapshot.angle_to_target_degrees, "deg", 1)

    return "\n".join(
        (
            "DRONE TELEMETRY",
            f"State   {snapshot.state.value}",
            f"Target  {target}",
            f"Current {_format_position_one_decimal(snapshot.current)}",
            f"Home    {home}",
            f"Speed   {snapshot.speed_mps:.2f} m/s",
            f"Height  {snapshot.height_m:.2f} m",
            f"VSpeed  {snapshot.vertical_speed_mps:.2f} m/s",
            f"Dist    {distance}",
            f"Angle   {angle}",
            (
                "Tilt    "
                f"R {snapshot.roll_degrees:.1f}  "
                f"P {snapshot.pitch_degrees:.1f}  "
                f"Y {snapshot.yaw_degrees:.1f} deg"
            ),
        )
    )


def _format_optional_position(position: Position | None) -> str:
    if position is None:
        return "--"
    return _format_position_one_decimal(position)


def _format_position_one_decimal(position: Position) -> str:
    return f"{position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}"


def _format_optional_value(
    value: float | None,
    unit: str,
    precision: int = 2,
) -> str:
    if value is None:
        return "--"
    return f"{value:.{precision}f} {unit}"
