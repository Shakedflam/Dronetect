import pytest

from controllers.autonomous_drone.flight_logic import FlightState
from controllers.autonomous_drone.hud import (
    HUD_STYLE,
    TelemetrySnapshot,
    distance_3d,
    format_telemetry_hud,
    signed_angle_to_target_degrees,
)


HOME = (0.0, 0.0, 0.1)


def test_distance_3d_includes_horizontal_and_vertical_difference():
    assert distance_3d((0.0, 0.0, 1.0), (3.0, 4.0, 13.0)) == pytest.approx(13.0)


def test_signed_angle_to_target_reports_left_as_positive_degrees():
    angle = signed_angle_to_target_degrees(
        position=(0.0, 0.0, 2.5),
        target=(0.0, 10.0, 2.5),
        yaw=0.0,
    )

    assert angle == pytest.approx(90.0)


def test_signed_angle_to_target_reports_none_without_target():
    assert signed_angle_to_target_degrees(HOME, None, yaw=0.0) is None


def test_format_telemetry_hud_contains_key_flight_values():
    hud = format_telemetry_hud(
        TelemetrySnapshot(
            state=FlightState.FLYING_RANDOMLY,
            current=(1.0, 2.0, 3.0),
            home=(0.0, 0.0, 0.5),
            target=(4.0, 6.0, 3.0),
            speed_mps=5.25,
            vertical_speed_mps=-0.2,
            height_m=2.5,
            distance_to_target_m=5.0,
            angle_to_target_degrees=53.13,
            roll_degrees=1.2,
            pitch_degrees=-3.4,
            yaw_degrees=90.0,
        )
    )

    assert "DRONE TELEMETRY" in hud
    assert "State   FLYING_RANDOMLY" in hud
    assert "Target  4.0, 6.0, 3.0" in hud
    assert "Speed   5.25 m/s" in hud
    assert "Angle   53.1 deg" in hud
    assert "Tilt    R 1.2  P -3.4  Y 90.0 deg" in hud


def test_hud_style_is_large_and_anchored_near_top_right():
    assert 0.0 <= HUD_STYLE.x <= 1.0
    assert 0.0 <= HUD_STYLE.y <= 1.0
    assert HUD_STYLE.x >= 0.55
    assert HUD_STYLE.y <= 0.04
    assert HUD_STYLE.font_size >= 0.03
    assert HUD_STYLE.shadow_enabled is False
    assert HUD_STYLE.font == "Arial"
