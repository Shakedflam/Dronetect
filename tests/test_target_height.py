"""Measuring how high the target is, and holding station above it.

The measurement is geometric: slant range from box area, camera pitch, where
the box sits vertically in frame, and the aircraft's own altitude. Nothing
here is a guess, so the aircraft settling metres above a target is always a
fault in one of those four inputs rather than something to tune around.
"""

import math

import pytest

from controllers.autonomous_drone.pursuit import (
    PursuitConfig,
    PursuitController,
    estimate_target_altitude,
    reference_area_ratio_from_span,
)


FOV = 0.7854
FRAME = (960, 540)
SPAN = 0.45
REFERENCE_DISTANCE_M = 2.0
REFERENCE_RATIO = reference_area_ratio_from_span(
    span_m=SPAN, distance_m=REFERENCE_DISTANCE_M, horizontal_fov_rad=FOV,
    frame_width=FRAME[0], frame_height=FRAME[1],
)


def sighting(*, horizontal_range_m, height_difference_m, camera_pitch_rad=None):
    """Build the box and pitch a target at that offset would really produce.

    Returns the box plus the camera pitch that centres it, so a test states
    the geometry it means and the projection maths is exercised for real.
    """
    slant = math.hypot(horizontal_range_m, height_difference_m)
    elevation = math.asin(height_difference_m / slant)
    pitch = -elevation if camera_pitch_rad is None else camera_pitch_rad

    # Box size from the true slant range.
    width_fraction = (SPAN / slant) / (2.0 * math.tan(FOV / 2.0))
    half = width_fraction * FRAME[0] / 2.0

    # Vertical image offset of a ray at this elevation, given the pitch.
    # down is the normalized image coordinate the projection inverts.
    forward_world = math.cos(elevation)
    up_world = math.sin(elevation)
    # Rotate into camera axes: pitch is positive nose-down.
    forward_cam = forward_world * math.cos(pitch) - up_world * math.sin(pitch)
    up_cam = forward_world * math.sin(pitch) + up_world * math.cos(pitch)
    down = -up_cam / forward_cam
    center_y = (
        down / (math.tan(FOV / 2.0) * FRAME[1] / FRAME[0]) + 1.0
    ) / 2.0 * FRAME[1]
    center_x = FRAME[0] / 2.0
    box = [center_x - half, center_y - half, center_x + half, center_y + half]
    return box, pitch


def measure(*, horizontal_range_m, height_difference_m):
    box, pitch = sighting(
        horizontal_range_m=horizontal_range_m,
        height_difference_m=height_difference_m,
    )
    return estimate_target_altitude(
        box=box, frame_width=FRAME[0], frame_height=FRAME[1],
        camera_pitch=pitch, horizontal_fov_rad=FOV,
        reference_distance_m=REFERENCE_DISTANCE_M,
        reference_area_ratio=REFERENCE_RATIO,
        max_range_m=200.0,
    )


# --- the measurement ---------------------------------------------------------


@pytest.mark.parametrize("horizontal_range_m,height_difference_m", [
    (20.0, -10.0),
    (20.0, +10.0),
    (40.0, -5.0),
    (8.0, -1.0),
    (30.0, 0.0),
])
def test_the_height_difference_is_measured_not_guessed(
    horizontal_range_m, height_difference_m
):
    """Slant range, pitch, image-y and our own altitude give it exactly."""
    estimate = measure(
        horizontal_range_m=horizontal_range_m,
        height_difference_m=height_difference_m,
    )
    assert estimate is not None
    assert estimate.height_error_m == pytest.approx(height_difference_m, abs=0.05)


def test_the_horizontal_range_falls_out_of_the_same_sighting():
    """The pursuit gates dodges on this, so it has to come out right too."""
    estimate = measure(horizontal_range_m=30.0, height_difference_m=-12.0)
    assert estimate is not None
    horizontal = estimate.slant_range_m * math.cos(estimate.elevation_rad)
    assert horizontal == pytest.approx(30.0, abs=0.2)


def test_a_target_below_reads_below_however_the_camera_is_pointed():
    """Pitch must not leak into the answer: it is an input, not a bias."""
    for pitch_offset in (-0.25, 0.0, 0.25):
        box, pitch = sighting(horizontal_range_m=25.0, height_difference_m=-8.0)
        estimate = estimate_target_altitude(
            box=box, frame_width=FRAME[0], frame_height=FRAME[1],
            camera_pitch=pitch + pitch_offset, horizontal_fov_rad=FOV,
            reference_distance_m=REFERENCE_DISTANCE_M,
            reference_area_ratio=REFERENCE_RATIO, max_range_m=200.0,
        )
        assert estimate is not None
        assert estimate.height_error_m < 0.0


def test_an_underreported_range_drags_the_height_toward_level():
    """The regression that put the aircraft metres above its target.

    Height is slant range times the sine of the elevation, so a range that is
    short by a factor reports a height difference short by the same factor --
    a target 10 m below looks 4 m below, and the aircraft settles high by the
    remainder while every gain and band involved is working correctly.
    """
    box, pitch = sighting(horizontal_range_m=20.0, height_difference_m=-10.0)
    shared = dict(
        box=box, frame_width=FRAME[0], frame_height=FRAME[1], camera_pitch=pitch,
        horizontal_fov_rad=FOV, reference_distance_m=REFERENCE_DISTANCE_M,
        max_range_m=200.0,
    )
    honest = estimate_target_altitude(reference_area_ratio=REFERENCE_RATIO, **shared)
    # The old calibration: the standoff setpoint pressed into service as one.
    stale = estimate_target_altitude(reference_area_ratio=0.025, **shared)

    assert honest.height_error_m == pytest.approx(-10.0, abs=0.05)
    assert stale.height_error_m > honest.height_error_m + 4.0


# --- holding station on it ---------------------------------------------------


def hold_altitude(*, band_m, gain_per_s, start_offset_m, ticks=80, dt=0.1):
    """Fly the vertical law alone and report the settled offset and overshoot."""
    config = PursuitConfig(
        max_speed_mps=6.0, acceleration_mps2=5.0, smoothing_time_s=0.2,
        altitude_offset_m=3.0,
    )
    pursuit = PursuitController(config)
    # Target sits at 20 m; the aircraft starts start_offset_m above station.
    target_altitude = 20.0
    station = target_altitude + config.altitude_offset_m
    altitude = station + start_offset_m
    worst = 0.0
    for _ in range(ticks):
        velocity = pursuit.update(
            position=(0.0, 0.0, altitude), home=(0.0, 0.0, 0.0), yaw=0.0,
            image_velocity=(0.0, 0.0), area=0.01, growth=0.0,
            desired_area=0.01, area_deadzone=0.001,
            collision_side=0.0, lateral_gain=0.0, dt=dt,
            height_error_m=target_altitude - altitude,
            altitude_deadband_m=band_m,
            altitude_gain_per_s=gain_per_s,
            max_vertical_speed_mps=2.0,
        )
        altitude += velocity[2] * dt
        worst = max(worst, station - altitude)
    return altitude - station, worst


def test_a_tighter_band_settles_closer_to_the_station_height():
    """The band absorbs box jitter; it should not licence a metre of droop."""
    loose, _ = hold_altitude(band_m=0.5, gain_per_s=1.0, start_offset_m=6.0)
    tight, _ = hold_altitude(band_m=0.3, gain_per_s=1.5, start_offset_m=6.0)
    assert abs(tight) < abs(loose)
    assert abs(tight) <= 0.35


def test_holding_tighter_does_not_start_hunting():
    """Quicker correction must not turn into oscillation around the station."""
    _, undershoot = hold_altitude(band_m=0.3, gain_per_s=1.5, start_offset_m=6.0)
    assert undershoot < 0.5


def test_the_aircraft_climbs_to_station_when_it_starts_below():
    settled, _ = hold_altitude(band_m=0.3, gain_per_s=1.5, start_offset_m=-6.0)
    assert abs(settled) <= 0.35


# --- yaw ---------------------------------------------------------------------


def yaw_rate_limit(*, bearing_error, time_constant_s, ceiling_deg=120.0,
                   crossing_speed_mps=None, range_m=10.0):
    """The per-frame yaw ceiling the tracking law derives for that sighting."""
    from controllers.autonomous_drone.flight_logic import FlightConfig, FlightLogic
    from controllers.autonomous_drone.pursuit import TargetState

    logic = FlightLogic(
        config=FlightConfig(
            target_yaw_time_constant_s=time_constant_s,
            target_max_yaw_rate_rad_s=math.radians(ceiling_deg),
        ),
        pursuit_config=PursuitConfig(horizontal_fov_rad=FOV),
    )
    logic.initialize_home((0.0, 0.0, 0.0))
    logic._target_last_center = (0.5 + bearing_error, 0.5)
    if crossing_speed_mps is not None:
        logic._target_state = TargetState(
            position=(range_m, 0.0, 15.0),
            velocity=(0.0, crossing_speed_mps, 0.0),
            range_m=range_m,
            speed_mps=abs(crossing_speed_mps),
            closure_mps=0.0,
        )
    logic._update_target_yaw_rate_limit((0.0, 0.0, 15.0))
    return logic.target_yaw_rate_limit


def test_a_bearing_error_is_taken_out_faster():
    """Keeping the target near frame centre is what keeps the track alive.

    A target drifting toward the edge is one the tracker is about to lose
    identity on, and whose box the range estimate distorts most.
    """
    before = yaw_rate_limit(bearing_error=0.3, time_constant_s=0.6)
    now = yaw_rate_limit(bearing_error=0.3, time_constant_s=0.45)
    assert now > before


def test_a_close_fast_crossing_target_is_tracked_up_to_the_ceiling():
    """The feed-forward is what a hard crossing pass actually needs.

    Holding one centred takes v_perp / range rad/s, which for a fast target
    at close range far exceeds anything the bearing error alone would ask
    for. The ceiling is what then bounds it: yaw fast enough and the detector
    loses the target to motion blur.
    """
    needed = yaw_rate_limit(
        bearing_error=0.0, time_constant_s=0.45,
        crossing_speed_mps=30.0, range_m=10.0,
    )
    assert needed == pytest.approx(math.radians(120.0))

    raised = yaw_rate_limit(
        bearing_error=0.0, time_constant_s=0.45, ceiling_deg=150.0,
        crossing_speed_mps=30.0, range_m=10.0,
    )
    assert raised > needed


def test_a_faster_crossing_target_is_turned_after_harder():
    """Speed is one of the three things that make a turn urgent."""
    slow = yaw_rate_limit(
        bearing_error=0.1, time_constant_s=0.45,
        crossing_speed_mps=2.0, range_m=40.0,
    )
    fast = yaw_rate_limit(
        bearing_error=0.1, time_constant_s=0.45,
        crossing_speed_mps=12.0, range_m=40.0,
    )
    assert fast > slow


def test_a_closer_target_is_turned_after_harder():
    """The same lateral motion sweeps frame faster the nearer the target.

    A correction that is unhurried at thirty metres is already too slow at
    ten, so the bearing time constant shortens as the target closes.
    """
    far = yaw_rate_limit(
        bearing_error=0.25, time_constant_s=0.45,
        crossing_speed_mps=0.0, range_m=40.0,
    )
    near = yaw_rate_limit(
        bearing_error=0.25, time_constant_s=0.45,
        crossing_speed_mps=0.0, range_m=8.0,
    )
    assert near > far


def test_the_close_range_shortening_has_a_floor():
    """A very close target must not demand an unbounded correction.

    Range appears in the denominator, so without a floor a target a metre
    away would ask for a rate set by nothing but how small that number got.
    """
    close = yaw_rate_limit(
        bearing_error=0.25, time_constant_s=0.45,
        crossing_speed_mps=0.0, range_m=6.0,
    )
    closer = yaw_rate_limit(
        bearing_error=0.25, time_constant_s=0.45,
        crossing_speed_mps=0.0, range_m=1.0,
    )
    assert closer == pytest.approx(close)


def test_a_target_at_the_frame_edge_turns_as_hard_as_it_is_allowed():
    """The last of the frame is worth defending with everything available.

    A target about to leave frame is a track about to be lost, so the edge
    urgency is what takes a bearing error -- bounded by the lens to half a
    field of view -- all the way to the ceiling.
    """
    at_edge = yaw_rate_limit(bearing_error=0.5, time_constant_s=0.45)
    assert at_edge == pytest.approx(math.radians(120.0))


def test_a_target_near_centre_is_not_turned_at_full_rate():
    """The boost is squared so it stays out of the way until it is needed."""
    near_centre = yaw_rate_limit(bearing_error=0.05, time_constant_s=0.45)
    assert near_centre < math.radians(120.0)


def test_a_centred_target_is_not_chased_with_twitchy_corrections():
    """The floor exists so a distant, centred target is held steadily."""
    rate = yaw_rate_limit(bearing_error=0.0, time_constant_s=0.45)
    assert rate == pytest.approx(math.radians(20.0))
