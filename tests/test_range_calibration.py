"""How far away the aircraft believes a target is, and what sets that belief."""

import math
import random

import pytest

from controllers.autonomous_drone.flight_logic import FlightConfig, FlightLogic
from controllers.autonomous_drone.pursuit import (
    PursuitConfig,
    calibrate_reference_area_ratio,
    estimate_target_altitude,
    reference_area_ratio_from_span,
)


FOV = 0.7854
FRAME = (960, 540)


def make_logic(**overrides):
    logic = FlightLogic(
        config=FlightConfig(**overrides),
        rng=random.Random(7),
        pursuit_config=PursuitConfig(horizontal_fov_rad=FOV),
    )
    logic.initialize_home((0.0, 0.0, 0.1))
    return logic


def box_for(span_m, distance_m, frame_width, frame_height):
    """A centred box the size the target really projects to at that range."""
    width_fraction = (span_m / distance_m) / (2.0 * math.tan(FOV / 2.0))
    half_w = width_fraction * frame_width / 2.0
    half_h = width_fraction * frame_width / 2.0
    cx, cy = frame_width / 2.0, frame_height / 2.0
    return [cx - half_w, cy - half_h, cx + half_w, cy + half_h]


# --- deriving the reference from real geometry -------------------------------


def test_the_reference_ratio_matches_what_the_lens_actually_sees():
    """The whole point is that this number stops being a guess."""
    ratio = reference_area_ratio_from_span(
        span_m=0.45, distance_m=2.0, horizontal_fov_rad=FOV,
        frame_width=FRAME[0], frame_height=FRAME[1],
    )
    box = box_for(0.45, 2.0, *FRAME)
    observed = (box[2] - box[0]) * (box[3] - box[1]) / (FRAME[0] * FRAME[1])
    assert ratio == pytest.approx(observed, rel=1e-9)


def test_the_reference_ratio_follows_the_lens():
    """A narrower lens magnifies, so the same target fills more of the frame."""
    wide = reference_area_ratio_from_span(
        span_m=0.45, distance_m=2.0, horizontal_fov_rad=1.2,
        frame_width=FRAME[0], frame_height=FRAME[1],
    )
    narrow = reference_area_ratio_from_span(
        span_m=0.45, distance_m=2.0, horizontal_fov_rad=0.4,
        frame_width=FRAME[0], frame_height=FRAME[1],
    )
    assert narrow > wide


@pytest.mark.parametrize("kwargs", [
    {"span_m": 0.0}, {"distance_m": 0.0}, {"horizontal_fov_rad": 0.0},
    {"frame_width": 0}, {"frame_height": 0}, {"span_m": float("nan")},
])
def test_degenerate_geometry_yields_no_reference(kwargs):
    base = dict(
        span_m=0.45, distance_m=2.0, horizontal_fov_rad=FOV,
        frame_width=FRAME[0], frame_height=FRAME[1],
    )
    assert reference_area_ratio_from_span(**{**base, **kwargs}) is None


# --- measuring it off a run --------------------------------------------------


def test_one_box_at_a_known_range_fixes_the_calibration():
    """A single honest observation is enough, because area falls off as 1/d^2."""
    box = box_for(0.45, 37.0, *FRAME)
    measured = (box[2] - box[0]) * (box[3] - box[1]) / (FRAME[0] * FRAME[1])

    suggested = calibrate_reference_area_ratio(
        measured_area_ratio=measured, measured_range_m=37.0, reference_distance_m=2.0,
    )

    truth = reference_area_ratio_from_span(
        span_m=0.45, distance_m=2.0, horizontal_fov_rad=FOV,
        frame_width=FRAME[0], frame_height=FRAME[1],
    )
    assert suggested == pytest.approx(truth, rel=1e-9)


def test_a_calibrated_reference_reports_the_true_range_back():
    """Round trip: calibrate from one pass, then measure a different range."""
    reference = reference_area_ratio_from_span(
        span_m=0.45, distance_m=2.0, horizontal_fov_rad=FOV,
        frame_width=FRAME[0], frame_height=FRAME[1],
    )
    estimate = estimate_target_altitude(
        box=box_for(0.45, 60.0, *FRAME),
        frame_width=FRAME[0], frame_height=FRAME[1], camera_pitch=0.0,
        horizontal_fov_rad=FOV, reference_distance_m=2.0,
        reference_area_ratio=reference, max_range_m=100.0,
    )
    assert estimate is not None
    assert estimate.slant_range_m == pytest.approx(60.0, rel=1e-6)


@pytest.mark.parametrize("kwargs", [
    {"measured_area_ratio": 0.0}, {"measured_range_m": 0.0},
    {"reference_distance_m": 0.0}, {"measured_range_m": float("inf")},
])
def test_a_useless_observation_calibrates_nothing(kwargs):
    base = dict(
        measured_area_ratio=0.01, measured_range_m=37.0, reference_distance_m=2.0,
    )
    assert calibrate_reference_area_ratio(**{**base, **kwargs}) is None


# --- what the aircraft actually uses -----------------------------------------


def test_the_standoff_setpoint_no_longer_doubles_as_the_calibration():
    """These were one pair, and that is what made every range wrong.

    The setpoint says when to stop closing. The calibration says how big the
    target is. Tying them together asserts a target size nobody measured, so
    moving one must not move the other.
    """
    logic = make_logic(
        target_standoff_box_area_ratio=0.025,
        target_standoff_distance_m=2.0,
        target_reference_span_m=0.45,
        target_reference_distance_m=2.0,
    )
    derived = logic.reference_area_ratio(*FRAME)
    assert derived != pytest.approx(0.025)

    tighter = make_logic(
        target_standoff_box_area_ratio=0.2,
        target_standoff_distance_m=2.0,
        target_reference_span_m=0.45,
        target_reference_distance_m=2.0,
    )
    assert tighter.reference_area_ratio(*FRAME) == pytest.approx(derived)


def test_a_measured_ratio_overrides_the_derived_one():
    """The detector's boxes are looser than the airframe, so measurement wins."""
    logic = make_logic(target_reference_area_ratio=0.077)
    assert logic.reference_area_ratio(*FRAME) == pytest.approx(0.077)


def test_the_aircraft_reports_the_true_range_to_a_target_it_can_see():
    """The regression this whole change exists for.

    The old calibration described a 0.19 m target, so a Mavic read as 2.4x
    closer than it was -- harmless while pursuit was image-space, wrong the
    moment it started steering to world coordinates and gating dodges on
    absolute range.
    """
    logic = make_logic(target_reference_span_m=0.45, target_reference_distance_m=2.0)
    estimate = estimate_target_altitude(
        box=box_for(0.45, 48.0, *FRAME),
        frame_width=FRAME[0], frame_height=FRAME[1], camera_pitch=0.0,
        horizontal_fov_rad=FOV, reference_distance_m=2.0,
        reference_area_ratio=logic.reference_area_ratio(*FRAME),
        max_range_m=100.0,
    )
    assert estimate is not None
    assert estimate.slant_range_m == pytest.approx(48.0, rel=1e-6)


def test_an_unusable_frame_falls_back_instead_of_losing_range_entirely():
    """A degenerate frame must not silently disable range estimation."""
    logic = make_logic(target_standoff_box_area_ratio=0.025)
    assert logic.reference_area_ratio(0, 0) == pytest.approx(0.025)


@pytest.mark.parametrize("ratio", [0.0, -0.1, 1.5])
def test_an_impossible_measured_ratio_is_refused(ratio):
    with pytest.raises(ValueError):
        FlightConfig(target_reference_area_ratio=ratio)
