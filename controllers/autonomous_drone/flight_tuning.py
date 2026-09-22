"""Canonical tuned autonomy configuration, shared by the Webots controller
(autonomous_drone.py) and the real-drone flight bridge
(adapters/dji/flight_bridge.py).

This is the single place Dronetect's tuned flight/pursuit/manual-control/
per-state-kinematic values are assembled. Neither caller should construct its
own FlightConfig()/PursuitConfig()/ManualFlightConfig()/KinematicFlightConfig()
from bare dataclass defaults -- those defaults are untuned placeholders, not
the values Dronetect actually flies with. Everything here was moved out of
autonomous_drone.py unchanged; nothing was retuned.

Import pattern: same try/except relative-import fallback used throughout
this directory (see manual_input.py). Webots loads every module in this
directory as a flat top-level name, not controllers.autonomous_drone.X, so a
plain `from .flight_logic import X` fails under that loader; falling back to
`from flight_logic import X` there resolves to the SAME module object
autonomous_drone.py itself already imports flat. When this module is instead
imported the normal, dotted way (controllers.autonomous_drone.flight_tuning,
as adapters/dji/flight_bridge.py does), the relative import succeeds and
resolves to controllers.autonomous_drone.flight_logic -- also the same
module flight_bridge.py imports directly. Either way, both callers end up
constructing FlightConfig/PursuitConfig/etc. instances of the identical
class, which is exactly what's required (see
services/operating_config_apply.py's own comment on this same hazard).
"""

from __future__ import annotations

from dataclasses import replace
import math
import os

try:
    from .flight_logic import FlightConfig, FlightState, KinematicFlightConfig, ManualFlightConfig
except ImportError:  # Webots loads this directory as top-level modules.
    from flight_logic import FlightConfig, FlightState, KinematicFlightConfig, ManualFlightConfig
try:
    from .pursuit import PursuitConfig
except ImportError:
    from pursuit import PursuitConfig

from config.shared_config import default_operating_config


def _optional_float_env(name: str) -> float | None:
    """Read a float override, treating unset and empty alike as absent."""
    raw = os.getenv(name)
    if raw is None or not raw.strip():
        return None
    return float(raw)


def build_flight_config() -> FlightConfig:
    """Assemble the flight configuration from the environment.

    Callable outside ``AutonomousDrone.__init__`` so the defaults and the
    cross-field constraints between them can be exercised without Webots.
    They are otherwise only ever built at simulator startup, where a bad
    combination surfaces as a controller that dies before it connects.
    """
    return FlightConfig(
        takeoff_altitude=float(
            os.getenv(
                "DRONTECT_TAKEOFF_ALTITUDE_M",
                str(default_operating_config.cruise_altitude_m),
            )
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
        target_ignore_timeout_s=float(os.getenv("DRONTECT_TARGET_IGNORE_TIMEOUT_S", "12")),
        target_reject_grace_s=float(os.getenv("DRONTECT_TARGET_REJECT_GRACE_S", "1.5")),
        target_projection_min_angle_rad=math.radians(float(os.getenv("DRONTECT_TARGET_PROJECTION_MIN_ANGLE_DEG", "5"))),
        target_projection_max_range_m=float(os.getenv("DRONTECT_TARGET_PROJECTION_MAX_RANGE_M", "250")),
        target_acquisition_timeout_s=float(
            os.getenv("DRONTECT_TARGET_ACQUISITION_TIMEOUT_S", "2.5")
        ),
        target_yaw_time_constant_s=float(
            os.getenv("DRONTECT_TARGET_YAW_TIME_CONSTANT_S", "0.45")
        ),
        target_yaw_edge_gain=float(
            os.getenv("DRONTECT_TARGET_YAW_EDGE_GAIN", "2.0")
        ),
        target_yaw_range_reference_m=float(
            os.getenv("DRONTECT_TARGET_YAW_RANGE_REFERENCE_M", "30.0")
        ),
        target_yaw_range_min_factor=float(
            os.getenv("DRONTECT_TARGET_YAW_RANGE_MIN_FACTOR", "0.3")
        ),
        target_lost_search_timeout_s=float(
            os.getenv("DRONTECT_TARGET_LOST_SEARCH_TIMEOUT_S", "3.0")
        ),
        target_lost_search_climb_m=float(
            os.getenv("DRONTECT_TARGET_LOST_SEARCH_CLIMB_M", "1.0")
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
            float(os.getenv("DRONTECT_TARGET_MAX_YAW_RATE_DEG_S", "120.0"))
        ),
        target_search_yaw_rate_rad_s=math.radians(
            float(os.getenv("DRONTECT_TARGET_SEARCH_YAW_RATE_DEG_S", "25.0"))
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
        target_standoff_distance_m=float(
            os.getenv("DRONTECT_TARGET_STANDOFF_DISTANCE_M", "2.0")
        ),
        target_reference_span_m=float(
            os.getenv("DRONTECT_TARGET_REFERENCE_SPAN_M", "0.45")
        ),
        target_reference_distance_m=float(
            os.getenv("DRONTECT_TARGET_REFERENCE_DISTANCE_M", "2.0")
        ),
        # Unset means derive it from the span and the real lens. Set it only
        # from a measured run -- DRONTECT_TARGET_RANGE_CALIBRATION prints the
        # value to use.
        target_reference_area_ratio=_optional_float_env(
            "DRONTECT_TARGET_REFERENCE_AREA_RATIO"
        ),
        target_altitude_target_band_m=float(
            os.getenv("DRONTECT_TARGET_ALTITUDE_TARGET_BAND_M", "0.3")
        ),
        target_altitude_gain_per_s=float(
            os.getenv("DRONTECT_TARGET_ALTITUDE_GAIN_PER_S", "1.5")
        ),
        target_max_vertical_speed_mps=float(
            os.getenv("DRONTECT_TARGET_MAX_VERTICAL_SPEED_MPS", "2.0")
        ),
        target_range_estimate_max_m=float(
            os.getenv("DRONTECT_TARGET_RANGE_ESTIMATE_MAX_M", "100.0")
        ),
        target_collision_avoidance_step_m=float(
            os.getenv("DRONTECT_TARGET_COLLISION_AVOIDANCE_STEP_M", "3.0")
        ),
        target_min_yaw_rate_rad_s=math.radians(
            float(os.getenv("DRONTECT_TARGET_MIN_YAW_RATE_DEG_S", "20.0"))
        ),
        target_takeoff_pursuit_clearance_m=float(
            os.getenv("DRONTECT_TARGET_TAKEOFF_PURSUIT_CLEARANCE_M", "2.0")
        ),
        target_takeoff_pursuit_ascent_mps=float(
            os.getenv("DRONTECT_TARGET_TAKEOFF_PURSUIT_ASCENT_MPS", "2.0")
        ),
    )


def build_manual_flight_config() -> ManualFlightConfig:
    """Assemble the manual-flight limits from the environment."""
    return ManualFlightConfig(
        max_horizontal_speed_mps=float(
            os.getenv("DRONTECT_MANUAL_MAX_SPEED_MPS", "6.0")
        ),
        max_vertical_speed_mps=float(
            os.getenv("DRONTECT_MANUAL_MAX_VERTICAL_SPEED_MPS", "2.0")
        ),
        max_yaw_rate_rad_s=math.radians(
            float(os.getenv("DRONTECT_MANUAL_MAX_YAW_RATE_DEG_S", "60.0"))
        ),
        min_altitude_m=float(os.getenv("DRONTECT_MANUAL_MIN_ALTITUDE_M", "2.0")),
        command_timeout_s=float(
            os.getenv("DRONTECT_MANUAL_COMMAND_TIMEOUT_SECONDS", "1.0")
        ),
    )


def build_pursuit_config(*, geofence_radius_m: float) -> PursuitConfig:
    """Assemble the pursuit configuration from the environment.

    Does not set horizontal_fov_rad -- that is a camera-hardware property
    (Webots' simulated camera vs. the Mini 4 Pro's real lens), not tuning,
    and is applied by each caller separately after construction. See
    autonomous_drone.py's __init__ (`self.camera.getFov()`) and
    DjiFlightBridge's `camera_horizontal_fov_rad` constructor parameter.
    """
    return PursuitConfig(
        stable_duration_s=float(os.getenv("DRONTECT_PURSUIT_STABLE_SECONDS", "0.8")),
        observation_timeout_s=float(os.getenv("DRONTECT_PURSUIT_OBSERVATION_TIMEOUT_SECONDS", "1.5")),
        # A stern chase needs a real margin over the target, not parity: at
        # equal top speeds the gap can only ever hold or grow, so the aircraft
        # never gets onto the six however well the station law is tuned. The
        # simulated traffic drones cruise at 6.0-6.6 m/s.
        max_speed_mps=float(os.getenv("DRONTECT_PURSUIT_MAX_SPEED_MPS", "10.0")),
        acceleration_mps2=float(os.getenv("DRONTECT_PURSUIT_ACCELERATION_MPS2", "5.0")),
        smoothing_time_s=float(os.getenv("DRONTECT_PURSUIT_SMOOTHING_SECONDS", "0.2")),
        standoff_distance_m=float(os.getenv("DRONTECT_PURSUIT_STANDOFF_M", "5.0")),
        station_keeping_gain_per_s=float(
            os.getenv("DRONTECT_PURSUIT_STATION_KEEPING_GAIN", "1.6")
        ),
        altitude_offset_m=float(os.getenv("DRONTECT_PURSUIT_ALTITUDE_OFFSET_M", "3.0")),
        entry_time_to_contact_s=float(
            os.getenv("DRONTECT_PURSUIT_DODGE_ENTRY_SECONDS", "1.8")
        ),
        exit_time_to_contact_s=float(
            os.getenv("DRONTECT_PURSUIT_DODGE_EXIT_SECONDS", "3.0")
        ),
        geofence_radius_m=geofence_radius_m,
    )


# -- per-flight-state kinematic tuning ---------------------------------------
# Moved from AutonomousDrone's class-level constants
# (KINEMATIC_FLIGHT_CONFIG/SEARCH_KINEMATIC_FLIGHT_CONFIG/
# MANUAL_KINEMATIC_FLIGHT_CONFIG) and _kinematic_config_for_state's selection
# logic, verbatim -- these are hardcoded (not environment-driven) in the
# original, so they are hardcoded here too.

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
# Manual flight integrates the operator's stick directly, so the only limit
# that still applies from this profile is the yaw slew rate.
MANUAL_KINEMATIC_FLIGHT_CONFIG = KinematicFlightConfig(
    max_horizontal_speed_mps=6.0,
    horizontal_acceleration_mps2=3.0,
    max_ascent_speed_mps=2.0,
    max_descent_speed_mps=2.0,
    max_yaw_rate_rad_s=math.radians(60.0),
)


def target_tracking_kinematic_config(target_max_yaw_rate_rad_s: float) -> KinematicFlightConfig:
    """Mirrors AutonomousDrone.__init__'s self._target_tracking_flight_config
    construction: SEARCH's speed/acceleration axes, with
    FlightConfig.target_max_yaw_rate_rad_s as its yaw rate (itself replaced
    per-tick by kinematic_config_for_state's target_yaw_rate_limit -- this is
    only the base a live yaw-rate override is applied on top of)."""
    return KinematicFlightConfig(
        max_horizontal_speed_mps=SEARCH_KINEMATIC_FLIGHT_CONFIG.max_horizontal_speed_mps,
        horizontal_acceleration_mps2=SEARCH_KINEMATIC_FLIGHT_CONFIG.horizontal_acceleration_mps2,
        max_ascent_speed_mps=SEARCH_KINEMATIC_FLIGHT_CONFIG.max_ascent_speed_mps,
        max_descent_speed_mps=SEARCH_KINEMATIC_FLIGHT_CONFIG.max_descent_speed_mps,
        max_yaw_rate_rad_s=target_max_yaw_rate_rad_s,
    )


def kinematic_config_for_state(
    state: FlightState,
    *,
    target_tracking_flight_config: KinematicFlightConfig,
    target_yaw_rate_limit: float,
) -> KinematicFlightConfig:
    """Mirrors AutonomousDrone._kinematic_config_for_state exactly.

    `target_tracking_flight_config` is normally
    target_tracking_kinematic_config(...)'s result, built once at startup;
    `target_yaw_rate_limit` is FlightLogic.target_yaw_rate_limit, live and
    recomputed every pursuit tick from the target's speed and range.
    """
    if state is FlightState.MANUAL:
        return MANUAL_KINEMATIC_FLIGHT_CONFIG
    if state is FlightState.TARGET_RETURN:
        return replace(SEARCH_KINEMATIC_FLIGHT_CONFIG, yaw_hold_horizontal_distance_m=0.0)
    if state in {
        FlightState.TARGET_ACQUISITION,
        FlightState.TARGET_FOLLOW,
        FlightState.TARGET_PURSUIT,
    }:
        return replace(target_tracking_flight_config, max_yaw_rate_rad_s=target_yaw_rate_limit)
    if state in {
        FlightState.ROI_PATROL,
        FlightState.PATROLLING,
        FlightState.REPOSITIONING,
    }:
        return SEARCH_KINEMATIC_FLIGHT_CONFIG
    return KINEMATIC_FLIGHT_CONFIG
