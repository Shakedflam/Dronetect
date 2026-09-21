"""Webots-independent autonomous flight state machine."""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
from enum import Enum
import math
import random
from typing import Any, Callable, Protocol

try:
    from .target_decisions import (
        TargetDecisions, ground_projection, range_estimated_projection,
    )
    from .pursuit import (
        PursuitConfig, PursuitController, TargetAltitudeEstimate,
        estimate_target_altitude,
    )
    from .roi_patrol import LawnMowerRoiPatrolPlanner, RoiPatrolPlan
except ImportError:  # Webots loads this directory as top-level modules.
    from target_decisions import (
        TargetDecisions, ground_projection, range_estimated_projection,
    )
    from pursuit import (
        PursuitConfig, PursuitController, TargetAltitudeEstimate,
        estimate_target_altitude,
    )
    from roi_patrol import LawnMowerRoiPatrolPlanner, RoiPatrolPlan


Position = tuple[float, float, float]


class RandomSource(Protocol):
    def uniform(self, a: float, b: float) -> float: ...


class FlightState(Enum):
    IDLE = "IDLE"
    TAKING_OFF = "TAKING_OFF"
    HOLDING = "HOLDING"
    TRANSITING_TO_ROI = "TRANSITING_TO_ROI"
    ROI_PATROL = "ROI_PATROL"
    PATROLLING = "PATROLLING"
    REPOSITIONING = "REPOSITIONING"
    TARGET_ACQUISITION = "TARGET_ACQUISITION"
    TARGET_FOLLOW = "TARGET_FOLLOW"
    TARGET_PURSUIT = "TARGET_PURSUIT"
    TARGET_RETURN = "TARGET_RETURN"
    RETURNING_HOME = "RETURNING_HOME"
    LANDING = "LANDING"
    LANDED = "LANDED"


@dataclass(frozen=True, slots=True)
class FlightConfig:
    idle_duration: float = 1.0
    takeoff_altitude: float = 2.5
    x_bounds: tuple[float, float] = (-8.0, 8.0)
    y_bounds: tuple[float, float] = (-8.0, 8.0)
    z_bounds: tuple[float, float] = (2.5, 5.0)
    # What we count as arrived at a target, for both horizontal and vertical directions, how far away from point of interest.
    horizontal_tolerance: float = 0.5
    vertical_tolerance: float = 0.25
    # Returning to an interruption checkpoint deliberately skips the dead zone
    # above so the aircraft flies the whole way back instead of stopping half a
    # metre short. It still cannot demand an exact match: the controller
    # commands a pose and reads the next one back off a stepped physics body,
    # so the reported position always trails the commanded one slightly, and a
    # gate tighter than that residual never opens at all.
    checkpoint_tolerance: float = 0.05
    landing_rate: float = 0.35
    landed_height: float = 0.12
    roi_arrival_tolerance: float = 10.0
    target_min_confidence: float = 0.43
    target_confirmation_hits: int = 2
    target_observation_s: float = 0.85
    target_observation_min_samples: int = 3
    target_stationary_center_ratio: float = 0.02
    target_stationary_observer_m: float = 0.25
    target_stationary_angle_rad: float = math.radians(2.0)
    target_low_confidence: float = 0.60
    target_confidence_range: float = 0.20
    target_ignore_radius_m: float = 5.0
    target_ignore_timeout_s: float = 6.0
    target_reject_grace_s: float = 1.7
    target_projection_min_angle_rad: float = math.radians(5.0)
    target_projection_max_range_m: float = 250.0
    # Must exceed the expected interval between vision frames so a tentative
    # track can receive its confirming hit while the aircraft remains stopped.
    target_acquisition_timeout_s: float = 2.5
    target_lost_search_timeout_s: float = 2.0
    target_yaw_gain: float = 0.35
    target_yaw_prediction_horizon_s: float = 0.5
    target_yaw_deadband_ratio: float = 0.04
    target_velocity_smoothing: float = 0.35
    target_max_yaw_rate_rad_s: float = math.radians(18.0)
    target_gimbal_pitch_gain: float = 0.25
    target_center_tolerance_x: float = 0.08
    target_center_tolerance_y: float = 0.12
    target_search_yaw_rate_rad_s: float = math.radians(10.0)
    target_follow_step_m: float = 6.0
    target_lateral_motion_gain_m: float = 4.0
    target_standoff_box_area_ratio: float = 0.025
    target_standoff_distance_m: float = 2.0
    target_altitude_target_band_m: float = 0.5
    target_altitude_gain_per_s: float = 1.0
    target_max_vertical_speed_mps: float = 2.0
    target_range_estimate_max_m: float = 100.0
    target_standoff_deadband_ratio: float = 0.005
    target_approach_growth_ratio_per_s: float = 0.04
    target_collision_avoidance_step_m: float = 3.0
    # A takeoff interrupted by a confirmed target may begin cautious pursuit
    # above the hard floor instead of climbing to the normal cruise altitude.
    target_takeoff_pursuit_clearance_m: float = 2.0
    target_takeoff_pursuit_ascent_mps: float = 2.0

    def __post_init__(self):
        for name in ("target_observation_s", "target_stationary_center_ratio",
                     "target_stationary_observer_m", "target_stationary_angle_rad",
                     "target_low_confidence", "target_confidence_range", "target_ignore_radius_m",
                     "target_ignore_timeout_s", "target_reject_grace_s",
                     "target_projection_min_angle_rad", "target_projection_max_range_m",
                     "target_standoff_distance_m", "target_altitude_target_band_m",
                     "target_altitude_gain_per_s", "target_max_vertical_speed_mps",
                     "target_range_estimate_max_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.target_observation_min_samples < 3:
            raise ValueError("target_observation_min_samples must be at least 3")
        if self.target_low_confidence > 1 or self.target_confidence_range > 1:
            raise ValueError("Confidence thresholds must be at most 1")
        for name in (
            "target_standoff_distance_m",
            "target_altitude_gain_per_s",
            "target_max_vertical_speed_mps",
            "target_range_estimate_max_m",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True, slots=True)
class KinematicFlightConfig:
    max_horizontal_speed_mps: float = 14.0
    horizontal_acceleration_mps2: float = 6.0
    max_ascent_speed_mps: float = 5.0
    max_descent_speed_mps: float = 3.0
    max_yaw_rate_rad_s: float = math.radians(150.0)
    yaw_hold_horizontal_distance_m: float = 0.25


@dataclass(frozen=True, slots=True)
class KinematicFlightStep:
    position: Position
    yaw: float
    horizontal_speed_mps: float
    speed_mps: float
    vertical_speed_mps: float


class FlightLogic:
    """Select targets and flight states without controlling motors directly."""

    def __init__(
        self,
        config: FlightConfig | None = None,
        rng: RandomSource | None = None,
        roi_patrol_planner: RoiPatrolPlan | None = None,
        patrol_waypoint_provider: Callable[[Position, bool], Position] | None = None,
        pursuit_config: PursuitConfig | None = None,
    ) -> None:
        self.config = config or FlightConfig()
        self.target_decisions = TargetDecisions(self.config)
        self.rng = rng or random.Random()
        self.roi_patrol_planner = roi_patrol_planner or LawnMowerRoiPatrolPlanner()
        self.patrol_waypoint_provider = patrol_waypoint_provider
        self.state = FlightState.IDLE
        self.home: Position | None = None
        self.target: Position | None = None
        self._idle_elapsed = 0.0
        self._landing_target_altitude = self.config.takeoff_altitude
        self._takeoff_anchor: Position | None = None
        self._queued_waypoint: Position | None = None
        self._queued_state: FlightState | None = None
        self._queued_mission_stale = False
        self._roi_center: tuple[float, float] | None = None
        self._roi_radius = 0.0
        self._roi_first_waypoint: Position | None = None
        self._elapsed_time = 0.0
        self._patrol_resume_target: Position | None = None
        self._patrol_resume_checkpoint: Position | None = None
        self._target_resume_state: FlightState | None = None
        self._tracked_target_id: str | None = None
        self._followed_track_ids: set[str] = set()
        self._target_hits = 0
        self._target_confidence: float | None = None
        self._target_last_seen_at = 0.0
        self._target_last_center = (0.5, 0.5)
        self._target_velocity = (0.0, 0.0)
        self._target_area_ratio = 0.0
        self._target_previous_area_ratio: float | None = None
        self._target_area_growth_per_s = 0.0
        self._target_visible = False
        self._target_confirmed = False
        self._target_yaw: float | None = None
        self._target_camera_pitch: float | None = None
        self._target_last_yaw = 0.0
        self._target_altitude_estimate: TargetAltitudeEstimate | None = None
        self.pursuit = PursuitController(pursuit_config or PursuitConfig())
        self._stable_since: float | None = None
        self._pursuit_usable = False
        self._pursuit_started = False
        self._pursuit_growth = 0.0
        self._pursuit_sample: tuple[float, float, float] | None = None
        self._pursuit_image_motion = (0.0, 0.0)
        self._takeoff_pursuit_vertical_velocity = 0.0
        self._target_return_waypoints: deque[Position] = deque()
        self._target_return_ready = True
        self._reposition_waypoints: deque[Position] = deque()
        self._reposition_resume_state: FlightState | None = None

    def initialize_home(self, position: Position) -> None:
        """Capture home once from the first valid simulated GPS reading."""
        if self.home is None:
            self.home = (
                float(position[0]),
                float(position[1]),
                float(position[2]),
            )
            self.target = self.home

    def update(self, position: Position, dt: float, *, yaw: float | None = None,
               camera_pitch: float | None = None) -> None:
        """Advance state and target selection by one simulation timestep."""
        if self.home is None:
            raise RuntimeError("Home position must be initialized before update().")
        dt = max(0.0, dt)
        self._elapsed_time += dt
        self.target_decisions.prune(self._elapsed_time)
        if (
            self.is_target_tracking
            and yaw is not None and camera_pitch is not None
        ):
            # The false-positive evaluation keeps accumulating evidence in
            # parallel with an already-started follow/pursuit.
            self.target_decisions.record_pose(position, yaw, camera_pitch)

        if self.state is FlightState.IDLE:
            self.target = self.home

        elif self.state is FlightState.TAKING_OFF:
            self.target = self._takeoff_target()
            if self._has_completed_takeoff(position, self.target):
                self.state = self._queued_state or FlightState.HOLDING
                queued_waypoint = self._queued_waypoint
                needs_refresh = self._queued_mission_stale
                self._queued_waypoint = None
                self._queued_state = None
                self._queued_mission_stale = False
                if needs_refresh and self.state in {FlightState.TRANSITING_TO_ROI, FlightState.PATROLLING}:
                    # The waypoint queued at request_roi()/request_patrol() time
                    # (and, for ROI, the clipped geometry behind it) is stale --
                    # the operating config changed during this climb. Recompute
                    # fresh from the actual arrival position and current config.
                    self.target = self._fresh_queued_mission_target(self.state, position)
                else:
                    self.target = queued_waypoint or self.target
                if (
                    self.state is FlightState.TRANSITING_TO_ROI
                    and self._is_inside_roi(position)
                ):
                    self._start_roi_patrol(position)

        elif self.state is FlightState.HOLDING:
            # Mission navigation is not implemented yet. Hold the position
            # reached by the commanded takeoff instead of choosing a waypoint.
            if self.target is None:
                self.target = position

        elif self.state is FlightState.TRANSITING_TO_ROI:
            if self._is_inside_roi(position):
                self._start_roi_patrol(position)

        elif self.state is FlightState.ROI_PATROL:
            if not self._is_inside_roi(position):
                # Recover to the nearest point in the clipped ROI/home area.
                self.target = self._roi_center_target()
            elif self.target is None or self._has_reached(position, self.target):
                self.target = self.roi_patrol_planner.next_waypoint()

        elif self.state is FlightState.PATROLLING:
            if self.target is None or self._has_reached(position, self.target):
                self.target = self._next_patrol_waypoint(position, completed=True)

        elif self.state is FlightState.REPOSITIONING:
            if self.target is not None and self._has_reached(position, self.target):
                if self._reposition_waypoints:
                    self.target = self._reposition_waypoints.popleft()
                elif self._reposition_resume_state is FlightState.ROI_PATROL:
                    self._reposition_resume_state = None
                    try:
                        assert self._roi_center is not None
                        self._roi_first_waypoint = self.roi_patrol_planner.reset(
                            self._roi_center, self._roi_radius,
                            self.config.takeoff_altitude, position,
                        )
                        self._start_roi_patrol(position)
                    except ValueError as exc:
                        # The ROI no longer overlaps the (now smaller) flight
                        # radius at all -- degrade to normal area patrol
                        # rather than leaving the drone stuck here.
                        print(f"ROI no longer reachable after reconfiguration, falling back to patrol: {exc}")
                        self.state = FlightState.PATROLLING
                        self.target = self._next_patrol_waypoint(position, completed=False)
                else:
                    self._reposition_resume_state = None
                    self.state = FlightState.PATROLLING
                    self.target = self._next_patrol_waypoint(position, completed=False)

        elif self.state is FlightState.TARGET_ACQUISITION:
            self.target = self._target_acquisition_target(position)
            # Acceptance/rejection is evaluated only on fresh detector matches;
            # a controller tick cannot promote a cached confirmed box.
            if self._elapsed_time - self._target_last_seen_at > self.config.target_acquisition_timeout_s:
                self.complete_target_follow(position)

        elif self.state in {FlightState.TARGET_FOLLOW, FlightState.TARGET_PURSUIT}:
            if (
                self._target_visible
                and self._elapsed_time - self._target_last_seen_at
                > self.config.target_acquisition_timeout_s
            ):
                self._target_visible = False
            if self._target_visible and self._elapsed_time - self._target_last_seen_at > self.pursuit.config.observation_timeout_s:
                self._target_visible = False
                self._pursuit_usable = False
            if self._target_visible:
                self.target = self._target_tracking_hold_target(position)
                # Approach begins as soon as the tracker is usable and flight
                # safety allows movement -- no additional stability wait.
                if (self.state is FlightState.TARGET_FOLLOW
                        and self._pursuit_usable
                        and self._pursuit_altitude_is_safe(position)):
                    self.state = FlightState.TARGET_PURSUIT
                    self._pursuit_started = True
                if self.state is FlightState.TARGET_PURSUIT and self._pursuit_usable:
                    self.pursuit.update(
                        position=position, home=self.home, yaw=self._target_last_yaw,
                        image_velocity=self._pursuit_image_motion, area=self._target_area_ratio,
                        growth=self._pursuit_growth,
                        desired_area=self.config.target_standoff_box_area_ratio,
                        area_deadzone=self.config.target_standoff_deadband_ratio,
                        collision_growth=self.config.target_approach_growth_ratio_per_s,
                        collision_detected=(self._target_area_growth_per_s >= self.config.target_approach_growth_ratio_per_s),
                        collision_side=(-1.0 if self._target_last_center[0] >= 0.5 else 1.0)
                            * self.config.target_collision_avoidance_step_m,
                        lateral_gain=self.config.target_lateral_motion_gain_m,
                        height_error_m=(
                            self._target_altitude_estimate.height_error_m
                            if self._target_altitude_estimate is not None else None
                        ),
                        altitude_deadband_m=self.config.target_altitude_target_band_m,
                        altitude_gain_per_s=self.config.target_altitude_gain_per_s,
                        max_vertical_speed_mps=self.config.target_max_vertical_speed_mps,
                        dt=dt,
                    )
                    self._continue_takeoff_during_pursuit(position, dt)
                else:
                    self.pursuit.stop()
                    self._takeoff_pursuit_vertical_velocity = 0.0
            else:
                self.pursuit.stop()
                self._takeoff_pursuit_vertical_velocity = 0.0
                self._stable_since = None
                self._pursuit_sample = None
                self._pursuit_growth = 0.0
                self._pursuit_image_motion = (0.0, 0.0)
                self.target = self._target_tracking_hold_target(position)
                lost_for = self._elapsed_time - self._target_last_seen_at
                predicted_x = self._target_last_center[0] + self._target_velocity[0] * lost_for
                image_error = predicted_x - 0.5
                search_direction = math.copysign(1.0, image_error) if abs(image_error) > 1e-6 else 1.0
                self._target_yaw = normalized_angle(
                    (self._target_yaw or self._target_last_yaw)
                    - self._target_velocity[0] * self.config.target_yaw_gain * dt
                    - search_direction * self.config.target_search_yaw_rate_rad_s * dt
                )
                if lost_for > self.config.target_lost_search_timeout_s:
                    self.complete_target_follow(position)

        elif self.state is FlightState.TARGET_RETURN:
            if not self._target_return_ready:
                self.target = position
            elif (
                self.target is not None
                and math.dist(position, self.target) <= self.config.checkpoint_tolerance
            ):
                if self._target_return_waypoints:
                    self.target = self._target_return_waypoints.popleft()
                else:
                    self._resume_patrol_after_target(position)

        elif self.state is FlightState.RETURNING_HOME:
            self.target = self._home_at_cruise_altitude()
            if self._has_reached(position, self.target):
                self.state = FlightState.LANDING
                self._landing_target_altitude = self.config.takeoff_altitude
                self.target = self._landing_target()

        elif self.state is FlightState.LANDING:
            if position[2] <= self.home[2] + self.config.landed_height:
                self.state = FlightState.LANDED
                self.target = self.home
            else:
                floor_altitude = self.home[2]
                self._landing_target_altitude = max(
                    floor_altitude,
                    self._landing_target_altitude
                    - self.config.landing_rate * max(0.0, dt),
                )
                self.target = self._landing_target()

        elif self.state is FlightState.LANDED:
            self.target = self.home

    def request_return_home(self) -> bool:
        """Replace an active flight target with home at a safe altitude."""
        if self.home is None or self.state not in {
            FlightState.TAKING_OFF,
            FlightState.HOLDING,
            FlightState.TRANSITING_TO_ROI,
            FlightState.ROI_PATROL,
            FlightState.PATROLLING,
            FlightState.REPOSITIONING,
            FlightState.TARGET_ACQUISITION,
            FlightState.TARGET_FOLLOW,
            FlightState.TARGET_PURSUIT,
            FlightState.TARGET_RETURN,
        }:
            return False

        self.state = FlightState.RETURNING_HOME
        self._queued_waypoint = None
        self._queued_state = None
        self._roi_center = None
        self._roi_radius = 0.0
        self.roi_patrol_planner.clear()
        self._reposition_waypoints.clear()
        self._reposition_resume_state = None
        self._clear_target_tracking()
        self.target = self._home_at_cruise_altitude()
        return True

    def request_mission_takeoff(self, position: Position) -> None:
        """Take off for a mission, or hold immediately when already airborne."""
        if self.home is None:
            raise RuntimeError("Home position must be initialized before commands.")

        self._clear_target_tracking()
        self._queued_waypoint = None
        self._queued_state = None
        self._roi_center = None
        self._roi_radius = 0.0
        self.roi_patrol_planner.clear()
        self._reposition_waypoints.clear()
        self._reposition_resume_state = None
        safe_altitude = self.config.takeoff_altitude - self.config.vertical_tolerance
        if position[2] < safe_altitude:
            self._start_takeoff(position)
        else:
            self.state = FlightState.HOLDING
            self.target = position

    def request_patrol(self, position: Position) -> Position:
        """Start continuous normal patrol, taking off first when necessary."""
        if self.home is None:
            raise RuntimeError("Home position must be initialized before commands.")

        self._roi_center = None
        self._roi_radius = 0.0
        self.roi_patrol_planner.clear()
        self._reposition_waypoints.clear()
        self._reposition_resume_state = None
        self._clear_target_tracking()
        waypoint = self._next_patrol_waypoint(position, completed=False)
        safe_altitude = self.config.takeoff_altitude - self.config.vertical_tolerance
        if position[2] < safe_altitude:
            self._queued_waypoint = waypoint
            self._queued_state = FlightState.PATROLLING
            self._start_takeoff(position)
        else:
            self._queued_waypoint = None
            self._queued_state = None
            self.state = FlightState.PATROLLING
            self.target = waypoint
        return waypoint

    def request_roi(
        self,
        position: Position,
        center: tuple[float, float],
        radius_m: float,
    ) -> None:
        """Take off if needed, transit into the ROI, then patrol continuously."""
        if self.home is None:
            raise RuntimeError("Home position must be initialized before commands.")

        self._roi_center = center
        self._roi_radius = radius_m
        self._reposition_waypoints.clear()
        self._reposition_resume_state = None
        self._clear_target_tracking()
        self._roi_first_waypoint = self.roi_patrol_planner.reset(
            center,
            radius_m,
            self.config.takeoff_altitude,
            position,
        )
        roi_target = self.roi_patrol_planner.entry_target(
            center,
            self.config.takeoff_altitude,
        )
        safe_altitude = self.config.takeoff_altitude - self.config.vertical_tolerance
        if position[2] < safe_altitude:
            self._queued_waypoint = roi_target
            self._queued_state = FlightState.TRANSITING_TO_ROI
            self._start_takeoff(position)
        else:
            self._queued_waypoint = None
            self._queued_state = None
            self.state = FlightState.TRANSITING_TO_ROI
            self.target = roi_target

    def clear_roi(self, position: Position) -> None:
        """Cancel ROI guidance without initiating any new movement."""
        self.target_decisions.memories.clear()
        was_queued_for_roi = self._queued_state is FlightState.TRANSITING_TO_ROI
        was_repositioning_for_roi = (
            self.state is FlightState.REPOSITIONING
            and self._reposition_resume_state is FlightState.ROI_PATROL
        )
        self._roi_center = None
        self._roi_radius = 0.0
        self._roi_first_waypoint = None
        self.roi_patrol_planner.clear()
        if was_queued_for_roi:
            self._queued_waypoint = None
            self._queued_state = None
        if was_repositioning_for_roi:
            self._reposition_waypoints.clear()
            self._reposition_resume_state = None
        if (
            self.state in {FlightState.TRANSITING_TO_ROI, FlightState.ROI_PATROL}
            or was_repositioning_for_roi
        ):
            self.state = FlightState.HOLDING
            self.target = position

    def mark_queued_mission_config_stale(self) -> None:
        """Flag that the operating config changed while a mission was queued
        behind an in-progress takeoff, so its precomputed waypoint/geometry
        must be recomputed fresh once that takeoff completes rather than
        trusted as-is."""
        self._queued_mission_stale = True

    def start_reposition(
        self,
        waypoints: list[Position],
        resume_state: FlightState,
    ) -> None:
        """Fly a route (already routed clear of no-fly sectors), then resume
        PATROLLING or ROI_PATROL under whatever geometry is current once
        the route completes."""
        if not waypoints:
            raise ValueError("Reposition route must contain at least one waypoint")
        if resume_state not in {FlightState.PATROLLING, FlightState.ROI_PATROL}:
            raise ValueError("Reposition resume state must be PATROLLING or ROI_PATROL")
        self.state = FlightState.REPOSITIONING
        self.target = waypoints[0]
        self._reposition_waypoints = deque(waypoints[1:])
        self._reposition_resume_state = resume_state

    def choose_random_waypoint(self) -> Position:
        """Create a waypoint within the configured safe flight volume."""
        previous = self.target
        candidate: Position = (0.0, 0.0, self.config.takeoff_altitude)
        for _ in range(10):
            candidate = (
                self.rng.uniform(*self.config.x_bounds),
                self.rng.uniform(*self.config.y_bounds),
                self.rng.uniform(*self.config.z_bounds),
            )
            if previous is None or not self._has_reached(candidate, previous):
                return candidate
        return candidate

    @property
    def reposition_resume_state(self) -> FlightState | None:
        """Which mission an in-progress reposition leg resumes into."""
        return self._reposition_resume_state

    @property
    def reposition_destination(self) -> Position | None:
        """Where an in-progress reposition leg ultimately ends.

        This is the proportional point the last operating-config change aimed
        for, so a further change mid-leg rescales *this* rather than the
        aircraft's current position -- which is only partway along the leg and
        no longer sits at any meaningful proportion of the boundary.
        """
        if self.state is not FlightState.REPOSITIONING:
            return None
        if self._reposition_waypoints:
            return self._reposition_waypoints[-1]
        return self.target

    @property
    def target_yaw(self) -> float | None:
        """Absolute yaw guidance used while acquiring or following a target."""
        return self._target_yaw

    @property
    def tracked_target_id(self) -> str | None:
        return self._tracked_target_id

    @property
    def target_camera_pitch(self) -> float | None:
        """Absolute gimbal pitch guidance for vertical image centering."""
        return self._target_camera_pitch

    @property
    def patrol_resume_checkpoint(self) -> Position | None:
        """Exact patrol position captured before any target movement begins."""
        return self._patrol_resume_checkpoint

    @property
    def pending_mission_state(self) -> FlightState | None:
        return self._queued_state

    @property
    def pending_mission_waypoint(self) -> Position | None:
        return self._queued_waypoint

    @property
    def target_return_ready(self) -> bool:
        return self._target_return_ready

    @property
    def is_target_tracking(self) -> bool:
        """Whether the drone is currently acquiring, following, or pursuing a target."""
        return self.state in {
            FlightState.TARGET_ACQUISITION,
            FlightState.TARGET_FOLLOW,
            FlightState.TARGET_PURSUIT,
        }

    @property
    def diagnostic_state(self) -> str:
        """Externally visible autonomous state without changing flight behavior."""
        if self.state in {FlightState.TARGET_FOLLOW, FlightState.TARGET_PURSUIT} and not self._target_visible:
            return "TARGET_LOST_SEARCH"
        if self.state is FlightState.PATROLLING:
            return "PATROL"
        return self.state.value

    def target_follow_diagnostics(self) -> dict[str, Any]:
        """Return a JSON-compatible snapshot for simulator validation."""
        required_hits = max(1, self.config.target_confirmation_hits)
        confirmation_hits = min(self._target_hits, required_hits)
        velocity_x, velocity_y = self._target_velocity
        velocity_magnitude = math.hypot(velocity_x, velocity_y)
        velocity_direction_deg = (
            math.degrees(math.atan2(velocity_y, velocity_x))
            if velocity_magnitude > 1e-9
            else None
        )
        box_scale = math.sqrt(max(0.0, self._target_area_ratio))
        standoff_error = (
            self.config.target_standoff_box_area_ratio - self._target_area_ratio
        )
        target_lost_duration_s = (
            max(0.0, self._elapsed_time - self._target_last_seen_at)
            if self._tracked_target_id is not None and not self._target_visible
            else 0.0
        )
        resume_state = self._target_resume_state
        resume_state_name = (
            "PATROL"
            if resume_state is FlightState.PATROLLING
            else resume_state.value if resume_state is not None else None
        )
        patrol_interrupted = resume_state is not None
        altitude_estimate = self._target_altitude_estimate
        return {
            "targetDecision": self.target_decision_diagnostics(),
            "pursuit": {
                "state": self.state.value,
                "velocityCommand": dict(zip(("x", "y", "z"), self.pursuit.velocity)),
                "estimatedMotion": dict(zip(("x", "y", "z"), self.pursuit.motion)),
                "constraint": self.pursuit.constraint,
            },
            "flightState": self.diagnostic_state,
            "trackerId": self._tracked_target_id,
            "detectionConfidence": self._target_confidence,
            "imageCenterError": {
                "x": self._target_last_center[0] - 0.5,
                "y": self._target_last_center[1] - 0.5,
            },
            "confirmation": {
                "hits": confirmation_hits,
                "requiredHits": required_hits,
                "progress": confirmation_hits / required_hits,
            },
            "estimatedImageVelocity": {
                "x": velocity_x,
                "y": velocity_y,
                "magnitude": velocity_magnitude,
                "directionDeg": velocity_direction_deg,
            },
            "boundingBoxScale": box_scale,
            "boundingBoxAreaRatio": self._target_area_ratio,
            "standoffError": standoff_error,
            "estimatedTargetDistanceM": (
                altitude_estimate.slant_range_m if altitude_estimate is not None else None
            ),
            "estimatedTargetElevationDeg": (
                math.degrees(altitude_estimate.elevation_rad)
                if altitude_estimate is not None else None
            ),
            "estimatedTargetHeightDifferenceM": (
                altitude_estimate.height_error_m if altitude_estimate is not None else None
            ),
            "altitudeMatchingActive": (
                self.state is FlightState.TARGET_PURSUIT
                and self._pursuit_usable
                and altitude_estimate is not None
            ),
            "targetLostDurationS": target_lost_duration_s,
            "patrolInterrupted": patrol_interrupted,
            "patrolResumeState": resume_state_name,
            "patrolResumeStatus": ("returning_to_checkpoint" if self.state is FlightState.TARGET_RETURN
                                   else "interrupted" if patrol_interrupted else "inactive"),
        }

    def observe_detections(
        self,
        detections: list[dict[str, Any]],
        *,
        frame_width: int,
        frame_height: int,
        position: Position,
        yaw: float,
        camera_pitch: float = 0.0,
        allow_landing_intercept: bool = False,
    ) -> None:
        """Apply one tracker frame to patrol interception/follow guidance.

        Takeoff and mission navigation may be interrupted, but their exact
        state and target remain independent from target-follow guidance.
        """
        if frame_width <= 0 or frame_height <= 0:
            return
        project = lambda item: self.project_detection(item, frame_width, frame_height, position, yaw, camera_pitch)
        self.target_decisions.update_visibility(detections, self._elapsed_time, project)
        interruptible_states = {
            FlightState.TAKING_OFF,
            FlightState.TRANSITING_TO_ROI,
            FlightState.ROI_PATROL,
            FlightState.PATROLLING,
            FlightState.REPOSITIONING,
            FlightState.TARGET_ACQUISITION,
            FlightState.TARGET_FOLLOW,
            FlightState.TARGET_PURSUIT,
        }
        if allow_landing_intercept:
            interruptible_states.add(FlightState.LANDING)
        if self.state not in interruptible_states:
            return

        if self.is_target_tracking:
            self.target_decisions.record_pose(position, yaw, camera_pitch)
            # Retain weak matches as evidence even when they cannot guide flight.
            weak = next((item for item in detections
                         if item.get("trackId") == self._tracked_target_id
                         and item.get("trackState") != "lost"
                         and float(item.get("confidence", 0)) < self.config.target_min_confidence), None)
            if weak is not None:
                box = weak["box"]
                self.target_decisions.sample(weak, self._elapsed_time, (
                    (box[0] + box[2]) / (2 * frame_width), (box[1] + box[3]) / (2 * frame_height)))

        selected = self._select_target_detection(detections, project=project)
        if selected is None:
            self.pursuit.stop()
            self._target_altitude_estimate = None
            self._stable_since = None
            self._pursuit_usable = False
            self._pursuit_sample = None
            self._pursuit_growth = 0.0
            self._pursuit_image_motion = (0.0, 0.0)
            if self.state is FlightState.TARGET_ACQUISITION:
                self._target_visible = False
            elif self.state in {FlightState.TARGET_FOLLOW, FlightState.TARGET_PURSUIT}:
                self._target_visible = False
                self.target = position
            self._target_camera_pitch = None
            return

        track_id = str(selected["trackId"])
        interception_start_states = {
            FlightState.TAKING_OFF,
            FlightState.TRANSITING_TO_ROI,
            FlightState.ROI_PATROL,
            FlightState.PATROLLING,
            FlightState.REPOSITIONING,
        }
        if allow_landing_intercept:
            interception_start_states.add(FlightState.LANDING)
        if self.state in interception_start_states:
            if track_id in self._followed_track_ids:
                return
            self._target_resume_state = self.state
            self._patrol_resume_target = self.target
            self._patrol_resume_checkpoint = (
                float(position[0]), float(position[1]), float(position[2])
            )
            self.state = FlightState.TARGET_ACQUISITION
            self.target = self._target_acquisition_target(position)
            self._tracked_target_id = track_id
            self.target_decisions.start(self._elapsed_time, position, yaw, camera_pitch)
            self._target_hits = 0
            self._target_velocity = (0.0, 0.0)
        elif track_id != self._tracked_target_id:
            self.pursuit.stop()
            self._stable_since = None
            self._pursuit_sample = None
            self._pursuit_growth = 0.0
            self._pursuit_image_motion = (0.0, 0.0)
            if self.state is FlightState.TARGET_PURSUIT:
                self.state = FlightState.TARGET_FOLLOW
            # A tentative false positive or a lost track must not prevent a
            # newly confirmed target from taking over the same hover/search
            # session. This is a handoff, not a duplicate follow session.
            self._tracked_target_id = track_id
            self.target_decisions.start(self._elapsed_time, position, yaw, camera_pitch)
            self._target_velocity = (0.0, 0.0)
            if self.state is FlightState.TARGET_ACQUISITION:
                self._target_hits = 0
                self._target_confirmed = False
            elif selected.get("trackState") != "confirmed":
                self.state = FlightState.TARGET_ACQUISITION
                self.target = self._target_acquisition_target(position)
                self._target_hits = 0
                self._target_confirmed = False
            else:
                self._followed_track_ids.add(track_id)

        box = selected["box"]
        if selected.get("trackState") != "lost":
            self.target_decisions.remember(selected, self._elapsed_time, project(selected))
        center = (
            (float(box[0]) + float(box[2])) / (2.0 * frame_width),
            (float(box[1]) + float(box[3])) / (2.0 * frame_height),
        )
        velocity = selected.get("velocityPxPerSec", {})
        measured_velocity = (
            float(velocity.get("x", 0.0)) / frame_width,
            float(velocity.get("y", 0.0)) / frame_height,
        )
        smoothing = max(0.0, min(1.0, self.config.target_velocity_smoothing))
        normalized_velocity = (
            self._target_velocity[0] * (1.0 - smoothing)
            + measured_velocity[0] * smoothing,
            self._target_velocity[1] * (1.0 - smoothing)
            + measured_velocity[1] * smoothing,
        )
        area_ratio = max(0.0, float(box[2]) - float(box[0])) * max(
            0.0, float(box[3]) - float(box[1])
        ) / (frame_width * frame_height)
        is_visible = selected.get("trackState") != "lost"
        self._pursuit_usable = is_visible and selected.get("trackState") == "confirmed"
        stable = self._pursuit_usable and abs(center[0] - 0.5) <= self.config.target_center_tolerance_x and abs(center[1] - 0.5) <= self.config.target_center_tolerance_y
        if not stable:
            self._stable_since = None
        elif self.state is FlightState.TARGET_FOLLOW and self._stable_since is None:
            self._stable_since = self._elapsed_time
        if not self._pursuit_usable:
            self.pursuit.stop()
            self._target_altitude_estimate = None
            self._pursuit_sample = None
            self._pursuit_growth = 0.0
            self._pursuit_image_motion = (0.0, 0.0)
            if self.state is FlightState.TARGET_PURSUIT:
                self.target = position
                is_visible = False
        else:
            self._target_altitude_estimate = estimate_target_altitude(
                box=box,
                frame_width=frame_width,
                frame_height=frame_height,
                camera_pitch=camera_pitch,
                horizontal_fov_rad=self.pursuit.config.horizontal_fov_rad,
                reference_distance_m=self.config.target_standoff_distance_m,
                reference_area_ratio=self.config.target_standoff_box_area_ratio,
                max_range_m=self.config.target_range_estimate_max_m,
            )
            if self._pursuit_sample is not None:
                sample_time, sample_area, sample_yaw = self._pursuit_sample
                sample_dt = self._elapsed_time - sample_time
                if sample_dt > 1e-6:
                    alpha = 1 - math.exp(-sample_dt / self.pursuit.config.smoothing_time_s)
                    self._pursuit_growth += alpha * ((area_ratio - sample_area) / sample_dt - self._pursuit_growth)
                    # Near image center, remove optical flow due to own yaw.
                    # This preserves lateral following while yaw holds a target
                    # centered. It does not recover absolute metric velocity.
                    yaw_rate = normalized_angle(yaw - sample_yaw) / sample_dt
                    motion_x = measured_velocity[0] - yaw_rate / self.pursuit.config.horizontal_fov_rad
                    self._pursuit_image_motion = (
                        self._pursuit_image_motion[0] + alpha * (motion_x - self._pursuit_image_motion[0]),
                        0.0,
                    )
            self._pursuit_sample = (self._elapsed_time, area_ratio, yaw)
        elapsed_since_seen = max(1e-6, self._elapsed_time - self._target_last_seen_at)
        if is_visible and self._target_previous_area_ratio is not None:
            self._target_area_growth_per_s = (
                area_ratio - self._target_previous_area_ratio
            ) / elapsed_since_seen
        if is_visible:
            self._target_previous_area_ratio = area_ratio
            self._target_last_seen_at = self._elapsed_time
        self._target_area_ratio = area_ratio
        self._target_confidence = float(selected.get("confidence", 0.0))
        self._target_last_center = center
        self._target_velocity = normalized_velocity
        self._target_visible = is_visible
        self._target_last_yaw = yaw
        predicted_error = max(
            -0.5,
            min(
                0.5,
                center[0]
                - 0.5
                + normalized_velocity[0]
                * self.config.target_yaw_prediction_horizon_s,
            ),
        )
        if abs(predicted_error) <= self.config.target_yaw_deadband_ratio:
            predicted_error = 0.0
        self._target_yaw = normalized_angle(
            yaw - predicted_error * self.config.target_yaw_gain
        )
        # Image Y grows downward and positive gimbal pitch tilts downward.
        self._target_camera_pitch = (
            camera_pitch + (center[1] - 0.5) * self.config.target_gimbal_pitch_gain
            if is_visible
            else None
        )

        if not self._target_visible:
            return
        self._target_hits += 1
        confirmed = (
            selected.get("trackState") == "confirmed"
            and self._target_hits >= max(1, self.config.target_confirmation_hits)
        )
        self._target_confirmed = self._target_confirmed or confirmed
        # The false-positive observation window runs in parallel with an
        # already-started follow/pursuit; it must never gate the transition
        # into TARGET_FOLLOW below.
        if self.is_target_tracking:
            self.target_decisions.record_pose(position, yaw, camera_pitch)
            self.target_decisions.sample(selected, self._elapsed_time, center)
            if not self.target_decisions.evaluated and self.target_decisions.ready(self._elapsed_time):
                if self.target_decisions.should_reject():
                    self.reject_target(track_id, position)
                    self.target_decisions.last_rejection_reason = "stationary_with_weak_confidence"
                    return
                self.target_decisions.evaluated = True
        # Start visual follow as soon as the tracker confirms -- takeoff
        # climb and altitude/translation safety are handled independently
        # by _target_tracking_hold_target / _pursuit_altitude_is_safe.
        if self.state is FlightState.TARGET_ACQUISITION and confirmed:
            self._start_target_follow(position, yaw)

    def would_intercept(
        self,
        detections: list[dict[str, Any]],
        *,
        allow_landing_intercept: bool = False,
        frame_width: int = 0,
        frame_height: int = 0,
        position: Position | None = None,
        yaw: float = 0.0,
        camera_pitch: float = 0.0,
    ) -> bool:
        """Return whether a tracker frame can interrupt normal patrol."""
        states = {
            FlightState.TAKING_OFF,
            FlightState.TRANSITING_TO_ROI,
            FlightState.ROI_PATROL,
            FlightState.PATROLLING,
            FlightState.REPOSITIONING,
        }
        if allow_landing_intercept:
            states.add(FlightState.LANDING)
        project = (lambda item: self.project_detection(item, frame_width, frame_height, position, yaw, camera_pitch)) if position is not None else None
        return self.state in states and self._select_target_detection(detections, project=project) is not None

    def _select_target_detection(
        self,
        detections: list[dict[str, Any]],
        *, project=None,
    ) -> dict[str, Any] | None:
        candidates = [
            detection
            for detection in detections
            if isinstance(detection, dict)
            and float(detection.get("confidence", 0.0)) >= self.config.target_min_confidence
            and isinstance(detection.get("trackId"), str)
            and isinstance(detection.get("box"), (list, tuple))
            and len(detection["box"]) == 4
            and not self.target_decisions.is_ignored(
                detection["trackId"],
                # A late rejection of the previous candidate may cover this
                # location, but must not cancel a different active candidate.
                project(detection) if project and detection["trackId"] != self._tracked_target_id else None,
                self._elapsed_time)
        ]
        if self._tracked_target_id is not None:
            active = next(
                (item for item in candidates if item["trackId"] == self._tracked_target_id),
                None,
            )
            alternatives = [
                item
                for item in candidates
                if item["trackId"] != self._tracked_target_id
                and item["trackId"] not in self._followed_track_ids
                and item.get("trackState") != "lost"
            ]
            confirmed_alternatives = [
                item for item in alternatives if item.get("trackState") == "confirmed"
            ]
            if active is None or active.get("trackState") == "lost":
                pool = confirmed_alternatives or alternatives
                return max(
                    pool,
                    key=lambda item: float(item["confidence"]),
                    default=active,
                )
            if (
                self.state is FlightState.TARGET_ACQUISITION
                and active.get("trackState") != "confirmed"
                and confirmed_alternatives
            ):
                return max(
                    confirmed_alternatives,
                    key=lambda item: float(item["confidence"]),
                )
            return active
        candidates = [
            item
            for item in candidates
            if item["trackId"] not in self._followed_track_ids
            and item.get("trackState") != "lost"
        ]
        return max(candidates, key=lambda item: float(item["confidence"]), default=None)

    def _target_acquisition_target(self, position: Position) -> Position:
        if (
            self._target_resume_state is FlightState.TAKING_OFF
            and self._patrol_resume_target is not None
        ):
            return (position[0], position[1], self._patrol_resume_target[2])
        return position

    def _target_tracking_hold_target(self, position: Position) -> Position:
        """Hover horizontally and preserve the pursuit altitude floor."""
        if self._target_resume_state is FlightState.TAKING_OFF:
            return (
                position[0],
                position[1],
                max(position[2], self._takeoff_pursuit_altitude()),
            )
        if self._target_resume_state is FlightState.LANDING:
            assert self.home is not None
            floor = self.home[2] + self.pursuit.config.minimum_height_m
            return (position[0], position[1], max(position[2], floor))
        return position

    def _pursuit_altitude_is_safe(self, position: Position) -> bool:
        assert self.home is not None
        if self._target_resume_state is FlightState.TAKING_OFF:
            floor = self.home[2] + self.pursuit.config.minimum_height_m
            return position[2] >= floor + self.config.vertical_tolerance
        return position[2] >= self.home[2] + self.pursuit.config.minimum_height_m

    def _takeoff_pursuit_altitude(self) -> float:
        """Return a safe launch height without requiring normal cruise height."""
        assert self.home is not None
        floor = self.home[2] + self.pursuit.config.minimum_height_m
        minimum = floor + self.config.vertical_tolerance
        useful_clearance = floor + self.config.target_takeoff_pursuit_clearance_m
        return max(minimum, min(self.config.takeoff_altitude, useful_clearance))

    def _continue_takeoff_during_pursuit(self, position: Position, dt: float) -> None:
        """Combine height-limited horizontal pursuit with a bounded ascent."""
        if self._target_resume_state is not FlightState.TAKING_OFF or dt <= 0.0:
            self._takeoff_pursuit_vertical_velocity = 0.0
            return
        if self.pursuit.constraint == "collision":
            self._takeoff_pursuit_vertical_velocity = 0.0
            return
        altitude_error = self._takeoff_pursuit_altitude() - position[2]
        if altitude_error <= 0.0:
            self._takeoff_pursuit_vertical_velocity = 0.0
            return
        desired = min(
            self.config.target_takeoff_pursuit_ascent_mps,
            max(0.0, altitude_error / dt),
        )
        max_delta = self.pursuit.config.acceleration_mps2 * dt
        delta = desired - self._takeoff_pursuit_vertical_velocity
        self._takeoff_pursuit_vertical_velocity += max(
            -max_delta,
            min(max_delta, delta),
        )
        velocity = self.pursuit.velocity
        self.pursuit.velocity = (
            velocity[0],
            velocity[1],
            self._takeoff_pursuit_vertical_velocity,
        )

    def _target_is_centered(self) -> bool:
        return (
            abs(self._target_last_center[0] - 0.5)
            <= self.config.target_center_tolerance_x
            and abs(self._target_last_center[1] - 0.5)
            <= self.config.target_center_tolerance_y
        )

    def _start_target_follow(self, position: Position, yaw: float) -> None:
        self.state = FlightState.TARGET_FOLLOW
        if self._tracked_target_id is not None:
            self._followed_track_ids.add(self._tracked_target_id)
        self.target = position
        self._stable_since = self._elapsed_time if self._target_is_centered() else None

    def project_detection(self, detection, width, height, position, yaw, camera_pitch):
        if self.home is None:
            return None
        ground = ground_projection(
            detection["box"], width, height, position, yaw, camera_pitch,
            self.pursuit.config.horizontal_fov_rad, self.home[2],
            self.config.target_projection_min_angle_rad, self.config.target_projection_max_range_m)
        if ground is not None:
            return ground
        # The camera ray never reaches the ground for anything at or above our
        # own height, which is every airborne target. Fall back to the same
        # calibrated-area range estimate pursuit already steers by, so an
        # operator's rejection sticks to a place and survives the tracker
        # re-identifying that object under a new id.
        return range_estimated_projection(
            detection["box"], width, height, position, yaw, camera_pitch,
            self.pursuit.config.horizontal_fov_rad,
            self.config.target_standoff_distance_m,
            self.config.target_standoff_box_area_ratio,
            self.config.target_range_estimate_max_m)

    def approve_target(self, track_id: str, position: Position) -> bool:
        self.target_decisions.prune(self._elapsed_time)
        memory = self.target_decisions.memories.get(track_id)
        if (track_id != self._tracked_target_id or memory is None or memory.missing_at is not None
                or not self._target_visible or not self.is_target_tracking):
            return False
        self._target_confirmed = True
        # Cancel any pending automatic false-positive rejection for good.
        self.target_decisions.evaluated = True
        if self.state is FlightState.TARGET_ACQUISITION:
            self._start_target_follow(position, self._target_last_yaw)
        # Already following/pursuing: confirm in place, do not restart
        # tracking, reset the target, or pause movement.
        return True

    def reject_target(self, track_id: str, position: Position | None = None) -> bool:
        if not self.target_decisions.reject(
            track_id,
            self._elapsed_time,
            # Only the target actually being followed earns a blanket pause:
            # rejecting some other remembered candidate must not blind the
            # aircraft to the one it is on.
            suppress_all_if_unlocatable=track_id == self._tracked_target_id,
        ):
            return False
        self._followed_track_ids.discard(track_id)
        if track_id == self._tracked_target_id and self.is_target_tracking:
            self.complete_target_follow(position)
        return True

    def target_decision_diagnostics(self):
        self.target_decisions.prune(self._elapsed_time)
        memories = self.target_decisions.memories
        track_id = self._tracked_target_id if self._tracked_target_id in memories else next(reversed(memories), None)
        memory = memories.get(track_id) if track_id is not None else None
        return {
            "trackId": track_id,
            "canApprove": bool(memory and memory.missing_at is None and self._target_visible
                               and track_id == self._tracked_target_id and not self.target_decisions.evaluated
                               and self.is_target_tracking),
            "canReject": memory is not None,
            "rejectableTrackIds": list(memories),
            "lastRejectionReason": self.target_decisions.last_rejection_reason,
        }

    def complete_target_follow(self, position: Position | None = None) -> None:
        """Stop pursuit, return to the interruption point, then restore navigation."""
        self.pursuit.stop()
        self._target_yaw = None
        self._target_camera_pitch = None
        self._target_altitude_estimate = None
        if (
            self._pursuit_started and self._patrol_resume_checkpoint is not None
            and self._target_resume_state is not FlightState.TAKING_OFF
        ):
            self.state = FlightState.TARGET_RETURN
            self.target = self._patrol_resume_checkpoint
            self._target_return_waypoints.clear()
            self._target_return_ready = True
        else:
            # A takeoff-interrupted follow has no route to egress: the
            # checkpoint is just wherever the climb happened to be
            # interrupted, often low altitude. Resume the takeoff climb
            # itself instead of flying back down to that point.
            self._resume_patrol_after_target(position)

    def set_target_return_route(self, waypoints: list[Position]) -> None:
        """Install a safety-routed path whose last point is the checkpoint."""
        if self.state is not FlightState.TARGET_RETURN:
            raise RuntimeError("Target return route requires TARGET_RETURN state")
        if not waypoints:
            raise ValueError("Target return route must contain at least one waypoint")
        self.target = waypoints[0]
        self._target_return_waypoints = deque(waypoints[1:])
        self._target_return_ready = True

    def hold_target_return(self, position: Position) -> None:
        """Hover while a safe mission-resume route is unavailable."""
        if self.state is not FlightState.TARGET_RETURN:
            raise RuntimeError("Target return hold requires TARGET_RETURN state")
        self.pursuit.stop()
        self.target = position
        self._target_return_waypoints.clear()
        self._target_return_ready = False

    def _resume_patrol_after_target(self, position: Position | None = None) -> None:
        resume_state = self._target_resume_state or FlightState.PATROLLING
        resume_target = self._patrol_resume_target
        if resume_state is FlightState.TAKING_OFF and position is not None:
            safe_altitude = self.config.takeoff_altitude - self.config.vertical_tolerance
            if position[2] >= safe_altitude:
                # Already effectively airborne at cruise altitude (e.g. kept
                # climbing while chasing the target) -- go straight to the
                # queued mission instead of re-entering a stale takeoff climb
                # that would leave state/camera stuck on TAKING_OFF.
                resume_state = self._queued_state or FlightState.HOLDING
                queued_waypoint = self._queued_waypoint
                needs_refresh = self._queued_mission_stale
                self._queued_waypoint = None
                self._queued_state = None
                self._queued_mission_stale = False
                if needs_refresh and resume_state in {FlightState.TRANSITING_TO_ROI, FlightState.PATROLLING}:
                    # Same staleness concern as the TAKING_OFF branch in
                    # update(): the queued waypoint/geometry predates an
                    # operating-config change that happened during pursuit.
                    resume_target = self._fresh_queued_mission_target(resume_state, position)
                else:
                    resume_target = queued_waypoint or resume_target
        self.state = resume_state
        self.target = resume_target
        self._clear_target_tracking(preserve_followed_ids=True)

    def _fresh_queued_mission_target(self, state: FlightState, position: Position) -> Position:
        """Recompute the entry waypoint for a takeoff-queued mission from the
        actual arrival position and current config, instead of trusting a
        value computed back when the mission was originally queued -- which
        may now be stale (e.g. the operating config changed during the
        climb). For TRANSITING_TO_ROI this also refreshes the clipped ROI
        geometry itself, since `_is_inside_roi` must agree with whatever
        target is returned here or the aircraft can arrive at the target
        without ever being recognized as "inside" the ROI."""
        if state is FlightState.TRANSITING_TO_ROI:
            assert self._roi_center is not None
            self._roi_first_waypoint = self.roi_patrol_planner.reset(
                self._roi_center, self._roi_radius, self.config.takeoff_altitude, position,
            )
            return self.roi_patrol_planner.entry_target(
                self._roi_center, self.config.takeoff_altitude,
            )
        assert state is FlightState.PATROLLING
        return self._next_patrol_waypoint(position, completed=False)

    def _clear_target_tracking(self, *, preserve_followed_ids: bool = False) -> None:
        self.target_decisions.reset_observation()
        if not preserve_followed_ids:
            self.target_decisions.memories.clear()
        self.pursuit.stop()
        self._stable_since = None
        self._pursuit_usable = False
        self._pursuit_started = False
        self._pursuit_growth = 0.0
        self._pursuit_sample = None
        self._pursuit_image_motion = (0.0, 0.0)
        self._takeoff_pursuit_vertical_velocity = 0.0
        self._target_return_waypoints.clear()
        self._target_return_ready = True
        self._patrol_resume_target = None
        self._patrol_resume_checkpoint = None
        self._target_resume_state = None
        self._tracked_target_id = None
        self._target_hits = 0
        self._target_confidence = None
        self._target_visible = False
        self._target_confirmed = False
        self._target_yaw = None
        self._target_camera_pitch = None
        self._target_altitude_estimate = None
        self._target_previous_area_ratio = None
        self._target_area_growth_per_s = 0.0
        if not preserve_followed_ids:
            self._followed_track_ids.clear()

    def _home_at_cruise_altitude(self) -> Position:
        assert self.home is not None
        return (self.home[0], self.home[1], self.config.takeoff_altitude)

    def _start_takeoff(self, anchor: Position) -> None:
        self._takeoff_anchor = anchor
        self.state = FlightState.TAKING_OFF
        self.target = self._takeoff_target()

    def _takeoff_target(self) -> Position:
        assert self.home is not None
        anchor = self._takeoff_anchor or self.home
        return (anchor[0], anchor[1], self.config.takeoff_altitude)

    def _landing_target(self) -> Position:
        assert self.home is not None
        return (self.home[0], self.home[1], self._landing_target_altitude)

    def _start_roi_patrol(self, position: Position) -> None:
        if self._roi_center is None or self._roi_first_waypoint is None:
            raise RuntimeError("Cannot start ROI patrol without an ROI")
        self.state = FlightState.ROI_PATROL
        self.target = self._roi_first_waypoint

    def _roi_center_target(self) -> Position:
        if self._roi_center is None:
            raise RuntimeError("Cannot recover into a missing ROI")
        return self.roi_patrol_planner.entry_target(
            self._roi_center,
            self.config.takeoff_altitude,
        )

    def _next_patrol_waypoint(
        self,
        position: Position,
        *,
        completed: bool,
    ) -> Position:
        if self.patrol_waypoint_provider is None:
            return self.choose_random_waypoint()
        return self.patrol_waypoint_provider(position, completed)

    def _is_inside_roi(self, position: Position) -> bool:
        return self._roi_center is not None and self.roi_patrol_planner.contains(position)

    def _has_reached(self, position: Position, target: Position) -> bool:
        """Check if drone reached it's target"""
        horizontal_distance = math.hypot(
            target[0] - position[0], target[1] - position[1]
        )
        vertical_distance = abs(target[2] - position[2])
        return (
            horizontal_distance <= self.config.horizontal_tolerance
            and vertical_distance <= self.config.vertical_tolerance
        )

    def _has_completed_takeoff(self, position: Position, target: Position) -> bool:
        """Accept the configured cruise altitude within the existing tolerance."""
        horizontal_distance = math.hypot(
            target[0] - position[0], target[1] - position[1]
        )
        return (
            horizontal_distance <= self.config.horizontal_tolerance
            and position[2] >= self.config.takeoff_altitude - self.config.vertical_tolerance
        )


def navigation_disturbances(
    position: Position,
    target: Position,
    yaw: float,
    horizontal_tolerance: float,
    yaw_gain: float = 0.4,
) -> tuple[float, float]:
    """Return the yaw and forward-pitch commands for a horizontal target."""
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    if math.hypot(dx, dy) <= horizontal_tolerance:
        return (0.0, 0.0)

    desired_yaw = math.atan2(dy, dx)
    #normalize angle to [-pi, pi]
    angle_left = (desired_yaw - yaw + math.pi) % (2.0 * math.pi) - math.pi
    yaw_command = yaw_gain * angle_left / (2.0 * math.pi)

    # The drone sample pitches forward most strongly after it faces the target.
    alignment_command = math.log10(max(abs(angle_left), 1e-6))
    pitch_command = min(max(alignment_command, -1.0), 0.1)
    return (yaw_command, pitch_command)


def boost_forward_pitch(
    pitch_command: float,
    forward_speed_gain: float,
) -> float:
    """Increase forward pitch while preserving the command direction."""
    if pitch_command >= 0.0:
        return pitch_command

    gain = max(0.1, forward_speed_gain)
    return pitch_command * gain


def ramp_command(current: float, target: float, max_delta: float) -> float:
    """Move a command toward its target without exceeding one-step change."""
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


def advance_kinematic_flight(
    *,
    position: Position,
    target: Position,
    yaw: float,
    horizontal_speed_mps: float,
    dt: float,
    config: KinematicFlightConfig | None = None,
    desired_yaw: float | None = None,
    velocity_command: Position | None = None,
) -> KinematicFlightStep:
    """Advance a DJI-like high-level flight model toward a target pose.

    This intentionally controls position/yaw directly instead of raw propeller
    thrust. It models the kind of bounded speed commands a real DJI adapter
    would send to the aircraft's own flight controller.
    """
    if dt <= 0.0:
        return KinematicFlightStep(position, yaw, horizontal_speed_mps, 0.0, 0.0)

    cfg = config or KinematicFlightConfig()
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    dz = target[2] - position[2]
    horizontal_distance = math.hypot(dx, dy)

    should_hold_horizontal = horizontal_distance <= cfg.yaw_hold_horizontal_distance_m
    # A bearing to a target within a few centimeters is dominated by
    # floating-point noise (atan2 becomes numerically unstable), which can
    # otherwise make yaw oscillate indefinitely while a waypoint with no
    # horizontal dead zone (e.g. TARGET_RETURN) is being reached exactly.
    # Freeze yaw this close regardless of the configured dead zone, without
    # affecting how closely the position itself is allowed to converge.
    should_hold_yaw = horizontal_distance <= max(cfg.yaw_hold_horizontal_distance_m, 0.05)
    navigation_yaw = yaw if should_hold_yaw else math.atan2(dy, dx)
    commanded_yaw = navigation_yaw if desired_yaw is None else desired_yaw
    next_yaw = move_angle_toward(
        yaw,
        commanded_yaw,
        cfg.max_yaw_rate_rad_s * dt,
    )

    if velocity_command is not None:
        # Pursuit has already smoothed and constrained the vector. Integrating
        # it directly avoids waypoint dead zones and a second speed ramp.
        next_position: Position = (
            position[0] + velocity_command[0] * dt,
            position[1] + velocity_command[1] * dt,
            position[2] + velocity_command[2] * dt,
        )
        return KinematicFlightStep(
            position=next_position,
            yaw=next_yaw,
            horizontal_speed_mps=math.hypot(velocity_command[0], velocity_command[1]),
            speed_mps=math.sqrt(sum(v * v for v in velocity_command)),
            vertical_speed_mps=velocity_command[2],
        )

    desired_horizontal_speed = (
        0.0
        if should_hold_horizontal
        else min(
            cfg.max_horizontal_speed_mps,
            horizontal_distance / dt,
        )
    )
    commanded_horizontal_speed = ramp_command(
        horizontal_speed_mps,
        desired_horizontal_speed,
        cfg.horizontal_acceleration_mps2 * dt,
    )
    horizontal_step = min(horizontal_distance, commanded_horizontal_speed * dt)
    if horizontal_distance > 1e-9 and not should_hold_horizontal:
        next_x = position[0] + dx / horizontal_distance * horizontal_step
        next_y = position[1] + dy / horizontal_distance * horizontal_step
    else:
        next_x = position[0]
        next_y = position[1]

    vertical_limit = (
        cfg.max_ascent_speed_mps if dz >= 0.0 else cfg.max_descent_speed_mps
    )
    vertical_step = clamp_magnitude(dz, vertical_limit * dt)
    next_position = (next_x, next_y, position[2] + vertical_step)
    actual_horizontal_speed = horizontal_step / dt
    vertical_speed_mps = vertical_step / dt
    speed_mps = distance_3d(position, next_position) / dt

    return KinematicFlightStep(
        position=next_position,
        yaw=next_yaw,
        horizontal_speed_mps=actual_horizontal_speed,
        speed_mps=speed_mps,
        vertical_speed_mps=vertical_speed_mps,
    )


def move_angle_toward(current: float, target: float, max_delta: float) -> float:
    delta = normalized_angle(target - current)
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


def normalized_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp_magnitude(value: float, maximum_magnitude: float) -> float:
    limit = max(0.0, maximum_magnitude)
    if abs(value) <= limit:
        return value
    return math.copysign(limit, value)


def distance_3d(first: Position, second: Position) -> float:
    return math.sqrt(
        (second[0] - first[0]) ** 2
        + (second[1] - first[1]) ** 2
        + (second[2] - first[2]) ** 2
    )
