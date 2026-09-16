"""Webots-independent autonomous flight state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import random
from typing import Callable, Protocol

try:
    from .roi_patrol import LawnMowerRoiPatrolPlanner, RoiPatrolPlan
except ImportError:  # Webots loads this directory as top-level modules.
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
    FLYING_RANDOMLY = "FLYING_RANDOMLY"
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
    landing_rate: float = 0.35
    landed_height: float = 0.12
    roi_arrival_tolerance: float = 10.0


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
    ) -> None:
        self.config = config or FlightConfig()
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
        self._roi_center: tuple[float, float] | None = None
        self._roi_radius = 0.0

    def initialize_home(self, position: Position) -> None:
        """Capture home once from the first valid simulated GPS reading."""
        if self.home is None:
            self.home = (
                float(position[0]),
                float(position[1]),
                float(position[2]),
            )
            self.target = self.home

    def update(self, position: Position, dt: float) -> None:
        """Advance state and target selection by one simulation timestep."""
        if self.home is None:
            raise RuntimeError("Home position must be initialized before update().")

        if self.state is FlightState.IDLE:
            self.target = self.home

        elif self.state is FlightState.TAKING_OFF:
            self.target = self._takeoff_target()
            if self._has_reached(position, self.target):
                self.state = self._queued_state or FlightState.HOLDING
                self.target = self._queued_waypoint or self.target
                self._queued_waypoint = None
                self._queued_state = None
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
                # Recover toward the center after an external disturbance.
                self.target = self._roi_center_target()
            elif self.target is None or self._has_reached(position, self.target):
                self.target = self.roi_patrol_planner.next_waypoint()

        elif self.state is FlightState.PATROLLING:
            if self.target is None or self._has_reached(position, self.target):
                self.target = self._next_patrol_waypoint(position, completed=True)

        elif self.state is FlightState.FLYING_RANDOMLY:
            if self.target is None:
                self.target = self.choose_random_waypoint()
            elif self._has_reached(position, self.target):
                self.request_return_home()

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
            FlightState.FLYING_RANDOMLY,
        }:
            return False

        self.state = FlightState.RETURNING_HOME
        self._queued_waypoint = None
        self._queued_state = None
        self._roi_center = None
        self._roi_radius = 0.0
        self.roi_patrol_planner.clear()
        self.target = self._home_at_cruise_altitude()
        return True

    def request_mission_takeoff(self, position: Position) -> None:
        """Take off for a mission, or hold immediately when already airborne."""
        if self.home is None:
            raise RuntimeError("Home position must be initialized before commands.")

        self._queued_waypoint = None
        self._queued_state = None
        self._roi_center = None
        self._roi_radius = 0.0
        self.roi_patrol_planner.clear()
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
        self.roi_patrol_planner.clear()
        roi_target = (center[0], center[1], self.config.takeoff_altitude)
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
        was_queued_for_roi = self._queued_state is FlightState.TRANSITING_TO_ROI
        self._roi_center = None
        self._roi_radius = 0.0
        self.roi_patrol_planner.clear()
        if was_queued_for_roi:
            self._queued_waypoint = None
            self._queued_state = None
        if self.state in {FlightState.TRANSITING_TO_ROI, FlightState.ROI_PATROL}:
            self.state = FlightState.HOLDING
            self.target = position

    def request_new_target(self, position: Position) -> Position:
        """Start a new random mission safely from any flight state."""
        if self.home is None:
            raise RuntimeError("Home position must be initialized before commands.")

        self._roi_center = None
        self._roi_radius = 0.0
        self.roi_patrol_planner.clear()
        waypoint = self.choose_random_waypoint()
        safe_altitude = (
            self.config.takeoff_altitude - self.config.vertical_tolerance
        )
        if position[2] < safe_altitude:
            self._queued_waypoint = waypoint
            self._queued_state = FlightState.FLYING_RANDOMLY
            self._start_takeoff(position)
        else:
            self._queued_waypoint = None
            self._queued_state = None
            self.state = FlightState.FLYING_RANDOMLY
            self.target = waypoint
        return waypoint

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
        if self._roi_center is None:
            raise RuntimeError("Cannot start ROI patrol without an ROI")
        self.state = FlightState.ROI_PATROL
        self.target = self.roi_patrol_planner.reset(
            self._roi_center,
            self._roi_radius,
            self.config.takeoff_altitude,
            position,
        )

    def _roi_center_target(self) -> Position:
        if self._roi_center is None:
            raise RuntimeError("Cannot recover into a missing ROI")
        return (
            self._roi_center[0],
            self._roi_center[1],
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
        if self._roi_center is None:
            return False
        distance_m = math.hypot(
            position[0] - self._roi_center[0],
            position[1] - self._roi_center[1],
        )
        # Patrol must not begin in the arrival-tolerance halo outside the ROI.
        # The configured tolerance remains useful to mission/UI observers, but
        # flight guidance uses the physical boundary as its safety invariant.
        return distance_m <= self._roi_radius

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
    desired_yaw = yaw if should_hold_horizontal else math.atan2(dy, dx)
    next_yaw = move_angle_toward(
        yaw,
        desired_yaw,
        cfg.max_yaw_rate_rad_s * dt,
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
