"""Webots-independent autonomous flight state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import random
from typing import Protocol


Position = tuple[float, float, float]


class RandomSource(Protocol):
    def uniform(self, a: float, b: float) -> float: ...


class FlightState(Enum):
    IDLE = "IDLE"
    TAKING_OFF = "TAKING_OFF"
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


class FlightLogic:
    """Select targets and flight states without controlling motors directly."""

    def __init__(
        self,
        config: FlightConfig | None = None,
        rng: RandomSource | None = None,
    ) -> None:
        self.config = config or FlightConfig()
        self.rng = rng or random.Random()
        self.state = FlightState.IDLE
        self.home: Position | None = None
        self.target: Position | None = None
        self._idle_elapsed = 0.0
        self._landing_target_altitude = self.config.takeoff_altitude
        self._takeoff_anchor: Position | None = None
        self._queued_waypoint: Position | None = None

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
            self._idle_elapsed += max(0.0, dt)
            self.target = self.home
            if self._idle_elapsed >= self.config.idle_duration:
                self._start_takeoff(self.home)

        elif self.state is FlightState.TAKING_OFF:
            self.target = self._takeoff_target()
            if self._has_reached(position, self.target):
                self.state = FlightState.FLYING_RANDOMLY
                self.target = self._queued_waypoint or self.choose_random_waypoint()
                self._queued_waypoint = None

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
            FlightState.FLYING_RANDOMLY,
        }:
            return False

        self.state = FlightState.RETURNING_HOME
        self._queued_waypoint = None
        self.target = self._home_at_cruise_altitude()
        return True

    def request_new_target(self, position: Position) -> Position:
        """Start a new random mission safely from any flight state."""
        if self.home is None:
            raise RuntimeError("Home position must be initialized before commands.")

        waypoint = self.choose_random_waypoint()
        safe_altitude = (
            self.config.takeoff_altitude - self.config.vertical_tolerance
        )
        if position[2] < safe_altitude:
            self._queued_waypoint = waypoint
            self._start_takeoff(position)
        else:
            self._queued_waypoint = None
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
) -> tuple[float, float]:
    """Return the yaw and forward-pitch commands for a horizontal target."""
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    if math.hypot(dx, dy) <= horizontal_tolerance:
        return (0.0, 0.0)

    desired_yaw = math.atan2(dy, dx)
    #normalize angle to [-pi, pi]
    angle_left = (desired_yaw - yaw + math.pi) % (2.0 * math.pi) - math.pi
    yaw_command = 0.4 * angle_left / (2.0 * math.pi)

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
