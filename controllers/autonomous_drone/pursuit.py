"""Bounded visual pursuit velocities. Image scale is not metric range."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PursuitConfig:
    stable_duration_s: float = 0.8
    observation_timeout_s: float = 1.5
    max_speed_mps: float = 4.0
    acceleration_mps2: float = 2.0
    smoothing_time_s: float = 0.35
    motion_deadzone: float = 0.015
    geofence_radius_m: float = 2500.0
    minimum_height_m: float = 3.5
    maximum_height_m: float = 120.0
    horizontal_fov_rad: float = 1.0

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"Pursuit {name} must be finite and positive")
        if self.maximum_height_m < self.minimum_height_m:
            raise ValueError("Pursuit maximum height must exceed minimum height")
        if self.minimum_height_m < 3.5:
            raise ValueError("Pursuit minimum height must be at least 3.5 m")


class PursuitController:
    def __init__(self, config: PursuitConfig) -> None:
        self.config = config
        self.velocity = (0.0, 0.0, 0.0)
        self.motion = (0.0, 0.0, 0.0)
        self.constraint: str | None = None

    def stop(self) -> None:
        self.velocity = (0.0, 0.0, 0.0)
        self.motion = (0.0, 0.0, 0.0)
        self.constraint = None

    def update(
        self, *, position: tuple[float, float, float],
        home: tuple[float, float, float], yaw: float,
        image_velocity: tuple[float, float], area: float, growth: float,
        desired_area: float, area_deadzone: float, collision_growth: float,
        collision_side: float, lateral_gain: float, dt: float,
        collision_detected: bool = False,
    ) -> tuple[float, float, float]:
        if dt <= 0:
            return self.velocity
        cfg = self.config
        self.constraint = None
        # History-derived expansion/contraction estimates radial motion; image X
        # estimates lateral motion. These are control cues, not world velocities.
        radial = -growth / max(desired_area, 1e-6)
        lateral = image_velocity[0] * lateral_gain
        radial = 0.0 if abs(growth) < cfg.motion_deadzone * desired_area else radial
        lateral = 0.0 if abs(image_velocity[0]) < cfg.motion_deadzone else lateral
        forward_x, forward_y = math.cos(yaw), math.sin(yaw)
        right_x, right_y = math.sin(yaw), -math.cos(yaw)
        self.motion = (forward_x * radial + right_x * lateral,
                       forward_y * radial + right_y * lateral, 0.0)
        error = desired_area - area
        standoff = 0.0 if abs(error) <= area_deadzone else error / max(desired_area, 1e-6)
        forward = standoff + radial
        collision = collision_detected or growth >= collision_growth
        if collision:
            forward = min(-0.5, forward)
            lateral = collision_side
            self.constraint = "collision"
        desired = (forward_x * forward + right_x * lateral,
                   forward_y * forward + right_y * lateral, 0.0)
        speed = math.hypot(*desired[:2])
        limit = cfg.max_speed_mps
        # Height clearance bounds speed by braking distance, v²/(2a).
        clearance = max(0.0, position[2] - home[2] - cfg.minimum_height_m)
        limit = min(limit, math.sqrt(2 * cfg.acceleration_mps2 * clearance))
        if speed > limit:
            desired = tuple(v * limit / speed for v in desired)
        alpha = 1 - math.exp(-dt / cfg.smoothing_time_s)
        delta = tuple((wanted - old) * alpha for wanted, old in zip(desired, self.velocity))
        magnitude = math.sqrt(sum(v * v for v in delta))
        scale = min(1.0, cfg.acceleration_mps2 * dt / max(magnitude, 1e-12))
        velocity = tuple(old + d * scale for old, d in zip(self.velocity, delta))
        velocity = (velocity[0], velocity[1], 0.0)
        if speed == 0 and math.hypot(*velocity[:2]) < 0.02:
            velocity = (0.0, 0.0, 0.0)
        if collision:
            # Safety can brake immediately; smoothing must not retain approach.
            toward = velocity[0] * forward_x + velocity[1] * forward_y
            if toward > 0:
                velocity = (velocity[0] - toward * forward_x,
                            velocity[1] - toward * forward_y, velocity[2])
        actual_speed = math.hypot(*velocity[:2])
        if actual_speed > limit:
            velocity = (velocity[0] * limit / actual_speed, velocity[1] * limit / actual_speed, velocity[2])
        if clearance <= 0:
            # Recover upward if already below the floor; never command descent.
            velocity = (0.0, 0.0, min(1.0, max(0.0, home[2] + cfg.minimum_height_m - position[2]) / dt))
            self.constraint = "height_floor"
        elif position[2] > home[2] + cfg.maximum_height_m:
            velocity = (0.0, 0.0, -min(1.0, (position[2] - home[2] - cfg.maximum_height_m) / dt))
            self.constraint = "height_ceiling"
        rx, ry = position[0] - home[0], position[1] - home[1]
        speed = math.hypot(*velocity[:2])
        braking = speed * speed / (2 * cfg.acceleration_mps2)
        if math.hypot(rx + velocity[0] * dt, ry + velocity[1] * dt) + braking >= cfg.geofence_radius_m:
            velocity = (0.0, 0.0, velocity[2])
            self.constraint = "geofence"
        self.velocity = velocity
        return velocity
