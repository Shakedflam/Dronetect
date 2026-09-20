"""Bounded visual pursuit velocities. Image scale is not metric range."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TargetAltitudeEstimate:
    """Camera-only target geometry derived from one detector frame."""

    slant_range_m: float
    elevation_rad: float
    height_error_m: float


def estimate_target_altitude(
    *,
    box: tuple[float, float, float, float] | list[float],
    frame_width: int,
    frame_height: int,
    camera_pitch: float,
    horizontal_fov_rad: float,
    reference_distance_m: float,
    reference_area_ratio: float,
    max_range_m: float,
) -> TargetAltitudeEstimate | None:
    """Estimate target height from a pinhole ray and calibrated box area.

    Positive camera pitch points down. The range estimate assumes the same
    target size used to calibrate ``reference_area_ratio`` and is therefore a
    control cue, not an absolute geometric measurement.
    """
    values = (
        *box,
        frame_width,
        frame_height,
        camera_pitch,
        horizontal_fov_rad,
        reference_distance_m,
        reference_area_ratio,
        max_range_m,
    )
    if (
        len(box) != 4
        or not all(math.isfinite(float(value)) for value in values)
        or frame_width <= 0
        or frame_height <= 0
        or not 0.0 < horizontal_fov_rad < math.pi
        or reference_distance_m <= 0.0
        or reference_area_ratio <= 0.0
        or max_range_m <= 0.0
    ):
        return None

    box_width = max(0.0, float(box[2]) - float(box[0]))
    box_height = max(0.0, float(box[3]) - float(box[1]))
    area_ratio = box_width * box_height / (frame_width * frame_height)
    if area_ratio <= 1e-9:
        return None

    slant_range_m = reference_distance_m * math.sqrt(
        reference_area_ratio / area_ratio
    )
    if not math.isfinite(slant_range_m) or slant_range_m > max_range_m:
        return None

    center_x = (float(box[0]) + float(box[2])) / (2.0 * frame_width)
    center_y = (float(box[1]) + float(box[3])) / (2.0 * frame_height)
    right = (2.0 * center_x - 1.0) * math.tan(horizontal_fov_rad / 2.0)
    down = (
        (2.0 * center_y - 1.0)
        * math.tan(horizontal_fov_rad / 2.0)
        * frame_height
        / frame_width
    )
    forward = math.cos(camera_pitch) - down * math.sin(camera_pitch)
    vertical = -math.sin(camera_pitch) - down * math.cos(camera_pitch)
    ray_length = math.sqrt(forward * forward + right * right + vertical * vertical)
    if ray_length <= 1e-9 or not math.isfinite(ray_length):
        return None

    vertical_component = max(-1.0, min(1.0, vertical / ray_length))
    elevation_rad = math.asin(vertical_component)
    return TargetAltitudeEstimate(
        slant_range_m=slant_range_m,
        elevation_rad=elevation_rad,
        height_error_m=slant_range_m * vertical_component,
    )


@dataclass(frozen=True)
class PursuitConfig:
    stable_duration_s: float = 0.8
    observation_timeout_s: float = 1.5
    max_speed_mps: float = 5.0
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
        height_error_m: float | None = None,
        altitude_deadband_m: float = 0.0,
        altitude_gain_per_s: float = 0.0,
        max_vertical_speed_mps: float = 0.0,
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
        # Normalized error alone saturates near 1.0 regardless of top speed;
        # scale by max_speed_mps so a far target is approached at cruise
        # speed and the commanded speed tapers to zero at the standoff point.
        standoff = 0.0 if abs(error) <= area_deadzone else (error / max(desired_area, 1e-6)) * cfg.max_speed_mps
        forward = standoff + radial
        collision = collision_detected or growth >= collision_growth
        if collision:
            forward = min(-0.5, forward)
            lateral = collision_side
            self.constraint = "collision"
        # Height error is metric and positive when the target is above us.
        # Subtracting the deadband before applying gain avoids a command jump
        # as the estimate crosses the target band.
        excess_height = 0.0
        if height_error_m is not None and math.isfinite(height_error_m):
            excess_height = max(0.0, abs(height_error_m) - altitude_deadband_m)
        desired_vertical = math.copysign(
            min(max_vertical_speed_mps, excess_height * altitude_gain_per_s),
            height_error_m or 1.0,
        ) if excess_height > 0.0 else 0.0
        desired = (forward_x * forward + right_x * lateral,
                   forward_y * forward + right_y * lateral, desired_vertical)
        speed = math.hypot(*desired[:2])
        limit = cfg.max_speed_mps
        # Height clearance bounds speed by braking distance, v²/(2a).
        clearance = max(0.0, position[2] - home[2] - cfg.minimum_height_m)
        limit = min(limit, math.sqrt(2 * cfg.acceleration_mps2 * clearance))
        if speed > limit:
            desired = (desired[0] * limit / speed, desired[1] * limit / speed, desired[2])
        alpha = 1 - math.exp(-dt / cfg.smoothing_time_s)
        delta = tuple((wanted - old) * alpha for wanted, old in zip(desired, self.velocity))
        magnitude = math.sqrt(sum(v * v for v in delta))
        scale = min(1.0, cfg.acceleration_mps2 * dt / max(magnitude, 1e-12))
        velocity = tuple(old + d * scale for old, d in zip(self.velocity, delta))
        if speed == 0 and desired_vertical == 0.0 and math.hypot(*velocity[:2]) < 0.02:
            velocity = (0.0, 0.0, velocity[2])
        if collision:
            # Safety can brake immediately; smoothing must not retain approach.
            toward = velocity[0] * forward_x + velocity[1] * forward_y
            if toward > 0:
                velocity = (velocity[0] - toward * forward_x,
                            velocity[1] - toward * forward_y, velocity[2])
        actual_speed = math.hypot(*velocity[:2])
        if actual_speed > limit:
            velocity = (velocity[0] * limit / actual_speed, velocity[1] * limit / actual_speed, velocity[2])
        floor = home[2] + cfg.minimum_height_m
        ceiling = home[2] + cfg.maximum_height_m
        minimum_vertical = min(0.0, (floor - position[2]) / dt)
        maximum_vertical = max(0.0, (ceiling - position[2]) / dt)
        bounded_vertical = min(max(velocity[2], minimum_vertical), maximum_vertical)
        if bounded_vertical != velocity[2]:
            self.constraint = "height_floor" if velocity[2] < minimum_vertical else "height_ceiling"
            velocity = (velocity[0], velocity[1], bounded_vertical)
        if clearance <= 0:
            # Recover upward if already below the floor; never command descent.
            velocity = (0.0, 0.0, min(1.0, max(0.0, home[2] + cfg.minimum_height_m - position[2]) / dt))
            self.constraint = "height_floor"
        elif position[2] > ceiling:
            velocity = (0.0, 0.0, -min(1.0, (position[2] - ceiling) / dt))
            self.constraint = "height_ceiling"
        rx, ry = position[0] - home[0], position[1] - home[1]
        speed = math.hypot(*velocity[:2])
        braking = speed * speed / (2 * cfg.acceleration_mps2)
        if math.hypot(rx + velocity[0] * dt, ry + velocity[1] * dt) + braking >= cfg.geofence_radius_m:
            velocity = (0.0, 0.0, velocity[2])
            self.constraint = "geofence"
        self.velocity = velocity
        return velocity
