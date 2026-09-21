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
class TargetState:
    """World-frame target estimate derived from successive camera fixes.

    Positions come from the same calibrated-area projection pursuit steers
    by, so this is a filtered control estimate and not a measured track.
    """

    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    range_m: float
    speed_mps: float
    closure_mps: float


def reference_area_ratio_from_span(
    *,
    span_m: float,
    distance_m: float,
    horizontal_fov_rad: float,
    frame_width: int,
    frame_height: int,
) -> float | None:
    """Fraction of frame a target of ``span_m`` fills at ``distance_m``.

    The calibrated-area range estimate needs to be told how big the thing it
    is looking at actually is. Deriving that from the airframe span and the
    real camera geometry keeps it honest: a magic ratio silently encodes an
    assumed target size and an assumed lens, and is wrong the moment either
    changes -- which is how a range estimate ends up off by a factor while
    still looking plausible.
    """
    if (
        span_m <= 0.0
        or distance_m <= 0.0
        or not 0.0 < horizontal_fov_rad < math.pi
        or frame_width <= 0
        or frame_height <= 0
        or not all(math.isfinite(v) for v in (span_m, distance_m, horizontal_fov_rad))
    ):
        return None
    # Pinhole projection: image offset goes as tan(angle), not as the angle.
    # Using the angle directly costs ~0.5% at close range, which would be
    # tolerable on its own -- except that it also breaks the inverse-square
    # law the range estimate inverts, so the reference distance would stop
    # being an arbitrary anchor and quietly become a third calibration.
    half_width = math.tan(horizontal_fov_rad / 2.0)
    if half_width <= 0.0:
        return None
    width_fraction = (span_m / distance_m) / (2.0 * half_width)
    # The vertical half-angle is the horizontal one scaled by the aspect.
    height_fraction = width_fraction * frame_width / frame_height
    ratio = width_fraction * height_fraction
    if not math.isfinite(ratio) or ratio <= 0.0:
        return None
    return min(1.0, ratio)


def calibrate_reference_area_ratio(
    *,
    measured_area_ratio: float,
    measured_range_m: float,
    reference_distance_m: float,
) -> float | None:
    """Reference ratio implied by one box seen at a known range.

    Box area falls off as the square of range, so a single honest observation
    fixes the calibration. This is the inverse of the range estimate itself,
    and exists so the value can be measured off a real run rather than
    assumed from airframe drawings and a nominal lens.
    """
    if (
        measured_area_ratio <= 0.0
        or measured_range_m <= 0.0
        or reference_distance_m <= 0.0
        or not all(
            math.isfinite(v)
            for v in (measured_area_ratio, measured_range_m, reference_distance_m)
        )
    ):
        return None
    return measured_area_ratio * (measured_range_m / reference_distance_m) ** 2


def time_to_contact_s(relative_growth: float) -> float:
    """Seconds to contact implied by a scale-free box-growth cue.

    For an inverse-square box ``d(area)/dt / area == 2 * v / d == 2 / tau``,
    so the growth cue yields time to contact directly. Reading it this way
    needs no range estimate, which matters because the area-derived range
    degrades as the square of distance exactly where a dodge is decided.
    """
    if not math.isfinite(relative_growth) or relative_growth <= 1e-9:
        return math.inf
    return 2.0 / relative_growth


@dataclass(frozen=True)
class PursuitConfig:
    stable_duration_s: float = 0.8
    observation_timeout_s: float = 1.5
    max_speed_mps: float = 5.0
    acceleration_mps2: float = 5.0
    smoothing_time_s: float = 0.2
    motion_deadzone: float = 0.015
    geofence_radius_m: float = 2500.0
    minimum_height_m: float = 3.5
    maximum_height_m: float = 120.0
    horizontal_fov_rad: float = 1.0
    # Station held behind and above the target once its heading is known.
    standoff_distance_m: float = 5.0
    altitude_offset_m: float = 3.0
    # How hard the stern chase pulls toward the six o'clock station. Closing
    # the gap is capped separately by approach_time_to_contact_s, so this
    # governs the part that actually takes time: swinging around behind a
    # target that is already at range.
    station_keeping_gain_per_s: float = 1.6
    # Below this the heading estimate is noise, so hold the image-space law.
    tail_chase_min_speed_mps: float = 0.8
    # Dodge thresholds are times, not distances: the clearance a lateral
    # escape can generate depends on how long there is to generate it, not
    # on how far away the threat currently is.
    entry_time_to_contact_s: float = 1.8
    exit_time_to_contact_s: float = 3.0
    emergency_time_to_contact_s: float = 0.8
    # Our own closing speed is capped to keep time to contact above this,
    # so a shorter one means the target is closing on us, not the reverse.
    approach_time_to_contact_s: float = 2.5
    # Hard floor on how close the aircraft may be driven to the target,
    # beneath the dodge threshold so the dodge acts first and this only ever
    # catches what the dodge missed. It exists because braking distance grows
    # as the square of speed: at 10 m/s with 5 m/s2 the aircraft needs 10 m to
    # stop and the standoff is 5, so the tapering closure cap is otherwise the
    # only thing between it and the target -- and that cap needs a valid range
    # estimate, which a single bad box takes away.
    minimum_separation_m: float = 2.0
    pass_under_min_range_m: float = 2.5
    pass_under_max_range_m: float = 25.0
    target_closure_threshold_mps: float = 1.0

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"Pursuit {name} must be finite and positive")
        if self.maximum_height_m < self.minimum_height_m:
            raise ValueError("Pursuit maximum height must exceed minimum height")
        if self.minimum_height_m < 3.5:
            raise ValueError("Pursuit minimum height must be at least 3.5 m")
        if not (
            self.emergency_time_to_contact_s
            < self.entry_time_to_contact_s
            < self.exit_time_to_contact_s
        ):
            raise ValueError(
                "Pursuit dodge times must increase: emergency < entry < exit"
            )
        if self.approach_time_to_contact_s <= self.entry_time_to_contact_s:
            raise ValueError(
                "Pursuit approach time to contact must exceed the dodge entry time"
            )
        if self.pass_under_max_range_m <= self.pass_under_min_range_m:
            raise ValueError("Pursuit pass-under range window must be non-empty")
        if self.minimum_separation_m <= 0.0:
            raise ValueError("Pursuit minimum separation must be positive")
        if self.minimum_separation_m > self.pass_under_min_range_m:
            raise ValueError(
                "Pursuit minimum separation must not exceed the pass-under range"
            )
        if self.altitude_offset_m >= self.maximum_height_m - self.minimum_height_m:
            raise ValueError("Pursuit altitude offset must fit inside the height band")


class PursuitController:
    def __init__(self, config: PursuitConfig) -> None:
        self.config = config
        self.velocity = (0.0, 0.0, 0.0)
        self.motion = (0.0, 0.0, 0.0)
        self.constraint: str | None = None
        self.avoidance_side = 0.0
        self.time_to_contact_s = math.inf

    @property
    def avoidance_active(self) -> bool:
        return self.avoidance_side != 0.0

    def stop(self, *, preserve_avoidance: bool = False) -> None:
        self.velocity = (0.0, 0.0, 0.0)
        self.motion = (0.0, 0.0, 0.0)
        self.constraint = None
        self.time_to_contact_s = math.inf
        if not preserve_avoidance:
            self.avoidance_side = 0.0

    def coast(
        self,
        dt: float,
        *,
        toward: tuple[float, float] = (0.0, 0.0),
        climb: float = 0.0,
        position: tuple[float, float, float] | None = None,
        home: tuple[float, float, float] | None = None,
    ) -> tuple[float, float, float]:
        """Ease toward a velocity at the acceleration limit, zero by default.

        One tentative or dropped tracker frame must not discard the approach
        ramp: rebuilding it from rest takes seconds at the pursuit
        acceleration limit, which reads on screen as a drone that refuses to
        chase.

        ``climb`` is the vertical rate to ease toward, used by the search
        after a loss to gain a little height: a target that went behind a
        roofline is found by seeing over it, not by staring at it.

        ``toward`` lets a lost target be coasted after rather than braked on
        -- keeping pace with where it was last going, so it is still roughly
        ahead when it clears whatever it went behind. Callers decay it to
        zero over the search window; the geofence is rechecked here because
        this path moves without a fix to steer by.
        """
        self.motion = (0.0, 0.0, 0.0)
        self.constraint = None
        if dt <= 0.0:
            return self.velocity
        desired_x, desired_y = toward
        speed = math.hypot(desired_x, desired_y)
        limit = self.config.max_speed_mps
        if speed > limit:
            desired_x, desired_y = desired_x * limit / speed, desired_y * limit / speed
        desired = (desired_x, desired_y, climb)
        delta = tuple(want - have for want, have in zip(desired, self.velocity))
        magnitude = math.sqrt(sum(value * value for value in delta))
        step = self.config.acceleration_mps2 * dt
        if magnitude <= step or magnitude <= 1e-12:
            velocity = desired
        else:
            scale = step / magnitude
            velocity = (
                self.velocity[0] + delta[0] * scale,
                self.velocity[1] + delta[1] * scale,
                self.velocity[2] + delta[2] * scale,
            )
        if position is not None and home is not None:
            radial_x, radial_y = position[0] - home[0], position[1] - home[1]
            horizontal = math.hypot(velocity[0], velocity[1])
            braking = horizontal * horizontal / (2 * self.config.acceleration_mps2)
            if math.hypot(
                radial_x + velocity[0] * dt, radial_y + velocity[1] * dt
            ) + braking >= self.config.geofence_radius_m:
                velocity = (0.0, 0.0, velocity[2])
                self.constraint = "geofence"
        self.velocity = velocity
        return self.velocity

    def update(
        self, *, position: tuple[float, float, float],
        home: tuple[float, float, float], yaw: float,
        image_velocity: tuple[float, float], area: float, growth: float,
        desired_area: float, area_deadzone: float,
        collision_side: float, lateral_gain: float, dt: float,
        raw_relative_growth: float = 0.0,
        estimated_horizontal_range_m: float | None = None,
        height_error_m: float | None = None,
        altitude_deadband_m: float = 0.0,
        altitude_gain_per_s: float = 0.0,
        max_vertical_speed_mps: float = 0.0,
        target_state: TargetState | None = None,
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

        # Both the smoothed and the raw cue are read as times to contact: the
        # raw one reacts a frame sooner, the smoothed one survives box jitter.
        self.time_to_contact_s = min(
            time_to_contact_s(growth / max(area, 1e-6)),
            time_to_contact_s(raw_relative_growth),
        )
        time_to_contact = self.time_to_contact_s
        range_is_valid = (
            estimated_horizontal_range_m is not None
            and math.isfinite(estimated_horizontal_range_m)
            and estimated_horizontal_range_m >= 0.0
        )
        estimated_range = (
            float(estimated_horizontal_range_m)
            if estimated_horizontal_range_m is not None and range_is_valid
            else math.inf
        )
        # A passive target is answered by braking; only something flying at us
        # has to be stepped around. Where no heading estimate exists, the
        # approach speed cap below is what rules out our own closure instead.
        target_receding = (
            target_state is not None
            and target_state.closure_mps < cfg.target_closure_threshold_mps
        )
        if range_is_valid and estimated_range <= cfg.pass_under_min_range_m:
            at_risk = True
        elif time_to_contact <= cfg.emergency_time_to_contact_s:
            at_risk = True
        elif (
            time_to_contact <= cfg.entry_time_to_contact_s
            and range_is_valid
            and estimated_range <= cfg.pass_under_max_range_m
        ):
            at_risk = not target_receding
        else:
            at_risk = False

        if self.avoidance_active:
            risk_cleared = (
                time_to_contact >= cfg.exit_time_to_contact_s
                and not (
                    range_is_valid and estimated_range <= cfg.pass_under_min_range_m
                )
            )
            if risk_cleared:
                self.avoidance_side = 0.0
        elif at_risk:
            self.avoidance_side = 1.0 if collision_side >= 0.0 else -1.0

        avoiding = self.avoidance_active
        # Height error is metric and positive when the target is above us.
        # Subtracting the deadband before applying gain avoids a command jump
        # as the estimate crosses the target band.
        altitude_error = None
        if height_error_m is not None and math.isfinite(height_error_m):
            # Hold station above the target rather than level with it.
            altitude_error = height_error_m + cfg.altitude_offset_m
        excess_height = (
            max(0.0, abs(altitude_error) - altitude_deadband_m)
            if altitude_error is not None else 0.0
        )
        desired_vertical = math.copysign(
            min(max_vertical_speed_mps, excess_height * altitude_gain_per_s),
            altitude_error or 1.0,
        ) if excess_height > 0.0 else 0.0

        tail_chase = (
            target_state is not None
            and not avoiding
            and target_state.speed_mps >= cfg.tail_chase_min_speed_mps
        )
        if avoiding:
            forward = min(-0.5, forward)
            lateral = self.avoidance_side * abs(collision_side)
            self.constraint = "pass_under_avoidance"
            # Stepping aside is the escape; climbing toward an oncoming
            # aircraft is not, so the standoff offset is suspended here.
            desired_vertical = 0.0
            desired = (forward_x * forward + right_x * lateral,
                       forward_y * forward + right_y * lateral, desired_vertical)
        elif tail_chase and target_state is not None:
            # Fly the six o'clock: hold a point one standoff behind the
            # target along its own heading and one offset above it, with the
            # target velocity as feed-forward so station can actually be kept
            # instead of perpetually lagging a box-area error.
            target_x, target_y, target_z = target_state.position
            target_vx, target_vy, target_vz = target_state.velocity
            heading = math.hypot(target_vx, target_vy)
            heading_x, heading_y = (
                (target_vx / heading, target_vy / heading)
                if heading > 1e-9 else (forward_x, forward_y)
            )
            gain = cfg.station_keeping_gain_per_s
            station_x = target_x - cfg.standoff_distance_m * heading_x
            station_y = target_y - cfg.standoff_distance_m * heading_y
            station_z = target_z + cfg.altitude_offset_m
            desired_vertical = max(
                -max_vertical_speed_mps,
                min(
                    max_vertical_speed_mps,
                    target_vz + gain * (station_z - position[2]),
                ),
            )
            desired = (
                target_vx + gain * (station_x - position[0]),
                target_vy + gain * (station_y - position[1]),
                desired_vertical,
            )
            # No constraint is recorded here: the stern chase is a mode, not a
            # limit, and claiming the field would mask a real one below.
        else:
            desired = (forward_x * forward + right_x * lateral,
                       forward_y * forward + right_y * lateral, desired_vertical)

        # Cap only the component that closes the gap, never total speed: a
        # stern chase matching a fast target closes at zero and must not be
        # slowed. Holding our own closure below the dodge entry time is what
        # lets a shorter time to contact be read as the target's doing.
        if range_is_valid and not avoiding:
            if target_state is not None:
                line_x = target_state.position[0] - position[0]
                line_y = target_state.position[1] - position[1]
            else:
                line_x, line_y = forward_x, forward_y
            line = math.hypot(line_x, line_y)
            if line > 1e-9:
                line_x, line_y = line_x / line, line_y / line
                # Closure is relative: matching a fleeing target's speed
                # shrinks no gap at all, and must not be billed as if it did.
                target_vx, target_vy = (
                    target_state.velocity[:2] if target_state is not None else (0.0, 0.0)
                )
                closing = (
                    (desired[0] - target_vx) * line_x
                    + (desired[1] - target_vy) * line_y
                )
                closing_cap = estimated_range / cfg.approach_time_to_contact_s
                if closing > closing_cap:
                    excess = closing - closing_cap
                    desired = (
                        desired[0] - excess * line_x,
                        desired[1] - excess * line_y,
                        desired[2],
                    )
                    self.constraint = self.constraint or "approach_speed"

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
        velocity = (self.velocity[0] + delta[0] * scale,
                    self.velocity[1] + delta[1] * scale,
                    self.velocity[2] + delta[2] * scale)
        if speed == 0 and desired[2] == 0.0 and math.hypot(*velocity[:2]) < 0.02:
            # Nothing is being commanded: settle rather than leave an
            # exponential tail as a standing micro-command on either axis.
            velocity = (0.0, 0.0, velocity[2] if abs(velocity[2]) >= 0.02 else 0.0)
        if avoiding:
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
        velocity = self._respect_minimum_separation(
            velocity,
            position=position,
            forward=(forward_x, forward_y),
            estimated_range=estimated_range if range_is_valid else None,
            target_state=target_state,
            dt=dt,
        )
        self.velocity = velocity
        return velocity

    def _respect_minimum_separation(
        self,
        velocity: tuple[float, float, float],
        *,
        position: tuple[float, float, float],
        forward: tuple[float, float],
        estimated_range: float | None,
        target_state: TargetState | None,
        dt: float,
    ) -> tuple[float, float, float]:
        """Refuse to close inside the separation floor, whatever asked for it.

        Every other limit here is a control law that can be wrong: the station
        point is only as good as the world estimate behind it, the dodge needs
        a range that a single bad box removes, and the closure cap is gated on
        the same range. This is none of those. It takes the best estimate of
        where the target is, and removes the part of the commanded velocity
        that would close inside the floor this tick -- leaving everything
        crossways untouched, so the aircraft keeps manoeuvring rather than
        stopping dead.

        The aircraft is flown by a supervisor writing poses directly, so it
        does not bounce off anything: driving it into another airframe is an
        interpenetration the physics engine has to resolve, and the whole
        world pays for that, not just this aircraft.
        """
        cfg = self.config
        if dt <= 0.0:
            return velocity

        if target_state is not None:
            target_x, target_y = target_state.position[0], target_state.position[1]
            target_vx, target_vy = target_state.velocity[0], target_state.velocity[1]
        elif estimated_range is not None:
            target_x = position[0] + forward[0] * estimated_range
            target_y = position[1] + forward[1] * estimated_range
            target_vx = target_vy = 0.0
        else:
            return velocity

        line_x, line_y = target_x - position[0], target_y - position[1]
        separation = math.hypot(line_x, line_y)
        if separation <= 1e-6:
            return velocity
        line_x, line_y = line_x / separation, line_y / separation

        # Relative, because matching a fleeing target's speed closes nothing.
        closing = (velocity[0] - target_vx) * line_x + (velocity[1] - target_vy) * line_y
        if closing <= 0.0:
            return velocity
        allowed = max(0.0, (separation - cfg.minimum_separation_m) / dt)
        if closing <= allowed:
            return velocity

        excess = closing - allowed
        self.constraint = "minimum_separation"
        return (
            velocity[0] - excess * line_x,
            velocity[1] - excess * line_y,
            velocity[2],
        )
