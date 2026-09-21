"""Conservative acquisition evidence and short-lived rejection memory.

Ground projections are approximate locations, never measured target GPS.
All deadlines use the flight controller's simulation clock.
"""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import math
from typing import TYPE_CHECKING, Any, Callable

if TYPE_CHECKING:
    from .flight_logic import FlightConfig

Position = tuple[float, float, float]
Point2 = tuple[float, float]
Detection = dict[str, Any]


@dataclass
class CandidateMemory:
    track_id: str
    point: tuple[float, float] | None
    last_frame_at: float
    missing_at: float | None = None
    timestamp: float | None = None


@dataclass
class IgnoredTarget:
    track_id: str
    point: tuple[float, float] | None
    expires_at: float


def ground_projection(box, width, height, position, yaw, camera_pitch, fov,
                      ground_height, min_angle, max_range) -> Point2 | None:
    """Pinhole ray: forward +X, image-right -Y, positive pitch downward."""
    if (not all(math.isfinite(v) for v in (*box, width, height, *position, yaw, camera_pitch, fov, ground_height))
            or width <= 0 or height <= 0 or not 0 < fov < math.pi):
        return None
    cx = (float(box[0]) + float(box[2])) / (2 * width)
    cy = (float(box[1]) + float(box[3])) / (2 * height)
    right = (2 * cx - 1) * math.tan(fov / 2)
    down = (2 * cy - 1) * math.tan(fov / 2) * height / width
    forward = math.cos(camera_pitch) - down * math.sin(camera_pitch)
    vertical = -math.sin(camera_pitch) - down * math.cos(camera_pitch)
    horizontal = math.hypot(forward, right)
    altitude = position[2] - ground_height
    if not all(math.isfinite(v) for v in (*position, yaw, vertical, horizontal)):
        return None
    if altitude <= 0 or vertical >= 0 or math.atan2(-vertical, horizontal) < min_angle:
        return None
    scale = altitude / -vertical
    if horizontal * scale > max_range:
        return None
    return (position[0] + scale * (forward * math.cos(yaw) + right * math.sin(yaw)),
            position[1] + scale * (forward * math.sin(yaw) - right * math.cos(yaw)))


def range_estimated_projection(box, width, height, position, yaw, camera_pitch, fov,
                               reference_distance_m, reference_area_ratio,
                               max_range) -> Point2 | None:
    """Locate a target by calibrated box area instead of a ground intersection.

    A ground fix needs the camera ray to actually reach the ground, which it
    never does for a target at or above the aircraft's own height -- the usual
    case for another drone. Box area still gives a usable range there. It
    assumes the target size that calibrated ``reference_area_ratio``, so this
    is an approximation: good enough to tell "the thing just rejected" from
    "something else", not a survey measurement.
    """
    if (not all(math.isfinite(v) for v in (*box, width, height, *position, yaw, camera_pitch, fov))
            or width <= 0 or height <= 0 or not 0 < fov < math.pi
            or reference_distance_m <= 0 or reference_area_ratio <= 0 or max_range <= 0):
        return None
    area_ratio = (
        max(0.0, float(box[2]) - float(box[0]))
        * max(0.0, float(box[3]) - float(box[1]))
        / (width * height)
    )
    if area_ratio <= 1e-9:
        return None
    slant_range = reference_distance_m * math.sqrt(reference_area_ratio / area_ratio)
    if not math.isfinite(slant_range) or slant_range > max_range:
        return None
    cx = (float(box[0]) + float(box[2])) / (2 * width)
    cy = (float(box[1]) + float(box[3])) / (2 * height)
    right = (2 * cx - 1) * math.tan(fov / 2)
    down = (2 * cy - 1) * math.tan(fov / 2) * height / width
    forward = math.cos(camera_pitch) - down * math.sin(camera_pitch)
    vertical = -math.sin(camera_pitch) - down * math.cos(camera_pitch)
    ray = math.sqrt(forward * forward + right * right + vertical * vertical)
    horizontal = math.hypot(forward, right)
    if ray <= 1e-9 or not math.isfinite(ray):
        return None
    if horizontal <= 1e-9:
        return (position[0], position[1])
    horizontal_range = horizontal / ray * slant_range
    unit_forward, unit_right = forward / horizontal, right / horizontal
    return (position[0] + horizontal_range * (unit_forward * math.cos(yaw) + unit_right * math.sin(yaw)),
            position[1] + horizontal_range * (unit_forward * math.sin(yaw) - unit_right * math.cos(yaw)))


class TargetDecisions:
    def __init__(self, config: FlightConfig):
        self.config = config
        self.memories: dict[str, CandidateMemory] = {}
        self.ignored: list[IgnoredTarget] = []
        self.last_rejection_reason: str | None = None
        # Set only when an operator rejects a target that could not be located
        # at all. Without a place to remember, the rejection would survive
        # only as a track id, and the tracker re-identifying the same object
        # seconds later would put the aircraft straight back onto it.
        self.suppress_until: float | None = None
        self.reset_observation()

    def reset_observation(self) -> None:
        self.started_at: float | None = None
        self.samples: deque[tuple[float, float, tuple[float, float]]] = deque(maxlen=256)
        self.evaluated = False
        self.sample_timestamp: float | None = None
        self.origin: Position | None = None
        self.last_pose: Point2 | None = None
        self.displacement = self.yaw_travel = self.pitch_travel = 0.0

    def start(self, now: float, position: Position, yaw: float, pitch: float) -> None:
        self.reset_observation()
        self.started_at = now
        self.origin = position
        self.record_pose(position, yaw, pitch)

    def record_pose(self, position: Position, yaw: float, pitch: float) -> None:
        if self.started_at is None or self.origin is None:
            return
        self.displacement = max(self.displacement, math.dist(position, self.origin))
        if self.last_pose is not None:
            previous_yaw, previous_pitch = self.last_pose
            self.yaw_travel += abs((yaw - previous_yaw + math.pi) % (2 * math.pi) - math.pi)
            self.pitch_travel += abs(pitch - previous_pitch)
        self.last_pose = (yaw, pitch)

    def ready(self, now: float) -> bool:
        return self.started_at is not None and now - self.started_at + 1e-9 >= self.config.target_observation_s

    def prune(self, now: float) -> None:
        self.ignored[:] = [item for item in self.ignored if now < item.expires_at]
        if self.suppress_until is not None and now >= self.suppress_until:
            self.suppress_until = None
        for key, memory in list(self.memories.items()):
            # A stalled stream must not keep operator actions alive indefinitely.
            if memory.missing_at is None and now - memory.last_frame_at > self.config.target_acquisition_timeout_s:
                memory.missing_at = memory.last_frame_at + self.config.target_acquisition_timeout_s
            if memory.missing_at is not None and now - memory.missing_at > self.config.target_reject_grace_s:
                del self.memories[key]

    def update_visibility(self, detections: list[Detection], now: float,
                          project: Callable[[Detection], Point2 | None]) -> None:
        self.prune(now)
        visible = {d['trackId']: d for d in detections if d.get('trackState') != 'lost'}
        for key, memory in self.memories.items():
            detection = visible.get(key)
            if detection is None:
                if memory.missing_at is None:
                    memory.missing_at = now
            else:
                memory.last_frame_at = now
                memory.missing_at = None
                stamp = detection.get('lastSeenTimestamp')
                if stamp != memory.timestamp:
                    memory.point = project(detection)
                    memory.timestamp = stamp

    def remember(self, detection: Detection, now: float, point: Point2 | None) -> None:
        key = detection['trackId']
        memory = self.memories.get(key)
        if memory is None:
            self.memories[key] = CandidateMemory(key, point, now, timestamp=detection.get('lastSeenTimestamp'))

    def sample(self, detection: Detection, now: float, center: Point2) -> bool:
        stamp = detection.get('lastSeenTimestamp')
        # No timestamp is supported only by legacy local callers; production
        # protocol requires it. Lost/repeated timestamps are never evidence.
        stamp = now if stamp is None else float(stamp)
        if not math.isfinite(stamp):
            return False
        if detection.get('trackState') == 'lost' or (self.sample_timestamp is not None and stamp <= self.sample_timestamp):
            return False
        self.sample_timestamp = stamp
        if not self.evaluated:
            self.samples.append((now, float(detection['confidence']), center))
        return True

    def should_reject(self) -> bool:
        cfg = self.config
        if len(self.samples) < cfg.target_observation_min_samples:
            return False
        if self.samples[-1][0] - self.samples[0][0] + 1e-9 < cfg.target_observation_s:
            return False
        if (self.displacement > cfg.target_stationary_observer_m
                or self.yaw_travel > cfg.target_stationary_angle_rad
                or self.pitch_travel > cfg.target_stationary_angle_rad):
            return False
        for axis in (0, 1):
            values = [sample[2][axis] for sample in self.samples]
            if max(values) - min(values) > cfg.target_stationary_center_ratio:
                return False
        scores = [sample[1] for sample in self.samples]
        return (sum(scores) / len(scores) < cfg.target_low_confidence
                or (max(scores) - min(scores) >= cfg.target_confidence_range
                    and min(scores) < cfg.target_low_confidence))

    def is_ignored(self, track_id: str, point: Point2 | None, now: float) -> bool:
        self.prune(now)
        if self.suppress_until is not None:
            return True
        return any(item.track_id == track_id or (
            point is not None and item.point is not None
            and math.dist(point, item.point) <= self.config.target_ignore_radius_m
        ) for item in self.ignored)

    def reject(
        self,
        track_id: str,
        now: float,
        *,
        suppress_all_if_unlocatable: bool = False,
    ) -> bool:
        self.prune(now)
        if any(item.track_id == track_id for item in self.ignored):
            return False
        memory = self.memories.get(track_id)
        if memory is None:
            return False
        expires_at = now + self.config.target_ignore_timeout_s
        self.ignored.append(IgnoredTarget(track_id, memory.point, expires_at))
        if suppress_all_if_unlocatable and memory.point is None:
            self.suppress_until = expires_at
        self.memories.pop(track_id)
        return True
