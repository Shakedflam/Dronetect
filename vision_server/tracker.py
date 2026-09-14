"""Lightweight bounding-box tracker for YOLO drone detections."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

from vision_server.detector import Box, Detection


TrackState = Literal["tentative", "confirmed", "lost"]
Velocity = tuple[float, float]

MIN_IOU_MATCH = 0.2
MAX_CENTER_DISTANCE_DIAGONALS = 1.0
MIN_CENTER_DISTANCE_FRAME_DIAGONALS = 0.20
CONFIRMATION_HITS = 2
LOST_AFTER_SECONDS = 0.25
REMOVE_AFTER_SECONDS = 0.80
MAX_VELOCITY_PX_PER_SEC = 5000.0


@dataclass(frozen=True, slots=True)
class TrackedDetection:
    class_name: str
    confidence: float
    box: Box
    track_id: str
    track_state: TrackState
    last_seen_timestamp: float
    velocity_px_per_sec: Velocity


@dataclass(slots=True)
class _Track:
    track_id: str
    class_name: str
    confidence: float
    box: Box
    track_state: TrackState
    last_seen_timestamp: float
    velocity_px_per_sec: Velocity
    hits: int
    consecutive_misses: int

    def to_detection(
        self,
        *,
        timestamp: float,
        frame_width: int,
        frame_height: int,
        prediction_horizon_seconds: float,
    ) -> TrackedDetection:
        return TrackedDetection(
            class_name=self.class_name,
            confidence=self.confidence,
            box=_predict_box(
                self,
                timestamp=timestamp,
                frame_width=frame_width,
                frame_height=frame_height,
                prediction_horizon_seconds=prediction_horizon_seconds,
            ),
            track_id=self.track_id,
            track_state=self.track_state,
            last_seen_timestamp=self.last_seen_timestamp,
            velocity_px_per_sec=self.velocity_px_per_sec,
        )


class DroneTracker:
    """Match raw detector boxes into short-lived persistent drone tracks."""

    def __init__(
        self,
        *,
        min_iou_match: float = MIN_IOU_MATCH,
        max_center_distance_diagonals: float = MAX_CENTER_DISTANCE_DIAGONALS,
        min_center_distance_frame_diagonals: float = MIN_CENTER_DISTANCE_FRAME_DIAGONALS,
        confirmation_hits: int = CONFIRMATION_HITS,
        lost_after_seconds: float = LOST_AFTER_SECONDS,
        remove_after_seconds: float = REMOVE_AFTER_SECONDS,
        max_velocity_px_per_sec: float = MAX_VELOCITY_PX_PER_SEC,
    ) -> None:
        self.min_iou_match = min_iou_match
        self.max_center_distance_diagonals = max_center_distance_diagonals
        self.min_center_distance_frame_diagonals = min_center_distance_frame_diagonals
        self.confirmation_hits = confirmation_hits
        self.lost_after_seconds = max(0.0, lost_after_seconds)
        self.remove_after_seconds = max(remove_after_seconds, self.lost_after_seconds)
        self.max_velocity_px_per_sec = max_velocity_px_per_sec
        self._next_track_number = 1
        self._tracks: list[_Track] = []

    def update(
        self,
        detections: list[Detection],
        *,
        timestamp: float,
        frame_width: int,
        frame_height: int,
    ) -> list[TrackedDetection]:
        timestamp = _valid_timestamp(timestamp)
        unmatched_track_indexes = set(range(len(self._tracks)))
        next_tracks: list[_Track] = []

        for detection in detections:
            track_index = self._best_track_index(
                detection,
                unmatched_track_indexes,
                timestamp=timestamp,
                frame_width=frame_width,
                frame_height=frame_height,
            )
            if track_index is None:
                next_tracks.append(self._new_track(detection, timestamp))
                continue

            unmatched_track_indexes.remove(track_index)
            track = self._tracks[track_index]
            self._update_track(track, detection, timestamp)
            next_tracks.append(track)

        for track_index in unmatched_track_indexes:
            track = self._tracks[track_index]
            track.consecutive_misses += 1
            elapsed = _elapsed_seconds(timestamp, track.last_seen_timestamp)
            if elapsed >= self.remove_after_seconds:
                continue
            if elapsed >= self.lost_after_seconds:
                track.track_state = "lost"
            next_tracks.append(track)

        self._tracks = next_tracks
        return [
            track.to_detection(
                timestamp=timestamp,
                frame_width=frame_width,
                frame_height=frame_height,
                prediction_horizon_seconds=self.remove_after_seconds,
            )
            for track in self._tracks
        ]

    def _best_track_index(
        self,
        detection: Detection,
        candidates: set[int],
        *,
        timestamp: float,
        frame_width: int,
        frame_height: int,
    ) -> int | None:
        best_index: int | None = None
        best_score = float("-inf")

        for track_index in candidates:
            track = self._tracks[track_index]
            predicted_box = _predict_box(
                track,
                timestamp=timestamp,
                frame_width=frame_width,
                frame_height=frame_height,
                prediction_horizon_seconds=self.remove_after_seconds,
            )
            score = _match_score(
                predicted_box,
                detection.box,
                min_iou_match=self.min_iou_match,
                max_center_distance_diagonals=self.max_center_distance_diagonals,
                min_center_distance_frame_diagonals=self.min_center_distance_frame_diagonals,
                frame_width=frame_width,
                frame_height=frame_height,
            )
            if score is not None and score > best_score:
                best_score = score
                best_index = track_index

        return best_index

    def _new_track(self, detection: Detection, timestamp: float) -> _Track:
        track = _Track(
            track_id=f"drone-{self._next_track_number}",
            class_name=detection.class_name,
            confidence=detection.confidence,
            box=detection.box,
            track_state="confirmed" if self.confirmation_hits <= 1 else "tentative",
            last_seen_timestamp=timestamp,
            velocity_px_per_sec=(0.0, 0.0),
            hits=1,
            consecutive_misses=0,
        )
        self._next_track_number += 1
        return track

    def _update_track(
        self,
        track: _Track,
        detection: Detection,
        timestamp: float,
    ) -> None:
        previous_center = _center(track.box)
        next_center = _center(detection.box)
        dt = _elapsed_seconds(timestamp, track.last_seen_timestamp)
        velocity = _bounded_velocity(
            (
                (next_center[0] - previous_center[0]) / dt,
                (next_center[1] - previous_center[1]) / dt,
            )
            if dt > 0.0
            else (0.0, 0.0),
            max_velocity_px_per_sec=self.max_velocity_px_per_sec,
        )

        track.class_name = detection.class_name
        track.confidence = detection.confidence
        track.box = detection.box
        track.last_seen_timestamp = max(timestamp, track.last_seen_timestamp)
        track.velocity_px_per_sec = velocity
        track.hits += 1
        track.consecutive_misses = 0
        track.track_state = "confirmed" if track.hits >= self.confirmation_hits else "tentative"


def _match_score(
    track_box: Box,
    detection_box: Box,
    *,
    min_iou_match: float,
    max_center_distance_diagonals: float,
    min_center_distance_frame_diagonals: float,
    frame_width: int,
    frame_height: int,
) -> float | None:
    overlap = _iou(track_box, detection_box)
    if overlap >= min_iou_match:
        return 2.0 + overlap

    distance = _center_distance(track_box, detection_box)
    max_distance = (
        max(_diagonal(track_box), _diagonal(detection_box))
        * max_center_distance_diagonals
    )
    min_distance = (
        _frame_diagonal(frame_width, frame_height)
        * min_center_distance_frame_diagonals
    )
    max_distance = max(max_distance, min_distance)
    if max_distance > 0.0 and distance <= max_distance:
        return 1.0 - distance / max_distance

    return None


def _iou(a: Box, b: Box) -> float:
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    intersection = _area((x1, y1, x2, y2))
    if intersection <= 0.0:
        return 0.0

    union = _area(a) + _area(b) - intersection
    return intersection / union if union > 0.0 else 0.0


def _area(box: Box) -> float:
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def _center(box: Box) -> tuple[float, float]:
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


def _center_distance(a: Box, b: Box) -> float:
    ac = _center(a)
    bc = _center(b)
    return ((bc[0] - ac[0]) ** 2 + (bc[1] - ac[1]) ** 2) ** 0.5


def _diagonal(box: Box) -> float:
    return ((box[2] - box[0]) ** 2 + (box[3] - box[1]) ** 2) ** 0.5


def _frame_diagonal(frame_width: int, frame_height: int) -> float:
    if frame_width <= 0 or frame_height <= 0:
        return 0.0
    return (frame_width**2 + frame_height**2) ** 0.5


def _predict_box(
    track: _Track,
    *,
    timestamp: float,
    frame_width: int,
    frame_height: int,
    prediction_horizon_seconds: float,
) -> Box:
    elapsed = min(
        _elapsed_seconds(timestamp, track.last_seen_timestamp),
        max(0.0, prediction_horizon_seconds),
    )
    dx = track.velocity_px_per_sec[0] * elapsed
    dy = track.velocity_px_per_sec[1] * elapsed
    predicted = (
        track.box[0] + dx,
        track.box[1] + dy,
        track.box[2] + dx,
        track.box[3] + dy,
    )
    return _clamp_box(predicted, frame_width=frame_width, frame_height=frame_height)


def _clamp_box(box: Box, *, frame_width: int, frame_height: int) -> Box:
    width = box[2] - box[0]
    height = box[3] - box[1]
    if width <= 0.0 or height <= 0.0 or frame_width <= 0 or frame_height <= 0:
        return box

    max_x1 = max(0.0, frame_width - width)
    max_y1 = max(0.0, frame_height - height)
    x1 = min(max(0.0, box[0]), max_x1)
    y1 = min(max(0.0, box[1]), max_y1)
    return (x1, y1, x1 + width, y1 + height)


def _elapsed_seconds(timestamp: float, previous_timestamp: float) -> float:
    if not math.isfinite(timestamp) or not math.isfinite(previous_timestamp):
        return 0.0
    return max(0.0, timestamp - previous_timestamp)


def _bounded_velocity(
    velocity: Velocity,
    *,
    max_velocity_px_per_sec: float,
) -> Velocity:
    vx, vy = velocity
    if not math.isfinite(vx) or not math.isfinite(vy):
        return (0.0, 0.0)

    speed = (vx**2 + vy**2) ** 0.5
    if speed <= max_velocity_px_per_sec or speed == 0.0:
        return velocity

    scale = max_velocity_px_per_sec / speed
    return (vx * scale, vy * scale)


def _valid_timestamp(timestamp: float) -> float:
    return timestamp if math.isfinite(timestamp) else 0.0
