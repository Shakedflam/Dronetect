"""Gate raw tracker detections into "actionable" ones before flight logic.

The tablet always receives the raw vision stream; this filter only decides
which detections may *start* a target interruption (TARGET_ACQUISITION and,
from there, follow/pursuit). A track becomes actionable once it has passed
every per-frame gate for `required_frames` consecutive vision frames. One
missed or failed frame is tolerated (`max_missed_frames`) without resetting
the streak, and an already-actionable track is held to a lower confidence
bar (`keep_confidence`) so a brief dip does not drop it.

One detection message is one vision frame: the vision server publishes a
message for every processed frame, including frames with no detections, so
a track absent from a message counts as a miss.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Callable


@dataclass(frozen=True, slots=True)
class ActionableDetectionConfig:
    min_confidence: float = 0.6
    # Hysteresis: a track that is already actionable only has to stay above this.
    keep_confidence: float = 0.5
    required_frames: int = 3
    # Grace: this many consecutive missed/failed frames keep the streak alive.
    max_missed_frames: int = 1
    # ~10x10 px on the 960x540 vision frame; below that YOLO is mostly noise.
    min_box_side_px: float = 6.0
    min_box_area_ratio: float = 0.0002
    # A box covering most of the frame is not a plausible distant drone.
    max_box_area_ratio: float = 0.6
    max_aspect_ratio: float = 5.0


@dataclass(slots=True)
class _TrackStreak:
    hits: int = 0
    misses: int = 0
    actionable: bool = False
    last_confidence: float = 0.0
    last_rejection: str | None = None


@dataclass(frozen=True, slots=True)
class FilterResult:
    actionable: list[dict[str, Any]]
    newly_accepted: tuple[str, ...] = ()


@dataclass(slots=True)
class ActionableDetectionFilter:
    config: ActionableDetectionConfig = field(default_factory=ActionableDetectionConfig)
    log: Callable[[str], None] = print
    _tracks: dict[str, _TrackStreak] = field(default_factory=dict)

    def reset(self) -> None:
        self._tracks.clear()

    def is_actionable(self, track_id: str) -> bool:
        streak = self._tracks.get(track_id)
        return streak is not None and streak.actionable

    def update(
        self,
        detections: list[dict[str, Any]],
        *,
        frame_width: int,
        frame_height: int,
        passthrough_track_id: str | None = None,
    ) -> FilterResult:
        """Advance one vision frame; return the detections flight logic may see.

        `passthrough_track_id` is the target flight logic is already tracking:
        its frames are forwarded unfiltered so an established chase keeps its
        own loss/re-acquisition handling. The filter gates entry, not pursuit.
        """
        actionable: list[dict[str, Any]] = []
        newly_accepted: list[str] = []
        seen: set[str] = set()
        cfg = self.config

        for detection in detections:
            if not isinstance(detection, dict):
                continue
            track_id = detection.get("trackId")
            if not isinstance(track_id, str) or not track_id:
                continue
            if track_id == passthrough_track_id:
                actionable.append(detection)
            if track_id in seen:
                continue
            seen.add(track_id)
            streak = self._tracks.setdefault(track_id, _TrackStreak())
            confidence = _number(detection.get("confidence"))
            threshold = cfg.keep_confidence if streak.actionable else cfg.min_confidence
            reason = _rejection_reason(detection, confidence, threshold, frame_width, frame_height, cfg)
            if reason is None:
                streak.hits += 1
                streak.misses = 0
                streak.last_confidence = confidence
                streak.last_rejection = None
                if not streak.actionable and streak.hits >= cfg.required_frames:
                    streak.actionable = True
                    newly_accepted.append(track_id)
                    self.log(
                        f"detection=accepted track={track_id} conf={confidence:.2f} "
                        f"frames={streak.hits}/{cfg.required_frames} box={_box_text(detection)}"
                    )
                if streak.actionable and track_id != passthrough_track_id:
                    actionable.append(detection)
            else:
                if reason != streak.last_rejection:
                    streak.last_rejection = reason
                    self.log(
                        f"detection=rejected track={track_id} reason={reason} "
                        f"conf={confidence:.2f} frames={streak.hits}/{cfg.required_frames} "
                        f"box={_box_text(detection)}"
                    )
                self._miss(track_id, streak, present=True)

        for track_id in [tid for tid in self._tracks if tid not in seen]:
            self._miss(track_id, self._tracks[track_id], present=False)

        return FilterResult(actionable=actionable, newly_accepted=tuple(newly_accepted))

    def _miss(self, track_id: str, streak: _TrackStreak, *, present: bool) -> None:
        """A frame where the track was absent (or present but failed a gate)."""
        streak.misses += 1
        if streak.misses <= self.config.max_missed_frames:
            return
        if streak.actionable:
            self.log(f"detection=released track={track_id} reason=missed_frames "
                     f"misses={streak.misses} lastConf={streak.last_confidence:.2f}")
        elif streak.hits > 0:
            # The one-frame false positive ends here: it never reached the bar.
            self.log(f"detection=rejected track={track_id} reason=not_persistent "
                     f"frames={streak.hits}/{self.config.required_frames} "
                     f"lastConf={streak.last_confidence:.2f}")
        if present:
            # Still in view but failing a gate: restart the streak but keep the
            # record, so the same rejection reason is not re-logged every frame.
            streak.hits = 0
            streak.misses = 0
            streak.actionable = False
        else:
            del self._tracks[track_id]


def _rejection_reason(
    detection: dict[str, Any], confidence: float, threshold: float,
    frame_width: int, frame_height: int, cfg: ActionableDetectionConfig,
) -> str | None:
    if detection.get("trackState") == "lost":
        return "track_lost"
    if not math.isfinite(confidence) or confidence < threshold:
        return "low_confidence"
    box = detection.get("box")
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        return "box_invalid"
    x1, y1, x2, y2 = (_number(value) for value in box)
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        return "box_invalid"
    width, height = x2 - x1, y2 - y1
    if width <= 0.0 or height <= 0.0 or frame_width <= 0 or frame_height <= 0:
        return "box_invalid"
    center_x, center_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    if not (0.0 <= center_x <= frame_width and 0.0 <= center_y <= frame_height):
        return "box_outside_frame"
    if min(width, height) < cfg.min_box_side_px:
        return "box_too_small"
    area_ratio = (width * height) / float(frame_width * frame_height)
    if area_ratio < cfg.min_box_area_ratio:
        return "box_too_small"
    if area_ratio > cfg.max_box_area_ratio:
        return "box_too_large"
    if max(width, height) / min(width, height) > cfg.max_aspect_ratio:
        return "box_implausible_aspect"
    return None


def _number(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def _box_text(detection: dict[str, Any]) -> str:
    box = detection.get("box")
    if isinstance(box, (list, tuple)) and len(box) == 4:
        try:
            return "[" + ",".join(f"{float(v):.0f}" for v in box) + "]"
        except (TypeError, ValueError):
            pass
    return "invalid"
