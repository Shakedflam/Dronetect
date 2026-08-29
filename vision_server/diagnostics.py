"""Basic diagnostics for received vision frames."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import time

from services.video import VideoFrame


@dataclass(slots=True)
class VisionDiagnostics:
    received_count: int = 0
    latest_frame_id: int | None = None
    _received_times: deque[float] = field(default_factory=lambda: deque(maxlen=120))

    def record(self, frame: VideoFrame) -> None:
        self.received_count += 1
        self.latest_frame_id = frame.frame_id
        self._received_times.append(time.monotonic())

    @property
    def fps(self) -> float:
        if len(self._received_times) < 2:
            return 0.0
        elapsed = self._received_times[-1] - self._received_times[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self._received_times) - 1) / elapsed

    def to_dict(self) -> dict:
        return {
            "received_count": self.received_count,
            "latest_frame_id": self.latest_frame_id,
            "fps": self.fps,
        }
