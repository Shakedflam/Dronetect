"""Small camera-gimbal state helper for keyboard control."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class CameraGimbal:
    pitch: float = 0.7
    minimum_pitch: float = -1.4
    maximum_pitch: float = 1.0
    step: float = 0.03

    def tilt_down(self) -> float:
        self.pitch = min(self.maximum_pitch, self.pitch + self.step)
        return self.pitch

    def tilt_up(self) -> float:
        self.pitch = max(self.minimum_pitch, self.pitch - self.step)
        return self.pitch
