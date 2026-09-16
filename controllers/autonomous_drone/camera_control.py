"""Small camera-gimbal state helper for keyboard control."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class CameraGimbal:
    pitch: float = 0.7
    minimum_pitch: float = -0.5
    maximum_pitch: float = 1.7
    step: float = 0.03

    def set_pitch(self, pitch: float) -> float:
        self.pitch = min(max(pitch, self.minimum_pitch), self.maximum_pitch)
        return self.pitch

    def tilt_down(self) -> float:
        return self.set_pitch(self.pitch + self.step)

    def tilt_up(self) -> float:
        return self.set_pitch(self.pitch - self.step)
