"""Kinematic autonomous controller for airborne bird distractors."""

from __future__ import annotations

import json
from pathlib import Path
import sys

from controller import Supervisor

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_actor_paths import BirdPath, Position, bird_position, yaw_toward


class SimBird(Supervisor):
    def __init__(self) -> None:
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.profile = _load_profile()
        self.self_node = self.getSelf()
        self.translation = self.self_node.getField("translation")
        self.rotation = self.self_node.getField("rotation")
        if self.translation is None or self.rotation is None:
            raise RuntimeError("Bird requires translation and rotation fields.")
        self.previous_position: Position = _position(self.translation.getSFVec3f())
        self.previous_yaw = float(self.profile.get("yaw", 0.0))

    def run(self) -> None:
        while self.step(self.time_step) != -1:
            now = self.getTime()
            position = bird_position(
                BirdPath(
                    center=_position(self.profile["center"]),
                    radius_x_m=float(self.profile["radius_x_m"]),
                    radius_y_m=float(self.profile["radius_y_m"]),
                    altitude_amplitude_m=float(self.profile["altitude_amplitude_m"]),
                    period_s=float(self.profile["period_s"]),
                    phase_rad=float(self.profile.get("phase_rad", 0.0)),
                ),
                now,
            )
            yaw = yaw_toward(self.previous_position, position, self.previous_yaw)
            self.translation.setSFVec3f(list(position))
            self.rotation.setSFRotation([0.0, 0.0, 1.0, yaw])
            self.self_node.resetPhysics()
            self.previous_position = position
            self.previous_yaw = yaw


def _load_profile() -> dict:
    if len(sys.argv) < 2:
        raise RuntimeError("Bird controller requires a JSON profile argument.")
    return json.loads(sys.argv[1])


def _position(values) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


if __name__ == "__main__":
    SimBird().run()
