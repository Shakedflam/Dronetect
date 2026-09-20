"""Kinematic autonomous controller for non-Dronetect traffic drones."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys

from controller import Supervisor

CONTROLLERS_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROLLERS_ROOT))
sys.path.insert(0, str(CONTROLLERS_ROOT / "simulation_scenario"))

from scenario_config import load_scenario_config
from sim_actor_profiles import traffic_drone_profile
from sim_actor_paths import (
    CircularPath,
    Position,
    WaypointPath,
    circular_position,
    waypoint_position,
    yaw_toward,
)


CONFIG_PATH = CONTROLLERS_ROOT.parent / "config" / "simulation_scenario.json"
HIDDEN_POSITION: Position = (0.0, 0.0, -80.0)


class SimTrafficDrone(Supervisor):
    def __init__(self) -> None:
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.actor_index, self.profile = _load_actor()
        if (
            self.actor_index == 0
            and os.getenv("DRONTECT_WEBOTS_ALTITUDE_SMOKE", "0") == "1"
        ):
            self.profile = {
                "behavior": "circle",
                "start": [-2.0, 0.0, 14.0],
                "yaw": 1.5708,
                "center": [-2.0, 0.0, 14.0],
                "radius_m": 0.0,
                "altitude_amplitude_m": 0.25,
                "period_s": 120.0,
                "phase_rad": 0.0,
            }
        self.self_node = self.getSelf()
        self.translation = self.self_node.getField("translation")
        self.rotation = self.self_node.getField("rotation")
        if self.translation is None or self.rotation is None:
            raise RuntimeError("Traffic drone requires translation and rotation fields.")
        self.previous_position: Position = _position(self.translation.getSFVec3f())
        self.previous_yaw = float(self.profile.get("yaw", 0.0))

    def run(self) -> None:
        if not self._is_active():
            self.translation.setSFVec3f(list(HIDDEN_POSITION))
            self.self_node.resetPhysics()
            while self.step(self.time_step) != -1:
                pass
            return

        print(
            f"TRAFFIC_DRONE_{self.actor_index + 1} active: "
            f"{self.profile['behavior']} from {self.profile['start']}"
        )
        while self.step(self.time_step) != -1:
            now = self.getTime()
            position = self._position_at(now)
            yaw = yaw_toward(self.previous_position, position, self.previous_yaw)
            self.translation.setSFVec3f(list(position))
            self.rotation.setSFRotation([0.0, 0.0, 1.0, yaw])
            self.self_node.resetPhysics()
            self.previous_position = position
            self.previous_yaw = yaw

    def _is_active(self) -> bool:
        config = load_scenario_config(CONFIG_PATH)
        return self.actor_index < config.traffic_drone_count

    def _position_at(self, elapsed_s: float) -> tuple[float, float, float]:
        behavior = self.profile.get("behavior")
        if behavior in {"patrol", "flythrough"}:
            return waypoint_position(
                WaypointPath(
                    waypoints=tuple(
                        _position(point) for point in self.profile["waypoints"]
                    ),
                    speed_mps=float(self.profile["speed_mps"]),
                    phase_s=float(self.profile.get("phase_s", 0.0)),
                ),
                elapsed_s,
            )
        if behavior == "circle":
            return circular_position(
                CircularPath(
                    center=_position(self.profile["center"]),
                    radius_m=float(self.profile["radius_m"]),
                    altitude_amplitude_m=float(self.profile["altitude_amplitude_m"]),
                    period_s=float(self.profile["period_s"]),
                    phase_rad=float(self.profile.get("phase_rad", 0.0)),
                ),
                elapsed_s,
            )
        return self.previous_position


def _load_actor() -> tuple[int, dict]:
    if len(sys.argv) < 2:
        raise RuntimeError(
            "Traffic drone controller requires an index or JSON profile argument."
        )

    raw = sys.argv[1]
    if raw.lstrip("-").isdigit():
        index = max(0, int(raw))
        return index, traffic_drone_profile(index)

    profile = json.loads(raw)
    return int(profile.get("index", 0)), profile


def _position(values) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


if __name__ == "__main__":
    SimTrafficDrone().run()
