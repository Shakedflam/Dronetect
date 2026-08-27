"""Spawn configurable background actors for the Dronetect Webots scenario."""

from __future__ import annotations

import json
from pathlib import Path

from controller import Supervisor

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_actor_profiles import bird_profile
from scenario_config import ScenarioConfig, load_scenario_config


REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = REPO_ROOT / "config" / "simulation_scenario.json"
MAX_CLEANUP_ACTORS = 200
STATIC_TRAFFIC_DRONE_POOL = 12


class SimulationScenario(Supervisor):
    def __init__(self) -> None:
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.root_children = self.getRoot().getField("children")
        if self.root_children is None:
            raise RuntimeError("World root children field was not found.")

    def run(self) -> None:
        config = load_scenario_config(CONFIG_PATH)
        self._remove_previous_dynamic_birds()
        self._spawn_birds(config)
        print(
            "Simulation scenario loaded: "
            f"{config.traffic_drone_count} traffic drones, {config.bird_count} birds."
        )
        print(
            "Traffic drones use the pre-placed Mavic2Pro pool: "
            f"showing {min(config.traffic_drone_count, STATIC_TRAFFIC_DRONE_POOL)} "
            f"of {STATIC_TRAFFIC_DRONE_POOL}."
        )

        while self.step(self.time_step) != -1:
            pass

    def _remove_previous_dynamic_birds(self) -> None:
        for index in range(1, MAX_CLEANUP_ACTORS + 1):
            node = self.getFromDef(f"BIRD_{index}")
            if node is not None:
                node.remove()

    def _spawn_birds(self, config: ScenarioConfig) -> None:
        for index in range(config.bird_count):
            profile = bird_profile(index)
            start = profile["start"]
            scale = profile["scale"]
            node = f"""
DEF BIRD_{index + 1} Robot {{
  translation {start[0]:.3f} {start[1]:.3f} {start[2]:.3f}
  rotation 0 0 1 {profile["yaw"]:.6f}
  name "Bird {index + 1}"
  controller "sim_bird"
  supervisor TRUE
  controllerArgs [
    "{_escape_controller_arg(profile)}"
  ]
  children [
    Pose {{
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.08 0.08 0.075
            roughness 0.8
          }}
          geometry Box {{
            size {0.30 * scale:.3f} {0.07 * scale:.3f} {0.08 * scale:.3f}
          }}
        }}
        Pose {{
          translation 0 {-0.22 * scale:.3f} 0
          rotation 0 0 1 -0.22
          children [
            Shape {{
              appearance PBRAppearance {{
                baseColor 0.03 0.03 0.03
                roughness 0.85
              }}
              geometry Box {{
                size {0.18 * scale:.3f} {0.42 * scale:.3f} {0.018 * scale:.3f}
              }}
            }}
          ]
        }}
        Pose {{
          translation 0 {0.22 * scale:.3f} 0
          rotation 0 0 1 0.22
          children [
            Shape {{
              appearance PBRAppearance {{
                baseColor 0.03 0.03 0.03
                roughness 0.85
              }}
              geometry Box {{
                size {0.18 * scale:.3f} {0.42 * scale:.3f} {0.018 * scale:.3f}
              }}
            }}
          ]
        }}
      ]
    }}
  ]
  boundingObject Box {{
    size {0.40 * scale:.3f} {0.55 * scale:.3f} {0.12 * scale:.3f}
  }}
}}
"""
            self.root_children.importMFNodeFromString(-1, node)
            print(f"Spawned BIRD_{index + 1}: orbit at {start}")

def _escape_controller_arg(profile: dict) -> str:
    return (
        json.dumps(profile, separators=(",", ":"))
        .replace("\\", "\\\\")
        .replace('"', '\\"')
    )


if __name__ == "__main__":
    SimulationScenario().run()
