# Webots Autonomous Drone PoC

A safe, software-only autonomous quadcopter demonstration for Webots R2025a.
The simulated drone flies through Webots' official rural 3D scene, visits one
random waypoint, then returns home and lands automatically. Pressing `H`
returns early, while `N` starts a new mission from any state. There is no real drone connection,
physical GPS, computer vision, or external autopilot stack.

## What is included

- `worlds/drone_poc.wbt` — a small Webots world containing a ground plane and
  the simulated quadcopter, rural scenery, and navigation markers.
- `controllers/autonomous_drone/autonomous_drone.py` — the Webots sensor,
  keyboard, stabilization, and motor adapter.
- `controllers/autonomous_drone/flight_logic.py` — the pure-Python flight state
  machine and waypoint logic.
- `controllers/autonomous_drone/hud.py` — telemetry calculations and the
  text shown in the top-right Webots overlay.
- `controllers/autonomous_drone/world_markers.py` — home and target marker
  positioning in the Webots world.
- `tests/test_flight_logic.py` — deterministic tests that do not require Webots
  to be running.
- `tests/test_hud.py` — HUD formatting and telemetry math tests.

The controller states are:

```text
IDLE -> TAKING_OFF -> FLYING_RANDOMLY
                          |
                          v
                  RETURNING_HOME -> LANDING -> LANDED
```

## Inspect the official example

The installed reference world is:

```text
C:\Program Files\Webots\projects\robots\dji\mavic\worlds\mavic_2_pro.wbt
```

In Webots, use **File > Open Sample World** and browse to
`robots > dji > mavic > mavic_2_pro.wbt`. Alternatively, use
**File > Open World** and paste the path above.

Do not edit the installed example. This repository owns its world and
controller. The Mavic model is used only as a convenient generic simulated
quadcopter.

## Python setup

From the repository root, install the test dependency into the existing
virtual environment:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

In **Webots > Tools > Preferences > General**, set **Python command** to:

```text
C:\Users\User\Desktop\Drontect\.venv\Scripts\python.exe
```

The Webots `controller` module is provided by Webots when it starts the
controller; it is not a package that should be installed with pip. The
repository's `pyrightconfig.json` points Pylance at the R2025a controller API
so VS Code can resolve the import as well. If the warning remains after opening
the project, run **Developer: Reload Window** from the VS Code command palette.

## Run the simulation

1. Open Webots R2025a.
2. Choose **File > Open World**.
3. Open this repository's `worlds\drone_poc.wbt`.
4. Press **Run**.
5. Watch the console for state and target changes.
6. Click the 3D view and press `H` to cancel random flight and return home.
7. Press `N` at any time to generate a new target and start a new mission.
8. Press `U` to tilt the onboard camera upward, or `J` to tilt it downward.
9. A small telemetry table appears in the top-right of the 3D view with the
   current state, target, current position, home, speed, height, distance,
   angle-to-target, and roll/pitch/yaw tilt values.
10. Right-click the drone itself in the 3D view or Scene Tree and select
   **Robot window** to reopen the small native camera/device window. This
   project deliberately uses Webots' native `generic_window` viewer instead
   of opening the HTML Robot Window in an external browser.
11. Use **Simulation > Reset Simulation**, then press **Run**, to restart.

After editing either Python file, save it and reset the simulation so Webots
starts a fresh controller process.

## Expected behavior

1. The drone waits at home for one second.
2. It rises vertically to 15 metres so it clears the rural scenery.
3. It selects one random target over the manor/road area, within X `-60..-30`,
   Y `5..20`, and Z `13..18` metres.
4. During horizontal flight it uses a modest forward-speed gain so it travels
   faster without simply adding vertical motor power.
5. Reaching that target automatically starts Return Home.
6. Pressing `H` replaces the active target with home early.
7. Above home, it descends gradually, stops its motors, and enters `LANDED`.
8. Pressing `N` after landing takes off again without changing the original
   blue HOME location.

The blue pad marks the permanent home coordinate. The glowing red sphere marks
the current airborne target, and the red ground disc shows its X/Y projection.
The top-right telemetry HUD is drawn by the Python controller directly over
the main Webots 3D view, so it does not depend on the robot camera window.

The `H` command is accepted during takeoff or random flight. `N` works from
every state and safely climbs before departing when the drone is too low.
The `U` and `J` keys move only the camera pitch, so the camera can look more
up or down while its left/right direction still follows the drone body.

## Run tests

```powershell
.\.venv\Scripts\python.exe -m pytest -v
```

## Attribution

The motor stabilization approach and empirical Mavic constants are adapted
from Cyberbotics' Webots R2025a `mavic2pro_patrol.py` example, licensed under
the Apache License 2.0. The world references the official R2025a Webots PROTO
assets rather than copying model meshes or textures into this repository.
