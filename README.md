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
- `gateway/` — the local Edge Gateway WebSocket server and documented message
  protocol used between drone adapters and tablet clients.
- `adapters/simulator/` — the Webots simulator adapter that publishes
  telemetry and receives gateway commands.
- `services/` — shared backend helpers for GPS conversion, telemetry payloads,
  and supported command names.
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

After editing controller Python files, save them and reset the simulation so
Webots starts fresh controller processes.

## Run the edge gateway

Start the local gateway from the repository root:

```powershell
.\.venv\Scripts\python.exe -m gateway.server
```

By default it listens on:

```text
ws://127.0.0.1:8765/ws
```

Override the bind address with `DRONTECT_GATEWAY_HOST` and
`DRONTECT_GATEWAY_PORT`. The simulator adapter can override its outbound URL
with `DRONTECT_SIMULATOR_WS_URL`.

Use the same `/ws` endpoint with a required role query parameter:

```text
ws://127.0.0.1:8765/ws?role=simulator
ws://127.0.0.1:8765/ws?role=tablet
```

Only one simulator may be connected at a time. Multiple tablet clients may
connect and receive telemetry.

Simulator telemetry is broadcast to all connected tablet clients:

```json
{
  "type": "telemetry",
  "timestamp": 1720000000.123,
  "source": "simulator",
  "data": {
    "gps": { "lat": 32.0853548, "lng": 34.7816696 },
    "altitudeM": 14.8,
    "headingDeg": 91.4,
    "speedMps": 4.2,
    "batteryPercent": 100,
    "state": "FLYING_RANDOMLY"
  }
}
```

The simulator keeps Webots local meter coordinates internally. Before
telemetry is sent, local `x` is converted to longitude, local `y` is converted
to latitude, and local `z` is sent only as altitude. The WGS84 home point is
`32.085300, 34.781800`.

Tablet clients send commands through the gateway:

```json
{
  "type": "command",
  "commandId": "client-generated-id",
  "command": "return_home",
  "params": {}
}
```

Supported commands are `return_home`, `new_mission`, `camera_tilt_up`, and
`camera_tilt_down`. Accepted commands receive:

```json
{
  "type": "command_ack",
  "commandId": "client-generated-id",
  "status": "accepted"
}
```

Invalid messages receive structured errors with `type`, `code`, and `message`.
Video streaming is not implemented yet; future WebRTC signaling and media relay
belong under `gateway/webrtc/` while preserving this command and telemetry
contract.

## Configure background actors

The simulation includes autonomous non-Dronetect drones and birds so future
camera-based detection can be tested against moving distractors. They are
spawned by a separate scenario supervisor and use their own controllers; the
Dronetect drone controller is not given their positions or types.

Before starting or resetting Webots, edit:

```text
config\simulation_scenario.json
```

The default scenario starts three traffic drones and six birds already airborne:

```json
{
  "traffic_drones": {
    "count": 3
  },
  "birds": {
    "count": 6
  }
}
```

Negative counts are treated as zero, and missing values fall back to the
defaults above. Traffic drones use a pre-placed pool of 12 same-model Mavic2Pro
nodes for reliable Webots visibility, so values above 12 require adding more
pool nodes to the world. The main Dronetect drone still starts from the ground.

## Expected behavior

1. The drone waits at home for one second.
2. It rises quickly to 15 metres with DJI Mini 4 Pro-inspired climb authority
   so it clears the rural scenery.
3. It selects one random target over the manor/road area, within X `-60..-30`,
   Y `5..20`, and Z `13..18` metres.
4. During horizontal flight it uses a DJI Mini 4 Pro-inspired high-level
   kinematic flight profile with bounded horizontal speed, acceleration, climb,
   descent, and yaw rate instead of relying on raw motor-gain tuning.
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
