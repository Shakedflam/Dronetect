# Webots Autonomous Drone PoC

A safe, software-only autonomous quadcopter demonstration for Webots R2025a.
The simulated drone supports continuous priority-weighted patrol, ROI
investigation, visual target pursuit, mission resume, Return Home, and landing
in Webots' official rural 3D scene. Pressing `H` returns home, while `N` starts
the legacy random-waypoint mission. There is no real drone connection, physical
GPS, or external autopilot stack; target detection and pursuit use the
repository's simulated camera/vision/realtime pipeline.

## What is included

- `worlds/drone_poc.wbt` — a small Webots world containing a ground plane and
  the simulated quadcopter, rural scenery, and navigation markers.
- `controllers/autonomous_drone/autonomous_drone.py` — the Webots sensor,
  keyboard, stabilization, and motor adapter.
- `controllers/autonomous_drone/flight_logic.py` — the pure-Python flight state
  machine and waypoint logic.
- `controllers/autonomous_drone/pursuit.py` — bounded target-motion and
  image-scale pursuit commands.
- `controllers/autonomous_drone/hud.py` — telemetry calculations and the
  text shown in the top-right Webots overlay.
- `controllers/autonomous_drone/world_markers.py` — home and target marker
  positioning in the Webots world.
- `realtime_server/` — the local realtime WebSocket server and documented message
  protocol used between drone adapters and tablet clients.
- `adapters/simulator/` — the Webots simulator adapter that publishes
  telemetry and receives realtime server commands.
- `app/android/` — the Capacitor Android shell that packages the existing
  React/Vite UI as a directly installable tablet APK.
- `services/` — shared backend helpers for GPS conversion, telemetry payloads,
  supported command names, no-fly routing, and resumable mission context.
- `tests/test_flight_logic.py` — deterministic tests that do not require Webots
  to be running.
- `tests/test_hud.py` — HUD formatting and telemetry math tests.

The main mission and visual-tracking states are:

```text
IDLE -> TAKING_OFF -> PATROLLING / TRANSITING_TO_ROI -> ROI_PATROL
                              |
                              v
                   TARGET_ACQUISITION -> TARGET_FOLLOW -> TARGET_PURSUIT
                              ^                |                 |
                              +---- reacquire -+-- lost search --+
                                                               |
                                                               v
                   interrupted mission <- TARGET_RETURN <- complete/lost

Any airborne mission -> RETURNING_HOME -> LANDING -> LANDED
```

## Android tablet app

The browser application remains the local simulator and development workflow.
The same React/Vite frontend is also packaged in a Capacitor Android shell; it
has not been rewritten as a native-only UI.

From `app`, build the directly installable debug APK with:

```powershell
npm.cmd run android:apk
```

The build command selects a compatible JDK and Android SDK 35 automatically.
The APK is written to:

```text
app\android\app\build\outputs\apk\debug\app-debug.apk
```

The Android-native DJI boundary is currently fail-closed and contains no DJI
SDK dependency or speculative flight APIs. Local browser → backend → simulator
operation remains unchanged. See [docs/android.md](docs/android.md) for Android
tooling, endpoint configuration, installation, LAN testing, APK signing, and
the planned RC-N3/Mini 4 Pro integration boundary.

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

## Run the realtime server

Start the local realtime server from the repository root:

```powershell
.\.venv\Scripts\python.exe -m realtime_server.server
```

By default it listens on:

```text
ws://127.0.0.1:8765/ws
```

Override the bind address with `DRONTECT_REALTIME_HOST` and
`DRONTECT_REALTIME_PORT`. The simulator adapter can override its outbound
telemetry URL with `DRONTECT_SIMULATOR_WS_URL` and its outbound video URL with
`DRONTECT_SIMULATOR_VIDEO_WS_URL`.

Battery safety RTH defaults to 20%. Override the threshold with
`DRONTECT_LOW_BATTERY_RTH_PERCENT`. For simulator testing, set
`DRONTECT_SIMULATED_BATTERY_PERCENT` at or below that threshold; the realtime
server will latch safety RTH, command the simulator home, and keep mission
commands from replacing it until landing completes.

After the simulator has connected at least once, losing its realtime command
link for 3 seconds also latches safety RTH. Override that grace period with
`DRONTECT_CONNECTION_LOSS_RTH_SECONDS`. Initial server unavailability before
the first successful connection does not trigger this failsafe.

The **MANUAL** toggle on the right edge of the action bar takes the aircraft off
the computer entirely and flies it from an on-screen stick pad, handing the
mission back on release. A safety RTH always outranks it. See
[docs/manual-control.md](docs/manual-control.md).

Use the same `/ws` endpoint with a required role query parameter:

```text
ws://127.0.0.1:8765/ws?role=drone
ws://127.0.0.1:8765/ws?role=tablet
```

The older `role=simulator` value is still accepted as a compatibility alias
for `role=drone`. Only one drone source and one tablet client may be connected
at a time.

Simulator telemetry is sent to the connected tablet client:

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
    "state": "PATROLLING"
  }
}
```

The simulator keeps Webots local meter coordinates internally. Before
telemetry is sent, local `x` is converted to longitude, local `y` is converted
to latitude, and local `z` is sent only as altitude. The WGS84 home point is
`32.085300, 34.781800`.

Tablet clients send commands through the realtime server:

```json
{
  "type": "command",
  "commandId": "client-generated-id",
  "command": "return_home",
  "params": {}
}
```

Supported commands are `start_patrol`, `set_patrol_priorities`,
`set_patrol_area_types`, `set_roi`, `clear_roi`, `return_home`,
`camera_tilt_up`, and `camera_tilt_down`. Accepted commands receive:

```json
{
  "type": "command_ack",
  "commandId": "client-generated-id",
  "status": "accepted"
}
```

Invalid messages receive structured errors with `type`, `code`, and `message`.

## Run the video pipeline

Video uses separate realtime server paths from telemetry and commands:

```text
ws://127.0.0.1:8765/video/ingest?role=drone
ws://127.0.0.1:8765/video/webrtc?role=viewer
```

The simulator adapter reuses the Webots camera device, encodes frames as JPEG,
assigns each frame a `frame_id` and timestamp, and sends them to
`/video/ingest`. The realtime server exposes the full latest-frame stream to
browser viewers over WebRTC and forwards configured frames to `vision_server`.

Video settings live in:

```text
config\video.json
```

The default `source_target_fps` is `30`. The default
`vision_frame_interval` is `2`, so every second frame ID is eligible for vision.
Queues are intentionally single-slot/latest-frame handoffs and are not
configurable, because stale frames should be dropped instead of accumulated.

Start the vision receiver from the repository root:

```powershell
.\.venv\Scripts\python.exe -m vision_server.server
```

By default it receives frames on:

```text
ws://127.0.0.1:8870/frames
```

and exposes diagnostics at:

```text
http://127.0.0.1:8871/diagnostics
```

The diagnostics response includes `received_count`, `latest_frame_id`, and
`fps`.

The vision server loads a YOLO drone detector when model weights are available
at:

```text
models\drone.pt
```

The model must expose a drone class, preferably class `0` named `drone`.
Detection settings can be tuned with environment variables:

```powershell
$env:DRONTECT_YOLO_MODEL_PATH="models\drone.pt"
$env:DRONTECT_YOLO_CONFIDENCE="0.25"
$env:DRONTECT_YOLO_IMGSZ="960"
.\.venv\Scripts\python.exe -m vision_server.server
```

For current simulator screenshots, `models\drone.pt` with confidence `0.25`
and image size `960` gives the best balance found so far between detecting
small drones and avoiding bird/background false positives.

The tablet video can switch between full-frame (`contain`) and fill-frame
(`cover`) presentation and apply digital zoom. Detection boxes use the same
fit/zoom transform as the video, so overlays remain aligned through resizing,
letterboxing, center cropping, and zoom changes.

Browser UI code should connect to `/video/webrtc?role=viewer` with a WebSocket,
create an `RTCPeerConnection`, add a receive-only video transceiver, send a JSON
SDP offer:

```json
{ "type": "offer", "sdp": "..." }
```

and apply the JSON SDP answer from the realtime server:

```json
{ "type": "answer", "sdp": "..." }
```

## Patrol regions, priorities, and area types

The home circle is divided into 32 fixed regions: eight compass directions
across four equal-area radial bands. The tablet map can rank PATROL regions and
mark regions as `IGNORE` or `NO_FLY`; authoritative priorities and area types
are carried in mission-status messages and restored in the UI after reconnects
or rejected updates.

Continuous patrol chooses only reachable `PATROL` regions. Its score combines
time since observation, operator priority, and safe route travel time. Once a
region is selected, the simulator generates an in-polygon zigzag coverage path
with 75 m lane spacing by default (`DRONTECT_PATROL_LANE_SPACING_M`). A region's
observation timestamp is recorded only after all of that coverage path is
finished.

`IGNORE` regions are excluded from selection but are not physical obstacles.
`NO_FLY` regions are excluded from selection, subtracted from coverage geometry,
and avoided by transit routing with a safety margin. If no continuous safe
coverage or transit path exists, the aircraft holds instead of crossing the
restriction. Active visual pursuit is the deliberate exception described below.

If a fresh patrol selection is requested while the aircraft is already
sitting inside a `NO_FLY` cell (for example, clearing an ROI that overlapped
one), it flies to the nearest safe exit point first and only plans which
region to patrol, and the full route to it, once it has arrived there and is
clear of every restricted cell. This keeps the aircraft moving immediately
instead of holding in place while every candidate region is scored, and the
scoring itself no longer needs a per-region escape leg once outside. This
exit-first step only applies to picking a brand-new patrol target from inside
a `NO_FLY` cell; normal in-progress patrol coverage (already outside every
restricted cell) is unaffected.

Region scoring stays a single synchronous step on arrival at the exit point
rather than running in a background thread while the aircraft is still
flying there. A background thread would only fight the same-process
simulation loop for the GIL on this CPU-bound geometry work, could read
`patrol_regions` mid-edit if an operator changes priorities/area types while
it runs, and adds cancellation logic for no guaranteed win. The remaining
pause is short and one-off, so the added risk isn't worth it here.

## Region of interest (ROI) on the tablet map

The tablet map (`app/src/components/map/MapView.tsx`) lets the operator
draw one region of interest: a "Define ROI" button arms placement mode, a
click on the map sets the center, and a draggable handle (or the numeric
field in the same panel) sets the radius. Choosing the ROI mission sends a
`set_roi` command through the realtime server; the simulator transits to the
ROI and runs a continuous lawnmower investigation path. Its fill is clipped
to the blue home operational-range circle
(`HOME_RADIUS_METERS` in `app/src/config/mapConfig.ts`); the complete ROI
outline and center remain visible so the operator can still adjust it.

ROI state lives behind `app/src/services/roiService.ts`, shaped like every
other `subscribe`-based frontend service
(`locationService`, `detectionService`, ...):

```ts
{ center: { lat, lon }, radiusM }
```

The map remains the owner of editable ROI state. `ActionBar` sends that state
through `droneCommandService`, and updates to an active ROI are debounced so
dragging the map handle does not flood the command link.

## Configure background actors

The simulation includes autonomous non-Dronetect drones and birds so future
camera-based detection can be tested against moving distractors. They are
spawned by a separate scenario supervisor and use their own controllers; the
Dronetect drone controller is not given their positions or types.

Before starting or resetting Webots, edit:

```text
config\simulation_scenario.json
```

The default scenario starts three traffic drones and three birds already airborne:

```json
{
  "traffic_drones": {
    "count": 3
  },
  "birds": {
    "count": 3
  }
}
```

Negative counts are treated as zero, missing values fall back to the defaults
above, and values above the supported pool are capped. Traffic drones use a
pre-placed pool of five same-model Mavic2Pro nodes, and birds are capped at
three dynamic actors. The main Dronetect drone still starts from the ground.

## Expected behavior

1. The drone starts idle at home and waits for a tablet/keyboard mission.
2. `START PATROL` begins continuous region coverage; an ROI command takes off,
   transits into the circle, and begins its lawnmower investigation. Pressing
   `N` still starts the legacy random-waypoint mission.
3. Normal mission takeoff rises to 15 metres with DJI Mini 4 Pro-inspired climb
   authority so it clears the rural scenery. A confirmed target can switch to
   the lower, height-limited pursuit takeoff described below.
4. During horizontal flight it uses a DJI Mini 4 Pro-inspired high-level
   kinematic flight profile with bounded horizontal speed, acceleration, climb,
   descent, and yaw rate instead of relying on raw motor-gain tuning.
5. Patrol and ROI missions continue until replaced, cleared, or sent home; the
   legacy random mission starts Return Home after reaching its waypoint.
6. Pressing `H` replaces the active mission with Return Home.
7. Above home, the drone descends gradually, stops its motors, and enters
   `LANDED`.
8. A new mission after landing takes off again without changing the original
   blue HOME location.

The blue pad marks the permanent home coordinate. The glowing red sphere marks
the current airborne target, and the red ground disc shows its X/Y projection.
The top-right telemetry HUD is drawn by the Python controller directly over
the main Webots 3D view, so it does not depend on the robot camera window.

The `H` command is accepted during takeoff or random flight. `N` works from
every state and safely climbs before departing when the drone is too low.
The `U` and `J` keys move only the camera pitch, so the camera can look more
up or down while its left/right direction still follows the drone body.

## Visual target pursuit (simulator)

`TARGET_ACQUISITION` stops mission translation while a tracker is confirmed.
`TARGET_FOLLOW` continues yaw and gimbal centering while the track stabilizes.
A confirmed, centered track held for `DRONTECT_PURSUIT_STABLE_SECONDS`
(default `0.8`) enters `TARGET_PURSUIT`. Translation uses smoothed tracker
image motion and bounding-box area history. Image expansion brakes approach;
rapid expansion takes collision priority and introduces lateral avoidance.
This is a visual heuristic, not an obstacle-range sensor or a guarantee of
collision clearance.

During takeoff, confirmation enters visual follow immediately. Pursuit may
start once the aircraft is above the 3.5 m floor plus its vertical tolerance,
instead of waiting for the normal 15 m cruise altitude. Horizontal speed is
limited from current height and braking clearance while a bounded ascent runs
in parallel. By default that ascent aims for 2 m of clearance above the floor
(5.5 m when home altitude is zero), capped by the normal takeoff altitude. Use
`DRONTECT_TARGET_TAKEOFF_PURSUIT_CLEARANCE_M` and
`DRONTECT_TARGET_TAKEOFF_PURSUIT_ASCENT_MPS` to tune it.

An operator-requested landing may also be interrupted for pursuit.
Low-battery, connection-loss, and other backend safety RTH/landing sequences
carry a safety reason to the simulator and cannot be interrupted by tracking.

Pursuit defaults to 6 m/s and 5 m/s² vector acceleration, configured through
`DRONTECT_PURSUIT_MAX_SPEED_MPS` and `DRONTECT_PURSUIT_ACCELERATION_MPS2`,
with a `0.2 s` command smoothing time (`DRONTECT_PURSUIT_SMOOTHING_SECONDS`).
`DRONTECT_PURSUIT_OBSERVATION_TIMEOUT_SECONDS` defaults to 1.5 seconds. A
missing, lost, unconfirmed or unusable-confidence observation **decelerates at
the acceleration limit rather than zeroing the command**: rebuilding the ramp
from rest costs seconds, so one dropped frame must not discard the approach.
Translation still ends within a fraction of a second, and a sustained loss
stops completely. Fresh confirmation of the same track resumes pursuit; a
different tracker must stabilize again. Temporary-loss search always hovers
rather than continuing the last translation command.

The yaw-rate limit is derived per frame instead of being a fixed cap. Holding a
target centered needs exactly `v_perp / range` rad/s, so that is the
feed-forward term, with a proportional term for the residual bearing error,
clamped between `DRONTECT_TARGET_MIN_YAW_RATE_DEG_S` (20 deg/s) and
`DRONTECT_TARGET_MAX_YAW_RATE_DEG_S` (70 deg/s). It is therefore fast only for
a close, fast-crossing target and slow for a distant one. The ceiling protects
the detector from motion blur and the own-yaw term subtracted out of the lateral
image cue, which is only a first-order correction. The yaw setpoint commands the
true bearing the image error represents; treating that normalized error as
radians silently scaled every correction by the camera FOV.

The interruption context is mission-neutral and records the source mission,
exact interruption position, active patrol-region or ROI ID, active waypoint,
and remaining path/progress. Permanent loss or `complete_target_follow()` enters
`TARGET_RETURN`, returns exactly to that position, then restores unfinished
PATROL coverage or the ROI planner's exact cursor. Starting pursuit does not
record a patrol observation or complete ROI work; normal mission code does that
only after its own remaining path finishes.

`DRONTECT_TARGET_STANDOFF_BOX_AREA_RATIO` and
`DRONTECT_TARGET_STANDOFF_DISTANCE_M` are a **range calibration pair**: the
relative image area the target occupies at a known distance, for a fixed target
size and camera setup. They are not the station the aircraft holds. That is
`DRONTECT_PURSUIT_STANDOFF_M` (5 m) with `DRONTECT_PURSUIT_ALTITUDE_OFFSET_M`
(3 m above), converted into an area setpoint through the calibration. Box area
does not measure absolute distance, so no separation is guaranteed. Estimated
motion is likewise an image-derived control cue, not a measured world velocity.

Once the target's own heading can be estimated, pursuit flies its six o'clock:
it holds a point one standoff behind the target along that heading and one
offset above it, using the estimated target velocity as feed-forward so station
can be kept rather than perpetually lagging a box-area error. The estimate is an
alpha-beta filter over the same calibrated-area projection, gated against
implausible jumps; below `tail_chase_min_speed_mps` the heading is noise and the
older image-space law is held instead.

During confirmed pursuit, altitude control combines the target's frame
position, the actual gimbal pitch, camera FOV, and an inverse-area range
estimate. The reference range defaults to 2 m at the configured standoff box
area. The controller holds the offset **above** the target rather than level
with it, inside a `0.5 m` band, and caps vertical tracking at `2 m/s`; tune
these with
`DRONTECT_TARGET_STANDOFF_DISTANCE_M`,
`DRONTECT_TARGET_ALTITUDE_TARGET_BAND_M`,
`DRONTECT_TARGET_ALTITUDE_GAIN_PER_S`,
`DRONTECT_TARGET_MAX_VERTICAL_SPEED_MPS`, and
`DRONTECT_TARGET_RANGE_ESTIMATE_MAX_M`.

The +/-0.5 m band is a control target validated in the included Webots
scenario, **not a geometric or real-world accuracy guarantee**. Bounding-box
area depends on target size, pose, and detector behavior and is not an
absolutely calibrated range measurement. A different target/camera setup must
be recalibrated; dependable real-world metric accuracy requires a metric range
source.

Pursuit uses the patrol home-circle geofence, a 3.5 m minimum height above the
home ground datum, and a 120 m ceiling. Altitude matching remains bounded by
those limits; available height clearance additionally limits speed using
braking distance.
Height is relative to home, not terrain clearance over uneven ground.

Collision avoidance triggers on **time to contact, not distance**. The
clearance a lateral escape can generate depends on how long there is to
generate it, so a fixed trigger distance is simultaneously too late for a fast
closure and needlessly early for a slow one. For an inverse-square box,
`growth / area == 2 / tau`, so time to contact comes straight out of the growth
cue without a range estimate -- which matters because the area-derived range
degrades as the square of distance exactly where the decision is made.

Pursuit latches a lateral pass-under when contact is within
`DRONTECT_PURSUIT_DODGE_ENTRY_SECONDS` (1.8 s) and releases it past
`DRONTECT_PURSUIT_DODGE_EXIT_SECONDS` (3.0 s). At 1.8 s, roughly 1.4 s of that
is usable after detector and smoothing lag, which is what a 3 m clearance costs
at the pursuit acceleration limit; below about 1.5 s the clearance cannot be
generated at all. A range window (2.5 m to 25 m) suppresses the decision where
the box is too small for the cue to mean anything, an unconditional threshold
covers imminent contact when no range is available, and a hard floor triggers
on proximity alone however slowly it was reached.

The aircraft's own closing speed is separately capped to keep time to contact
above `approach_time_to_contact_s`, so a shorter one can only be the target's
doing -- flying at something is not the same as being flown at. That cap is on
*relative* closure: matching a fleeing target's speed shrinks no gap and is not
billed as if it did. Braking answers a passive target; only something closing on
us is stepped around.

Per simulator policy, NO_FLY regions do not block active pursuit. If pursuit
ends inside one, `TARGET_RETURN` first chooses the shortest safe egress toward
the saved checkpoint without crossing another NO_FLY region. Normal NO_FLY
enforcement is restored after that first exit leg and remains active for the
rest of the return and resumed mission.

The existing realtime telemetry and unobtrusive debug HUD expose flight and
pursuit state, tracker ID, confidence, normalized image-center error,
confirmation progress, estimated image velocity/direction, constrained XYZ
velocity command (m/s), bounding-box scale/standoff error, target-lost duration,
mission interruption/resume status, and the active safety constraint.
`TARGET_LOST_SEARCH` remains a diagnostic search phase with zero pursuit
translation.

Run the real camera-to-controller altitude check with Webots installed:

```powershell
.\.venv\Scripts\python.exe tools\run_webots_altitude_smoke.py
```

The check requires `config\simulation_scenario.json` to contain one traffic
drone and zero birds. It starts the realtime and vision servers, commands patrol
through the tablet WebSocket, runs Webots in batch/realtime mode so simulated
motion remains synchronized with the external YOLO worker, and uses simulator
ground truth only to verify that the true height difference remains within the
0.5 m target for three simulated seconds. Logs and `result.json` are written
under `artifacts\webots_altitude_smoke\`.

## Run tests

```powershell
.\.venv\Scripts\python.exe -m pytest -v
```

## Deploy to the cloud

`realtime_server` and `vision_server` can run as Docker containers instead of
local processes -- see [docs/deployment.md](docs/deployment.md) for the full
env var matrix, TLS/WSS and WebRTC STUN notes, and a local Docker Compose
smoke test. This is scaffolding only: it does not create any cloud resources,
and every local/simulator workflow documented above keeps working unchanged
with zero new required steps. In short:

```powershell
docker build -f realtime_server/Dockerfile -t drontect-realtime-server .
docker build -f vision_server/Dockerfile -t drontect-vision-server .
docker compose up   # local smoke test of both containers together
```

## Attribution

The motor stabilization approach and empirical Mavic constants are adapted
from Cyberbotics' Webots R2025a `mavic2pro_patrol.py` example, licensed under
the Apache License 2.0. The world references the official R2025a Webots PROTO
assets rather than copying model meshes or textures into this repository.
