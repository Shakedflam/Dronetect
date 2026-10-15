# Dronetect

Dronetect is an autonomous aerial surveillance system built around a **DJI Mini 4 Pro**.
An Android tablet application provides the operator interface and the link to the
aircraft over the DJI Mobile SDK, while autonomy, computer vision and realtime
communications run as cloud services. The operator defines an operating area,
selects a region of interest on a live map, and the system plans and flies the
coverage pattern while streaming telemetry, video and drone detections back to
the tablet.

The same autonomy stack runs unchanged against a Webots simulation, which is how
the mission logic, detection and pursuit behaviour are developed and verified
without flying.

> **Scope note.** Sections below explicitly distinguish behaviour *validated in
> simulation* from behaviour *validated on the real aircraft*. See
> [Validation status](#validation-status) — real-hardware validation is
> deliberately narrower than the simulator feature set.

---

## Contents

- [Architecture](#architecture)
- [Features](#features)
- [Validation status](#validation-status)
- [Safety model](#safety-model)
  - [Flight lifecycle and control ownership](#flight-lifecycle-and-control-ownership)
- [Project structure](#project-structure)
- [Running the project](#running-the-project)
- [Configuration](#configuration)
- [Operator workflow](#operator-workflow)
- [Testing](#testing)
- [Development and test modes](#development-and-test-modes)
- [Known issues and limitations](#known-issues-and-limitations)
- [Attribution](#attribution)
- [Licensing](#licensing)

---

## Architecture

The development laptop is not part of the runtime flight path. In the field the
tablet talks to the cloud over the public internet; the laptop is only used for
building, deployment and debugging.

```text
                    ┌──────────────────────────┐
                    │      Operator tablet     │
                    │  React + Leaflet UI in a │
                    │   Capacitor Android app  │
                    └──────────┬───────────────┘
                               │  WebSocket (telemetry, commands, detections)
                               │  WebRTC / WebSocket video
                               ▼
        ┌──────────────────────────────────────────────┐
        │        AWS (Docker Compose on EC2)           │
        │                                              │
        │  realtime_server   mission state, command    │
        │                    routing, session home     │
        │  vision_server     YOLO drone detection      │
        │  dji_bridge        autonomy + FlightLogic,   │
        │                    setpoint generation       │
        └──────────────────────┬───────────────────────┘
                               │  authenticated WebSocket bridge
                               │  (Cloudflare Tunnel → api.dronetect.tech)
                               ▼
                    ┌──────────────────────────┐
                    │  Android DJI integration │
                    │   Kotlin + DJI MSDK 5.18 │
                    └──────────┬───────────────┘
                               │  DJI Mobile SDK / Virtual Stick
                               ▼
                    ┌──────────────────────────┐
                    │  RC-N3  ↔  DJI Mini 4 Pro │
                    └──────────────────────────┘
```

**Command flow toward the aircraft.** The operator selects an ROI on the tablet
map → `realtime_server` validates it against the confirmed session home and
forwards it → `dji_bridge` hands it to `FlightLogic`, which plans the transit
and coverage route → each tick the bridge converts the planned next position
into an ENU velocity plus a heading and sends it as a short-lived `dji_setpoint`
→ the Android client validates freshness, session and envelope, maps the command
into the DJI GROUND frame, and submits it through Virtual Stick.

**Telemetry flow back.** The Android layer subscribes to DJI keys (position,
attitude, battery, GPS, flight mode, RC sticks) and publishes them to the bridge,
which feeds FlightLogic and republishes mission telemetry to the tablet through
`realtime_server`. Camera frames go to `vision_server` for detection; detections
are returned to both the autonomy stack and the operator UI.

### Component responsibilities

| Path | Responsibility |
|---|---|
| `app/` | React + TypeScript operator UI (Vite), packaged for the tablet with Capacitor |
| `app/android/` | Kotlin Android shell, DJI MSDK integration, Virtual Stick actuator, telemetry bridge |
| `realtime_server/` | WebSocket hub: mission state, command validation and routing, session home, protocol |
| `vision_server/` | YOLO-based drone detection over a WebSocket frame feed |
| `adapters/dji/` | Cloud-side DJI flight bridge: autonomy tick loop, setpoint generation, Android link |
| `adapters/simulator/` | Webots adapter publishing simulator telemetry to the same realtime protocol |
| `controllers/autonomous_drone/` | `FlightLogic` state machine, ROI/patrol planners, pursuit, HUD |
| `services/` | Shared helpers: GPS conversion, telemetry payloads, command vocabulary, no-fly routing, mission context |
| `worlds/` | Webots world used for simulation |

---

## Features

### Autonomous flight

- Operator-defined **operating radius** and **cruise altitude**, applied as a
  runtime operating configuration; the autonomy stack refuses to fly until one
  has been applied.
- **Patrol regions** with priorities and area types, and polygon coverage
  planning that respects no-fly geometry.
- **ROI selection** from the tablet map, transit to the ROI, and **lawnmower
  coverage** of the selected area.
- **Single-cycle ROI mode** for demonstrations: one complete coverage cycle,
  then hold station (see [Configuration](#configuration)).
- **Automatic takeoff and landing on the real aircraft.** Starting an ROI or
  patrol on the ground lifts off on DJI's native takeoff; Dronetect then climbs
  to cruise altitude and flies the mission. Return to Home is Dronetect's own
  navigation back to the session home, ending with a hold low over home and a
  handover to DJI's native landing. See
  [Flight lifecycle and control ownership](#flight-lifecycle-and-control-ownership).
- **HOLDING** station-keeping, **Return to Home**, and landing.
- **Mission interruption and resume**, including returning to the previous
  patrol or ROI context after a detection-driven excursion.
- Independent **movement safety envelope** enforced on the aircraft-facing
  Android layer, not only in the cloud.

### Drone detection and tracking

- YOLO-based **drone detection** served by `vision_server`.
- **Confidence filtering** and acquisition logic before a track is committed.
- **Target following** and **target pursuit** states with bounded motion and
  image-scale-based standoff control.
- **Yaw and gimbal tracking** to keep an acquired target framed.
- **Target-loss** handling with a search window, then return to the interrupted
  patrol or ROI context.

### Operator interface

- Live map with aircraft position, home, operating radius and patrol geometry.
- Live telemetry: position, altitude, heading, speed, battery, mission state.
- Live camera view with detection overlays.
- ROI selection and radius adjustment directly on the map.
- Patrol and ROI mission controls, Return to Home, and manual takeover.
- Operating configuration dialog for radius and cruise altitude.
- Connection, GPS and safety indicators, including a pursuit-blocked control.

---

## Validation status

This distinction matters and is kept deliberately conservative.

### Validated on the real DJI Mini 4 Pro

- DJI Mini 4 Pro + RC-N3 connection through **DJI MSDK 5.18** on the tablet.
- Tablet ↔ AWS connectivity, authenticated bridge session, and **live aircraft
  telemetry** reaching the cloud and the operator UI.
- Live video and cloud service connectivity.
- **Virtual Stick authority acquisition** (`owner=MSDK`) on the real aircraft.
- **Real autonomous movement**: a commanded vertical velocity physically moved
  the aircraft, terminated by a measured-altitude cutoff, followed by a clean
  zero command and authority release.
- RC takeover, authority release and the stale-command fail-safe on hardware.
- ROI selection reaching `FlightLogic` and the mission state machine advancing.

### Validated in simulation only

- Patrol, polygon coverage, ROI lawnmower coverage flight, and mission resume.
- Detection-driven target acquisition, following and pursuit.
- Yaw and gimbal tracking of an acquired target.
- Return to Home and landing as autonomous manoeuvres.

### Not yet validated anywhere

- **DJI native takeoff and landing handover.** Implemented and covered by bridge
  and Android unit tests, but not yet flown. In particular, the Mini 4 Pro's
  takeoff hover height, pilot stick override during DJI's own takeoff/landing,
  and DJI landing-protection behaviour must be confirmed on the aircraft.

- **Horizontal axis and sign mapping on real hardware.** The ENU → DJI GROUND
  frame mapping is implemented from DJI's documented convention (positive pitch
  moves East, positive roll moves North in the GROUND frame) and covered by unit
  tests, but it has **not** been confirmed against the aircraft. A short
  East-then-North validation step must precede any autonomous horizontal flight.

---

## Safety model

Safety is enforced in two independent places: the cloud autonomy stack, and the
Android layer that actually talks to the aircraft. The Android layer does not
trust the cloud for physical limits.

**Android-side envelope (evaluated every control tick)**

| Guard | Behaviour |
|---|---|
| Horizontal speed | saturated to the configured demo limit, bearing preserved |
| Vertical speed | saturated to the configured demo limit |
| Altitude ceiling / floor | abort → zero → release authority |
| Radius backstop from arming position | abort → zero → release authority |
| RC stick input | immediate takeover: zero all axes, release authority |
| Aircraft or RC disconnect | zero → release |
| Stale or expired setpoint | bounded zero hold, then zero → release |
| Loss of Virtual Stick authority | abort |
| DJI `NEAR_BOUNDARY` / authority denial | abort, with flight-limit diagnostics logged |
| GPS below the required signal level | abort |
| Non-finite or incomplete command | reject and zero |
| Flight mode outside the allowed set | abort |

| Below the 1.5 m floor | only a pure climb (≥ 0.5 m, horizontal zeroed) is accepted, so the climb can continue from DJI's takeoff hover |

**Link and protocol safety**

- Every setpoint carries an **issue timestamp and TTL**; the client measures age
  against a clock estimate synchronised with the server and rejects stale
  commands.
- **Session and replay protection**: commands are bound to an authenticated
  session id with a monotonic sequence; a reconnect starts a fresh session.
- Loss of link degrades to zero output and authority release rather than
  continuing the last command.

**Mission-level safety**

- The autonomy stack refuses to run at all until an operating configuration has
  been applied.
- No-fly geometry is respected by the patrol and coverage planners.
- Return to Home is available from any airborne mission state, including
  `HOLDING` after a completed ROI cycle.
- A safety Return to Home is triggered on prolonged link loss while airborne.

### Flight lifecycle and control ownership

DJI's native APIs are used only for lift-off and touchdown. Everything in
between — climb, patrol, ROI, pursuit, target return, no-fly logic and Return to
Home — is Dronetect's own `FlightLogic`.

| Phase | Owner | What is sent to the aircraft |
|---|---|---|
| Grounded | nobody | no setpoints |
| Native takeoff (`TAKING_OFF`, before stable hover) | DJI (`KeyStartTakeoff`) | one takeoff request, no setpoints |
| Climb, patrol / ROI, pursuit, Return to Home | Dronetect | Virtual Stick setpoints with the mission's `controlEpoch` |
| Landing approach (`LANDING`, down to 3 m above home) | Dronetect | Virtual Stick setpoints |
| Native landing | DJI (`KeyStartAutoLanding`) | Virtual Stick released first, then one landing request; no setpoints |
| Manual takeover | pilot | Virtual Stick released, no setpoints |
| Touchdown (not flying for 2 s) | nobody | reset to `IDLE`, no setpoints |

- **Takeoff:** a mission started on the ground sends one DJI takeoff. Setpoints
  start only after telemetry shows the aircraft airborne in `GPS_NORMAL` /
  `VIRTUAL_STICK` for 1.5 s. A mission started while airborne skips this.
- **Landing handover:** DJI is asked to land only when home is known,
  telemetry is fresh, the aircraft is within 2 m of home, at most 4 m above it,
  slower than 1 m/s, in a normal flight mode, with no pilot control and no
  safety RTH — all held for 1.5 s. The 3 m hold is twice the Android Virtual
  Stick floor; 2 m covers real GPS noise. Virtual Stick must be seen released
  before the landing request is sent.
- **`controlEpoch`:** only an explicit new operator mission advances it.
  Releasing Virtual Stick — for a pilot takeover or for DJI's landing — marks
  the current epoch released on Android, so autonomy can never silently re-arm.
- **Failures:** a takeoff not airborne and stable within 20 s aborts the
  mission to `IDLE` without any Virtual Stick motion (handed to the pilot if
  already off the ground). A landing DJI does not start within 6 s, or abandons
  mid-air, is logged and handed to the pilot; it is never retried and autonomy
  does not resume. If Return to Home cannot settle over home, the aircraft keeps
  holding there and the bridge logs why the handover is waiting.
- **Pilot priority:** RC stick input during takeoff, climb, mission, Return to
  Home, landing approach or DJI's landing hands control to the pilot
  immediately. Dronetect never cancels DJI's own takeoff or landing and never
  confirms DJI landing protection on the pilot's behalf.
- Safety Return to Home (link loss, low battery) still uses DJI's native
  go-home, unchanged.

**What Dronetect does not do:** it never attempts to bypass, weaken or unlock
DJI's own FlySafe, GEO-zone or flight-limit restrictions. Where the aircraft's
own limits block a manoeuvre, Dronetect aborts and reports it.

---

## Project structure

```text
.
├── app/                        React + TypeScript operator UI
│   ├── src/                    components, hooks, services, map, HUD
│   └── android/                Capacitor shell + Kotlin DJI integration
│       └── app/src/main/java/com/dronetect/app/vehicle/
│                               MSDK runtime, telemetry, Virtual Stick actuator
├── adapters/
│   ├── dji/                    cloud DJI flight bridge and Android link
│   └── simulator/              Webots adapter
├── controllers/
│   └── autonomous_drone/       FlightLogic, ROI/patrol planners, pursuit, HUD
├── realtime_server/            WebSocket hub, protocol, mission state
├── vision_server/              YOLO detection service
├── services/                   GPS, telemetry, commands, no-fly, mission context
├── worlds/                     Webots world
├── tests/                      Python test suite
├── docs/                       deployment and design notes
├── docker-compose.yml          realtime_server + vision_server
└── docker-compose.dji.yml      dji_bridge service
```

---

## Running the project

### Prerequisites

- Python 3.11+ and a virtual environment
- Node.js 20+ for the operator UI
- Docker (for the cloud services)
- Webots R2025a (simulator only)
- Android Studio / JDK 17+ and an Android tablet (real hardware only)

### 1. Python environment

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

### 2. Simulator

Open `worlds/drone_poc.wbt` in Webots R2025a and start the simulation. The
controller in `controllers/autonomous_drone/` runs the same `FlightLogic` used
in real flight. Keyboard controls are shown in the on-screen HUD.

### 3. Operator UI (development)

```powershell
cd app
npm install
npm run dev
```

### 4. Cloud services

```powershell
docker build -f realtime_server/Dockerfile -t drontect-realtime-server .
docker build -f vision_server/Dockerfile -t drontect-vision-server .
docker compose up
```

See [docs/deployment.md](docs/deployment.md) for the full environment variable
matrix, TLS/WSS notes and WebRTC STUN configuration.

### 5. Android / DJI build

Create `app/android/local.properties` (git-ignored) with your own values:

```properties
DRONTECT_DJI_BRIDGE_TOKEN=<64 hex characters>
DRONTECT_DJI_AUTONOMOUS_ENABLED=<true|false>
```

Generate a bridge token with
`python -c "import secrets; print(secrets.token_hex(32))"` and set the **same**
value in the AWS host's private Compose `.env`. The DJI application key is
supplied through the Android manifest placeholder and must never be committed.

```powershell
cd app
npm run android:apk        # build UI, sync Capacitor, run unit tests, assemble APK
```

Install on the tablet and watch the logs:

```powershell
adb install -r app\android\app\build\outputs\apk\debug\app-debug.apk
adb logcat -c
adb logcat -s DronetectDJI:*
```

### 6. Real-hardware deployment

On the AWS host, with the realtime and vision services already running:

```sh
docker compose -f docker-compose.dji.yml up -d --build
docker compose -f docker-compose.dji.yml logs -f dji_bridge
```

The DJI bridge is reached from the field through a Cloudflare Tunnel at
`api.dronetect.tech`; the bridge port binds to loopback on the host and is not
exposed in the security group. Add the `/dji/bridge` ingress rule from
`deploy/cloudflared-dji-ingress.example.yml` **before** the catch-all rule, then
reload the tunnel.

---

## Configuration

### Normal configuration

| Setting | Where | Meaning |
|---|---|---|
| Operating radius | Tablet UI | Maximum mission radius from the session home |
| Cruise altitude | Tablet UI | Altitude used for transit and coverage |
| Patrol priorities / area types | Tablet UI | Region weighting for patrol planning |
| `DRONTECT_DJI_REALTIME_WS_URL` | Bridge env | Realtime server endpoint |
| `DRONTECT_DJI_LOCAL_HOST` / `_PORT` | Bridge env | Loopback bind for the Android bridge |

### Real-flight safety configuration

| Setting | Where | Meaning |
|---|---|---|
| `DRONTECT_DJI_BRIDGE_TOKEN` | Bridge env + `local.properties` | Shared secret authenticating the tablet ↔ bridge session |
| `DRONTECT_DJI_AUTONOMOUS_ENABLED` | `local.properties` | Enables the autonomous actuator path in the APK |
| `DRONTECT_DJI_ROI_SINGLE_CYCLE` | Bridge env | Real-demo only: stop after one complete ROI coverage cycle and hold. Simulator behaviour stays continuous by default |

Actuator limits (horizontal and vertical speed caps, altitude ceiling and floor,
radius backstop) are compiled into the Android layer so that no cloud-side
misconfiguration can exceed them. They are deliberately conservative for
demonstration flights.

### Development and test flags

These are **not** normal production configuration. They exist for staged
hardware bring-up and must all be unset for autonomous operation.

| Flag | Purpose |
|---|---|
| `DRONTECT_DJI_PHASE1_TELEMETRY_ONLY` | Telemetry only; no aircraft commands |
| `DRONTECT_DJI_PHASE2_ACTIONS` | Gimbal and Return-to-Home actions only |
| `DRONTECT_DJI_PHASE3_ZERO_STICK` | Ground test: zero-valued Virtual Stick output only |
| `DRONTECT_DJI_PHASE4_VERTICAL_PULSE` | Single bounded vertical climb pulse |

Any one of these forces telemetry-only mode and disables the autonomous path.

---

## Operator workflow

1. Start the cloud services and confirm the DJI bridge is running.
2. Power the aircraft and RC-N3, connect the tablet, and launch the app. Wait
   for aircraft telemetry and a confirmed session home. **The bridge must see a
   grounded fix before takeoff** — session home is only captured on the ground.
3. Set the operating radius and cruise altitude in the configuration dialog.
4. Confirm GPS quality, home point and battery on the HUD.
5. With the aircraft still on the ground, select a region of interest on the
   map or start a patrol mission. The aircraft takes off on DJI's native
   takeoff, climbs to cruise altitude and flies the mission. (Starting a
   mission while already airborne also works and skips the takeoff.)
6. Observe the map, live video, detections and mission state.
7. End the mission with Return to Home: the aircraft flies home, descends to a
   low hold over home, and DJI lands it. The mission resets to `IDLE` after
   touchdown.
8. Take manual control at any time by moving the RC sticks. Autonomy does not
   resume on its own; only a new mission from the tablet regains control.

---

## Testing

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

```powershell
cd app
npm run android:test
```

The Android suite is JVM-only and covers the command protocol, telemetry
mapping, the ENU → DJI frame conversion, the actuator safety envelope and the
autonomous control lifecycle. When re-running after a configuration change, use
`--rerun-tasks`: Gradle will otherwise report a cached, stale result.

---

## Development and test modes

Hardware bring-up was staged so that each layer was proven before the next was
enabled: telemetry only, then non-flight actions, then zero-valued Virtual Stick
output on the ground, then a single bounded vertical pulse in the air, and only
then continuous autonomous control. The corresponding flags are listed under
[Configuration](#configuration) and remain available for re-validating a layer
in isolation. They are not part of normal operation.

---

## Known issues and limitations

- **Horizontal axis/sign mapping is unvalidated on hardware.** See
  [Validation status](#validation-status).
- **Aircraft-side DJI flight limits are independent of Dronetect.** During field
  testing, an aircraft-enforced maximum-distance limit repeatedly caused DJI to
  refuse or reclaim Virtual Stick authority (`NEAR_BOUNDARY`) whenever the
  aircraft approached that boundary. Because the operator interface does not
  display the aircraft's own distance-to-limit, this was difficult to attribute
  and cost significant field time: the mission would simply stop with no
  on-screen explanation. The aircraft's own limits must be checked in the DJI
  Fly app before a flight. Surfacing distance-to-limit in the Dronetect HUD is a
  known gap.
- **Session home requires a grounded fix.** If the bridge only links to the
  aircraft after takeoff, no session home is established and no setpoints are
  produced. The bridge now logs this explicitly rather than stalling silently.
- ROI coverage at cruise altitude means selecting an ROI from a low hover will
  first command a climb to the configured cruise altitude.
- **DJI takeoff/landing results are not reported back to the bridge.** Failures
  are detected from telemetry timeouts (20 s takeoff, 6 s landing start); the
  tablet logs DJI's own error codes.
- **After a failed native landing the mission is `IDLE`,** and Return to Home is
  not accepted from `IDLE`. The pilot lands manually, or the operator starts a
  new mission.
- If DJI's landing protection stops above the ground to ask for confirmation,
  that confirmation is left to the pilot.
- Detection performance has not been characterised against real airborne
  targets.

---

## Licensing

Original project code is licensed under AGPL-3.0-only, with original `app/` code
under MIT and identified third-party exceptions. See [LICENSING.md](LICENSING.md)
for scope, full license texts, dependency notices, and outstanding model provenance
checks. Adding these texts does not by itself clear the custom models for redistribution.

## Attribution

The motor stabilization approach and empirical Mavic constants are adapted from
Cyberbotics' Webots R2025a `mavic2pro_patrol.py` example, licensed under the
Apache License 2.0. The world references the official R2025a Webots PROTO assets
rather than copying model meshes or textures into this repository.
