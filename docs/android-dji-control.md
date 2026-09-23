# Android DJI drone socket and action boundary

This build adds the native `/ws?role=drone` transport and a limited set of DJI
actions. It is **not autonomous-flight ready**. No APK installation or hardware
actuation was performed during implementation.

## Existing protocol and compatibility gaps

`realtime_server/protocol.py` carries mission intents (`start_patrol`, `set_roi`,
`return_home`, configuration/region changes, manual sticks, pursuit hold and
camera tilt). Vision detections are also forwarded to the drone socket. It does
not send autonomous velocity or waypoint setpoints. The simulation runs the
actual planner and control loop inside the Webots controller.

`adapters/dji/flight_bridge.py` already reuses the same Python FlightLogic,
patrol, no-fly routing, pursuit and safety modules for DJI. It expects a local
Android link on port 8790 using `adapters/dji/local_protocol.py`. That local
runtime is not present in the Android app. It also owns the cloud drone-role
socket, so it cannot run alongside this native client as a second drone source.
Connecting that existing controller, or packaging it locally, needs an explicit
architecture decision; copying its algorithms into Kotlin would duplicate the
state machine and its safety checks.

Other existing server limitations, left unchanged:

- GPS, altitude, heading, speed and battery are required numbers. There is no
  null-GPS or position-unavailable message. Native uplink waits for valid real
  values and closes on loss of those values. The local Capacitor snapshot/video
  continue independently. The map may retain its last server position while
  mission status reports disconnection; no default coordinates are sent.
- Only `set_operating_config` has a drone-side result message. All other tablet
  acknowledgements confirm forwarding, not aircraft execution. An unsupported
  mission can therefore appear accepted in the existing UI while native logs
  correctly say rejected. Do not use that acknowledgement as permission to fly.
- ROI/radius checks still use configured `DEFAULT_HOME`, not DJI's recorded home.
  This must be reconciled before enabling real patrol/ROI execution.
- The inspected server authenticates by role only and supports one drone and one
  tablet. WSS transport alone does not pair or authorize an operator/aircraft.
  This is another prerequisite for operational remote flight control.

## Native behavior

After SDK registration, DjiSdkRuntime starts the process-lifetime adapter/client.
Activity recreation does not restart that link. Its origin is the existing
`BuildConfig.DRONTECT_REALTIME_WS_ORIGIN` generated from `.env.android` (currently
`wss://api.dronetect.tech`). TLS certificate verification uses OkHttp defaults.

Once aircraft connection and a complete GPS sample are available:

`DJI listeners -> DjiTelemetryMapper -> DroneRealtimeClient -> /ws?role=drone`

The client sends the existing `telemetry` envelope with `source=dji` at 2 Hz.
The existing server forwards this to the tablet telemetry/location services and
map. Speed is the measured 3D velocity magnitude; compass heading is normalized
to [0, 360). Flight state is derived from DJI flight mode/isFlying/go-home state,
never inferred from a requested mission. Uplink timestamps are Unix seconds at
serialization of the latest SDK values. GPS requires a valid coordinate pair
and reported signal LEVEL_2 or better. Unknown values are not replaced with
simulator defaults. The local nullable WebView telemetry remains available.

The separate camera/video ingest classes and their lifecycle calls are unchanged.

| Existing command | Native behavior |
| --- | --- |
| `return_home` | Read aircraft connection, isFlying, flight mode and home-set state; call DJI `KeyStartGoHome`. Reject on ground, unknown state/home, landing, unsupported keys or SDK errors. Already-GO_HOME is a successful no-op. |
| `camera_tilt_up`, `camera_tilt_down` | Read main gimbal connection, pitch and supported range; rotate only pitch through `KeyRotateByAngle`. |
| `set_pursuit_block` | Retain/report the Boolean operator hold flag. Does not start or resume autonomy. |
| `set_operating_config` | Return existing `operating_config_result` with `accepted=false`, `pending=false`, explaining that the local autonomy controller is required. No limits are falsely advertised as applied. |
| Patrol, ROI, region changes, target rejection, manual-control takeover and sticks | Reject with `local_autonomy_controller_required`; no aircraft actuation. |
| Vision detections | Ignore with a throttled explanation until the existing local controller is integrated. |
| Unknown commands, including raw takeoff/land/Virtual Stick | Reject. |

## Verified SDK surface and limits

Signatures were inspected with JDK 21 `javap` in the installed
`com.dji:dji-sdk-v5-aircraft-provided:5.18.0` artifact, including inherited
`DJIFlightControllerKey`/`DJIGimbalKey` declarations:

- `KeyManager.getValue(DJIKey<T>, CompletionCallbackWithParam<T>)`,
  `isKeySupported`, `performAction(ActionKey<?,R>, callback)` and
  `performAction(ActionKey<P,R>, P, callback)`.
- Flight-controller keys: `KeyConnection`, `KeyIsFlying`, `KeyFlightMode`,
  `KeyIsHomeLocationSet`, `KeyStartGoHome`, `KeyAircraftVelocity`,
  `KeyGoHomeStatus`, plus the previously integrated GPS/altitude/heading keys.
- Gimbal keys: `KeyConnection`, `KeyGimbalAttitude`, `KeyGimbalAttitudeRange`,
  `KeyRotateByAngle`, using `ComponentIndexType.LEFT_OR_MAIN`.
- `GimbalAngleRotation` uses `ABSOLUTE_ANGLE`, pitch enabled, yaw/roll ignored,
  duration 0.3 seconds and `jointReferenceUsed=false`. The simulator's
  0.03-radian step is converted to degrees with DJI's positive-up convention.
  Bounds are the intersection of the aircraft-reported range and the existing
  simulator pitch envelope. Missing/invalid ranges reject the command.

Official descriptions supplement the artifact signatures:
[RTH key](https://developer.dji.com/api-reference-v5/Components/IKeyManager/Key_FlightController_FlightControllerKey.html)
and [gimbal rotation](https://developer.dji.com/api-reference-v5/android-api/Components/IKeyManager/Value_Gimbal_Struct_GimbalAngleRotation.html).

There is one pending action at a time, at least 300 ms between camera actions and
5 seconds between RTH requests. RTH can preempt pending camera reads; a short
gimbal rotation already submitted is not cancelled. Preconditions must complete within 1.5 seconds
of receipt; callbacks time out after 5 seconds. Timeouts are reported as unknown
outcome and are never retried automatically. A socket/product generation guard
prevents stale asynchronous reads from starting an action after disconnection.
The last 256 command IDs are remembered across reconnects to suppress duplicates.
The existing wire protocol has no issued-at/expiry field for these cloud
commands; these guards bound local processing, not upstream network age.

Reconnect backoff is 1–10 seconds, with WebSocket pings and bounded send/receive
queues. Losing GPS/aircraft/socket cancels pending pre-action work, never replays
commands and never cancels an aircraft-native RTH/landing already underway.
The app does not acquire Virtual Stick authority, start motors or change DJI
obstacle avoidance, geofencing, RTH height or RC-loss settings. The RC and DJI
aircraft-native failsafes retain authority. No automatic cloud-loss RTH is
claimed in this limited mode: the app is not flying the aircraft.

## Build and tomorrow's review/ground check

From `app`, run `npm.cmd run android:apk`. This runs TypeScript/Vite, Capacitor
sync, Android unit tests and debug assembly. APK:
`app/android/app/build/outputs/apk/debug/app-debug.apk`.

After review and a separately authorized installation, use PowerShell:

```powershell
& "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe" logcat -v time -s "DronetectDJI:I" "*:S"
```

Expected observations, with aircraft stationary:

```text
registration=success
control=rc_owned autonomy=unavailable virtualStick=disabled
product=connected productId=0
telemetry_uplink=waiting reason=aircraft_or_valid_gps_sample_unavailable
# Once real GPS and the remaining fields are available:
drone_ws=connecting
drone_ws=connected
telemetry_uplink=sent source=dji state=LANDED gps=valid hz=2
command=received name=set_operating_config id=...
command=rejected name=set_operating_config id=... reason=local_autonomy_controller_required
```

The existing `video=...` and `telemetry=snapshot` logs should continue. Indoors,
remaining in `telemetry_uplink=waiting` is expected. Check source `dji` and real
position in the normal HUD/map once GPS becomes available. A simulator already
occupying `/ws?role=drone` is refused by the server, not displaced. Test aircraft
and network disconnect/reconnect without flight; no mission resumes on reconnect.
Review this limited command list and the server acknowledgement limitation
before any later command test. API support, callback results, home/RTH behavior,
gimbal direction/range and real GPS/map integration still need hardware checks.
