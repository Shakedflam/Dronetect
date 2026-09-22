# Android tablet foundation

Dronetect remains a browser-first React/Vite application. The Android project
under `app/android` is a Capacitor shell around the same compiled UI; it does
not fork or duplicate any mission, patrol, tracking, pursuit, or presentation
logic.

## Runtime layouts

Local development and demonstrations retain the existing path:

```text
Browser UI -> local realtime/vision services -> Webots simulator
```

The reserved real-aircraft path is:

```text
Capacitor UI -- role=tablet --> production realtime/vision services
Android vehicle gateway -- role=drone + video ingest --> production services
Android vehicle gateway -> DJI MSDK -> RC-N3 -> Mini 4 Pro
```

Only the first line is active in the current APK. The native vehicle adapter
is deliberately unavailable and cannot open a drone connection or execute a
command until the project hardware and DJI APIs have been verified.

## Browser development

The browser default remains `ws://127.0.0.1:8765`:

```powershell
cd app
npm.cmd run dev
```

Start the Python realtime server, vision server, and Webots exactly as described
in the root README. No Android tooling is required for this mode.

## Android prerequisites

- Android Studio with Android SDK 35 and platform/build tools installed.
- JDK 21. The repository's Android commands select it without changing the
  machine-wide `JAVA_HOME`.
- An arm64 Android tablet. The project intentionally filters to `arm64-v8a`,
  matching DJI MSDK v5.
- USB debugging enabled for direct installation.

Capacitor is pinned to 7.6.9 so the generated project compiles and targets API
35. DJI MSDK is not a dependency yet.

## Endpoint configuration

Copy `app/.env.android.example` to `app/.env.android` and set an origin only,
without `/ws` or a role query:

```dotenv
VITE_REALTIME_SERVER_WS_URL=wss://realtime.example.com
```

Production APKs require `wss://`. For a debug-only LAN simulator test, use the
development computer's reachable LAN address, not `127.0.0.1`:

```dotenv
VITE_REALTIME_SERVER_WS_URL=ws://192.168.1.20:8765
```

The debug manifest permits cleartext traffic for this test. Release builds
deny it. The Python server must bind to a LAN interface and the firewall must
allow the selected port.

## Build, install, and inspect

From `app`:

```powershell
npm.cmd run android:sync
npm.cmd run android:apk
C:\Users\User\AppData\Local\Android\Sdk\platform-tools\adb.exe install -r android\app\build\outputs\apk\debug\app-debug.apk
```

`android:sync` always rebuilds the Vite bundle before copying it into the
native project. `android:open` opens the generated project in Android Studio.

The debug APK is directly installable and is the expected project-tablet
artifact for this phase. `npm.cmd run android:apk:release` builds a release
variant. To sign it, copy `android/signing.properties.example` to
`android/signing.properties`, create a private keystore outside the repository,
and replace every placeholder. The real properties file, keystore, APKs, and
DJI keys are ignored by Git.

The npm Android commands use `scripts/run-android-gradle.mjs` to find JDK 21
and Android SDK 35. The selected paths are passed only to the spawned Gradle
process, so the global `JAVA_HOME` and Python/Docker/cloud workflows are not
changed. This also avoids accidentally running Gradle 8.11.1 with JDK 25,
which fails with `Unsupported class file major version 69`. Override discovery
with `DRONTECT_ANDROID_JAVA_HOME` or `DRONTECT_ANDROID_SDK_ROOT` if required.

The checked-in `android/gradle/gradle-daemon-jvm.properties` also requests a
Java 21 Gradle daemon for direct wrapper and Android Studio builds. It specifies
only the Java version, not a developer-specific installation path.

Gradle 8.11.1 supports running on JDK 21, and Capacitor 7 compiles its Android
library with Java 21. DJI's current MSDK v5 integration guide recommends a Java
17 runtime for its sample stack but does not state that Java 21 is unsupported.
When MSDK is added, its exact selected release must be validated on JDK 21 with
the project hardware before flight functionality is enabled.

## LAN smoke test

With the local Python services and simulator running, install the debug APK
configured with the computer's LAN address. Verify that the packaged UI shows
the same mission status, map, telemetry, detections, command acknowledgements,
and WebRTC video as the browser UI. Only one tablet client is supported by the
current realtime server, so close the browser UI before opening the APK.

The current WebRTC client and server now accept STUN configuration, but a
production network may still require a credentialed TURN service. A cloud
deployment also needs TLS termination, authentication, and explicit pairing of
tablet and aircraft sessions.

## DJI integration boundary

Native code lives in
`app/android/app/src/main/java/com/dronetect/app/vehicle`. `VehicleAdapter`
defines normalized lifecycle, command, telemetry, and JPEG-frame values.
`DjiVehicleAdapter`, `DjiTelemetryMapper`, `DjiCommandDispatcher`, and
`DjiVideoSource` are fail-closed seams with no DJI imports. `DroneRealtimeClient`
will eventually own the native drone-role and video-ingest connections, but it
refuses to connect while the adapter is unavailable.

Before enabling that boundary, verify all of the following on the actual
hardware: Mini 4 Pro and RC-N3 firmware, tablet Android/API and arm64 ABI, USB
link behavior, DJI developer App Key registration, aircraft telemetry, camera
frames, RC-stick priority, supported flight/gimbal/RTH commands, connection-loss
behavior, and aircraft-native failsafes. Store the DJI App Key in an untracked
local Gradle property or CI secret; never add it to TypeScript or source control.

The existing autonomous flight execution is currently part of the Webots
controller rather than the realtime server. Enabling a real aircraft therefore
requires a separate reviewed design for that execution layer; the Android shell
must not approximate those semantics with guessed MSDK calls.
