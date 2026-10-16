# Dronetect operator application

React and TypeScript operator interface, built with Vite and Leaflet and packaged
for Android with Capacitor. The Kotlin integration connects to DJI MSDK, publishes
telemetry and video, and applies aircraft-side command validation and safety limits.

See the [project README](../README.md) for architecture, features, prerequisites,
setup instructions, and the distinction between simulator and hardware validation.

- [Android setup](../docs/android.md)
- [Server authentication and private configuration](../docs/server-access.md)
- [Licensing](../LICENSING.md)

Access keys belong in ignored local configuration. Do not put them in `VITE_*`
variables or publish generated APKs containing private credentials.
