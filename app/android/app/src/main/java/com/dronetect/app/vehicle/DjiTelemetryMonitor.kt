package com.dronetect.app.vehicle

import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.v5.manager.KeyManager
import java.util.Locale

/** Latest read-only aircraft state retained for a future backend/UI adapter. */
data class DjiTelemetrySnapshot(
    val latitude: Double? = null,
    val longitude: Double? = null,
    val altitudeMeters: Double? = null,
    val headingDegrees: Double? = null,
    val batteryPercent: Int? = null,
    val gpsSatelliteCount: Int? = null,
    val gpsSignalLevel: String? = null,
    val isFlying: Boolean? = null,
    val aircraftConnected: Boolean? = null,
    val speedMetersPerSecond: Double? = null,
    val flightMode: String? = null,
    val goHomeState: String? = null,
    val gimbalPitchDegrees: Double? = null,
    val rcConnected: Boolean? = null,
    val rcLeftHorizontal: Int? = null,
    val rcLeftVertical: Int? = null,
    val rcRightHorizontal: Int? = null,
    val rcRightVertical: Int? = null,
)

/** Owns only listenable, non-settable DJI telemetry keys. */
class DjiTelemetryMonitor(private val onSnapshotChanged: (DjiTelemetrySnapshot) -> Unit = {}) {
    private val lock = Any()
    private val listenerHolder = Any()
    private val logHandler = Handler(Looper.getMainLooper())

    @Volatile
    private var latest = DjiTelemetrySnapshot()

    private var listening = false
    private var logScheduled = false
    private var lastLogTimeMs = 0L
    private var lastLoggedSnapshot: DjiTelemetrySnapshot? = null

    private val locationKey = KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D)
    private val altitudeKey = KeyTools.createKey(FlightControllerKey.KeyAltitude)
    private val headingKey = KeyTools.createKey(FlightControllerKey.KeyCompassHeading)
    private val batteryPercentKey = KeyTools.createKey(
        BatteryKey.KeyChargeRemainingInPercent,
        ComponentIndexType.LEFT_OR_MAIN,
    )
    private val gpsSatelliteCountKey = KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount)
    private val gpsSignalLevelKey = KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel)
    private val isFlyingKey = KeyTools.createKey(FlightControllerKey.KeyIsFlying)

    private val logRunnable = Runnable {
        val snapshot = synchronized(lock) {
            logScheduled = false
            latest
        }
        if (snapshot == lastLoggedSnapshot) return@Runnable

        lastLoggedSnapshot = snapshot
        lastLogTimeMs = SystemClock.elapsedRealtime()
        Log.i(DjiSdkRuntime.TAG, snapshot.toLogMessage())
    }

    fun snapshot(): DjiTelemetrySnapshot = latest

    fun start(productId: Int) {
        synchronized(lock) {
            if (listening) return
            listening = true
            latest = DjiTelemetrySnapshot()
            lastLoggedSnapshot = null
            lastLogTimeMs = 0L
        }

        Log.i(DjiSdkRuntime.TAG, "telemetry=listeners_started productId=$productId")
        listen("location", locationKey) { location ->
            update {
                copy(
                    latitude = location.validLatitude(),
                    longitude = location.validLongitude(),
                )
            }
        }
        listen("altitude", altitudeKey) { altitude ->
            update { copy(altitudeMeters = altitude?.takeIf(Double::isFinite)) }
        }
        listen("heading", headingKey) { heading ->
            update { copy(headingDegrees = heading?.takeIf(Double::isFinite)) }
        }
        listen("battery", batteryPercentKey) { percent ->
            update { copy(batteryPercent = percent?.takeIf { it in 0..100 }) }
        }
        listen("gps_satellites", gpsSatelliteCountKey) { count ->
            update { copy(gpsSatelliteCount = count?.takeIf { it >= 0 }) }
        }
        listen("gps_signal", gpsSignalLevelKey) { level ->
            update { copy(gpsSignalLevel = level?.name) }
        }
        listen("flight_state", isFlyingKey) { flying ->
            update { copy(isFlying = flying) }
        }
        listen("aircraft_connection", KeyTools.createKey(FlightControllerKey.KeyConnection)) { connected ->
            update { copy(aircraftConnected = connected) }
        }
        listen("velocity", KeyTools.createKey(FlightControllerKey.KeyAircraftVelocity)) { velocity ->
            val x = velocity?.x
            val y = velocity?.y
            val z = velocity?.z
            val speed = if (x != null && y != null && z != null &&
                x.isFinite() && y.isFinite() && z.isFinite()) Math.hypot(Math.hypot(x, y), z) else null
            update { copy(speedMetersPerSecond = speed) }
        }
        listen("flight_mode", KeyTools.createKey(FlightControllerKey.KeyFlightMode)) { mode ->
            update { copy(flightMode = mode?.name) }
        }
        listen("go_home_state", KeyTools.createKey(FlightControllerKey.KeyGoHomeStatus)) { state ->
            update { copy(goHomeState = state?.name) }
        }
        val main = ComponentIndexType.LEFT_OR_MAIN
        listen("gimbal_attitude", KeyTools.createKey(GimbalKey.KeyGimbalAttitude, main)) { attitude ->
            update { copy(gimbalPitchDegrees = attitude?.pitch?.takeIf(Double::isFinite)) }
        }
        listen("rc_connection", KeyTools.createKey(RemoteControllerKey.KeyConnection)) { connected ->
            update { copy(rcConnected = connected) }
        }
        listen("rc_left_horizontal", KeyTools.createKey(RemoteControllerKey.KeyStickLeftHorizontal)) { value ->
            update { copy(rcLeftHorizontal = value?.takeIf { it in -660..660 }) }
        }
        listen("rc_left_vertical", KeyTools.createKey(RemoteControllerKey.KeyStickLeftVertical)) { value ->
            update { copy(rcLeftVertical = value?.takeIf { it in -660..660 }) }
        }
        listen("rc_right_horizontal", KeyTools.createKey(RemoteControllerKey.KeyStickRightHorizontal)) { value ->
            update { copy(rcRightHorizontal = value?.takeIf { it in -660..660 }) }
        }
        listen("rc_right_vertical", KeyTools.createKey(RemoteControllerKey.KeyStickRightVertical)) { value ->
            update { copy(rcRightVertical = value?.takeIf { it in -660..660 }) }
        }
    }

    fun restart(productId: Int) {
        stop("product_changed")
        start(productId)
    }

    fun stop(reason: String) {
        val wasListening = synchronized(lock) {
            if (!listening) return
            listening = false
            logScheduled = false
            true
        }
        if (!wasListening) return

        KeyManager.getInstance().cancelListen(listenerHolder)
        logHandler.removeCallbacks(logRunnable)
        latest = DjiTelemetrySnapshot()
        Log.i(DjiSdkRuntime.TAG, "telemetry=listeners_stopped reason=$reason")
    }

    private fun <T> listen(name: String, key: DJIKey<T>, onValue: (T?) -> Unit) {
        try {
            val keyManager = KeyManager.getInstance()
            if (!keyManager.isKeySupported(key)) {
                Log.w(DjiSdkRuntime.TAG, "telemetry=$name state=unsupported")
                return
            }
            keyManager.listen(key, listenerHolder, true) { _, newValue -> onValue(newValue) }
        } catch (error: Throwable) {
            Log.e(DjiSdkRuntime.TAG, "telemetry=$name state=listener_failed", error)
        }
    }

    private fun update(transform: DjiTelemetrySnapshot.() -> DjiTelemetrySnapshot) {
        val changed = synchronized(lock) {
            if (!listening) return
            val next = latest.transform()
            if (next == latest) return
            latest = next
            next
        }
        onSnapshotChanged(changed)
        scheduleLog()
    }

    private fun scheduleLog() {
        val delayMs = synchronized(lock) {
            if (!listening || logScheduled) return
            logScheduled = true
            val elapsed = SystemClock.elapsedRealtime() - lastLogTimeMs
            if (lastLogTimeMs == 0L) INITIAL_LOG_DELAY_MS else (LOG_INTERVAL_MS - elapsed).coerceAtLeast(0L)
        }
        logHandler.postDelayed(logRunnable, delayMs)
    }

    private fun LocationCoordinate3D?.validLatitude(): Double? {
        val value = this?.latitude ?: return null
        val longitude = this.longitude ?: return null
        return value.takeIf {
            it.isFinite() && longitude.isFinite() && it in -90.0..90.0 &&
                longitude in -180.0..180.0 && !(it == 0.0 && longitude == 0.0)
        }
    }

    private fun LocationCoordinate3D?.validLongitude(): Double? {
        validLatitude() ?: return null
        return this?.longitude
    }

    private fun DjiTelemetrySnapshot.toLogMessage(): String = buildString {
        append("telemetry=snapshot")
        append(" latitude=${latitude.format(6)}")
        append(" longitude=${longitude.format(6)}")
        append(" altitudeM=${altitudeMeters.format(2)}")
        append(" headingDeg=${headingDegrees.format(1)}")
        append(" batteryPercent=${batteryPercent ?: "unavailable"}")
        append(" gpsSatellites=${gpsSatelliteCount ?: "unavailable"}")
        append(" gpsSignal=${gpsSignalLevel ?: "unavailable"}")
        append(" isFlying=${isFlying ?: "unavailable"}")
        append(
            " groundState=" + when (isFlying) {
                true -> "airborne"
                false -> "on_ground"
                null -> "unavailable"
            },
        )
    }

    private fun Double?.format(decimalPlaces: Int): String =
        this?.let { String.format(Locale.US, "%.${decimalPlaces}f", it) } ?: "unavailable"

    private companion object {
        const val INITIAL_LOG_DELAY_MS = 500L
        const val LOG_INTERVAL_MS = 2_000L
    }
}
