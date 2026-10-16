package com.dronetect.app

import android.os.Handler
import android.os.Looper
import com.dronetect.app.vehicle.DjiSdkRuntime
import com.dronetect.app.vehicle.DjiTelemetryState
import com.getcapacitor.JSObject
import com.getcapacitor.Plugin
import com.getcapacitor.PluginCall
import com.getcapacitor.annotation.CapacitorPlugin
import com.getcapacitor.PluginMethod
import org.json.JSONObject

/** Read-only Capacitor handoff for the existing WebView telemetry service. */
@CapacitorPlugin(name = "DjiTelemetry")
class DjiTelemetryPlugin : Plugin() {
    private val mainHandler = Handler(Looper.getMainLooper())
    private val observer: (DjiTelemetryState) -> Unit = { state ->
        mainHandler.post { notifyListeners("snapshot", state.toJsObject()) }
    }

    override fun load() {
        DjiSdkRuntime.addTelemetryObserver(observer)
    }

    override fun handleOnDestroy() {
        DjiSdkRuntime.removeTelemetryObserver(observer)
        mainHandler.removeCallbacksAndMessages(null)
    }

    @PluginMethod
    fun getServerCredentials(call: PluginCall) {
        call.resolve(JSObject().apply {
            put("operator", BuildConfig.DRONTECT_OPERATOR_TOKEN)
            put("viewer", BuildConfig.DRONTECT_VIEWER_TOKEN)
        })
    }

    @PluginMethod
    fun getSnapshot(call: PluginCall) {
        call.resolve(DjiSdkRuntime.telemetryState().toJsObject())
    }

    private fun DjiTelemetryState.toJsObject(): JSObject = JSObject().apply {
        put("source", "dji")
        put("timestampMs", timestampMs)
        put("connectionState", connectionState)
        put("latitude", snapshot.latitude ?: JSONObject.NULL)
        put("longitude", snapshot.longitude ?: JSONObject.NULL)
        put("altitudeM", snapshot.altitudeMeters ?: JSONObject.NULL)
        put("headingDeg", snapshot.headingDegrees ?: JSONObject.NULL)
        put("batteryPercent", snapshot.batteryPercent ?: JSONObject.NULL)
        put("gpsSatellites", snapshot.gpsSatelliteCount ?: JSONObject.NULL)
        put("gpsSignal", snapshot.gpsSignalLevel ?: JSONObject.NULL)
        put("isFlying", snapshot.isFlying ?: JSONObject.NULL)
        put("groundState", when (snapshot.isFlying) {
            true -> "airborne"
            false -> "on_ground"
            null -> "unavailable"
        })
    }
}
