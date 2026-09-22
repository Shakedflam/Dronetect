package com.dronetect.app.vehicle

/**
 * Native aircraft boundary. Implementations normalize vendor state into the
 * existing realtime protocol; the WebView never imports a vendor SDK.
 */
interface VehicleAdapter {
    val availability: VehicleAvailability

    fun start()

    fun stop()

    fun handleCommand(command: VehicleCommand): CommandResult

    fun telemetrySnapshot(): VehicleTelemetry?

    fun latestVideoFrame(): VehicleVideoFrame?
}

enum class VehicleAvailability {
    UNAVAILABLE,
    DISCONNECTED,
    CONNECTED,
}

data class VehicleCommand(
    val commandId: String,
    val name: String,
    val params: Map<String, Any?> = emptyMap(),
)

data class CommandResult(
    val accepted: Boolean,
    val message: String,
)

data class VehicleTelemetry(
    val timestampSeconds: Double,
    val latitude: Double,
    val longitude: Double,
    val altitudeMeters: Double,
    val headingDegrees: Double,
    val speedMetersPerSecond: Double,
    val batteryPercent: Double,
    val state: String,
    val aircraftSafetyRthActive: Boolean = false,
    val aircraftSafetyRthReason: String? = null,
    val manualInputSource: String = "rc",
)

data class VehicleVideoFrame(
    val frameId: Long,
    val timestampSeconds: Double,
    val width: Int,
    val height: Int,
    val jpeg: ByteArray,
)
