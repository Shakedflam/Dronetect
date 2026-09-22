package com.dronetect.app.vehicle

/**
 * Reserved owner of the native `/ws?role=drone` and `/video/ingest` links.
 * Networking stays disabled until a real adapter is available, preventing the
 * shell APK from advertising itself as an aircraft or displacing a simulator.
 */
class DroneRealtimeClient(
    private val realtimeOrigin: String,
    private val adapter: VehicleAdapter,
) {
    fun start(): CommandResult {
        if (adapter.availability == VehicleAvailability.UNAVAILABLE) {
            return CommandResult(false, "Vehicle adapter is unavailable; realtime link was not opened")
        }
        return CommandResult(
            false,
            "Native realtime transport is reserved but not implemented for $realtimeOrigin",
        )
    }

    fun stop() = Unit
}
