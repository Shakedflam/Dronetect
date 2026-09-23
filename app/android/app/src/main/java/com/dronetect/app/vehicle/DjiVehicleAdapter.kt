package com.dronetect.app.vehicle

import android.os.SystemClock

/** Native actions and telemetry; never impersonates the missing local autonomy controller. */
class DjiVehicleAdapter(
    private val state: () -> DjiTelemetryState = DjiSdkRuntime::telemetryState,
    actions: DjiAircraftActions = DjiSdkAircraftActions(),
    clockMs: () -> Long = SystemClock::elapsedRealtime,
    private val timestampSeconds: () -> Double = { System.currentTimeMillis() / 1000.0 },
) : VehicleAdapter {
    @Volatile private var started = false
    private val mapper = DjiTelemetryMapper()
    private val dispatcher = DjiCommandDispatcher(actions, { availability == VehicleAvailability.CONNECTED }, clockMs)

    override val availability: VehicleAvailability
        get() = if (!started) VehicleAvailability.UNAVAILABLE else {
            val current = state()
            if (current.connectionState == "connected" && current.snapshot.aircraftConnected == true)
                VehicleAvailability.CONNECTED else VehicleAvailability.DISCONNECTED
        }

    override fun start() {
        started = true
    }

    override fun stop() {
        started = false
        onControlLinkLost()
    }

    override fun handleCommand(command: VehicleCommand, complete: (CommandResult) -> Unit) =
        dispatcher.dispatch(command, complete)

    override fun onControlLinkLost() = dispatcher.invalidate()

    override fun telemetrySnapshot(): VehicleTelemetry? {
        dispatcher.expire()
        if (availability != VehicleAvailability.CONNECTED) return null
        return mapper.map(state(), timestampSeconds())?.copy(pursuitBlocked = dispatcher.pursuitBlocked)
    }

    override fun latestVideoFrame(): VehicleVideoFrame? = null

}
