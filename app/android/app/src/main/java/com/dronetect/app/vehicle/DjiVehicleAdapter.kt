package com.dronetect.app.vehicle

import android.util.Log

/** Command-rejecting boundary retained while MSDK runs in read-only mode. */
class DjiVehicleAdapter : VehicleAdapter {
    override val availability = VehicleAvailability.UNAVAILABLE

    override fun start() {
        Log.i(TAG, "DJI vehicle adapter is read-only; aircraft commands are disabled")
    }

    override fun stop() = Unit

    override fun handleCommand(command: VehicleCommand) = CommandResult(
        accepted = false,
        message = "DJI vehicle adapter is unavailable; command ${command.commandId} was not executed",
    )

    override fun telemetrySnapshot(): VehicleTelemetry? = null

    override fun latestVideoFrame(): VehicleVideoFrame? = null

    private companion object {
        const val TAG = "DronetectDjiAdapter"
    }
}
