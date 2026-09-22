package com.dronetect.app.vehicle

import android.util.Log

/**
 * Deliberately inert until the RC-N3/tablet/firmware matrix and DJI MSDK APIs
 * are verified on project hardware. It cannot connect or execute commands.
 */
class DjiVehicleAdapter : VehicleAdapter {
    override val availability = VehicleAvailability.UNAVAILABLE

    override fun start() {
        Log.i(TAG, "DJI vehicle adapter is disabled; no MSDK dependency is installed")
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
