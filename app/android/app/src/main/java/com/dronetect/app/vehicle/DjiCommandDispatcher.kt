package com.dronetect.app.vehicle

/** Vendor command seam; rejects everything until supported MSDK calls are verified. */
class DjiCommandDispatcher {
    fun dispatch(command: VehicleCommand) = CommandResult(
        accepted = false,
        message = "No verified DJI command mapping exists for ${command.name}",
    )
}
