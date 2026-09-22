package com.dronetect.app.vehicle

/** Vendor video seam; no camera feed API is assumed by the Android shell. */
class DjiVideoSource {
    fun latestFrame(): VehicleVideoFrame? = null
}
