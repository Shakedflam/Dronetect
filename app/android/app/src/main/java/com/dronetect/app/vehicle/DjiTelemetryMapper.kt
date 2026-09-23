package com.dronetect.app.vehicle

/** Maps only complete physical samples into the server's non-null GPS contract. */
class DjiTelemetryMapper {
    fun map(state: DjiTelemetryState, timestampSeconds: Double): VehicleTelemetry? {
        val value = state.snapshot
        if (state.connectionState != "connected" || value.aircraftConnected != true) return null
        val lat = value.latitude ?: return null
        val lng = value.longitude ?: return null
        val altitude = value.altitudeMeters ?: return null
        val heading = value.headingDegrees ?: return null
        val speed = value.speedMetersPerSecond ?: return null
        val battery = value.batteryPercent ?: return null
        val flying = value.isFlying ?: return null
        val mode = value.flightMode ?: return null
        if (!timestampSeconds.isFinite() || !lat.isFinite() || !lng.isFinite() ||
            lat !in -90.0..90.0 || lng !in -180.0..180.0 || (lat == 0.0 && lng == 0.0) ||
            !altitude.isFinite() || !heading.isFinite() || !speed.isFinite() || speed < 0 ||
            battery !in 0..100 || value.gpsSignalLevel !in VALID_GPS_LEVELS) return null
        val rth = flying && (mode == "GO_HOME" || value.goHomeState == "RETURNING_TO_HOME")
        val normalizedState = when {
            !flying -> "LANDED"
            mode in LANDING_MODES || value.goHomeState == "LANDING" -> "LANDING"
            rth -> "RETURNING_HOME"
            mode == "AUTO_TAKE_OFF" -> "TAKING_OFF"
            // Never advertise patrol/follow or native tablet control that we do not own.
            else -> "MANUAL"
        }
        return VehicleTelemetry(
            timestampSeconds, lat, lng, altitude, (heading % 360 + 360) % 360,
            speed, battery.toDouble(), normalizedState,
            aircraftSafetyRthActive = rth,
            aircraftSafetyRthReason = if (rth) "aircraft_safety_rth" else null,
            manualInputSource = "rc",
        )
    }

    private companion object {
        val VALID_GPS_LEVELS = setOf("LEVEL_2", "LEVEL_3", "LEVEL_4", "LEVEL_5")
        val LANDING_MODES = setOf("AUTO_LANDING", "FORCE_LANDING", "ATTI_LANDING")
    }
}
