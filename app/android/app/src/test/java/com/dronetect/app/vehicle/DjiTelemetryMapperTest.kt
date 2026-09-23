package com.dronetect.app.vehicle

import org.json.JSONObject
import org.junit.Assert.*
import org.junit.Test

class DjiTelemetryMapperTest {
    private val mapper = DjiTelemetryMapper()
    private val valid = DjiTelemetrySnapshot(
        latitude = 31.5, longitude = 34.5, altitudeMeters = 0.0,
        headingDegrees = -90.0, batteryPercent = 64, gpsSatelliteCount = 12,
        gpsSignalLevel = "LEVEL_5", isFlying = false,
        aircraftConnected = true, speedMetersPerSecond = 0.0, flightMode = "GPS_NORMAL",
    )
    private fun map(snapshot: DjiTelemetrySnapshot) = mapper.map(DjiTelemetryState("connected", snapshot, 1000), 2.0)

    @Test fun noGpsNoConnectionOrUnknownMeasurementsNeverProduceDefaultPosition() {
        assertNull(map(valid.copy(latitude = null, longitude = null)))
        assertNull(map(valid.copy(latitude = Double.NaN)))
        assertNull(map(valid.copy(latitude = 0.0, longitude = 0.0)))
        assertNull(map(valid.copy(gpsSignalLevel = "LEVEL_0")))
        assertNull(map(valid.copy(speedMetersPerSecond = null)))
        assertNull(map(valid.copy(isFlying = null)))
        assertNull(map(valid.copy(aircraftConnected = false)))
        assertNull(mapper.map(DjiTelemetryState("disconnected", valid, 1000), 2.0))
    }

    @Test fun mapsRealStateAndUsesExistingServerEnvelope() {
        val grounded = map(valid)!!
        assertEquals("LANDED", grounded.state)
        assertEquals(270.0, grounded.headingDegrees, 0.0)
        val json = JSONObject(DroneWireProtocol.telemetry(grounded))
        assertEquals("dji", json.getString("source"))
        assertEquals(31.5, json.getJSONObject("data").getJSONObject("gps").getDouble("lat"), 0.0)
        assertEquals("rc", json.getJSONObject("data").getString("manualInputSource"))
        val rth = map(valid.copy(isFlying = true, flightMode = "GO_HOME"))!!
        assertTrue(rth.aircraftSafetyRthActive)
        assertEquals("RETURNING_HOME", rth.state)
        assertEquals("LANDING", map(valid.copy(isFlying = true, flightMode = "FORCE_LANDING"))!!.state)
    }

    @Test fun malformedCommandsAndUnimplementedConfigProduceExplicitRejection() {
        assertTrue(runCatching { DroneWireProtocol.parseCommand(JSONObject("""{"type":"command","commandId":"a\nb","command":"return_home"}""")) }.isFailure)
        val json = JSONObject(DroneWireProtocol.operatingConfigResult("id", CommandResult(false, "local_autonomy_controller_required")))
        assertEquals("operating_config_result", json.getString("type"))
        assertFalse(json.getBoolean("accepted"))
        assertFalse(json.getBoolean("pending"))
    }
}
