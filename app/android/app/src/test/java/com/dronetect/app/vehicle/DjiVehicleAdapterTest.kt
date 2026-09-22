package com.dronetect.app.vehicle

import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class DjiVehicleAdapterTest {
    @Test
    fun unavailableAdapterRejectsCommandsWithoutSideEffects() {
        val adapter = DjiVehicleAdapter()

        assertTrue(adapter.availability == VehicleAvailability.UNAVAILABLE)
        assertFalse(adapter.handleCommand(VehicleCommand("test-1", "return_home")).accepted)
        assertNull(adapter.telemetrySnapshot())
        assertNull(adapter.latestVideoFrame())
    }

    @Test
    fun realtimeClientDoesNotConnectWithoutVehicleSupport() {
        val client = DroneRealtimeClient("wss://realtime.example.com", DjiVehicleAdapter())

        assertFalse(client.start().accepted)
    }
}
