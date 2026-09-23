package com.dronetect.app.vehicle

import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Assert.assertEquals
import org.junit.Test

class DjiVehicleAdapterTest {
    @Test
    fun stoppedAndDisconnectedAdapterRejectWithoutSdkActions() {
        var calls = 0
        val actions = object : DjiAircraftActions {
            override fun returnHome(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) { calls++ }
            override fun tiltCamera(up: Boolean, isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) { calls++ }
        }
        val adapter = DjiVehicleAdapter(
            state = { DjiTelemetryState("disconnected", DjiTelemetrySnapshot(), 0) },
            actions = actions, clockMs = { 0L }, timestampSeconds = { 0.0 },
        )
        assertEquals(VehicleAvailability.UNAVAILABLE, adapter.availability)
        adapter.handleCommand(VehicleCommand("test-1", "return_home")) { assertFalse(it.accepted) }
        adapter.start()
        assertEquals(VehicleAvailability.DISCONNECTED, adapter.availability)
        adapter.handleCommand(VehicleCommand("test-2", "return_home")) { assertFalse(it.accepted) }
        assertNull(adapter.telemetrySnapshot())
        assertNull(adapter.latestVideoFrame())
        assertEquals(0, calls)
    }

    @Test
    fun endpointUsesTheDedicatedHardwareBridgeWithoutVideoPath() {
        assertEquals("https://api.dronetect.tech/dji/bridge", DroneRealtimeClient.endpoint("wss://api.dronetect.tech"))
        assertTrue(runCatching { DroneRealtimeClient.endpoint("wss://user:password@host") }.isFailure)
        assertTrue(runCatching { DroneRealtimeClient.endpoint("wss://host/video/ingest?role=drone") }.isFailure)
    }
}
