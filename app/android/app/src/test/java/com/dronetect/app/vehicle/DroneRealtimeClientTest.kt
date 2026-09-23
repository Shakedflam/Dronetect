package com.dronetect.app.vehicle

import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.CopyOnWriteArrayList
import java.util.concurrent.atomic.AtomicInteger
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import okhttp3.mockwebserver.MockResponse
import okhttp3.mockwebserver.MockWebServer
import org.json.JSONObject
import org.junit.Assert.*
import org.junit.Test

class DroneRealtimeClientTest {
    @Test fun sendsGroundSafetyStateWithoutGpsPosition() {
        val server = MockWebServer()
        val received = CountDownLatch(1)
        val telemetryCount = AtomicInteger(0)
        server.enqueue(MockResponse().withWebSocketUpgrade(object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                webSocket.send("""{"type":"dji_hello","sessionId":"${"b".repeat(32)}"}""")
            }
            override fun onMessage(webSocket: WebSocket, text: String) {
                val message = JSONObject(text)
                when (message.optString("type")) {
                    "dji_telemetry" -> telemetryCount.incrementAndGet()
                    "dji_rc_state" -> {
                        assertTrue(message.getBoolean("aircraftConnected"))
                        assertFalse(message.getBoolean("isFlying"))
                        assertTrue(message.getBoolean("connected"))
                        received.countDown()
                    }
                }
            }
        }))
        server.start()
        val snapshot = DjiTelemetrySnapshot(aircraftConnected = true, rcConnected = true,
            isFlying = false, rcLeftHorizontal = 0, rcLeftVertical = 0,
            rcRightHorizontal = 0, rcRightVertical = 0)
        val client = DroneRealtimeClient("ws://127.0.0.1:${server.port}", "a".repeat(64),
            { DjiTelemetryState("connected", snapshot, System.currentTimeMillis()) }, log = {})
        try {
            client.start()
            assertTrue(received.await(5, TimeUnit.SECONDS))
            assertEquals(0, telemetryCount.get())
        } finally { client.stop(); server.shutdown() }
    }

    @Test fun freshAwsSetpointSurvivesTabletClockSkewButExpiredAndNonzeroStillFail() {
        val server = MockWebServer()
        val session = "b".repeat(32)
        val accepted = CountDownLatch(1)
        val rejectedNonzero = CountDownLatch(1)
        val count = AtomicInteger(0)
        val remainingTtlMs = CopyOnWriteArrayList<Long>()
        val logs = CopyOnWriteArrayList<String>()
        val stick = object : ZeroStickControl {
            override fun newSession() = Unit
            override fun acceptZero(snapshot: DjiTelemetrySnapshot, deadlineMs: Long, nowMs: Long): Boolean {
                count.incrementAndGet()
                remainingTtlMs.add(deadlineMs - nowMs)
                accepted.countDown()
                return true
            }
            override fun tick(snapshot: DjiTelemetrySnapshot, nowMs: Long) = Unit
            override fun stop(reason: String) = Unit
        }
        server.enqueue(MockResponse().withWebSocketUpgrade(object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                webSocket.send("""{"type":"dji_hello","sessionId":"$session"}""")
            }
            override fun onMessage(webSocket: WebSocket, text: String) {
                val probe = JSONObject(text)
                if (probe.optString("type") != "dji_clock_probe") return
                val tabletTime = probe.getDouble("timestamp")
                val awsTime = tabletTime - 0.75 // Live failure: 750 ms cross-device skew exceeds 250 ms TTL.
                webSocket.send(JSONObject().put("type", "dji_clock_sync")
                    .put("sessionId", session).put("probeTimestamp", tabletTime)
                    .put("serverReceivedAt", awsTime).toString())
                fun send(seq: Int, issued: Double, x: Double) {
                    webSocket.send(JSONObject().put("type", "dji_setpoint")
                        .put("sessionId", session).put("seq", seq).put("issuedAt", issued)
                        .put("ttlMs", 250).put("velocity", JSONObject()
                            .put("x", x).put("y", 0.0).put("z", 0.0))
                        .put("yawDeg", 90.0).toString())
                }
                // Live failure: multiple 50 ms-spaced setpoints arrive in a burst.
                for (seq in 1..7) send(seq, awsTime - (1.45 - seq * 0.05), 0.0)
                send(8, awsTime, 0.0)       // Fresh despite tablet clock offset.
                send(9, awsTime + 0.5, 0.0) // Slightly future timestamp cannot extend lease.
                send(10, awsTime, 1.0)      // Non-zero remains blocked.
            }
        }))
        server.start()
        val snapshot = DjiTelemetrySnapshot(aircraftConnected = true)
        val client = DroneRealtimeClient("ws://127.0.0.1:${server.port}", "a".repeat(64),
            { DjiTelemetryState("connected", snapshot, System.currentTimeMillis()) },
            log = {
                logs.add(it)
                if (it.contains("nonzero_or_invalid_ground_test")) rejectedNonzero.countDown()
            }, zeroStickControl = stick)
        try {
            client.start()
            assertTrue(accepted.await(5, TimeUnit.SECONDS))
            assertTrue(rejectedNonzero.await(5, TimeUnit.SECONDS))
            assertEquals(2, count.get())
            assertTrue(remainingTtlMs.all { it in 0L..250L })
            assertTrue(logs.any { it.contains("expired_or_invalid") &&
                it.contains("rxGapMs=") && it.contains("queueDelayMs=") &&
                it.contains("ageAtReceiveMs=") &&
                it.contains("syncRttMs=") && it.contains("syncAgeMs=") })
            assertTrue(logs.any { it.contains("bridge=clock_sync rttMs=") })
            assertTrue(logs.any { it.contains("bridge_action=accepted name=dji_setpoint") })
        } finally { client.stop(); server.shutdown() }
    }

    @Test fun zeroStickAcceptsOnlyFreshZeroCurrentSessionAndStopsOnDisconnect() {
        val server = MockWebServer()
        val session = "b".repeat(32)
        val accepted = CountDownLatch(1)
        val stopped = CountDownLatch(1)
        val stick = object : ZeroStickControl {
            override fun newSession() = Unit
            override fun acceptZero(snapshot: DjiTelemetrySnapshot, deadlineMs: Long, nowMs: Long): Boolean {
                accepted.countDown()
                return true
            }
            override fun tick(snapshot: DjiTelemetrySnapshot, nowMs: Long) = Unit
            override fun stop(reason: String) { if (reason.startsWith("bridge_")) stopped.countDown() }
        }
        server.enqueue(MockResponse().withWebSocketUpgrade(object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                webSocket.send("""{"type":"dji_hello","sessionId":"$session"}""")
            }
            override fun onMessage(webSocket: WebSocket, text: String) {
                val probe = JSONObject(text)
                if (probe.optString("type") != "dji_clock_probe") return
                val now = System.currentTimeMillis() / 1000.0
                webSocket.send(JSONObject().put("type", "dji_clock_sync")
                    .put("sessionId", session).put("probeTimestamp", probe.getDouble("timestamp"))
                    .put("serverReceivedAt", now).toString())
                fun send(id: String, seq: Int, issued: Double, x: Double) {
                    webSocket.send(JSONObject().put("type", "dji_setpoint").put("sessionId", id)
                        .put("seq", seq).put("issuedAt", issued).put("ttlMs", 250)
                        .put("velocity", JSONObject().put("x", x).put("y", 0).put("z", 0))
                        .put("yawDeg", 90).toString())
                }
                send("c".repeat(32), 1, now, 0.0)
                send(session, 2, now - 5, 0.0)
                send(session, 3, now, 1.0)
                send(session, 4, now, 0.0)
            }
        }))
        server.start()
        val snapshot = DjiTelemetrySnapshot(aircraftConnected = true, rcConnected = true,
            isFlying = false, rcLeftHorizontal = 0, rcLeftVertical = 0,
            rcRightHorizontal = 0, rcRightVertical = 0)
        val client = DroneRealtimeClient("ws://127.0.0.1:${server.port}", "a".repeat(64),
            { DjiTelemetryState("connected", snapshot, System.currentTimeMillis()) },
            log = {}, zeroStickControl = stick)
        try {
            client.start()
            assertTrue(accepted.await(5, TimeUnit.SECONDS))
        } finally {
            client.stop()
            assertTrue(stopped.await(5, TimeUnit.SECONDS))
            server.shutdown()
        }
    }

    @Test fun acceptsOnlyFreshCurrentSessionGimbalAndRthAndNeverSetpoint() {
        val server = MockWebServer()
        val session = "b".repeat(32)
        val accepted = CountDownLatch(2)
        val names = CopyOnWriteArrayList<String>()
        val adapter = object : VehicleAdapter {
            override val availability = VehicleAvailability.CONNECTED
            override fun start() = Unit
            override fun stop() = Unit
            override fun onControlLinkLost() = Unit
            override fun telemetrySnapshot(): VehicleTelemetry? = null
            override fun latestVideoFrame(): VehicleVideoFrame? = null
            override fun handleCommand(command: VehicleCommand, complete: (CommandResult) -> Unit) {
                names.add(command.name)
                complete(CommandResult(true, "test_success"))
                accepted.countDown()
            }
        }
        server.enqueue(MockResponse().withWebSocketUpgrade(object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                val now = System.currentTimeMillis() / 1000.0
                webSocket.send("""{"type":"dji_hello","sessionId":"$session"}""")
                fun send(type: String, id: String, seq: Int, issued: Double, ttl: Int) {
                    webSocket.send(JSONObject().put("type", type).put("sessionId", id)
                        .put("seq", seq).put("issuedAt", issued).put("ttlMs", ttl)
                        .put("pitchDeg", -15.0).toString())
                }
                send("dji_gimbal_command", "c".repeat(32), 1, now, 1000)
                send("dji_gimbal_command", session, 1, now - 10, 1000)
                send("dji_gimbal_command", session, 2, now, 1000)
                send("dji_rth", session, 3, now, 1500)
                send("dji_setpoint", session, 4, now, 250)
            }
        }))
        server.start()
        val snapshot = DjiTelemetrySnapshot(aircraftConnected = true)
        val client = DroneRealtimeClient("ws://127.0.0.1:${server.port}", "a".repeat(64),
            { DjiTelemetryState("connected", snapshot, System.currentTimeMillis()) }, adapter, log = {})
        try {
            client.start()
            assertTrue(accepted.await(5, TimeUnit.SECONDS))
            assertEquals(listOf("gimbal_pitch", "return_home"), names.toList())
        } finally { client.stop(); server.shutdown() }
    }

    @Test fun authenticatedHardwareSessionSendsMeasuredTelemetryAndNeverClaimsDroneRole() {
        val server = MockWebServer()
        val received = CountDownLatch(3)
        val token = "a".repeat(64)
        server.enqueue(MockResponse().withWebSocketUpgrade(object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                webSocket.send("""{"type":"dji_hello","sessionId":"${"b".repeat(32)}"}""")
            }
            override fun onMessage(webSocket: WebSocket, text: String) {
                val message = JSONObject(text)
                assertEquals("b".repeat(32), message.getString("sessionId"))
                when (message.getString("type")) {
                    "dji_telemetry" -> {
                        assertEquals(31.5, message.getJSONObject("gps").getDouble("lat"), 0.00001)
                        assertEquals(97.8, message.getDouble("headingDeg"), 0.001)
                        received.countDown()
                    }
                    "dji_rc_state" -> received.countDown()
                    "dji_gimbal_state" -> received.countDown()
                    "dji_clock_probe" -> Unit
                    else -> fail("Unexpected hardware message")
                }
            }
        }))
        server.start()
        val snapshot = DjiTelemetrySnapshot(latitude = 31.5, longitude = 34.5,
            altitudeMeters = 0.0, headingDegrees = 97.8, speedMetersPerSecond = 0.0,
            batteryPercent = 64, gpsSignalLevel = "LEVEL_4", aircraftConnected = true,
            gimbalPitchDegrees = -15.0, rcConnected = true,
            rcLeftHorizontal = 0, rcLeftVertical = 0, rcRightHorizontal = 0, rcRightVertical = 0)
        val client = DroneRealtimeClient("ws://127.0.0.1:${server.port}", token,
            { DjiTelemetryState("connected", snapshot, System.currentTimeMillis()) }, log = {})
        try {
            client.start()
            assertTrue(received.await(5, TimeUnit.SECONDS))
            val request = server.takeRequest(1, TimeUnit.SECONDS)!!
            assertEquals("/dji/bridge", request.path)
            assertEquals("Bearer $token", request.getHeader("Authorization"))
        } finally { client.stop(); server.shutdown() }
    }
}
