package com.dronetect.app.vehicle

import android.util.Log
import com.dronetect.app.BuildConfig
import java.net.URI
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import org.json.JSONObject

/** Hardware telemetry and command link. The Python bridge alone owns role=drone. */
class DroneRealtimeClient(
    realtimeOrigin: String,
    private val bridgeToken: String,
    private val state: () -> DjiTelemetryState,
    private val adapter: VehicleAdapter = DjiVehicleAdapter(state = state),
    private val log: (String) -> Unit = { Log.i(DjiSdkRuntime.TAG, it) },
    private val client: OkHttpClient = OkHttpClient.Builder()
        .readTimeout(0, TimeUnit.MILLISECONDS)
        .connectTimeout(10, TimeUnit.SECONDS)
        .pingInterval(5, TimeUnit.SECONDS)
        .build(),
    zeroStickControl: ZeroStickControl? = null,
    private val phase4PulseEnabled: Boolean =
        BuildConfig.DRONTECT_DJI_PHASE4_VERTICAL_PULSE_ENABLED,
    private val autonomousEnabled: Boolean =
        BuildConfig.DRONTECT_DJI_AUTONOMOUS_ENABLED,
) {
    private val url = endpoint(realtimeOrigin)
    private val controlMode = when {
        autonomousEnabled -> "autonomous_flight_logic"
        phase4PulseEnabled -> "phase4_vertical_pulse"
        else -> "zero_stick_ground_test"
    }
    private val worker = Executors.newSingleThreadScheduledExecutor { task ->
        Thread(task, "dronetect-dji-bridge").apply { isDaemon = true }
    }
    private var zeroStick: ZeroStickControl? = zeroStickControl
    private fun stick(): ZeroStickControl = zeroStick ?: DjiZeroStickControl(
        dispatch = { worker.execute(it) }, log = log,
        phase4PulseEnabled = phase4PulseEnabled,
        autonomousEnabled = autonomousEnabled,
    ).also { zeroStick = it }
    private var running = false
    private var ticker: ScheduledFuture<*>? = null
    private var socket: WebSocket? = null
    private var sessionId: String? = null
    private var epoch = 0L
    private var sequence = 0L
    private var downlinkSequence = 0L
    private var nextConnectMs = 0L
    private var retryMs = 1_000L
    private var connectingAtMs = 0L
    private var lastLogMs = 0L
    private var lastTelemetryTickMs = 0L
    private var lastSetpointLogMs = 0L
    private var lastZeroRejectReason: String? = null
    private var lastZeroRejectLogMs = 0L
    private var lastExpiredLogMs = 0L
    private var lastClockSyncLogMs = 0L
    private var lastClockRoundTripMs = 0L
    private var phase4SetpointReceivedLogged = false
    private var lastSetpointReceivedElapsedMs = 0L
    private var pendingProbeTimestamp: Double? = null
    private var lastProbeElapsedMs = 0L
    private var lastClockSyncElapsedMs = 0L
    private var syncProbeSentElapsedMs = 0L
    private var clockServerReceivedAtMs: Double? = null

    fun start() {
        worker.execute {
            if (running) return@execute
            if (!bridgeToken.matches(Regex("[A-Fa-f0-9]{64}"))) {
                log("bridge=disabled reason=missing_private_token")
                return@execute
            }
            running = true
            adapter.start()
            log("bridge=starting mode=$controlMode")
            ticker = worker.scheduleWithFixedDelay({ tickSafely() }, 0, 100, TimeUnit.MILLISECONDS)
        }
    }

    fun stop() {
        worker.execute {
            running = false
            ticker?.cancel(false)
            ticker = null
            disconnect("stopped")
            adapter.stop()
        }
    }

    fun onAircraftChanged() {
        worker.execute { disconnect("aircraft_changed") }
    }

    private fun tickSafely() {
        try { tick() } catch (error: Exception) {
            log("bridge=error type=${error.javaClass.simpleName}")
            disconnect("tick_error")
        }
    }

    private fun tick() {
        if (!running) return
        adapter.telemetrySnapshot() // expires any pending SDK action without requiring GPS.
        val current = state()
        zeroStick?.tick(current.snapshot, System.nanoTime() / 1_000_000L)
        if (current.connectionState != "connected" || current.snapshot.aircraftConnected != true) {
            if (socket != null) disconnect("aircraft_disconnected")
            return
        }
        val now = System.currentTimeMillis()
        if (socket == null && now >= nextConnectMs) connect()
        val active = socket ?: return
        val session = sessionId
        if (session == null) {
            if (now - connectingAtMs > 10_000) disconnect("handshake_timeout")
            return
        }
        val elapsedMs = System.nanoTime() / 1_000_000L
        if (elapsedMs - lastProbeElapsedMs >= 2_000L) {
            val probeTimestamp = now / 1_000.0
            pendingProbeTimestamp = probeTimestamp
            lastProbeElapsedMs = elapsedMs
            if (!active.send(JSONObject().put("type", "dji_clock_probe")
                    .put("sessionId", session).put("seq", ++sequence)
                    .put("timestamp", probeTimestamp).toString())) {
                disconnect("clock_probe_send_failed")
                return
            }
        }
        if (active.queueSize() > 16_384) {
            disconnect("uplink_backpressure")
            return
        }
        if (now - lastTelemetryTickMs < 500L) return
        lastTelemetryTickMs = now
        val snapshot = current.snapshot
        val timestamp = now / 1000.0
        fun send(message: JSONObject): Boolean {
            message.put("sessionId", session).put("seq", ++sequence).put("timestamp", timestamp)
            return active.send(message.toString())
        }
        val latitude = snapshot.latitude
        val longitude = snapshot.longitude
        val altitude = snapshot.altitudeMeters
        val heading = snapshot.headingDegrees
        val speed = snapshot.speedMetersPerSecond
        val battery = snapshot.batteryPercent
        if (latitude != null && longitude != null && altitude != null && heading != null &&
            speed != null && battery != null && snapshot.gpsSignalLevel in VALID_GPS_LEVELS) {
            val telemetry = JSONObject().put("type", "dji_telemetry")
                .put("gps", JSONObject().put("lat", latitude).put("lng", longitude))
                .put("altitudeM", altitude).put("headingDeg", heading)
                .put("speedMps", speed).put("batteryPercent", battery)
                .put("gimbalPitchDeg", snapshot.gimbalPitchDegrees ?: JSONObject.NULL)
                .put("flightMode", snapshot.flightMode ?: "")
                .put("isFlying", snapshot.isFlying ?: JSONObject.NULL)
            if (!send(telemetry)) { disconnect("send_failed"); return }
            if (now - lastLogMs >= 5_000) {
                lastLogMs = now
                log("bridge=telemetry_sent gps=valid mode=$controlMode")
            }
        }
        val rc = snapshot.rcConnected
        val lh = snapshot.rcLeftHorizontal
        val lv = snapshot.rcLeftVertical
        val rh = snapshot.rcRightHorizontal
        val rv = snapshot.rcRightVertical
        if (rc != null && lh != null && lv != null && rh != null && rv != null) {
            val sticks = JSONObject().put("forward", rv / 660.0).put("right", rh / 660.0)
                .put("up", lv / 660.0).put("yawRate", lh / 660.0)
            if (!send(JSONObject().put("type", "dji_rc_state").put("connected", rc)
                    .put("aircraftConnected", snapshot.aircraftConnected ?: JSONObject.NULL)
                    .put("isFlying", snapshot.isFlying ?: JSONObject.NULL)
                    .put("stick", sticks))) {
                disconnect("send_failed"); return
            }
        }
        snapshot.gimbalPitchDegrees?.let { pitch ->
            if (!send(JSONObject().put("type", "dji_gimbal_state").put("pitchDeg", pitch))) {
                disconnect("send_failed")
            }
        }
    }

    private fun connect() {
        val token = ++epoch
        connectingAtMs = System.currentTimeMillis()
        log("bridge=connecting endpoint=/dji/bridge")
        val request = Request.Builder().url(url).header("Authorization", "Bearer $bridgeToken").build()
        socket = client.newWebSocket(request, object : WebSocketListener() {
            override fun onMessage(webSocket: WebSocket, text: String) {
                val receivedElapsedMs = System.nanoTime() / 1_000_000L
                worker.execute {
                    if (!running || token != epoch || text.length > 4096) return@execute
                    try {
                        val message = JSONObject(text)
                        if (message.optString("type") == "dji_hello" && sessionId == null) {
                            val id = message.optString("sessionId")
                            if (id.matches(Regex("[a-f0-9]{32}"))) {
                                sessionId = id
                                sequence = 0
                                downlinkSequence = 0
                                retryMs = 1_000
                                clockServerReceivedAtMs = null
                                pendingProbeTimestamp = null
                                lastProbeElapsedMs = 0L
                                lastClockSyncElapsedMs = 0L
                                lastClockRoundTripMs = 0L
                                lastSetpointReceivedElapsedMs = 0L
                                lastZeroRejectReason = null
                                phase4SetpointReceivedLogged = false
                                zeroStick?.newSession()
                                log("bridge=connected mode=$controlMode")
                            }
                        } else if (message.optString("type") == "dji_clock_sync") {
                            receiveClockSync(message)
                        } else {
                            receiveAction(message, receivedElapsedMs)
                        }
                    } catch (_: Exception) {
                        log("bridge=message_ignored reason=invalid_json")
                        if (phase4PulseEnabled) zeroStick?.stop("invalid_message")
                    }
                }
            }
            override fun onFailure(webSocket: WebSocket, error: Throwable, response: Response?) {
                worker.execute { if (token == epoch) disconnect("http_${response?.code() ?: 0}_${error.javaClass.simpleName}") }
            }
            override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                worker.execute { if (token == epoch) disconnect("server_close_$code") }
            }
            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                worker.execute { if (token == epoch) disconnect("closed_$code") }
            }
        })
    }

    private fun receiveClockSync(message: JSONObject) {
        val probe = (message.opt("probeTimestamp") as? Number)?.toDouble()
        val received = (message.opt("serverReceivedAt") as? Number)?.toDouble()
        val nowElapsed = System.nanoTime() / 1_000_000L
        if (message.optString("sessionId") != sessionId || probe == null || received == null ||
            !probe.isFinite() || !received.isFinite() || probe != pendingProbeTimestamp ||
            nowElapsed - lastProbeElapsedMs !in 0L..500L) return
        // S_receive + tablet monotonic time since C_send is an upper bound
        // on AWS-now (it includes upstream transit), so expiry stays strict.
        clockServerReceivedAtMs = received * 1_000.0
        syncProbeSentElapsedMs = lastProbeElapsedMs
        lastClockSyncElapsedMs = nowElapsed
        lastClockRoundTripMs = nowElapsed - lastProbeElapsedMs
        pendingProbeTimestamp = null
        val nowMs = System.currentTimeMillis()
        if (nowMs - lastClockSyncLogMs >= 5_000L) {
            lastClockSyncLogMs = nowMs
            log("bridge=clock_sync rttMs=$lastClockRoundTripMs")
        }
    }

    private fun receiveAction(message: JSONObject, receivedElapsedMs: Long) {
        val name = message.optString("type")
        if (name !in setOf("dji_setpoint", "dji_gimbal_command", "dji_rth")) return
        val session = sessionId ?: return
        val seq = (message.opt("seq") as? Number)?.toLong()
        if (message.optString("sessionId") != session || seq == null || seq <= downlinkSequence) {
            log("bridge_action=rejected name=$name reason=wrong_session_or_replay")
            if (phase4PulseEnabled && name == "dji_setpoint") {
                zeroStick?.stop("wrong_session_or_replay")
            }
            return
        }
        downlinkSequence = seq
        val receiveGapMs = if (name == "dji_setpoint") {
            val gap = if (lastSetpointReceivedElapsedMs == 0L) -1L else
                receivedElapsedMs - lastSetpointReceivedElapsedMs
            lastSetpointReceivedElapsedMs = receivedElapsedMs
            gap
        } else -1L
        val issuedAt = (message.opt("issuedAt") as? Number)?.toDouble()
        val ttlMs = (message.opt("ttlMs") as? Number)?.toInt()
        val processingElapsedMs = System.nanoTime() / 1_000_000L
        // S_receive plus monotonic time since the probe left counts one whole
        // uplink transit, so every age was inflated by ~RTT and healthy field
        // commands expired. Half the measured round trip removes that bias
        // without weakening the staleness test itself.
        val serverNowEstimateMs = if (name == "dji_setpoint") {
            clockServerReceivedAtMs?.takeIf {
                processingElapsedMs - lastClockSyncElapsedMs <= 5_000L
            }?.plus(processingElapsedMs - syncProbeSentElapsedMs)
                ?.minus(lastClockRoundTripMs / 2.0)
        } else System.currentTimeMillis().toDouble()
        if (serverNowEstimateMs == null) {
            log("bridge_action=rejected name=$name reason=clock_sync_unavailable")
            if (phase4PulseEnabled && name == "dji_setpoint") {
                zeroStick?.stop("clock_sync_unavailable")
            }
            return
        }
        val ageMs = if (issuedAt == null) Double.POSITIVE_INFINITY else
            serverNowEstimateMs - issuedAt * 1_000.0
        val maximumTtl = when (name) {
            "dji_rth" -> 1_500
            "dji_setpoint" -> 500
            else -> 1_000
        }
        if (issuedAt == null || !issuedAt.isFinite() || ttlMs == null || ttlMs !in 1..maximumTtl ||
            ageMs < -1_000.0 || ageMs > ttlMs) {
            val nowMs = System.currentTimeMillis()
            if (name != "dji_setpoint" || nowMs - lastExpiredLogMs >= 2_000L) {
                lastExpiredLogMs = nowMs
                val queueDelayMs = processingElapsedMs - receivedElapsedMs
                val ageAtReceiveMs = ageMs - queueDelayMs
                val syncAgeMs = processingElapsedMs - lastClockSyncElapsedMs
                log("bridge_action=rejected name=$name reason=expired_or_invalid seq=$seq ageMs=${ageMs.toLong()} ttlMs=$ttlMs rxGapMs=$receiveGapMs queueDelayMs=$queueDelayMs ageAtReceiveMs=${ageAtReceiveMs.toLong()} syncRttMs=$lastClockRoundTripMs syncAgeMs=$syncAgeMs")
            }
            if (phase4PulseEnabled && name == "dji_setpoint") {
                zeroStick?.stop("stale_or_invalid_setpoint")
            }
            return
        }
        if (name == "dji_setpoint") {
            val velocity = message.optJSONObject("velocity")
            val x = (velocity?.opt("x") as? Number)?.toDouble()
            val y = (velocity?.opt("y") as? Number)?.toDouble()
            val z = (velocity?.opt("z") as? Number)?.toDouble()
            val heading = (message.opt("yawDeg") as? Number)?.toDouble()
            if (autonomousEnabled) {
                val invalid = autonomousCommandRejection(x, y, z, heading)
                if (invalid != null || x == null || y == null || z == null || heading == null) {
                    val reason = invalid ?: "command_incomplete"
                    log("bridge_action=rejected name=dji_setpoint reason=$reason")
                    zeroStick?.stop(reason)
                    return
                }
                val remainingMs = (ttlMs - ageMs).toLong().coerceIn(0L, ttlMs.toLong())
                val nowMs = System.nanoTime() / 1_000_000L
                val snapshot = state().snapshot
                val controller = stick()
                val deadline = nowMs + remainingMs
                if (controller.acceptAutonomous(snapshot, x, y, z, heading, deadline, nowMs)) {
                    lastZeroRejectReason = null
                    if (nowMs - lastSetpointLogMs >= 5_000L) {
                        lastSetpointLogMs = nowMs
                        log("bridge_action=accepted name=dji_setpoint mode=autonomous_flight_logic")
                    }
                } else {
                    val reason = controller.autonomousRejectionReason(
                        snapshot, x, y, z, heading, deadline, nowMs) ?: "unknown"
                    if (reason != lastZeroRejectReason || nowMs - lastZeroRejectLogMs >= 2_000L) {
                        lastZeroRejectReason = reason
                        lastZeroRejectLogMs = nowMs
                        log("bridge_action=rejected name=dji_setpoint reason=$reason")
                    }
                }
                return
            }
            if (x == null || y == null || z == null || heading == null ||
                !x.isFinite() || !y.isFinite() || !z.isFinite() || !heading.isFinite() ||
                x != 0.0 || y != 0.0 || heading !in 0.0..360.0 ||
                (z != 0.0 && (!phase4PulseEnabled || z != PHASE4_VERTICAL_SPEED_MPS))) {
                log("bridge_action=rejected name=dji_setpoint reason=nonzero_or_invalid_ground_test")
                zeroStick?.stop("nonzero_setpoint")
                return
            }
            // Even an allowed slightly-future AWS timestamp cannot extend the local lease.
            val remainingMs = (ttlMs - ageMs).toLong().coerceIn(0L, ttlMs.toLong())
            val nowMs = System.nanoTime() / 1_000_000L
            val snapshot = state().snapshot
            val controller = stick()
            val phase4 = phase4PulseEnabled && snapshot.isFlying == true
            if (phase4 && heading != 0.0) {
                log("bridge_action=rejected name=dji_setpoint reason=phase4_yaw_must_be_zero")
                controller.stop("phase4_yaw_nonzero")
                return
            }
            if (z != 0.0 && !phase4) {
                log("bridge_action=rejected name=dji_setpoint reason=phase4_not_airborne")
                controller.stop("phase4_not_airborne")
                return
            }
            if (phase4 && z != 0.0 && !phase4SetpointReceivedLogged) {
                phase4SetpointReceivedLogged = true
                log("phase4_setpoint=received seq=$seq requestedVerticalMps=$z remainingTtlMs=$remainingMs")
            }
            val accepted = if (phase4) {
                controller.acceptPhase4Pulse(snapshot, z, nowMs + remainingMs, nowMs)
            } else if (z == 0.0) {
                controller.acceptZero(snapshot, nowMs + remainingMs, nowMs)
            } else false
            if (!accepted) {
                val reason = if (phase4) {
                    controller.phase4RejectionReason(snapshot, z, nowMs + remainingMs, nowMs)
                } else if (z != 0.0 && !phase4PulseEnabled) {
                    "phase4_apk_opt_in_disabled"
                } else {
                    controller.rejectionReason(snapshot, nowMs + remainingMs, nowMs)
                } ?: "unknown"
                if (reason != lastZeroRejectReason || nowMs - lastZeroRejectLogMs >= 2_000L) {
                    lastZeroRejectReason = reason
                    lastZeroRejectLogMs = nowMs
                    log("bridge_action=rejected name=dji_setpoint reason=$reason")
                }
            } else if (nowMs - lastSetpointLogMs >= 5_000L) {
                lastZeroRejectReason = null
                lastSetpointLogMs = nowMs
                log("bridge_action=accepted name=dji_setpoint mode=${if (phase4) "phase4_vertical_pulse" else "zero_only"}")
            } else {
                lastZeroRejectReason = null
            }
            return
        }
        val command = if (name == "dji_rth") {
            VehicleCommand("$session:$seq", "return_home")
        } else {
            val pitch = (message.opt("pitchDeg") as? Number)?.toDouble()
            if (pitch == null || !pitch.isFinite()) {
                log("bridge_action=rejected name=$name reason=invalid_pitch")
                return
            }
            VehicleCommand("$session:$seq", "gimbal_pitch", mapOf("pitchDeg" to pitch))
        }
        log("bridge_action=received name=$name")
        adapter.handleCommand(command) { result ->
            val safe = result.message.replace(Regex("[\\r\\n\\t]"), " ").take(160)
            log("bridge_action=${if (result.accepted) "executed" else "rejected"} name=$name reason=$safe")
        }
    }

    private fun disconnect(reason: String) {
        epoch++
        sessionId = null
        sequence = 0
        downlinkSequence = 0
        lastZeroRejectReason = null
        clockServerReceivedAtMs = null
        pendingProbeTimestamp = null
        lastClockSyncElapsedMs = 0L
        lastClockRoundTripMs = 0L
        lastSetpointReceivedElapsedMs = 0L
        zeroStick?.stop("bridge_$reason")
        adapter.onControlLinkLost()
        val old = socket
        socket = null
        old?.cancel()
        nextConnectMs = System.currentTimeMillis() + retryMs
        retryMs = minOf(retryMs * 2, 10_000)
        if (old != null) log("bridge=disconnected reason=$reason")
    }

    companion object {
        const val PHASE4_VERTICAL_SPEED_MPS = 0.30
        private val VALID_GPS_LEVELS = setOf("LEVEL_2", "LEVEL_3", "LEVEL_4", "LEVEL_5")
        fun endpoint(origin: String): String {
            val parsed = URI(origin.trim())
            require(parsed.scheme in setOf("ws", "wss") && parsed.host != null &&
                parsed.userInfo == null && parsed.query == null && parsed.fragment == null &&
                parsed.path in setOf("", "/")) { "Expected a ws/wss origin" }
            return URI(if (parsed.scheme == "wss") "https" else "http", null,
                parsed.host, parsed.port, "/dji/bridge", null, null).toString()
        }
    }
}
