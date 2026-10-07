package com.dronetect.app.vehicle

/** SDK calls live behind this port so command policy can be tested without an aircraft. */
interface DjiAircraftActions {
    fun returnHome(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit)
    fun tiltCamera(up: Boolean, isCurrent: () -> Boolean, complete: (CommandResult) -> Unit)
    fun setGimbalPitch(pitchDeg: Double, isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) =
        complete(CommandResult(false, "absolute_gimbal_unsupported"))
    /** DJI's own takeoff, for a bridge mission started on the ground. */
    fun takeoff(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) =
        complete(CommandResult(false, "native_takeoff_unsupported"))
    /** DJI's own landing, once the bridge's approach holds low over home. */
    fun land(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) =
        complete(CommandResult(false, "native_land_unsupported"))
}

/**
 * The cloud sends mission intents, not trajectory setpoints. Those intents need
 * adapters/dji/flight_bridge.py's local FlightLogic; they must not become a
 * guessed takeoff/Virtual Stick command here.
 */
class DjiCommandDispatcher(
    private val actions: DjiAircraftActions,
    private val connected: () -> Boolean,
    private val nowMs: () -> Long,
) {
    @Volatile var pursuitBlocked = false
        private set
    private var generation = 0L
    private var pending: ((CommandResult) -> Unit)? = null
    private var pendingName: String? = null
    private var issuedAtMs = 0L
    private var lastActionMs = Long.MIN_VALUE
    private var lastRthMs = Long.MIN_VALUE
    private val lastNativeFlightMs = mutableMapOf<String, Long>()

    @Synchronized
    fun invalidate() {
        generation++
        val callback = pending
        pending = null
        pendingName = null
        callback?.invoke(CommandResult(false, "control_link_lost"))
    }

    @Synchronized
    fun expire() {
        if (pending != null && nowMs() - issuedAtMs >= ACTION_TIMEOUT_MS) {
            generation++
            val callback = pending
            pending = null
            pendingName = null
            // A timed-out DJI action may have reached the aircraft. Never retry it.
            callback?.invoke(CommandResult(false, "sdk_timeout_outcome_unknown_no_retry"))
        }
    }

    @Synchronized
    fun dispatch(command: VehicleCommand, complete: (CommandResult) -> Unit) {
        expire()
        fun reject(reason: String) = complete(CommandResult(false, reason))
        if (!connected()) return reject("aircraft_disconnected")
        if (command.name == "set_pursuit_block") {
            val blocked = command.params["blocked"] as? Boolean ?: return reject("invalid_blocked_flag")
            if (command.params.size != 1) return reject("unexpected_params")
            pursuitBlocked = blocked
            return complete(CommandResult(true, "hold_recorded_no_autonomy_execution"))
        }
        when (command.name) {
            "set_operating_config", "start_patrol", "set_roi", "clear_roi",
            "set_patrol_priorities", "set_patrol_area_types", "reject_target",
            "set_manual_control", "set_manual_command" ->
                return reject("local_autonomy_controller_required")
            "return_home", "camera_tilt_up", "camera_tilt_down", "gimbal_pitch",
            NATIVE_TAKEOFF, NATIVE_LAND -> Unit
            else -> return reject("unsupported_command")
        }
        val pitchTarget = if (command.name == "gimbal_pitch") {
            val pitch = (command.params["pitchDeg"] as? Number)?.toDouble()
                ?: return reject("invalid_pitch")
            if (command.params.size != 1 || !pitch.isFinite() ||
                pitch !in -Math.toDegrees(1.7)..Math.toDegrees(0.5)) return reject("invalid_pitch")
            pitch
        } else {
            if (command.params.isNotEmpty()) return reject("unexpected_params")
            null
        }
        // Flight actions outrank the gimbal traffic the bridge sends every tick.
        val flightAction = command.name in FLIGHT_ACTIONS
        if (flightAction && pending != null && pendingName !in FLIGHT_ACTIONS) {
            generation++
            val interrupted = pending
            pending = null
            pendingName = null
            interrupted?.invoke(CommandResult(false,
                if (command.name == "return_home") "preempted_by_rth" else "preempted_by_${command.name}"))
        }
        if (pending != null) return reject("action_in_progress")
        val now = nowMs()
        if (!flightAction && lastActionMs != Long.MIN_VALUE && now - lastActionMs < MIN_ACTION_INTERVAL_MS) {
            return reject("rate_limited")
        }
        if (command.name == "return_home" && lastRthMs != Long.MIN_VALUE && now - lastRthMs < RTH_INTERVAL_MS) {
            return reject("rth_rate_limited")
        }
        // The bridge sends each of these once per sortie; a burst is a fault.
        val lastNative = lastNativeFlightMs[command.name]
        if (lastNative != null && now - lastNative < NATIVE_FLIGHT_INTERVAL_MS) {
            return reject("${command.name}_rate_limited")
        }
        lastActionMs = now
        if (command.name == "return_home") lastRthMs = now
        if (command.name == NATIVE_TAKEOFF || command.name == NATIVE_LAND) lastNativeFlightMs[command.name] = now
        issuedAtMs = now
        pending = complete
        pendingName = command.name
        val token = ++generation
        val isCurrent = {
            synchronized(this) {
                token == generation && pending != null && connected() &&
                    nowMs() - issuedAtMs < PRECONDITION_DEADLINE_MS
            }
        }
        val done: (CommandResult) -> Unit = { result ->
            synchronized(this) {
                if (token == generation && pending != null) {
                    pending = null
                    pendingName = null
                    complete(result)
                }
            }
        }
        try {
            when (command.name) {
                "return_home" -> actions.returnHome(isCurrent, done)
                NATIVE_TAKEOFF -> actions.takeoff(isCurrent, done)
                NATIVE_LAND -> actions.land(isCurrent, done)
                "gimbal_pitch" -> actions.setGimbalPitch(pitchTarget!!, isCurrent, done)
                else -> actions.tiltCamera(command.name == "camera_tilt_up", isCurrent, done)
            }
        } catch (error: Exception) {
            done(CommandResult(false, "sdk_exception_${error.javaClass.simpleName}"))
        }
    }

    companion object {
        const val PRECONDITION_DEADLINE_MS = 1_500L
        const val ACTION_TIMEOUT_MS = 5_000L
        const val MIN_ACTION_INTERVAL_MS = 300L
        const val RTH_INTERVAL_MS = 5_000L
        const val NATIVE_FLIGHT_INTERVAL_MS = 5_000L
        // Bridge-only names (DroneRealtimeClient is the sole caller); no
        // cloud/tablet command maps to them.
        const val NATIVE_TAKEOFF = "native_takeoff"
        const val NATIVE_LAND = "native_land"
        private val FLIGHT_ACTIONS = setOf("return_home", NATIVE_TAKEOFF, NATIVE_LAND)

        /** Same 0.03-radian step and pitch envelope as CameraGimbal, inverted for DJI. */
        fun nextPitch(current: Double, min: Double, max: Double, up: Boolean): Double? {
            if (!current.isFinite() || !min.isFinite() || !max.isFinite() || min > max) return null
            val lower = maxOf(min, -Math.toDegrees(1.7))
            val upper = minOf(max, Math.toDegrees(0.5))
            if (lower > upper || current !in lower..upper) return null
            return (current + (if (up) 1 else -1) * Math.toDegrees(0.03)).coerceIn(lower, upper)
        }
    }
}
