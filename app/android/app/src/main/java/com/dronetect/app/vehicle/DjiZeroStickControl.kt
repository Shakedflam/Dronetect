package com.dronetect.app.vehicle

import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthority
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickState
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener

/** Phase 3 ground-test seam. No non-zero motion is representable here. */
interface ZeroStickControl {
    fun newSession()
    fun acceptZero(snapshot: DjiTelemetrySnapshot, deadlineMs: Long, nowMs: Long): Boolean
    fun rejectionReason(snapshot: DjiTelemetrySnapshot, deadlineMs: Long, nowMs: Long): String? = "unknown"
    fun tick(snapshot: DjiTelemetrySnapshot, nowMs: Long)
    fun stop(reason: String)
}

/** A null reason means the aircraft is grounded and physical RC input is verified centered. */
internal fun zeroStickGroundRejection(s: DjiTelemetrySnapshot): String? {
    if (s.aircraftConnected != true) return if (s.aircraftConnected == false)
        "aircraft_disconnected" else "aircraft_connection_unavailable"
    if (s.rcConnected != true) return if (s.rcConnected == false)
        "rc_disconnected" else "rc_connection_unavailable"
    if (s.isFlying != false) return if (s.isFlying == true)
        "airborne" else "flight_state_unavailable"
    val sticks = listOf(
        "left_horizontal" to s.rcLeftHorizontal,
        "left_vertical" to s.rcLeftVertical,
        "right_horizontal" to s.rcRightHorizontal,
        "right_vertical" to s.rcRightVertical,
    )
    val unavailable = sticks.filter { it.second == null }.map { it.first }
    if (unavailable.isNotEmpty()) return "rc_sticks_unavailable_${unavailable.joinToString("_")}"
    val moved = sticks.firstOrNull { it.second?.let { value -> value !in -33..33 } == true }
    return moved?.let { "rc_stick_moved_${it.first}_${it.second}" }
}

/** Do not mistake an initial, pre-confirmation RC state for loss of MSDK authority. */
internal fun zeroStickAuthorityLost(
    active: Boolean, confirmed: Boolean, sdkEnabled: Boolean,
    ownerMsdk: Boolean, ownerRc: Boolean,
): Boolean = active && (
    confirmed && (!sdkEnabled || !ownerMsdk)
    )

/** Network commands expire at 250 ms; only local zero output may bridge a short gap. */
internal const val ZERO_STICK_LOCAL_HOLD_MS = 1_250L

internal enum class ZeroStickDeadlineState { FRESH, ZERO_HOLD, EXPIRED }

internal fun zeroStickDeadlineState(nowMs: Long, deadlineMs: Long): ZeroStickDeadlineState = when {
    nowMs < deadlineMs -> ZeroStickDeadlineState.FRESH
    nowMs - deadlineMs < ZERO_STICK_LOCAL_HOLD_MS -> ZeroStickDeadlineState.ZERO_HOLD
    else -> ZeroStickDeadlineState.EXPIRED
}

/** All state mutations are dispatched onto DroneRealtimeClient's single worker. */
class DjiZeroStickControl(
    private val dispatch: (Runnable) -> Unit,
    private val log: (String) -> Unit,
) : ZeroStickControl {
    private val manager = VirtualStickManager.getInstance()
    private var generation = 0L
    private var active = false
    private var enabling = false
    private var enabled = false
    private var owned = false
    private var authorityConfirmed = false
    private var lastAuthorityState = ""
    private var lastAuthorityChangeReason = "unavailable"
    private var latched = false
    private var latchReason = ""
    private var deadlineMs = 0L
    private var zeroHolding = false
    private var enableStartedMs = 0L
    private var lastSendMs = 0L
    private var listener: VirtualStickStateListener? = null

    override fun newSession() {
        stop("new_session")
        latched = false
        latchReason = ""
    }

    override fun acceptZero(snapshot: DjiTelemetrySnapshot, deadlineMs: Long, nowMs: Long): Boolean {
        if (rejectionReason(snapshot, deadlineMs, nowMs) != null) return false
        if (zeroHolding) log("virtual_stick=zero_hold_ended reason=fresh_setpoint")
        zeroHolding = false
        this.deadlineMs = deadlineMs
        if (active) return true
        active = true
        enabling = true
        authorityConfirmed = false
        lastAuthorityState = ""
        lastAuthorityChangeReason = "unavailable"
        enableStartedMs = nowMs
        val request = ++generation
        val observer = object : VirtualStickStateListener {
            override fun onVirtualStickStateUpdate(state: VirtualStickState) {
                dispatch(Runnable {
                    if (request != generation) return@Runnable
                    val sdkEnabled = state.isVirtualStickEnable
                    val owner = state.currentFlightControlAuthorityOwner
                    val confirmedBefore = authorityConfirmed
                    if (sdkEnabled && owner == FlightControlAuthority.MSDK) authorityConfirmed = true
                    owned = sdkEnabled && state.isVirtualStickAdvancedModeEnabled &&
                        owner == FlightControlAuthority.MSDK
                    val stateText = "sdkEnabled=$sdkEnabled advanced=${state.isVirtualStickAdvancedModeEnabled} owner=$owner active=$active enabling=$enabling enabled=$enabled confirmedBefore=$confirmedBefore confirmed=$authorityConfirmed changeReason=$lastAuthorityChangeReason"
                    if (stateText != lastAuthorityState) {
                        lastAuthorityState = stateText
                        log("virtual_stick=state $stateText")
                    }
                    if (zeroStickAuthorityLost(active, authorityConfirmed, sdkEnabled,
                            owner == FlightControlAuthority.MSDK, owner == FlightControlAuthority.RC)) {
                        log("virtual_stick=authority_lost $stateText")
                        stop("rc_authority")
                    }
                })
            }

            override fun onChangeReasonUpdate(reason: FlightControlAuthorityChangeReason) {
                dispatch(Runnable { if (request == generation) {
                    lastAuthorityChangeReason = reason.toString()
                    log("virtual_stick=authority_change reason=$reason active=$active enabling=$enabling enabled=$enabled confirmed=$authorityConfirmed state=$lastAuthorityState")
                } })
            }
        }
        listener = observer
        manager.setVirtualStickStateListener(observer)
        log("virtual_stick=enabling mode=zero_only_ground_test")
        manager.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                dispatch(Runnable {
                    if (request != generation || !active) {
                        manager.disableVirtualStick(null)
                        return@Runnable
                    }
                    enabling = false
                    enabled = true
                    log("virtual_stick=enable_succeeded state=$lastAuthorityState confirmed=$authorityConfirmed")
                    try {
                        manager.setVirtualStickAdvancedModeEnabled(true)
                        log("virtual_stick=enabled mode=zero_only_ground_test")
                    } catch (error: Exception) {
                        log("virtual_stick=advanced_mode_failed type=${error.javaClass.simpleName}")
                        stop("advanced_mode_failed")
                    }
                })
            }

            override fun onFailure(error: IDJIError) {
                dispatch(Runnable {
                    if (request == generation) {
                        log("virtual_stick=enable_failed code=${error.errorCode()} description=${error.description()}")
                        stop("enable_failed")
                    }
                })
            }
        })
        return true
    }

    override fun rejectionReason(snapshot: DjiTelemetrySnapshot, deadlineMs: Long, nowMs: Long): String? =
        when {
            latched -> "latched_after_$latchReason"
            else -> zeroStickGroundRejection(snapshot) ?:
                if (deadlineMs <= nowMs) "setpoint_deadline_exhausted" else null
        }

    override fun tick(snapshot: DjiTelemetrySnapshot, nowMs: Long) {
        if (!active) return
        zeroStickGroundRejection(snapshot)?.let { stop("guard_$it"); return }
        when (zeroStickDeadlineState(nowMs, deadlineMs)) {
            ZeroStickDeadlineState.EXPIRED -> { stop("setpoint_timeout"); return }
            ZeroStickDeadlineState.ZERO_HOLD -> if (!zeroHolding) {
                zeroHolding = true
                log("virtual_stick=zero_hold reason=setpoint_expired localZero=true maxHoldMs=$ZERO_STICK_LOCAL_HOLD_MS")
            }
            ZeroStickDeadlineState.FRESH -> Unit
        }
        if ((!enabled || !owned) && nowMs - enableStartedMs > 1_500L) {
            stop("authority_timeout"); return
        }
        if (!enabled || !owned || nowMs - lastSendMs < 100L) return
        manager.sendVirtualStickAdvancedParam(zeroParam())
        lastSendMs = nowMs
    }

    override fun stop(reason: String) {
        if (!active && !enabling && !enabled) return
        log("virtual_stick=stopping reason=$reason state=$lastAuthorityState changeReason=$lastAuthorityChangeReason confirmed=$authorityConfirmed owned=$owned")
        generation++
        latched = true
        latchReason = reason
        val shouldDisable = enabling || enabled
        val hadAuthority = enabled && owned
        active = false
        enabling = false
        enabled = false
        owned = false
        authorityConfirmed = false
        deadlineMs = 0L
        zeroHolding = false
        listener?.let(manager::removeVirtualStickStateListener)
        listener = null
        if (shouldDisable) {
            // A zero packet precedes release when we actually held authority.
            // When DJI/RC already took authority, do not send a control packet.
            if (hadAuthority) {
                try { manager.sendVirtualStickAdvancedParam(zeroParam()) } catch (_: Exception) { }
            }
            manager.setVirtualStickAdvancedModeEnabled(false)
            manager.disableVirtualStick(null)
            log("virtual_stick=disabled reason=$reason")
        }
    }

    private fun zeroParam() = VirtualStickFlightControlParam().apply {
        rollPitchCoordinateSystem = FlightCoordinateSystem.GROUND
        rollPitchControlMode = RollPitchControlMode.VELOCITY
        verticalControlMode = VerticalControlMode.VELOCITY
        yawControlMode = YawControlMode.ANGULAR_VELOCITY
        pitch = 0.0
        roll = 0.0
        verticalThrottle = 0.0
        yaw = 0.0
    }
}
