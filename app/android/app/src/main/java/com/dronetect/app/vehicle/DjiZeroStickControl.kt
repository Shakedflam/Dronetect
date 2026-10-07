package com.dronetect.app.vehicle

import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthority
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickState
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener

/** Phase 3 ground-test seam. No non-zero motion is representable here. */
interface ZeroStickControl {
    fun newSession()
    fun acceptZero(snapshot: DjiTelemetrySnapshot, deadlineMs: Long, nowMs: Long): Boolean
    fun rejectionReason(snapshot: DjiTelemetrySnapshot, deadlineMs: Long, nowMs: Long): String? = "unknown"
    fun acceptPhase4Pulse(
        snapshot: DjiTelemetrySnapshot, verticalMps: Double, deadlineMs: Long, nowMs: Long,
    ): Boolean = false
    fun phase4RejectionReason(
        snapshot: DjiTelemetrySnapshot, verticalMps: Double, deadlineMs: Long, nowMs: Long,
    ): String? = "phase4_unsupported"
    /** Continuous autonomous flight. Unlike Phase 4 this never one-shot latches. */
    fun acceptAutonomous(
        snapshot: DjiTelemetrySnapshot, eastMps: Double, northMps: Double, upMps: Double,
        headingDeg: Double, deadlineMs: Long, nowMs: Long,
    ): Boolean = false
    fun autonomousRejectionReason(
        snapshot: DjiTelemetrySnapshot, eastMps: Double, northMps: Double, upMps: Double,
        headingDeg: Double, deadlineMs: Long, nowMs: Long,
    ): String? = "autonomous_unsupported"
    fun tick(snapshot: DjiTelemetrySnapshot, nowMs: Long)
    fun stop(reason: String)
    /** The control epoch carried by the latest autonomous setpoint. */
    fun observeControlEpoch(epoch: Long) {}
    /** The bridge handed the aircraft to the pilot: stop, disable, stay released. */
    fun releaseToPilot(reason: String) {}
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

/** DJI authority-change reasons other than a successful MSDK request are terminal for Phase 4. */
internal fun phase4AuthorityAbortReason(reason: String): String? =
    when (reason) {
        "MSDK_REQUEST", "UNKNOWN" -> null
        else -> "authority_${reason.lowercase()}"
    }

internal fun virtualStickAuthorityOwned(
    sdkEnabled: Boolean, advancedEnabled: Boolean, ownerMsdk: Boolean,
): Boolean = sdkEnabled && advancedEnabled && ownerMsdk

/** Phase 4 is intentionally narrower than normal flight: one small climb pulse. */
internal fun phase4AirborneRejection(s: DjiTelemetrySnapshot): String? {
    if (s.aircraftConnected != true) return if (s.aircraftConnected == false)
        "aircraft_disconnected" else "aircraft_connection_unavailable"
    if (s.rcConnected != true) return if (s.rcConnected == false)
        "rc_disconnected" else "rc_connection_unavailable"
    if (s.isFlying != true) return if (s.isFlying == false)
        "grounded" else "flight_state_unavailable"
    val sticks = listOf(
        "left_horizontal" to s.rcLeftHorizontal,
        "left_vertical" to s.rcLeftVertical,
        "right_horizontal" to s.rcRightHorizontal,
        "right_vertical" to s.rcRightVertical,
    )
    val unavailable = sticks.filter { it.second == null }.map { it.first }
    if (unavailable.isNotEmpty()) return "rc_sticks_unavailable_${unavailable.joinToString("_")}"
    val moved = sticks.firstOrNull { it.second?.let { value -> value !in -33..33 } == true }
    if (moved != null) return "rc_stick_moved_${moved.first}_${moved.second}"
    if (s.batteryPercent == null || s.batteryPercent < PHASE4_MIN_BATTERY_PERCENT)
        return "battery_below_${PHASE4_MIN_BATTERY_PERCENT}_or_unavailable"
    if (s.gpsSignalLevel !in PHASE4_VALID_GPS_LEVELS) return "gps_not_strong"
    val altitude = s.altitudeMeters
    if (altitude == null || !altitude.isFinite() || altitude !in PHASE4_MIN_ALTITUDE_M..PHASE4_ABSOLUTE_CEILING_M)
        return "altitude_outside_test_band"
    val speed = s.speedMetersPerSecond
    if (speed == null || !speed.isFinite() || kotlin.math.abs(speed) > PHASE4_MAX_INITIAL_SPEED_MPS)
        return "aircraft_not_stably_hovering"
    if (s.flightMode !in PHASE4_ALLOWED_FLIGHT_MODES) return "flight_mode_not_allowed_${s.flightMode}"
    return null
}

/** Network commands expire at 250 ms; only local zero output may bridge a short gap. */
internal const val ZERO_STICK_LOCAL_HOLD_MS = 1_250L
// 1.0 m at 0.30 m/s needs ~3.3 s, so the altitude cutoff -- not this timer --
// normally ends the pulse. As a backstop it still bounds travel to 1.5 m, which
// keeps even a 7.0 m start under PHASE4_ABSOLUTE_CEILING_M.
internal const val PHASE4_PULSE_WINDOW_MS = 5_000L
internal const val PHASE4_MIN_BATTERY_PERCENT = 30
internal const val PHASE4_MIN_ALTITUDE_M = 2.0
internal const val PHASE4_START_MAX_ALTITUDE_M = 7.0
// Backstop only: a worst-case 7.0 m start plus the 1.0 m target leaves 1.0 m of
// clearance, so the ceiling can never mask a legitimate target_reached result.
internal const val PHASE4_ABSOLUTE_CEILING_M = 9.0
internal const val PHASE4_CLIMB_TARGET_M = 1.0
internal const val PHASE4_MAX_INITIAL_SPEED_MPS = 1.0
internal const val PHASE4_VERTICAL_SPEED_MPS = 0.30
internal val PHASE4_VALID_GPS_LEVELS = setOf("LEVEL_3", "LEVEL_4", "LEVEL_5")
internal val PHASE4_ALLOWED_FLIGHT_MODES = setOf("GPS_NORMAL", "VIRTUAL_STICK")

/** The pulse may only be armed from the start band; the wider guard band applies once climbing. */
internal fun phase4StartAltitudeRejection(s: DjiTelemetrySnapshot): String? {
    val altitude = s.altitudeMeters
    return if (altitude == null || !altitude.isFinite() ||
        altitude !in PHASE4_MIN_ALTITUDE_M..PHASE4_START_MAX_ALTITUDE_M
    ) "altitude_outside_start_band" else null
}

/** Local climb cutoff. AWS delay or loss can only end the pulse sooner, never later. */
internal fun phase4AltitudeTermination(altitudeM: Double?, startAltitudeM: Double): String? = when {
    altitudeM == null || !altitudeM.isFinite() -> "altitude_unavailable"
    altitudeM >= PHASE4_ABSOLUTE_CEILING_M -> "absolute_ceiling_reached"
    altitudeM >= startAltitudeM + PHASE4_CLIMB_TARGET_M -> "altitude_target_reached"
    else -> null
}

internal fun phase4VerticalOutput(nowMs: Long, pulseStartedMs: Long, requestedMps: Double): Double =
    if (nowMs - pulseStartedMs < PHASE4_PULSE_WINDOW_MS) requestedMps else 0.0

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
    private val phase4PulseEnabled: Boolean = false,
    private val autonomousEnabled: Boolean = false,
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
    private var phase4Active = false
    // Deliberately survives newSession(): a WSS reconnect cannot re-arm the pulse.
    private var phase4PulseConsumed = false
    private var phase4PulseStartedMs = 0L
    private var verticalMps = 0.0
    private var phase4NonzeroSubmitted = false
    private var phase4LocalDeadlineReached = false
    private var phase4LocalDeadlineZeroSubmitted = false
    private var phase4StartAltitudeM: Double? = null
    private var phase4AltitudeZeroSubmitted = false
    private var autonomousActive = false
    private var autonomousOriginLat: Double? = null
    private var autonomousOriginLng: Double? = null
    private var autonomousEastMps = 0.0
    private var autonomousNorthMps = 0.0
    private var autonomousYawAngleDeg = 0.0
    private var autonomousSaturationLogged = false
    private var lastAutonomousLogMs = 0L
    private var autonomousRearmAtMs = 0L
    // Virtual Stick is never (re-)acquired for an epoch at or below the last
    // one released to the pilot/DJI; the bridge only advances the epoch on an
    // explicit operator mission.
    private var controlEpoch = 0L
    private var releasedEpoch = -1L
    private var enableStartedMs = 0L
    private var lastSendMs = 0L
    private var listener: VirtualStickStateListener? = null

    override fun newSession() {
        stop("new_session")
        latched = false
        latchReason = ""
        // AWS creates a new authenticated session id and resets its sequence
        // space on reconnect. Old packets cannot pass DroneRealtimeClient's
        // session/replay checks, so the new session may safely arm one fresh
        // Phase 4 attempt after its server-side stable-hover dwell.
        phase4PulseConsumed = false
        phase4PulseStartedMs = 0L
        phase4NonzeroSubmitted = false
        phase4LocalDeadlineReached = false
        phase4LocalDeadlineZeroSubmitted = false
        phase4StartAltitudeM = null
        phase4AltitudeZeroSubmitted = false
        autonomousSaturationLogged = false
        // A new authenticated session starts a new epoch space on the bridge.
        controlEpoch = 0L
        releasedEpoch = -1L
    }

    override fun observeControlEpoch(epoch: Long) {
        controlEpoch = epoch
    }

    override fun releaseToPilot(reason: String) {
        releasedEpoch = controlEpoch
        log("autonomy=released_to_pilot reason=$reason controlEpoch=$controlEpoch")
        stop("bridge_release_$reason")
    }

    override fun acceptZero(snapshot: DjiTelemetrySnapshot, deadlineMs: Long, nowMs: Long): Boolean {
        if (rejectionReason(snapshot, deadlineMs, nowMs) != null) return false
        if (zeroHolding) log("virtual_stick=zero_hold_ended reason=fresh_setpoint")
        zeroHolding = false
        this.deadlineMs = deadlineMs
        verticalMps = 0.0
        if (active) return true
        phase4Active = false
        return start(deadlineMs, nowMs, "zero_only_ground_test")
    }

    override fun acceptPhase4Pulse(
        snapshot: DjiTelemetrySnapshot, verticalMps: Double, deadlineMs: Long, nowMs: Long,
    ): Boolean {
        if (phase4RejectionReason(snapshot, verticalMps, deadlineMs, nowMs) != null) return false
        if (!phase4PulseConsumed) {
            phase4PulseConsumed = true
            phase4PulseStartedMs = 0L
            log("virtual_stick=phase4_pulse_requested verticalMps=$PHASE4_VERTICAL_SPEED_MPS maxPulseMs=$PHASE4_PULSE_WINDOW_MS")
        }
        if (zeroHolding) log("virtual_stick=zero_hold_ended reason=fresh_setpoint")
        zeroHolding = false
        this.deadlineMs = deadlineMs
        this.verticalMps = if (phase4PulseStartedMs == 0L) verticalMps
            else phase4VerticalOutput(nowMs, phase4PulseStartedMs, verticalMps)
        if (active) return true
        phase4Active = true
        return start(deadlineMs, nowMs, "phase4_vertical_pulse")
    }

    override fun autonomousRejectionReason(
        snapshot: DjiTelemetrySnapshot, eastMps: Double, northMps: Double, upMps: Double,
        headingDeg: Double, deadlineMs: Long, nowMs: Long,
    ): String? = when {
        !autonomousEnabled -> "autonomous_apk_opt_in_disabled"
        latched -> "latched_after_$latchReason"
        autonomyReleaseRejection(controlEpoch, releasedEpoch) != null ->
            autonomyReleaseRejection(controlEpoch, releasedEpoch)
        nowMs < autonomousRearmAtMs -> "autonomous_rearm_cooldown"
        else -> autonomousCommandRejection(eastMps, northMps, upMps, headingDeg)
            ?: autonomousEnvelopeRejection(snapshot, autonomousOriginLat, autonomousOriginLng, upMps)
            ?: if (deadlineMs <= nowMs) "setpoint_deadline_exhausted" else null
    }

    override fun acceptAutonomous(
        snapshot: DjiTelemetrySnapshot, eastMps: Double, northMps: Double, upMps: Double,
        headingDeg: Double, deadlineMs: Long, nowMs: Long,
    ): Boolean {
        if (autonomousRejectionReason(
                snapshot, eastMps, northMps, upMps, headingDeg, deadlineMs, nowMs) != null) return false
        if (zeroHolding) log("virtual_stick=zero_hold_ended reason=fresh_setpoint")
        zeroHolding = false
        this.deadlineMs = deadlineMs
        // Android saturates independently of AWS: a bad planner value cannot
        // become a fast aircraft.
        val (east, north) = saturateHorizontal(eastMps, northMps)
        val up = saturateVertical(upMps)
        if (!autonomousSaturationLogged && (east != eastMps || north != northMps || up != upMps)) {
            autonomousSaturationLogged = true
            log("autonomous=saturated requestedEast=$eastMps requestedNorth=$northMps requestedUp=$upMps appliedEast=$east appliedNorth=$north appliedUp=$up")
        }
        autonomousEastMps = east
        autonomousNorthMps = north
        verticalMps = up
        autonomousYawAngleDeg = djiYawAngleFromCompassHeading(headingDeg)
        if (active) return true
        // AWS streams zero setpoints whenever the mission is not asking for
        // motion (IDLE before an ROI, HOLDING after one). Taking DJI authority
        // for those would arm autonomy underneath a manually flying pilot, and
        // the first stick input would then abort it. Stay passive until the
        // mission actually commands movement.
        if (east == 0.0 && north == 0.0 && up == 0.0) return true
        // The fence origin is latched once, from the aircraft's own position.
        autonomousOriginLat = snapshot.latitude
        autonomousOriginLng = snapshot.longitude
        autonomousActive = true
        phase4Active = false
        log("autonomous=arming originLat=${snapshot.latitude} originLng=${snapshot.longitude} radiusBackstopM=$AUTONOMOUS_RADIUS_BACKSTOP_M ceilingM=$AUTONOMOUS_ALTITUDE_CEILING_M")
        return start(deadlineMs, nowMs, "autonomous_flight_logic")
    }

    private fun start(deadlineMs: Long, nowMs: Long, mode: String): Boolean {
        this.deadlineMs = deadlineMs
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
                    val ownedBefore = owned
                    if (sdkEnabled && owner == FlightControlAuthority.MSDK) authorityConfirmed = true
                    owned = virtualStickAuthorityOwned(
                        sdkEnabled,
                        state.isVirtualStickAdvancedModeEnabled,
                        owner == FlightControlAuthority.MSDK,
                    )
                    if (phase4Active && !ownedBefore && owned) {
                        log("phase4_authority=confirmed sdkEnabled=true advanced=true owner=MSDK")
                    }
                    if (autonomousActive && !ownedBefore && owned) {
                        log("autonomous_authority=confirmed sdkEnabled=true advanced=true owner=MSDK")
                    }
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
                    if (reason == FlightControlAuthorityChangeReason.NEAR_BOUNDARY) {
                        logNearBoundaryDiagnostics()
                    }
                    if (phase4Active) {
                        phase4AuthorityAbortReason(reason.toString())?.let { abortReason ->
                            log("phase4_authority=denied reason=$reason abort=$abortReason")
                            stop(abortReason)
                        }
                    }
                } })
            }
        }
        listener = observer
        manager.setVirtualStickStateListener(observer)
        log("virtual_stick=enabling mode=$mode")
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
                        log("virtual_stick=enabled mode=$mode")
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

    override fun phase4RejectionReason(
        snapshot: DjiTelemetrySnapshot, verticalMps: Double, deadlineMs: Long, nowMs: Long,
    ): String? = when {
        !phase4PulseEnabled -> "phase4_apk_opt_in_disabled"
        latched -> "latched_after_$latchReason"
        else -> phase4AirborneRejection(snapshot) ?:
            // The start band only gates arming; a climbing pulse is bounded by the ceiling.
            (if (phase4StartAltitudeM == null) phase4StartAltitudeRejection(snapshot) else null) ?:
            if (deadlineMs <= nowMs) "setpoint_deadline_exhausted"
            else if (verticalMps != 0.0 && verticalMps != PHASE4_VERTICAL_SPEED_MPS)
                "phase4_velocity_outside_envelope"
            else if (!phase4PulseConsumed && verticalMps == 0.0) "phase4_pulse_not_started"
            else if (phase4PulseConsumed && !phase4Active && verticalMps != 0.0)
                "phase4_pulse_already_consumed"
            else if (phase4PulseStartedMs != 0L &&
                nowMs - phase4PulseStartedMs >= PHASE4_PULSE_WINDOW_MS &&
                verticalMps != 0.0) "phase4_pulse_window_closed"
            else null
    }

    override fun tick(snapshot: DjiTelemetrySnapshot, nowMs: Long) {
        if (!active) return
        val guard = when {
            autonomousActive -> autonomousEnvelopeRejection(
                snapshot, autonomousOriginLat, autonomousOriginLng, if (zeroHolding) 0.0 else verticalMps)
            phase4Active -> phase4AirborneRejection(snapshot)
            else -> zeroStickGroundRejection(snapshot)
        }
        guard?.let { stop("guard_$it"); return }
        if (phase4Active && enabled && owned && verticalMps != 0.0 && phase4PulseStartedMs == 0L) {
            phase4PulseStartedMs = nowMs
            // The guard above already proved this altitude is present and finite.
            phase4StartAltitudeM = snapshot.altitudeMeters
            log("phase4_altitude=start altitudeM=${snapshot.altitudeMeters} targetM=${snapshot.altitudeMeters?.plus(PHASE4_CLIMB_TARGET_M)} ceilingM=$PHASE4_ABSOLUTE_CEILING_M")
            log("virtual_stick=phase4_pulse_started verticalMps=$PHASE4_VERTICAL_SPEED_MPS maxPulseMs=$PHASE4_PULSE_WINDOW_MS")
        }
        val startAltitude = phase4StartAltitudeM
        if (phase4Active && startAltitude != null && verticalMps != 0.0) {
            val altitude = snapshot.altitudeMeters
            log("phase4_altitude=current altitudeM=$altitude deltaM=${altitude?.minus(startAltitude)}")
            phase4AltitudeTermination(altitude, startAltitude)?.let { reason ->
                val label = if (reason == "altitude_target_reached") "target_reached" else reason
                log("phase4_altitude=$label altitudeM=$altitude startAltitudeM=$startAltitude deltaM=${altitude?.minus(startAltitude)} ceilingM=$PHASE4_ABSOLUTE_CEILING_M")
                terminateForAltitude(reason)
                return
            }
        }
        if (phase4Active && phase4PulseStartedMs != 0L && verticalMps != 0.0 &&
            nowMs - phase4PulseStartedMs >= PHASE4_PULSE_WINDOW_MS) {
            verticalMps = 0.0
            phase4LocalDeadlineReached = true
            log("phase4_pulse=deadline_reached clock=android_monotonic localZero=true maxPulseMs=$PHASE4_PULSE_WINDOW_MS")
        }
        when (zeroStickDeadlineState(nowMs, deadlineMs)) {
            ZeroStickDeadlineState.EXPIRED -> { stop("setpoint_timeout"); return }
            ZeroStickDeadlineState.ZERO_HOLD -> if (!zeroHolding) {
                zeroHolding = true
                verticalMps = 0.0
                log("virtual_stick=zero_hold reason=setpoint_expired localZero=true maxHoldMs=$ZERO_STICK_LOCAL_HOLD_MS")
            }
            ZeroStickDeadlineState.FRESH -> Unit
        }
        if ((!enabled || !owned) && nowMs - enableStartedMs > 1_500L) {
            stop("authority_timeout"); return
        }
        if (!enabled || !owned || nowMs - lastSendMs < 100L) return
        if (autonomousActive) {
            // A zero hold or expiry must zero every axis, not just the vertical;
            // so must a climb-out below the floor (see autonomousBelowFloor).
            val holdHorizontal = zeroHolding || autonomousBelowFloor(snapshot.altitudeMeters)
            val east = if (holdHorizontal) 0.0 else autonomousEastMps
            val north = if (holdHorizontal) 0.0 else autonomousNorthMps
            val pitch = djiPitchFromEnu(east)
            val roll = djiRollFromEnu(north)
            val vertical = djiVerticalFromEnu(verticalMps)
            manager.sendVirtualStickAdvancedParam(
                controlParam(pitch, roll, vertical, autonomousYawAngleDeg))
            if (nowMs - lastAutonomousLogMs >= 1_000L) {
                lastAutonomousLogMs = nowMs
                log("autonomous_cmd enuEast=$east enuNorth=$north enuUp=$verticalMps | djiPitch=$pitch djiRoll=$roll djiVertical=$vertical djiYawAngle=$autonomousYawAngleDeg | lat=${snapshot.latitude} lng=${snapshot.longitude} altM=${snapshot.altitudeMeters} hdg=${snapshot.headingDegrees}")
            }
            lastSendMs = nowMs
            return
        }
        manager.sendVirtualStickAdvancedParam(controlParam(verticalMps))
        if (phase4Active && verticalMps != 0.0 && !phase4NonzeroSubmitted) {
            phase4NonzeroSubmitted = true
            log("phase4_command=submitted_to_dji verticalMps=$verticalMps pitch=0.0 roll=0.0 yawRate=0.0")
        } else if (phase4Active && verticalMps == 0.0 && phase4LocalDeadlineReached &&
            !phase4LocalDeadlineZeroSubmitted) {
            phase4LocalDeadlineZeroSubmitted = true
            log("phase4_command=zero_submitted_to_dji reason=local_deadline")
        }
        lastSendMs = nowMs
    }

    /** Zero first, then release: the aircraft is never left with a live climb command. */
    private fun terminateForAltitude(reason: String) {
        verticalMps = 0.0
        if (enabled && owned && !phase4AltitudeZeroSubmitted) {
            phase4AltitudeZeroSubmitted = true
            try {
                manager.sendVirtualStickAdvancedParam(controlParam(0.0))
                // Suppresses stop()'s duplicate abort-zero line; stop() still sends its own packet.
                phase4LocalDeadlineZeroSubmitted = true
                log("phase4_command=zero_submitted_to_dji reason=$reason")
            } catch (_: Exception) { }
        }
        stop(reason)
    }

    override fun stop(reason: String) {
        if (!active && !enabling && !enabled) return
        log("virtual_stick=stopping reason=$reason state=$lastAuthorityState changeReason=$lastAuthorityChangeReason confirmed=$authorityConfirmed owned=$owned")
        generation++
        // Phase 3/4 are one-shot tests, so any stop ends them for the session.
        // Continuous autonomy instead backs off and may re-arm once the guard
        // that stopped it has cleared -- otherwise one stick touch ends the
        // mission with nothing on the tablet to explain why.
        val stoppingAutonomous = autonomousActive
        latched = !stoppingAutonomous
        latchReason = reason
        if (stoppingAutonomous) {
            autonomousRearmAtMs = System.nanoTime() / 1_000_000L + AUTONOMOUS_REARM_COOLDOWN_MS
            if (isControlReleaseStop(reason)) {
                releasedEpoch = maxOf(releasedEpoch, controlEpoch)
                log("autonomous=released reason=$reason controlEpoch=$controlEpoch rearm=new_mission_only")
            } else {
                log("autonomous=backoff reason=$reason rearmInMs=$AUTONOMOUS_REARM_COOLDOWN_MS")
            }
        }
        val shouldDisable = enabling || enabled
        val hadAuthority = enabled && owned
        val wasPhase4 = phase4Active
        active = false
        enabling = false
        enabled = false
        owned = false
        authorityConfirmed = false
        deadlineMs = 0L
        zeroHolding = false
        phase4Active = false
        verticalMps = 0.0
        phase4StartAltitudeM = null
        val wasAutonomous = autonomousActive
        autonomousActive = false
        autonomousEastMps = 0.0
        autonomousNorthMps = 0.0
        autonomousYawAngleDeg = 0.0
        autonomousOriginLat = null
        autonomousOriginLng = null
        if (wasAutonomous) log("autonomous=stopping reason=$reason")
        listener?.let(manager::removeVirtualStickStateListener)
        listener = null
        if (shouldDisable) {
            // A zero packet precedes release when we actually held authority.
            // When DJI/RC already took authority, do not send a control packet.
            if (hadAuthority) {
                try {
                    manager.sendVirtualStickAdvancedParam(controlParam(0.0))
                    if (wasPhase4 && !phase4LocalDeadlineZeroSubmitted) {
                        log("phase4_command=zero_submitted_to_dji reason=abort_$reason")
                    }
                } catch (_: Exception) { }
            }
            manager.setVirtualStickAdvancedModeEnabled(false)
            if (wasPhase4) {
                log("phase4_authority=release_requested reason=$reason")
                manager.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() {
                        log("phase4_authority=released virtualStickDisabled=true reason=$reason")
                    }
                    override fun onFailure(error: IDJIError) {
                        log("phase4_authority=release_failed code=${error.errorCode()} description=${error.description()} reason=$reason")
                    }
                })
            } else {
                manager.disableVirtualStick(null)
                log("virtual_stick=disabled reason=$reason")
            }
        }
    }

    /**
     * Read-only NEAR_BOUNDARY diagnostics. Reads DJI's own flight-limit state
     * to identify which onboard boundary triggered the authority change; never
     * writes any key and never affects Phase 4 timing, guards, or authority
     * handling.
     */
    private fun logNearBoundaryDiagnostics() {
        try {
            val manager = KeyManager.getInstance()
            logKeyValue(manager, "heightLimitM", KeyTools.createKey(FlightControllerKey.KeyHeightLimit))
            logKeyValue(manager, "isNearHeightLimit", KeyTools.createKey(FlightControllerKey.KeyIsNearHeightLimit))
            logKeyValue(manager, "distanceLimitEnabled", KeyTools.createKey(FlightControllerKey.KeyDistanceLimitEnabled))
            logKeyValue(manager, "distanceLimitM", KeyTools.createKey(FlightControllerKey.KeyDistanceLimit))
            logKeyValue(manager, "isNearDistanceLimit", KeyTools.createKey(FlightControllerKey.KeyIsNearDistanceLimit))
            logKeyValue(manager, "homeLocationSet", KeyTools.createKey(FlightControllerKey.KeyIsHomeLocationSet))
            logKeyValue(manager, "homeLocation", KeyTools.createKey(FlightControllerKey.KeyHomeLocation)) { location ->
                "lat=${location?.latitude} lng=${location?.longitude}"
            }
        } catch (error: Exception) {
            log("phase4_boundary_diag=read_failed type=${error.javaClass.simpleName}")
        }
    }

    private fun <T> logKeyValue(
        manager: KeyManager, name: String, key: DJIKey<T>, format: (T?) -> String = { it.toString() },
    ) {
        if (!manager.isKeySupported(key)) {
            log("phase4_boundary_diag $name=unsupported")
            return
        }
        manager.getValue(key, object : CommonCallbacks.CompletionCallbackWithParam<T> {
            override fun onSuccess(value: T?) {
                log("phase4_boundary_diag $name=${format(value)}")
            }
            override fun onFailure(error: IDJIError) {
                log("phase4_boundary_diag $name=read_failed code=${error.errorCode()}")
            }
        })
    }

    private fun controlParam(verticalMps: Double) = controlParam(0.0, 0.0, verticalMps, null)

    /**
     * yawAngleDeg null keeps the proven zero/Phase-4 shape: ANGULAR_VELOCITY
     * with a zero rate. A value selects ANGLE, where DJI expects a GROUND-frame
     * heading in [-180,180] -- never an absolute heading in a rate field.
     */
    private fun controlParam(
        pitchMps: Double, rollMps: Double, verticalMps: Double, yawAngleDeg: Double?,
    ) = VirtualStickFlightControlParam().apply {
        rollPitchCoordinateSystem = FlightCoordinateSystem.GROUND
        rollPitchControlMode = RollPitchControlMode.VELOCITY
        verticalControlMode = VerticalControlMode.VELOCITY
        yawControlMode =
            if (yawAngleDeg == null) YawControlMode.ANGULAR_VELOCITY else YawControlMode.ANGLE
        pitch = pitchMps
        roll = rollMps
        verticalThrottle = verticalMps
        yaw = yawAngleDeg ?: 0.0
    }
}
