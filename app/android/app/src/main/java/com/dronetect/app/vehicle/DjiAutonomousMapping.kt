package com.dronetect.app.vehicle

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sign

/**
 * FlightLogic (ENU) -> DJI Virtual Stick (GROUND) command mapping.
 *
 * DJI MSDK documents the virtual-stick pair as "velocity along the y-axis" for
 * pitch and "velocity along the x-axis" for roll, and the GROUND coordinate
 * system as NED (x=North, y=East). The two together give the mapping DJI states
 * directly: positive pitch moves the aircraft East, positive roll moves it
 * North. That is a swap relative to the intuitive reading, which is exactly why
 * it lives here behind named functions instead of inside controlParam().
 *
 * FlightLogic/Webots ENU is +X=East, +Y=North, +Z=up.
 */

/** DJI pitch carries the EAST component. Verified against MSDK GROUND=NED docs. */
internal fun djiPitchFromEnu(velocityEastMps: Double): Double = velocityEastMps

/** DJI roll carries the NORTH component. Verified against MSDK GROUND=NED docs. */
internal fun djiRollFromEnu(velocityNorthMps: Double): Double = velocityNorthMps

/** ENU +Z is up; Phase 4 proved on the aircraft that positive throttle climbs. */
internal fun djiVerticalFromEnu(velocityUpMps: Double): Double = velocityUpMps

/**
 * AWS sends a compass heading in [0,360). DJI YawControlMode.ANGLE expects a
 * GROUND-frame angle in [-180,180] with 0=North, clockwise positive. Absolute
 * heading must never reach an ANGULAR_VELOCITY field.
 */
internal fun djiYawAngleFromCompassHeading(headingDeg: Double): Double {
    val wrapped = ((headingDeg % 360.0) + 360.0) % 360.0
    return if (wrapped > 180.0) wrapped - 360.0 else wrapped
}

// Real-demo profile: the actuator boundary clamps to walking pace. FlightLogic
// and the simulator keep their own (higher) limits -- this is the last gate
// before DJI, so nothing upstream can out-run it.
internal const val AUTONOMOUS_MAX_HORIZONTAL_MPS = 2.0
internal const val AUTONOMOUS_MAX_VERTICAL_MPS = 2.0
internal const val AUTONOMOUS_ALTITUDE_CEILING_M = 55.0
internal const val AUTONOMOUS_MIN_ALTITUDE_M = 1.5
// DJI's native takeoff settles at ~1.2 m, under that floor. From there (but
// never from lower) only a climb is accepted, with horizontal output zeroed,
// until the aircraft is back above AUTONOMOUS_MIN_ALTITUDE_M.
internal const val AUTONOMOUS_CLIMB_OUT_MIN_ALTITUDE_M = 0.5
internal const val AUTONOMOUS_RADIUS_BACKSTOP_M = 330.0
// An autonomous guard trip releases authority but must not poison the rest of
// the session: continuous flight has to survive a momentary RC touch or a
// transient boundary nudge. The cooldown stops a 20 Hz re-arm storm.
internal const val AUTONOMOUS_REARM_COOLDOWN_MS = 3_000L
internal val AUTONOMOUS_VALID_GPS_LEVELS = setOf("LEVEL_3", "LEVEL_4", "LEVEL_5")

/**
 * Stops that mean someone else now has the aircraft -- the pilot on the RC
 * sticks, the bridge handing it to the pilot, or DJI/RC reclaiming authority
 * (RC_PAUSE_STOP, RC_SWITCH, RC_LOST, NEAR_BOUNDARY, low-battery go-home...).
 * Unlike a transient guard trip, these must never be followed by a silent
 * re-arm: only a newer control epoch (an explicit operator mission) may.
 */
internal fun isControlReleaseStop(reason: String): Boolean =
    reason.startsWith("guard_rc_stick_moved") ||
        reason.startsWith("bridge_release") ||
        reason == "rc_authority"

/** Virtual Stick is only (re-)acquired for an epoch newer than the last release. */
internal fun autonomyReleaseRejection(controlEpoch: Long, releasedEpoch: Long): String? =
    if (controlEpoch <= releasedEpoch) "released_to_pilot_awaiting_new_mission" else null
internal val AUTONOMOUS_ALLOWED_FLIGHT_MODES = setOf("GPS_NORMAL", "VIRTUAL_STICK")

/**
 * Local gate on a bridge takeoff request, on top of DJI's own checks: on the
 * ground, RC linked with its sticks centred, and GPS strong enough for the
 * Virtual Stick climb that follows.
 */
internal fun nativeTakeoffRejection(s: DjiTelemetrySnapshot): String? =
    zeroStickGroundRejection(s)
        ?: if (s.gpsSignalLevel !in AUTONOMOUS_VALID_GPS_LEVELS) "gps_not_strong" else null

/** Saturation preserves heading: both components scale by one factor. */
internal fun saturateHorizontal(eastMps: Double, northMps: Double): Pair<Double, Double> {
    val speed = hypot(eastMps, northMps)
    if (speed <= AUTONOMOUS_MAX_HORIZONTAL_MPS || speed == 0.0) return eastMps to northMps
    val scale = AUTONOMOUS_MAX_HORIZONTAL_MPS / speed
    return eastMps * scale to northMps * scale
}

internal fun saturateVertical(upMps: Double): Double =
    if (abs(upMps) <= AUTONOMOUS_MAX_VERTICAL_MPS) upMps
    else AUTONOMOUS_MAX_VERTICAL_MPS * sign(upMps)

/** Rejects a command that is not physically representable, before any clamping. */
internal fun autonomousCommandRejection(
    eastMps: Double?, northMps: Double?, upMps: Double?, headingDeg: Double?,
): String? {
    if (eastMps == null || northMps == null || upMps == null || headingDeg == null)
        return "command_incomplete"
    if (!eastMps.isFinite() || !northMps.isFinite() || !upMps.isFinite() || !headingDeg.isFinite())
        return "command_nonfinite"
    if (headingDeg !in 0.0..360.0) return "heading_outside_range"
    return null
}

private const val METERS_PER_DEG_LAT = 111_320.0

/** Equirectangular distance: exact enough for a few-hundred-metre safety fence. */
internal fun horizontalDistanceMeters(
    latA: Double, lngA: Double, latB: Double, lngB: Double,
): Double {
    val northM = (latB - latA) * METERS_PER_DEG_LAT
    val eastM = (lngB - lngA) * METERS_PER_DEG_LAT * cos(Math.toRadians(latA))
    return hypot(eastM, northM)
}

/** Below the floor: horizontal output is held at zero during a climb-out. */
internal fun autonomousBelowFloor(altitudeM: Double?): Boolean =
    altitudeM != null && altitudeM < AUTONOMOUS_MIN_ALTITUDE_M

/**
 * Android's own envelope, evaluated every tick and independent of AWS. The
 * radius fence sits outside the 300 m operating radius on purpose: it is a
 * backstop against a runaway, not a second copy of the mission limit.
 * `upMps` is the vertical command in force; only a climb may run below the floor.
 */
internal fun autonomousEnvelopeRejection(
    s: DjiTelemetrySnapshot, originLat: Double?, originLng: Double?, upMps: Double = 0.0,
): String? {
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
    if (s.gpsSignalLevel !in AUTONOMOUS_VALID_GPS_LEVELS) return "gps_not_strong"
    if (s.flightMode !in AUTONOMOUS_ALLOWED_FLIGHT_MODES)
        return "flight_mode_not_allowed_${s.flightMode}"
    val altitude = s.altitudeMeters
    if (altitude == null || !altitude.isFinite()) return "altitude_unavailable"
    if (altitude > AUTONOMOUS_ALTITUDE_CEILING_M) return "altitude_above_ceiling"
    if (altitude < AUTONOMOUS_MIN_ALTITUDE_M &&
        !(upMps > 0.0 && altitude >= AUTONOMOUS_CLIMB_OUT_MIN_ALTITUDE_M)) return "altitude_below_floor"
    val latitude = s.latitude
    val longitude = s.longitude
    if (latitude == null || longitude == null || !latitude.isFinite() || !longitude.isFinite())
        return "position_unavailable"
    if (originLat != null && originLng != null &&
        horizontalDistanceMeters(originLat, originLng, latitude, longitude) >
        AUTONOMOUS_RADIUS_BACKSTOP_M
    ) return "outside_radius_backstop"
    return null
}
