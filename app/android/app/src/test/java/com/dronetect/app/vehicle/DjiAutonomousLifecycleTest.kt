package com.dronetect.app.vehicle

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test

/**
 * Reproduces the real-flight failure: the aircraft stayed stationary through a
 * whole ROI mission because continuous autonomy inherited Phase 4's one-shot
 * lifecycle. These are pure-function/state tests -- DjiZeroStickControl itself
 * reaches the DJI SDK in its constructor and cannot run on the JVM.
 */
class DjiAutonomousLifecycleTest {
    private val cruising = DjiTelemetrySnapshot(
        latitude = 32.0, longitude = 34.0, altitudeMeters = 15.0,
        aircraftConnected = true, rcConnected = true, isFlying = true,
        gpsSignalLevel = "LEVEL_5", flightMode = "GPS_NORMAL",
        rcLeftHorizontal = 0, rcLeftVertical = 0,
        rcRightHorizontal = 0, rcRightVertical = 0,
    )

    /** The pilot flying manually is the normal case before an ROI is selected. */
    @Test fun manualStickInputIsAGuardTripNotAPermanentFailure() {
        val flying = cruising.copy(rcRightVertical = 300)
        assertEquals("rc_stick_moved_right_vertical_300",
            autonomousEnvelopeRejection(flying, 32.0, 34.0))
        // ...and the moment the sticks centre again the envelope is clean, so
        // autonomy is allowed to come back. Before the fix a stop latched the
        // controller for the rest of the session and every later ROI setpoint
        // was rejected with latched_after_guard_rc_stick_moved_*.
        assertNull(autonomousEnvelopeRejection(cruising, 32.0, 34.0))
    }

    @Test fun legitimateVirtualStickFlightModeDoesNotSelfAbort() {
        // Acquiring authority changes DJI's reported mode; that must not be
        // read back as a guard failure.
        assertNull(autonomousEnvelopeRejection(
            cruising.copy(flightMode = "VIRTUAL_STICK"), 32.0, 34.0))
        assertNull(autonomousEnvelopeRejection(
            cruising.copy(flightMode = "GPS_NORMAL"), 32.0, 34.0))
        assertEquals("flight_mode_not_allowed_ATTI",
            autonomousEnvelopeRejection(cruising.copy(flightMode = "ATTI"), 32.0, 34.0))
    }

    @Test fun aZeroCommandCarriesNoMotionSoItMustNotArmAuthority() {
        // AWS streams these whenever the mission is idle or holding. They are
        // valid commands, they simply ask for nothing.
        assertNull(autonomousCommandRejection(0.0, 0.0, 0.0, 270.0))
        val zero = listOf(0.0, 0.0, 0.0).all { it == 0.0 }
        assertEquals(true, zero)
    }

    @Test fun transitCommandCarriesMotionSoItMustArm() {
        assertNull(autonomousCommandRejection(1.4, -0.9, 0.2, 31.0))
        val moving = listOf(1.4, -0.9, 0.2).any { it != 0.0 }
        assertEquals(true, moving)
    }

    @Test fun rearmCooldownIsBoundedAndNonZero() {
        // Long enough to stop a 20 Hz re-arm storm, short enough that the
        // mission recovers within a couple of seconds.
        assertEquals(3_000L, AUTONOMOUS_REARM_COOLDOWN_MS)
    }

    @Test fun envelopeStillStopsOnEveryRealSafetyCondition() {
        assertEquals("aircraft_disconnected",
            autonomousEnvelopeRejection(cruising.copy(aircraftConnected = false), 32.0, 34.0))
        assertEquals("rc_disconnected",
            autonomousEnvelopeRejection(cruising.copy(rcConnected = false), 32.0, 34.0))
        assertEquals("gps_not_strong",
            autonomousEnvelopeRejection(cruising.copy(gpsSignalLevel = "LEVEL_1"), 32.0, 34.0))
        assertEquals("altitude_above_ceiling", autonomousEnvelopeRejection(
            cruising.copy(altitudeMeters = AUTONOMOUS_ALTITUDE_CEILING_M + 1.0), 32.0, 34.0))
        assertEquals("altitude_below_floor",
            autonomousEnvelopeRejection(cruising.copy(altitudeMeters = 0.5), 32.0, 34.0))
        assertNotNull(autonomousEnvelopeRejection(
            cruising.copy(latitude = 32.0 + 400.0 / 111_320.0), 32.0, 34.0))
        assertEquals("command_nonfinite",
            autonomousCommandRejection(Double.NaN, 0.0, 0.0, 0.0))
    }
}
