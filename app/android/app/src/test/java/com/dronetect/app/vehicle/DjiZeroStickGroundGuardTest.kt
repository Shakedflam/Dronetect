package com.dronetect.app.vehicle

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class DjiZeroStickGroundGuardTest {
    private val grounded = DjiTelemetrySnapshot(
        aircraftConnected = true, rcConnected = true, isFlying = false,
        rcLeftHorizontal = 0, rcLeftVertical = 0,
        rcRightHorizontal = 0, rcRightVertical = 0,
    )
    private val phase4Hover = grounded.copy(
        isFlying = true, altitudeMeters = 3.0, batteryPercent = 80,
        gpsSignalLevel = "LEVEL_4", speedMetersPerSecond = 0.0,
        flightMode = "GPS_NORMAL",
    )

    @Test fun onGroundWithCenteredRcIsAllowed() {
        assertNull(zeroStickGroundRejection(grounded))
    }

    @Test fun absentOrMovingRcInputNeverPassesAsCentered() {
        assertEquals("rc_connection_unavailable",
            zeroStickGroundRejection(grounded.copy(rcConnected = null)))
        assertEquals("rc_disconnected",
            zeroStickGroundRejection(grounded.copy(rcConnected = false)))
        assertEquals("rc_sticks_unavailable_left_vertical_right_horizontal",
            zeroStickGroundRejection(grounded.copy(rcLeftVertical = null, rcRightHorizontal = null)))
        assertEquals("rc_stick_moved_right_vertical_50",
            zeroStickGroundRejection(grounded.copy(rcRightVertical = 50)))
    }

    @Test fun airborneAndUnknownFlightStateStayBlocked() {
        assertEquals("airborne", zeroStickGroundRejection(grounded.copy(isFlying = true)))
        assertEquals("flight_state_unavailable", zeroStickGroundRejection(grounded.copy(isFlying = null)))
    }

    @Test fun preConfirmationStateIsNotRcTakeoverButConfirmedLossIs() {
        assertEquals(false, zeroStickAuthorityLost(true, false, false, false, true))
        // Live startup: DJI can report VS enabled but RC-owned before MSDK ownership.
        assertEquals(false, zeroStickAuthorityLost(true, false, true, false, true))
        assertEquals(true, zeroStickAuthorityLost(true, true, false, false, true))
        assertEquals(true, zeroStickAuthorityLost(true, true, true, false, true))
        assertEquals(false, zeroStickAuthorityLost(true, true, true, true, false))
        assertEquals(false, zeroStickAuthorityLost(false, true, false, false, true))
    }

    @Test fun phase4RequiresAllThreeDjiAuthoritySignalsBeforeSubmission() {
        assertFalse(virtualStickAuthorityOwned(false, true, true))
        assertFalse(virtualStickAuthorityOwned(true, false, true))
        assertFalse(virtualStickAuthorityOwned(true, true, false))
        assertTrue(virtualStickAuthorityOwned(true, true, true))
    }

    @Test fun explicitDjiAuthorityDenialAbortsPhase4WithExactReason() {
        assertNull(phase4AuthorityAbortReason("MSDK_REQUEST"))
        assertNull(phase4AuthorityAbortReason("UNKNOWN"))
        assertEquals("authority_near_boundary",
            phase4AuthorityAbortReason("NEAR_BOUNDARY"))
        assertEquals("authority_rc_not_p_mode",
            phase4AuthorityAbortReason("RC_NOT_P_MODE"))
        assertEquals("authority_rc_pause_stop",
            phase4AuthorityAbortReason("RC_PAUSE_STOP"))
    }

    @Test fun expiredNetworkCommandGetsBoundedLocalZeroHoldThenTimesOut() {
        val deadlineMs = 1_250L // A fresh command was accepted at t=1,000 with 250 ms TTL.
        assertEquals(ZeroStickDeadlineState.FRESH, zeroStickDeadlineState(1_249, deadlineMs))
        assertEquals(ZeroStickDeadlineState.ZERO_HOLD, zeroStickDeadlineState(1_250, deadlineMs))
        assertEquals(ZeroStickDeadlineState.ZERO_HOLD, zeroStickDeadlineState(1_950, deadlineMs))
        assertEquals(ZeroStickDeadlineState.FRESH, zeroStickDeadlineState(1_950, 2_150))
        assertEquals(ZeroStickDeadlineState.ZERO_HOLD, zeroStickDeadlineState(2_499, deadlineMs))
        assertEquals(ZeroStickDeadlineState.EXPIRED, zeroStickDeadlineState(2_500, deadlineMs))
    }

    @Test fun phase4RequiresAStableLowGpsHoverAndCenteredRc() {
        assertNull(phase4AirborneRejection(phase4Hover))
        assertEquals("grounded", phase4AirborneRejection(phase4Hover.copy(isFlying = false)))
        assertEquals("gps_not_strong",
            phase4AirborneRejection(phase4Hover.copy(gpsSignalLevel = "LEVEL_2")))
        assertNull(phase4AirborneRejection(phase4Hover.copy(altitudeMeters = 8.9)))
        assertEquals("altitude_outside_test_band",
            phase4AirborneRejection(phase4Hover.copy(altitudeMeters = 9.1)))
        assertEquals("altitude_outside_test_band",
            phase4AirborneRejection(phase4Hover.copy(altitudeMeters = 1.9)))
        assertNull(phase4AirborneRejection(phase4Hover.copy(speedMetersPerSecond = 0.9)))
        assertEquals("aircraft_not_stably_hovering",
            phase4AirborneRejection(phase4Hover.copy(speedMetersPerSecond = 1.1)))
        assertEquals("battery_below_30_or_unavailable",
            phase4AirborneRejection(phase4Hover.copy(batteryPercent = 29)))
        assertEquals("flight_mode_not_allowed_CINEMATIC",
            phase4AirborneRejection(phase4Hover.copy(flightMode = "CINEMATIC")))
        assertEquals("rc_stick_moved_left_vertical_50",
            phase4AirborneRejection(phase4Hover.copy(rcLeftVertical = 50)))
    }

    @Test fun phase4ArmsOnlyFromTheStartBandButMayClimbAboveIt() {
        assertNull(phase4StartAltitudeRejection(phase4Hover.copy(altitudeMeters = 6.0)))
        assertNull(phase4StartAltitudeRejection(phase4Hover.copy(altitudeMeters = 7.0)))
        assertEquals("altitude_outside_start_band",
            phase4StartAltitudeRejection(phase4Hover.copy(altitudeMeters = 7.1)))
        assertEquals("altitude_outside_start_band",
            phase4StartAltitudeRejection(phase4Hover.copy(altitudeMeters = 1.9)))
        assertEquals("altitude_outside_start_band",
            phase4StartAltitudeRejection(phase4Hover.copy(altitudeMeters = null)))
    }

    @Test fun phase4ClimbStopsAtTheRelativeTargetOrTheAbsoluteCeiling() {
        assertNull(phase4AltitudeTermination(6.9, 6.0))
        assertEquals("altitude_target_reached", phase4AltitudeTermination(7.0, 6.0))
        assertEquals("altitude_target_reached", phase4AltitudeTermination(7.4, 6.0))
        // The ceiling outranks the relative target when both would fire.
        assertEquals("absolute_ceiling_reached", phase4AltitudeTermination(9.0, 8.5))
        assertEquals("absolute_ceiling_reached", phase4AltitudeTermination(9.0, 6.0))
        assertEquals("altitude_unavailable", phase4AltitudeTermination(null, 6.0))
        assertEquals("altitude_unavailable", phase4AltitudeTermination(Double.NaN, 6.0))
    }

    @Test fun phase4LocalDeadlineCannotBeExtendedByFreshSetpoints() {
        assertEquals(0.30, phase4VerticalOutput(5_999, 1_000, 0.30), 0.0)
        assertEquals(0.0, phase4VerticalOutput(6_000, 1_000, 0.30), 0.0)
        assertEquals(0.0, phase4VerticalOutput(7_000, 1_000, 0.30), 0.0)
    }

    /** The timer must outlast the climb, or the altitude cutoff could never fire. */
    @Test fun phase4TimeBackstopOutlastsTheClimbButStaysUnderTheCeiling() {
        val nominalClimbMs = (PHASE4_CLIMB_TARGET_M / PHASE4_VERTICAL_SPEED_MPS) * 1_000.0
        assertTrue(PHASE4_PULSE_WINDOW_MS > nominalClimbMs)
        val maxTravelM = PHASE4_VERTICAL_SPEED_MPS * (PHASE4_PULSE_WINDOW_MS / 1_000.0)
        assertTrue(PHASE4_START_MAX_ALTITUDE_M + maxTravelM < PHASE4_ABSOLUTE_CEILING_M)
    }
}
