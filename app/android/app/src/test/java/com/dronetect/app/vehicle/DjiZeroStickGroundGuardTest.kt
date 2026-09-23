package com.dronetect.app.vehicle

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class DjiZeroStickGroundGuardTest {
    private val grounded = DjiTelemetrySnapshot(
        aircraftConnected = true, rcConnected = true, isFlying = false,
        rcLeftHorizontal = 0, rcLeftVertical = 0,
        rcRightHorizontal = 0, rcRightVertical = 0,
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

    @Test fun expiredNetworkCommandGetsBoundedLocalZeroHoldThenTimesOut() {
        val deadlineMs = 1_250L // A fresh command was accepted at t=1,000 with 250 ms TTL.
        assertEquals(ZeroStickDeadlineState.FRESH, zeroStickDeadlineState(1_249, deadlineMs))
        assertEquals(ZeroStickDeadlineState.ZERO_HOLD, zeroStickDeadlineState(1_250, deadlineMs))
        assertEquals(ZeroStickDeadlineState.ZERO_HOLD, zeroStickDeadlineState(1_950, deadlineMs))
        assertEquals(ZeroStickDeadlineState.FRESH, zeroStickDeadlineState(1_950, 2_150))
        assertEquals(ZeroStickDeadlineState.ZERO_HOLD, zeroStickDeadlineState(2_499, deadlineMs))
        assertEquals(ZeroStickDeadlineState.EXPIRED, zeroStickDeadlineState(2_500, deadlineMs))
    }
}
