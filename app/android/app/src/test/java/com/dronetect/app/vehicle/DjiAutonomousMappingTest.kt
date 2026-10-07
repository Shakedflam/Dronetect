package com.dronetect.app.vehicle

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class DjiAutonomousMappingTest {
    private val cruising = DjiTelemetrySnapshot(
        latitude = 32.0, longitude = 34.0, altitudeMeters = 48.0,
        aircraftConnected = true, rcConnected = true, isFlying = true,
        gpsSignalLevel = "LEVEL_5", flightMode = "GPS_NORMAL",
        rcLeftHorizontal = 0, rcLeftVertical = 0,
        rcRightHorizontal = 0, rcRightVertical = 0,
    )

    /** DJI GROUND is NED and documents pitch as the y-axis, roll as the x-axis. */
    @Test fun enuMapsEastToPitchAndNorthToRoll() {
        assertEquals(5.0, djiPitchFromEnu(5.0), 0.0)
        assertEquals(3.0, djiRollFromEnu(3.0), 0.0)
        // Flying due East must produce pitch only, never roll.
        assertEquals(4.0, djiPitchFromEnu(4.0), 0.0)
        assertEquals(0.0, djiRollFromEnu(0.0), 0.0)
        // Flying due North must produce roll only, never pitch.
        assertEquals(0.0, djiPitchFromEnu(0.0), 0.0)
        assertEquals(6.0, djiRollFromEnu(6.0), 0.0)
        // West and South are the negatives of East and North.
        assertEquals(-4.0, djiPitchFromEnu(-4.0), 0.0)
        assertEquals(-6.0, djiRollFromEnu(-6.0), 0.0)
    }

    @Test fun verticalKeepsEnuUpPositiveAsPhase4Proved() {
        assertEquals(0.30, djiVerticalFromEnu(0.30), 0.0)
        assertEquals(-1.0, djiVerticalFromEnu(-1.0), 0.0)
    }

    /** Compass [0,360) -> DJI ANGLE [-180,180], 0=North, clockwise positive. */
    @Test fun compassHeadingBecomesSignedGroundFrameAngle() {
        assertEquals(0.0, djiYawAngleFromCompassHeading(0.0), 0.0)
        assertEquals(90.0, djiYawAngleFromCompassHeading(90.0), 0.0)
        assertEquals(180.0, djiYawAngleFromCompassHeading(180.0), 0.0)
        assertEquals(-90.0, djiYawAngleFromCompassHeading(270.0), 0.0)
        assertEquals(-1.0, djiYawAngleFromCompassHeading(359.0), 0.0)
        assertEquals(0.0, djiYawAngleFromCompassHeading(360.0), 0.0)
        // Out-of-range input still lands inside DJI's accepted band.
        assertTrue(djiYawAngleFromCompassHeading(720.0) in -180.0..180.0)
        assertTrue(djiYawAngleFromCompassHeading(-90.0) in -180.0..180.0)
    }

    @Test fun horizontalSaturationPreservesDirection() {
        val (east, north) = saturateHorizontal(0.6, 0.8) // speed 1.0, under the limit
        assertEquals(0.6, east, 1e-9)
        assertEquals(0.8, north, 1e-9)
        // FlightLogic may ask for patrol 6 or pursuit 14; the aircraft may only
        // ever see the demo cap.
        val (fastEast, fastNorth) = saturateHorizontal(14.0, 0.0)
        assertEquals(AUTONOMOUS_MAX_HORIZONTAL_MPS, fastEast, 1e-9)
        assertEquals(0.0, fastNorth, 1e-9)
        val (dx, dy) = saturateHorizontal(30.0, 40.0)
        assertEquals(AUTONOMOUS_MAX_HORIZONTAL_MPS, kotlin.math.hypot(dx, dy), 1e-9)
        assertEquals(30.0 / 40.0, dx / dy, 1e-9) // bearing unchanged
        assertEquals(0.0 to 0.0, saturateHorizontal(0.0, 0.0))
    }

    /** The demo profile is a walking-pace fence at the actuator boundary. */
    @Test fun realDemoProfileClampsToTwoMetresPerSecond() {
        assertEquals(2.0, AUTONOMOUS_MAX_HORIZONTAL_MPS, 0.0)
        assertTrue(AUTONOMOUS_MAX_VERTICAL_MPS <= 2.0)
        val (east, north) = saturateHorizontal(6.0, 6.0) // patrol speed request
        assertEquals(2.0, kotlin.math.hypot(east, north), 1e-9)
    }

    @Test fun verticalSaturationClampsBothDirections() {
        assertEquals(1.0, saturateVertical(1.0), 1e-9)
        assertEquals(AUTONOMOUS_MAX_VERTICAL_MPS, saturateVertical(5.0), 1e-9)
        assertEquals(-AUTONOMOUS_MAX_VERTICAL_MPS, saturateVertical(-3.0), 1e-9)
    }

    @Test fun nonFiniteOrIncompleteCommandsAreRejected() {
        assertNull(autonomousCommandRejection(1.0, 2.0, 0.5, 90.0))
        assertEquals("command_incomplete", autonomousCommandRejection(null, 2.0, 0.5, 90.0))
        assertEquals("command_nonfinite",
            autonomousCommandRejection(Double.NaN, 2.0, 0.5, 90.0))
        assertEquals("command_nonfinite",
            autonomousCommandRejection(1.0, Double.POSITIVE_INFINITY, 0.5, 90.0))
        assertEquals("heading_outside_range", autonomousCommandRejection(1.0, 2.0, 0.5, 361.0))
    }

    @Test fun envelopeHoldsAltitudeCeilingAndFloor() {
        assertNull(autonomousEnvelopeRejection(cruising, 32.0, 34.0))
        assertEquals("altitude_above_ceiling", autonomousEnvelopeRejection(
            cruising.copy(altitudeMeters = AUTONOMOUS_ALTITUDE_CEILING_M + 0.1), 32.0, 34.0))
        assertEquals("altitude_below_floor", autonomousEnvelopeRejection(
            cruising.copy(altitudeMeters = 1.0), 32.0, 34.0))
        assertEquals("altitude_unavailable", autonomousEnvelopeRejection(
            cruising.copy(altitudeMeters = null), 32.0, 34.0))
        // The configured 48 m cruise sits inside the backstop, not against it.
        assertNull(autonomousEnvelopeRejection(cruising.copy(altitudeMeters = 48.0), 32.0, 34.0))
    }

    /** DJI's native takeoff hovers at ~1.2 m: only a climb may continue from there. */
    @Test fun belowTheFloorOnlyAClimbOutFromDjisTakeoffHoverIsAccepted() {
        val hover = cruising.copy(altitudeMeters = 1.2)
        assertEquals("altitude_below_floor", autonomousEnvelopeRejection(hover, null, null))
        assertEquals("altitude_below_floor", autonomousEnvelopeRejection(hover, null, null, upMps = -0.5))
        assertNull(autonomousEnvelopeRejection(hover, null, null, upMps = 1.0))
        assertTrue(autonomousBelowFloor(1.2))
        // Never from right off the ground, and never once above the floor.
        assertEquals("altitude_below_floor", autonomousEnvelopeRejection(
            cruising.copy(altitudeMeters = 0.3), null, null, upMps = 1.0))
        assertEquals(false, autonomousBelowFloor(AUTONOMOUS_MIN_ALTITUDE_M))
        assertNull(autonomousEnvelopeRejection(cruising.copy(altitudeMeters = 2.0), null, null, upMps = -0.5))
    }

    @Test fun nativeTakeoffNeedsGroundCentredSticksAndStrongGps() {
        val ground = cruising.copy(isFlying = false, altitudeMeters = 0.0)
        assertNull(nativeTakeoffRejection(ground))
        assertEquals("airborne", nativeTakeoffRejection(cruising))
        assertEquals("rc_stick_moved_left_vertical_200",
            nativeTakeoffRejection(ground.copy(rcLeftVertical = 200)))
        assertEquals("rc_disconnected", nativeTakeoffRejection(ground.copy(rcConnected = false)))
        assertEquals("gps_not_strong", nativeTakeoffRejection(ground.copy(gpsSignalLevel = "LEVEL_2")))
    }

    @Test fun envelopeHoldsRadiusBackstopIndependentlyOfAws() {
        // ~300 m north of the origin: inside the 330 m backstop.
        val near = cruising.copy(latitude = 32.0 + 300.0 / 111_320.0)
        assertNull(autonomousEnvelopeRejection(near, 32.0, 34.0))
        val far = cruising.copy(latitude = 32.0 + 400.0 / 111_320.0)
        assertEquals("outside_radius_backstop", autonomousEnvelopeRejection(far, 32.0, 34.0))
        assertEquals("position_unavailable",
            autonomousEnvelopeRejection(cruising.copy(latitude = null), 32.0, 34.0))
    }

    @Test fun rcInputAndLinkLossAbortAutonomousFlight() {
        assertEquals("rc_stick_moved_right_vertical_200",
            autonomousEnvelopeRejection(cruising.copy(rcRightVertical = 200), 32.0, 34.0))
        assertEquals("rc_disconnected",
            autonomousEnvelopeRejection(cruising.copy(rcConnected = false), 32.0, 34.0))
        assertEquals("aircraft_disconnected",
            autonomousEnvelopeRejection(cruising.copy(aircraftConnected = false), 32.0, 34.0))
        assertEquals("grounded",
            autonomousEnvelopeRejection(cruising.copy(isFlying = false), 32.0, 34.0))
        assertEquals("gps_not_strong",
            autonomousEnvelopeRejection(cruising.copy(gpsSignalLevel = "LEVEL_2"), 32.0, 34.0))
        assertEquals("flight_mode_not_allowed_ATTI",
            autonomousEnvelopeRejection(cruising.copy(flightMode = "ATTI"), 32.0, 34.0))
    }

    @Test fun radiusBackstopSitsOutsideTheConfiguredOperatingRadius() {
        // 300 m is the mission limit; Android's fence must not duplicate it.
        assertTrue(AUTONOMOUS_RADIUS_BACKSTOP_M > 300.0)
        assertTrue(AUTONOMOUS_ALTITUDE_CEILING_M > 48.0)
    }

    /**
     * Whole-vocabulary regression guard: a swapped axis or flipped sign here is
     * the failure mode that turns into a flyaway, so it is asserted end to end
     * exactly as tick() composes it.
     */
    @Test fun fullCommandVocabularyComposesWithoutSwappingAxes() {
        // FlightLogic: 0.6 m/s East, 0.8 m/s North, 1 m/s down, facing West (270).
        val (east, north) = saturateHorizontal(0.6, 0.8)
        val up = saturateVertical(-1.0)
        assertEquals(0.6, djiPitchFromEnu(east), 1e-9)   // East -> pitch
        assertEquals(0.8, djiRollFromEnu(north), 1e-9)   // North -> roll
        assertEquals(-1.0, djiVerticalFromEnu(up), 1e-9) // down stays negative
        assertEquals(-90.0, djiYawAngleFromCompassHeading(270.0), 1e-9)

        // Pure due-North flight must never produce an East component.
        assertEquals(0.0, djiPitchFromEnu(saturateHorizontal(0.0, 6.0).first), 1e-9)
        assertEquals(2.0, djiRollFromEnu(saturateHorizontal(0.0, 6.0).second), 1e-9)
        // Pure due-East flight must never produce a North component.
        assertEquals(2.0, djiPitchFromEnu(saturateHorizontal(6.0, 0.0).first), 1e-9)
        assertEquals(0.0, djiRollFromEnu(saturateHorizontal(6.0, 0.0).second), 1e-9)
    }

    @Test fun distanceMatchesKnownOffsets() {
        assertEquals(111.32, horizontalDistanceMeters(0.0, 0.0, 0.001, 0.0), 0.5)
        assertEquals(0.0, horizontalDistanceMeters(32.0, 34.0, 32.0, 34.0), 1e-9)
    }
}
