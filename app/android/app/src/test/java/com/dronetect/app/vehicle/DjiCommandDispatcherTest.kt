package com.dronetect.app.vehicle

import org.junit.Assert.*
import org.junit.Test

class DjiCommandDispatcherTest {
    private class Actions : DjiAircraftActions {
        var calls = 0
        var guard: (() -> Boolean)? = null
        var done: ((CommandResult) -> Unit)? = null
        var targetPitch: Double? = null
        override fun returnHome(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
            calls++; guard = isCurrent; done = complete
        }
        override fun tiltCamera(up: Boolean, isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
            calls++; guard = isCurrent; done = complete
        }
        override fun setGimbalPitch(pitchDeg: Double, isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
            calls++; targetPitch = pitchDeg; guard = isCurrent; done = complete
        }
        val native = mutableListOf<String>()
        override fun takeoff(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
            calls++; native.add("takeoff"); guard = isCurrent; done = complete
        }
        override fun land(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
            calls++; native.add("land"); guard = isCurrent; done = complete
        }
    }

    /** The bridge streams gimbal commands every tick; they must not block a flight action. */
    @Test fun nativeTakeoffAndLandPreemptGimbalTrafficAndCannotBurst() {
        var time = 0L
        val actions = Actions()
        val dispatcher = DjiCommandDispatcher(actions, { true }, { time })
        val results = mutableListOf<CommandResult>()
        dispatcher.dispatch(VehicleCommand("g", "gimbal_pitch", mapOf("pitchDeg" to -20.0)), results::add)
        dispatcher.dispatch(VehicleCommand("t", DjiCommandDispatcher.NATIVE_TAKEOFF), results::add)
        assertEquals("preempted_by_native_takeoff", results.single().message)
        assertEquals(listOf("takeoff"), actions.native)
        // A gimbal command cannot preempt the takeoff in turn.
        dispatcher.dispatch(VehicleCommand("g2", "gimbal_pitch", mapOf("pitchDeg" to -20.0)), results::add)
        assertEquals("action_in_progress", results.last().message)
        actions.done!!(CommandResult(true, "sdk_action_success"))
        time = 1_000
        dispatcher.dispatch(VehicleCommand("t2", DjiCommandDispatcher.NATIVE_TAKEOFF), results::add)
        assertEquals("native_takeoff_rate_limited", results.last().message)
        dispatcher.dispatch(VehicleCommand("l", DjiCommandDispatcher.NATIVE_LAND), results::add)
        assertEquals(listOf("takeoff", "land"), actions.native)
    }

    @Test fun missionIntentsNeverTurnIntoSdkMovement() {
        val actions = Actions()
        val dispatcher = DjiCommandDispatcher(actions, { true }, { 0 })
        for (name in listOf("start_patrol", "set_roi", "clear_roi", "set_operating_config",
            "set_patrol_priorities", "set_patrol_area_types", "set_manual_control", "set_manual_command", "reject_target", "takeoff")) {
            var result: CommandResult? = null
            dispatcher.dispatch(VehicleCommand(name, name)) { result = it }
            assertEquals(false, result?.accepted)
        }
        assertEquals(0, actions.calls)
    }

    @Test fun disconnectAndTimeoutInvalidateAsyncPreconditionsWithoutRetry() {
        var time = 0L
        val actions = Actions()
        val dispatcher = DjiCommandDispatcher(actions, { true }, { time })
        val results = mutableListOf<CommandResult>()
        dispatcher.dispatch(VehicleCommand("1", "return_home"), results::add)
        assertTrue(actions.guard!!())
        dispatcher.invalidate()
        assertFalse(actions.guard!!())
        actions.done!!(CommandResult(true, "late_success"))
        assertEquals(1, results.size)
        assertFalse(results.single().accepted)
        time = 6_000
        dispatcher.dispatch(VehicleCommand("2", "return_home"), results::add)
        time += 1_501
        assertFalse(actions.guard!!())
        time += 4_000
        dispatcher.expire()
        assertEquals("sdk_timeout_outcome_unknown_no_retry", results.last().message)
        assertEquals(2, actions.calls)
    }

    @Test fun noQueuedActionsAndNoBurstOrUnexpectedArguments() {
        val actions = Actions()
        var time = 0L
        val dispatcher = DjiCommandDispatcher(actions, { true }, { time })
        val results = mutableListOf<CommandResult>()
        dispatcher.dispatch(VehicleCommand("1", "camera_tilt_up"), results::add)
        dispatcher.dispatch(VehicleCommand("2", "camera_tilt_down"), results::add)
        assertEquals("action_in_progress", results.last().message)
        actions.done!!(CommandResult(true, "success"))
        dispatcher.dispatch(VehicleCommand("3", "camera_tilt_up"), results::add)
        assertEquals("rate_limited", results.last().message)
        time = 500
        dispatcher.dispatch(VehicleCommand("4", "camera_tilt_up", mapOf("pitch" to 90)), results::add)
        assertEquals("unexpected_params", results.last().message)
        assertEquals(1, actions.calls)
    }

    @Test fun holdTracksOperatorIntentWithoutSdkActions() {
        val actions = Actions()
        val dispatcher = DjiCommandDispatcher(actions, { true }, { 0 })
        dispatcher.dispatch(VehicleCommand("1", "set_pursuit_block", mapOf("blocked" to true))) { assertTrue(it.accepted) }
        assertTrue(dispatcher.pursuitBlocked)
        dispatcher.dispatch(VehicleCommand("2", "set_pursuit_block", mapOf("blocked" to "false"))) { assertFalse(it.accepted) }
        assertTrue(dispatcher.pursuitBlocked)
        assertEquals(0, actions.calls)
    }

    @Test fun rthPreemptsPendingCameraReadWithoutReplayingIt() {
        val actions = Actions()
        val dispatcher = DjiCommandDispatcher(actions, { true }, { 0 })
        var cameraResult: CommandResult? = null
        dispatcher.dispatch(VehicleCommand("camera", "camera_tilt_up")) { cameraResult = it }
        val oldGuard = actions.guard!!
        dispatcher.dispatch(VehicleCommand("rth", "return_home")) {}
        assertFalse(oldGuard())
        assertEquals("preempted_by_rth", cameraResult?.message)
        assertTrue(actions.guard!!())
        assertEquals(2, actions.calls)
    }

    @Test fun pitchStepPreservesSimulatorUnitsAndHonorsReportedHardwareLimits() {
        assertEquals(Math.toDegrees(0.03), DjiCommandDispatcher.nextPitch(0.0, -90.0, 60.0, true)!!, 0.0001)
        assertEquals(-90.0, DjiCommandDispatcher.nextPitch(-89.9, -90.0, 60.0, false)!!, 0.0001)
        assertEquals(20.0, DjiCommandDispatcher.nextPitch(19.9, -90.0, 20.0, true)!!, 0.0001)
        assertNull(DjiCommandDispatcher.nextPitch(Double.NaN, -90.0, 60.0, true))
        assertNull(DjiCommandDispatcher.nextPitch(0.0, 90.0, -90.0, true))
        assertNull(DjiCommandDispatcher.nextPitch(45.0, -90.0, 60.0, true))
    }

    @Test fun absolutePitchUsesExistingActionGuardAndRejectsOutOfRange() {
        val actions = Actions()
        val dispatcher = DjiCommandDispatcher(actions, { true }, { 0L })
        dispatcher.dispatch(VehicleCommand("bad", "gimbal_pitch", mapOf("pitchDeg" to 90.0))) {
            assertFalse(it.accepted)
        }
        assertEquals(0, actions.calls)
        dispatcher.dispatch(VehicleCommand("good", "gimbal_pitch", mapOf("pitchDeg" to -15.0))) {}
        assertEquals(-15.0, actions.targetPitch!!, 0.001)
        assertTrue(actions.guard!!())
        dispatcher.invalidate()
        assertFalse(actions.guard!!())
    }
}
