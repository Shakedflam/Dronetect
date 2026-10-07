package com.dronetect.app.vehicle

import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager

/** Signatures verified with javap against dji-sdk-v5-aircraft-provided-5.18.0.jar. */
class DjiSdkAircraftActions : DjiAircraftActions {
    override fun setGimbalPitch(pitchDeg: Double, isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
        val index = ComponentIndexType.LEFT_OR_MAIN
        read(KeyTools.createKey(GimbalKey.KeyConnection, index), isCurrent, complete) connected@{ connected ->
            if (!connected) return@connected complete(CommandResult(false, "gimbal_disconnected"))
            read(KeyTools.createKey(GimbalKey.KeyGimbalAttitudeRange, index), isCurrent, complete) range@{ range ->
                val min = range.pitch?.min
                val max = range.pitch?.max
                if (min == null || max == null || pitchDeg !in min..max) {
                    return@range complete(CommandResult(false, "gimbal_pitch_outside_verified_limits"))
                }
                val key = KeyTools.createKey(GimbalKey.KeyRotateByAngle, index)
                guarded(isCurrent, complete) {
                    val manager = KeyManager.getInstance()
                    if (!manager.isKeySupported(key)) {
                        complete(CommandResult(false, "gimbal_action_unsupported"))
                    } else {
                        val rotation = GimbalAngleRotation().apply {
                            mode = GimbalAngleRotationMode.ABSOLUTE_ANGLE
                            pitch = pitchDeg
                            pitchIgnored = false
                            rollIgnored = true
                            yawIgnored = true
                            duration = 0.3
                            jointReferenceUsed = false
                        }
                        manager.performAction(key, rotation, completion(complete))
                    }
                }
            }
        }
    }

    override fun returnHome(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
        read(KeyTools.createKey(FlightControllerKey.KeyConnection), isCurrent, complete) { connected ->
            if (!connected) return@read complete(CommandResult(false, "aircraft_disconnected"))
            read(KeyTools.createKey(FlightControllerKey.KeyIsFlying), isCurrent, complete) flying@{ flying ->
                if (!flying) return@flying complete(CommandResult(false, "rth_requires_airborne_aircraft"))
                read(KeyTools.createKey(FlightControllerKey.KeyFlightMode), isCurrent, complete) mode@{ mode ->
                    if (mode == FlightMode.GO_HOME) {
                        return@mode complete(CommandResult(true, "already_returning_home"))
                    }
                    if (mode in setOf(FlightMode.AUTO_LANDING, FlightMode.FORCE_LANDING,
                            FlightMode.ATTI_LANDING, FlightMode.UNKNOWN)) {
                        return@mode complete(CommandResult(false, "rth_blocked_by_flight_mode"))
                    }
                    read(KeyTools.createKey(FlightControllerKey.KeyIsHomeLocationSet), isCurrent, complete) home@{ homeSet ->
                        if (!homeSet) return@home complete(CommandResult(false, "home_location_unavailable"))
                        val key = KeyTools.createKey(FlightControllerKey.KeyStartGoHome)
                        guarded(isCurrent, complete) {
                            val manager = KeyManager.getInstance()
                            if (!manager.isKeySupported(key)) {
                                complete(CommandResult(false, "rth_key_unsupported"))
                            } else {
                                // Keep DJI's GPS/home/obstacle/battery checks and configured RTH height.
                                manager.performAction(key, completion(complete))
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * DJI's own takeoff (KeyStartTakeoff): DJI spins up and climbs to its
     * standard hover. Requires a recorded home point so DJI's own RTH/RC-loss
     * fail-safes have somewhere to go. Every DJI takeoff check stays in force.
     */
    override fun takeoff(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
        read(KeyTools.createKey(FlightControllerKey.KeyConnection), isCurrent, complete) { connected ->
            if (!connected) return@read complete(CommandResult(false, "aircraft_disconnected"))
            read(KeyTools.createKey(FlightControllerKey.KeyIsFlying), isCurrent, complete) flying@{ flying ->
                if (flying) return@flying complete(CommandResult(false, "takeoff_requires_grounded_aircraft"))
                read(KeyTools.createKey(FlightControllerKey.KeyIsHomeLocationSet), isCurrent, complete) home@{ homeSet ->
                    if (!homeSet) return@home complete(CommandResult(false, "home_location_unavailable"))
                    perform(KeyTools.createKey(FlightControllerKey.KeyStartTakeoff), "takeoff_key_unsupported",
                        isCurrent, complete)
                }
            }
        }
    }

    /**
     * DJI's own landing (KeyStartAutoLanding) where the aircraft is. Virtual
     * Stick must already be released. KeyConfirmLanding is never sent: if DJI's
     * landing protection stops to ask, that confirmation stays with the pilot.
     */
    override fun land(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
        read(KeyTools.createKey(FlightControllerKey.KeyConnection), isCurrent, complete) { connected ->
            if (!connected) return@read complete(CommandResult(false, "aircraft_disconnected"))
            read(KeyTools.createKey(FlightControllerKey.KeyIsFlying), isCurrent, complete) flying@{ flying ->
                if (!flying) return@flying complete(CommandResult(false, "land_requires_airborne_aircraft"))
                read(KeyTools.createKey(FlightControllerKey.KeyFlightMode), isCurrent, complete) mode@{ mode ->
                    when (mode) {
                        FlightMode.AUTO_LANDING, FlightMode.FORCE_LANDING, FlightMode.ATTI_LANDING ->
                            return@mode complete(CommandResult(true, "already_landing"))
                        FlightMode.VIRTUAL_STICK ->
                            return@mode complete(CommandResult(false, "virtual_stick_still_active"))
                        FlightMode.GO_HOME, FlightMode.UNKNOWN ->
                            return@mode complete(CommandResult(false, "land_blocked_by_flight_mode_$mode"))
                        else -> perform(KeyTools.createKey(FlightControllerKey.KeyStartAutoLanding),
                            "land_key_unsupported", isCurrent, complete)
                    }
                }
            }
        }
    }

    private fun perform(
        key: DJIKey.ActionKey<EmptyMsg, EmptyMsg>, unsupported: String,
        isCurrent: () -> Boolean, complete: (CommandResult) -> Unit,
    ) = guarded(isCurrent, complete) {
        val manager = KeyManager.getInstance()
        if (!manager.isKeySupported(key)) complete(CommandResult(false, unsupported))
        else manager.performAction(key, completion(complete))
    }

    override fun tiltCamera(up: Boolean, isCurrent: () -> Boolean, complete: (CommandResult) -> Unit) {
        val index = ComponentIndexType.LEFT_OR_MAIN
        read(KeyTools.createKey(GimbalKey.KeyConnection, index), isCurrent, complete) connected@{ connected ->
            if (!connected) return@connected complete(CommandResult(false, "gimbal_disconnected"))
            read(KeyTools.createKey(GimbalKey.KeyGimbalAttitude, index), isCurrent, complete) { attitude ->
                read(KeyTools.createKey(GimbalKey.KeyGimbalAttitudeRange, index), isCurrent, complete) range@{ range ->
                    val current = attitude.pitch
                    val min = range.pitch?.min
                    val max = range.pitch?.max
                    if (current == null || min == null || max == null) {
                        return@range complete(CommandResult(false, "gimbal_limits_unavailable"))
                    }
                    val target = DjiCommandDispatcher.nextPitch(current, min, max, up)
                        ?: return@range complete(CommandResult(false, "gimbal_pitch_outside_verified_limits"))
                    val key = KeyTools.createKey(GimbalKey.KeyRotateByAngle, index)
                    guarded(isCurrent, complete) {
                        val manager = KeyManager.getInstance()
                        if (!manager.isKeySupported(key)) {
                            complete(CommandResult(false, "gimbal_action_unsupported"))
                        } else {
                            val rotation = GimbalAngleRotation().apply {
                                mode = GimbalAngleRotationMode.ABSOLUTE_ANGLE
                                pitch = target
                                pitchIgnored = false
                                rollIgnored = true
                                yawIgnored = true
                                duration = 0.3
                                jointReferenceUsed = false
                            }
                            manager.performAction(key, rotation, completion(complete))
                        }
                    }
                }
            }
        }
    }

    private fun <T> read(
        key: DJIKey<T>, isCurrent: () -> Boolean, complete: (CommandResult) -> Unit, onValue: (T) -> Unit,
    ) = guarded(isCurrent, complete) {
        val manager = KeyManager.getInstance()
        if (!manager.isKeySupported(key)) {
            complete(CommandResult(false, "precondition_key_unsupported"))
        } else {
            manager.getValue(key, object : CommonCallbacks.CompletionCallbackWithParam<T> {
                override fun onSuccess(value: T?) {
                    guarded(isCurrent, complete) {
                        if (value == null) complete(CommandResult(false, "precondition_unavailable"))
                        else onValue(value)
                    }
                }
                override fun onFailure(error: IDJIError) = complete(failure(error))
            })
        }
    }

    private fun guarded(isCurrent: () -> Boolean, complete: (CommandResult) -> Unit, action: () -> Unit) {
        if (!isCurrent()) {
            complete(CommandResult(false, "expired_or_disconnected_before_sdk_action"))
            return
        }
        try { action() } catch (error: Exception) {
            complete(CommandResult(false, "sdk_exception_${error.javaClass.simpleName}"))
        }
    }

    private fun completion(complete: (CommandResult) -> Unit) =
        object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(value: EmptyMsg?) = complete(CommandResult(true, "sdk_action_success"))
            override fun onFailure(error: IDJIError) = complete(failure(error))
        }

    private fun failure(error: IDJIError) = CommandResult(
        false, "dji_error type=${error.errorType()} code=${error.errorCode()} " +
            "innerCode=${error.innerCode()} description=${error.description()}",
    )
}
