package com.dronetect.app.vehicle

import org.json.JSONObject

/** Existing realtime_server/protocol.py vocabulary; no new cloud message types. */
object DroneWireProtocol {
    const val MAX_MESSAGE_CHARS = 65_536

    fun parseCommand(message: JSONObject): VehicleCommand {
        require(message.getString("type") == "command")
        val id = message.get("commandId") as? String ?: error("invalid_command_id")
        val name = message.get("command") as? String ?: error("invalid_command_name")
        require(id.length in 1..128 && id.all { it.code in 33..126 }) { "invalid_command_id" }
        require(name.matches(Regex("[a-z_]{1,64}"))) { "invalid_command_name" }
        val params = if (message.has("params")) message.getJSONObject("params") else JSONObject()
        return VehicleCommand(id, name, params.keys().asSequence().associateWith { key ->
            params.opt(key).takeUnless { it == JSONObject.NULL }
        })
    }

    fun telemetry(value: VehicleTelemetry): String = JSONObject().apply {
        put("type", "telemetry")
        put("source", "dji")
        put("timestamp", value.timestampSeconds)
        put("data", JSONObject().apply {
            put("gps", JSONObject().put("lat", value.latitude).put("lng", value.longitude))
            put("altitudeM", value.altitudeMeters)
            put("headingDeg", value.headingDegrees)
            put("speedMps", value.speedMetersPerSecond)
            put("batteryPercent", value.batteryPercent)
            put("state", value.state)
            put("targetPatrolRegionId", JSONObject.NULL)
            put("aircraftSafetyRthActive", value.aircraftSafetyRthActive)
            put("aircraftSafetyRthReason", value.aircraftSafetyRthReason ?: JSONObject.NULL)
            put("manualInputSource", value.manualInputSource)
            put("manualControlActive", false)
            put("pursuitBlocked", value.pursuitBlocked)
            put("operatingConfigPending", false)
        })
    }.toString()

    fun operatingConfigResult(id: String, result: CommandResult): String = JSONObject().apply {
        put("type", "operating_config_result")
        put("commandId", id)
        put("accepted", result.accepted)
        put("pending", false)
        put("message", result.message)
    }.toString()
}
