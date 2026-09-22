package com.dronetect.app.vehicle

/** Vendor mapping seam; intentionally contains no speculative DJI types. */
class DjiTelemetryMapper {
    fun map(unverifiedSdkValue: Any): VehicleTelemetry {
        throw UnsupportedOperationException(
            "DJI telemetry mapping requires hardware-verified MSDK types: $unverifiedSdkValue",
        )
    }
}
