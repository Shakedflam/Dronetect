package com.dronetect.app

import android.os.Bundle
import com.dronetect.app.vehicle.DjiVehicleAdapter
import com.dronetect.app.vehicle.VehicleAdapter
import com.getcapacitor.BridgeActivity

/** Hosts the unchanged React UI and the fail-closed native vehicle boundary. */
class MainActivity : BridgeActivity() {
    private val vehicleAdapter: VehicleAdapter = DjiVehicleAdapter()

    override fun onCreate(savedInstanceState: Bundle?) {
        registerPlugin(DjiTelemetryPlugin::class.java)
        super.onCreate(savedInstanceState)
        vehicleAdapter.start()
    }

    override fun onDestroy() {
        vehicleAdapter.stop()
        super.onDestroy()
    }
}
