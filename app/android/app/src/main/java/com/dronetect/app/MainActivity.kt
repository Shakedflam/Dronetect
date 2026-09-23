package com.dronetect.app

import android.os.Bundle
import com.getcapacitor.BridgeActivity

/** Hosts the React UI. The process-lifetime MSDK runtime owns the aircraft link. */
class MainActivity : BridgeActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        registerPlugin(DjiTelemetryPlugin::class.java)
        super.onCreate(savedInstanceState)
    }
}
