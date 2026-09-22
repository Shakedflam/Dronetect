package com.dronetect.app

import android.app.Application
import android.content.Context
import android.util.Log
import com.dronetect.app.vehicle.DjiSdkRuntime

/** Process-lifetime bootstrap for the read-only DJI MSDK runtime. */
class DronetectApplication : Application() {
    private var djiHelperInstalled = false

    override fun attachBaseContext(base: Context?) {
        super.attachBaseContext(base)
        try {
            com.cySdkyc.clx.Helper.install(this)
            djiHelperInstalled = true
            Log.i(DjiSdkRuntime.TAG, "helper=installed")
        } catch (error: Throwable) {
            Log.e(DjiSdkRuntime.TAG, "helper=failed; MSDK initialization disabled", error)
        }
    }

    override fun onCreate() {
        super.onCreate()
        if (djiHelperInstalled) {
            DjiSdkRuntime.initialize(this)
        } else {
            Log.e(DjiSdkRuntime.TAG, "initialization=skipped reason=helper_install_failed")
        }
    }
}
