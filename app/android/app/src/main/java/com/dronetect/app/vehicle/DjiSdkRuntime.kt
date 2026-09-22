package com.dronetect.app.vehicle

import android.content.Context
import android.util.Log
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.CopyOnWriteArraySet

data class DjiTelemetryState(
    val connectionState: String,
    val snapshot: DjiTelemetrySnapshot,
    val timestampMs: Long,
)

/**
 * Read-only MSDK lifecycle boundary. This class initializes and observes DJI
 * connection state only; it intentionally exposes no aircraft-control APIs.
 */
object DjiSdkRuntime {
    const val TAG = "DronetectDJI"

    private val initializationStarted = AtomicBoolean(false)
    private val registrationRequested = AtomicBoolean(false)
    private val productConnected = AtomicBoolean(false)
    private val telemetryObservers = CopyOnWriteArraySet<(DjiTelemetryState) -> Unit>()
    @Volatile private var latestTelemetryState = DjiTelemetryState(
        "disconnected", DjiTelemetrySnapshot(), System.currentTimeMillis(),
    )
    private val telemetryMonitor = DjiTelemetryMonitor { publishTelemetry(it) }

    fun telemetrySnapshot(): DjiTelemetrySnapshot = telemetryMonitor.snapshot()

    fun telemetryState(): DjiTelemetryState = latestTelemetryState

    fun addTelemetryObserver(observer: (DjiTelemetryState) -> Unit) {
        telemetryObservers.add(observer)
    }

    fun removeTelemetryObserver(observer: (DjiTelemetryState) -> Unit) {
        telemetryObservers.remove(observer)
    }

    private fun publishTelemetry(snapshot: DjiTelemetrySnapshot = telemetrySnapshot()) {
        val state = DjiTelemetryState(
            if (productConnected.get()) "connected" else "disconnected",
            if (productConnected.get()) snapshot else DjiTelemetrySnapshot(),
            System.currentTimeMillis(),
        )
        latestTelemetryState = state
        telemetryObservers.forEach { it(state) }
    }

    fun initialize(context: Context) {
        if (!initializationStarted.compareAndSet(false, true)) {
            Log.i(TAG, "initialization=already_started")
            return
        }

        Log.i(TAG, "initialization=starting sdkVersion=5.18.0")
        try {
            SDKManager.getInstance().init(
                context.applicationContext,
                object : SDKManagerCallback {
                    override fun onRegisterSuccess() {
                        Log.i(TAG, "registration=success")
                    }

                    override fun onRegisterFailure(error: IDJIError) {
                        Log.e(
                            TAG,
                            "registration=failure type=${error.errorType()} " +
                                "code=${error.errorCode()} innerCode=${error.innerCode()} " +
                                "description=${error.description()} hint=${error.hint()}",
                        )
                    }

                    override fun onProductDisconnect(productId: Int) {
                        productConnected.set(false)
                        telemetryMonitor.stop("product_disconnected")
                        publishTelemetry()
                        Log.i(TAG, "product=disconnected productId=$productId")
                    }

                    override fun onProductConnect(productId: Int) {
                        productConnected.set(true)
                        Log.i(TAG, "product=connected productId=$productId")
                        publishTelemetry(DjiTelemetrySnapshot())
                        telemetryMonitor.start(productId)
                    }

                    override fun onProductChanged(productId: Int) {
                        Log.i(TAG, "product=changed productId=$productId")
                        if (productConnected.get()) {
                            telemetryMonitor.restart(productId)
                            publishTelemetry(DjiTelemetrySnapshot())
                        }
                    }

                    override fun onInitProcess(event: DJISDKInitEvent, totalProcess: Int) {
                        Log.i(TAG, "initialization=progress event=$event progress=$totalProcess")
                        if (
                            event == DJISDKInitEvent.INITIALIZE_COMPLETE &&
                            registrationRequested.compareAndSet(false, true)
                        ) {
                            Log.i(TAG, "initialization=complete registration=requested")
                            SDKManager.getInstance().registerApp()
                        }
                    }

                    override fun onDatabaseDownloadProgress(current: Long, total: Long) {
                        Log.i(TAG, "database_download current=$current total=$total")
                    }
                },
            )
        } catch (error: Throwable) {
            Log.e(TAG, "initialization=failure", error)
        }
    }
}
