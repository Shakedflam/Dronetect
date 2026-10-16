package com.dronetect.app.vehicle

import android.graphics.Bitmap
import android.os.SystemClock
import android.util.Log
import com.dronetect.app.BuildConfig
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.manager.interfaces.ICameraStreamManager.FrameFormat
import java.util.Locale
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Feeds the Mini 4 Pro's camera into the existing Dronetect video pipeline.
 *
 *   Mini 4 Pro -> RC-N3 -> MSDK ICameraStreamManager.addFrameListener
 *     -> [SDK frame thread] FPS gate + latest-frame-wins handoff
 *     -> [encoder thread] Bitmap -> UI JPEG lane + vision JPEG lane
 *     -> VideoFrameBundleCodec (services/video.py wire format)
 *     -> VideoIngestClient -> wss://<origin>/video/ingest?role=drone
 *     -> realtime_server video_pipeline -> vision_server + WebRTC viewers
 *
 * Behavior deliberately mirrors `adapters/simulator/video.py`: target-FPS
 * throttling, a single-slot latest-frame handoff (stale frames are dropped,
 * never queued), bounded memory via two recycled buffers, and both JPEG
 * lanes on every frame -- `realtime_server` applies
 * `vision_frame_interval` itself when deciding which frames reach YOLO.
 *
 * This is video only. It does not open `/ws?role=drone`, so the aircraft
 * still does not act as a Dronetect mission drone.
 */
class DjiVideoIngestService {
    private val running = AtomicBoolean(false)
    private val frameLock = Object()

    @Volatile private var client: VideoIngestClient? = null
    @Volatile private var encoderThread: Thread? = null
    // Registration can be driven from the availability callback's thread as
    // well as start()/stop(), so this must be visible across them.
    @Volatile private var listenerRegistered = false

    // Single-slot latest-frame handoff. `spareBuffer` is handed back by the
    // encoder so a steady stream reuses two arrays instead of allocating per
    // frame.
    private var pendingBuffer: ByteArray? = null
    private var pendingLength = 0
    private var pendingWidth = 0
    private var pendingHeight = 0
    private var pendingFormat: FrameFormat? = null
    private var spareBuffer: ByteArray? = null

    private var frameId = 0L
    private var lastAcceptedFrameMs = 0L
    private var firstFrameLogged = false
    private var unsupportedFormatLogged = false
    private var lastCadenceLogMs = 0L
    private var framesReceived = 0
    private var framesSent = 0
    private var encodeFailures = 0
    private var encodeTimeMsTotal = 0L
    private var lastEncodeFailureLogMs = 0L

    private val frameListener = ICameraStreamManager.CameraFrameListener {
            data, offset, length, width, height, format ->
        onCameraFrame(data, offset, length, width, height, format)
    }

    private val availabilityListener =
        object : ICameraStreamManager.AvailableCameraUpdatedListener {
            override fun onAvailableCameraUpdated(availableCameras: List<ComponentIndexType>) {
                Log.i(
                    DjiSdkRuntime.TAG,
                    "video=available_cameras list=${availableCameras.joinToString(",")}",
                )
                if (availableCameras.contains(CAMERA_INDEX)) registerFrameListener()
            }

            /**
             * Must be implemented explicitly, not inherited.
             *
             * `dji-sdk-v5-aircraft-provided` (the compileOnly stub this
             * builds against) declares this as a Java `default` method, so
             * Kotlin is happy to omit it -- but the SecNeo-protected class
             * actually loaded at runtime from `dji-sdk-v5-aircraft` declares
             * it abstract. Omitting it compiles cleanly and then throws
             * AbstractMethodError the first time MSDK reports camera
             * availability. Keep this override even though the stub says it
             * is optional.
             */
            override fun onCameraStreamEnableUpdate(
                cameraStreamEnableMap: MutableMap<ComponentIndexType, Boolean>,
            ) {
                if (cameraStreamEnableMap.isEmpty()) return
                Log.i(
                    DjiSdkRuntime.TAG,
                    "video=camera_stream_enable_update " +
                        cameraStreamEnableMap.entries.joinToString(",") { "${it.key}=${it.value}" },
                )
                if (cameraStreamEnableMap[CAMERA_INDEX] == true) registerFrameListener()
            }
        }

    fun start(productId: Int) {
        if (!running.compareAndSet(false, true)) return

        val ingestUrl = BuildConfig.DRONTECT_REALTIME_WS_ORIGIN.trimEnd('/') + INGEST_PATH
        Log.i(
            DjiSdkRuntime.TAG,
            "video=starting productId=$productId targetFps=$TARGET_FPS url=$ingestUrl",
        )

        synchronized(frameLock) {
            pendingBuffer = null
            pendingFormat = null
            frameId = 0L
            lastAcceptedFrameMs = 0L
            firstFrameLogged = false
            unsupportedFormatLogged = false
            framesReceived = 0
            framesSent = 0
            encodeFailures = 0
            encodeTimeMsTotal = 0L
            lastCadenceLogMs = SystemClock.elapsedRealtime()
        }

        client = VideoIngestClient(ingestUrl, BuildConfig.DRONTECT_VIDEO_INGEST_TOKEN).also { it.start() }

        encoderThread = Thread(Runnable { encodeLoop() }, "dronetect-video-encoder").apply {
            isDaemon = true
            priority = Thread.NORM_PRIORITY - 1
            start()
        }

        try {
            streamManager().addAvailableCameraUpdatedListener(availabilityListener)
        } catch (error: Throwable) {
            Log.e(DjiSdkRuntime.TAG, "video=availability_listener_failed", error)
        }
        // The camera is often already available by the time the product
        // connects, in which case no availability callback follows.
        registerFrameListener()
    }

    fun stop(reason: String) {
        if (!running.compareAndSet(true, false)) return
        Log.i(DjiSdkRuntime.TAG, "video=stopping reason=$reason")

        try {
            streamManager().removeAvailableCameraUpdatedListener(availabilityListener)
        } catch (error: Throwable) {
            Log.w(DjiSdkRuntime.TAG, "video=availability_listener_remove_failed")
        }
        if (listenerRegistered) {
            try {
                streamManager().removeFrameListener(frameListener)
            } catch (error: Throwable) {
                Log.w(DjiSdkRuntime.TAG, "video=frame_listener_remove_failed")
            }
            listenerRegistered = false
        }

        synchronized(frameLock) {
            pendingBuffer = null
            pendingFormat = null
            frameLock.notifyAll()
        }
        encoderThread?.interrupt()
        encoderThread = null

        client?.stop(reason)
        client = null
    }

    fun restart(productId: Int) {
        stop("product_changed")
        start(productId)
    }

    private fun streamManager(): ICameraStreamManager =
        MediaDataCenter.getInstance().cameraStreamManager

    private fun registerFrameListener() {
        if (!running.get() || listenerRegistered) return
        try {
            streamManager().addFrameListener(CAMERA_INDEX, REQUESTED_FORMAT, frameListener)
            listenerRegistered = true
            Log.i(
                DjiSdkRuntime.TAG,
                "video=listener_registered camera=$CAMERA_INDEX format=$REQUESTED_FORMAT",
            )
        } catch (error: Throwable) {
            Log.e(DjiSdkRuntime.TAG, "video=listener_register_failed", error)
        }
    }

    /** Runs on an MSDK frame thread: copy and hand off, never block or encode here. */
    private fun onCameraFrame(
        data: ByteArray?,
        offset: Int,
        length: Int,
        width: Int,
        height: Int,
        format: FrameFormat?,
    ) {
        if (!running.get() || data == null || format == null || length <= 0) return

        val now = SystemClock.elapsedRealtime()
        synchronized(frameLock) {
            framesReceived++
            if (!firstFrameLogged) {
                firstFrameLogged = true
                Log.i(
                    DjiSdkRuntime.TAG,
                    "video=first_frame width=$width height=$height format=$format bytes=$length",
                )
            }
            // Target-FPS gate, matching the simulator's source_target_fps.
            if (lastAcceptedFrameMs != 0L && now - lastAcceptedFrameMs < MIN_FRAME_PERIOD_MS) {
                return
            }
            lastAcceptedFrameMs = now

            val reusable = spareBuffer
            val buffer = if (reusable != null && reusable.size >= length) {
                reusable
            } else {
                ByteArray(length)
            }
            System.arraycopy(data, offset, buffer, 0, length)
            spareBuffer = pendingBuffer
            pendingBuffer = buffer
            pendingLength = length
            pendingWidth = width
            pendingHeight = height
            pendingFormat = format
            frameLock.notifyAll()
        }
    }

    private fun encodeLoop() {
        while (running.get()) {
            var buffer: ByteArray? = null
            var length = 0
            var width = 0
            var height = 0
            var format: FrameFormat? = null

            synchronized(frameLock) {
                while (running.get() && pendingBuffer == null) {
                    try {
                        frameLock.wait(ENCODER_WAIT_MS)
                    } catch (error: InterruptedException) {
                        Thread.currentThread().interrupt()
                        return
                    }
                }
                buffer = pendingBuffer
                length = pendingLength
                width = pendingWidth
                height = pendingHeight
                format = pendingFormat
                pendingBuffer = null
                pendingFormat = null
            }

            val frame = buffer ?: continue
            val frameFormat = format ?: continue
            val startedAt = SystemClock.elapsedRealtime()
            val bundle = encodeBundle(frame, length, width, height, frameFormat)
            val elapsed = SystemClock.elapsedRealtime() - startedAt

            // Hand the buffer back for reuse before the next frame lands.
            synchronized(frameLock) {
                val spare = spareBuffer
                if (spare == null || spare.size < frame.size) spareBuffer = frame
                encodeTimeMsTotal += elapsed
                if (bundle == null) encodeFailures++
            }

            if (bundle != null && client?.send(bundle) == true) {
                synchronized(frameLock) { framesSent++ }
            }
            logCadenceIfDue()
        }
    }

    private fun encodeBundle(
        frame: ByteArray,
        length: Int,
        width: Int,
        height: Int,
        format: FrameFormat,
    ): ByteArray? {
        var source: Bitmap? = null
        try {
            source = VideoFrameBundleCodec.decodeFrame(frame, 0, length, width, height, format)
            if (source == null) {
                if (!unsupportedFormatLogged) {
                    unsupportedFormatLogged = true
                    Log.e(
                        DjiSdkRuntime.TAG,
                        "video=decode_unsupported format=$format width=$width height=$height",
                    )
                }
                return null
            }

            val ui = VideoFrameBundleCodec.encodeLane(
                source,
                VideoFrameBundleCodec.UI_MAX_WIDTH,
                VideoFrameBundleCodec.UI_MAX_HEIGHT,
                VideoFrameBundleCodec.UI_JPEG_QUALITY,
            ) ?: return null
            val vision = VideoFrameBundleCodec.encodeLane(
                source,
                VideoFrameBundleCodec.VISION_MAX_WIDTH,
                VideoFrameBundleCodec.VISION_MAX_HEIGHT,
                VideoFrameBundleCodec.VISION_JPEG_QUALITY,
            ) ?: return null

            // frame_id increments by one per sent frame: realtime_server's
            // vision_frame_interval selects vision frames with a modulo on it.
            frameId += 1
            return VideoFrameBundleCodec.encodeBundle(
                frameId,
                System.currentTimeMillis() / 1000.0,
                ui,
                vision,
            )
        } catch (error: Throwable) {
            logEncodeFailure(error)
            return null
        } finally {
            source?.recycle()
        }
    }

    private fun logEncodeFailure(error: Throwable) {
        val now = SystemClock.elapsedRealtime()
        if (now - lastEncodeFailureLogMs < ENCODE_FAILURE_LOG_INTERVAL_MS) return
        lastEncodeFailureLogMs = now
        Log.e(DjiSdkRuntime.TAG, "video=encode_failed error=${error.javaClass.simpleName}", error)
    }

    private data class CadenceSnapshot(
        val elapsedMs: Long,
        val received: Int,
        val sent: Int,
        val failures: Int,
        val encodeTimeMsTotal: Long,
    )

    private fun logCadenceIfDue() {
        val now = SystemClock.elapsedRealtime()
        val snapshot = synchronized(frameLock) {
            val elapsed = now - lastCadenceLogMs
            if (elapsed < CADENCE_LOG_INTERVAL_MS) return
            val taken = CadenceSnapshot(elapsed, framesReceived, framesSent, encodeFailures, encodeTimeMsTotal)
            framesReceived = 0
            framesSent = 0
            encodeFailures = 0
            encodeTimeMsTotal = 0
            lastCadenceLogMs = now
            taken
        }
        val seconds = snapshot.elapsedMs / 1000.0
        val receivedFps = if (seconds > 0) snapshot.received / seconds else 0.0
        val sentFps = if (seconds > 0) snapshot.sent / seconds else 0.0
        val encoded = snapshot.sent + snapshot.failures
        val averageEncodeMs = if (encoded > 0) snapshot.encodeTimeMsTotal / encoded else 0L
        Log.i(
            DjiSdkRuntime.TAG,
            "video=cadence " +
                "receivedFps=${String.format(Locale.US, "%.1f", receivedFps)} " +
                "sentFps=${String.format(Locale.US, "%.1f", sentFps)} " +
                "encodeMsAvg=$averageEncodeMs encodeFailures=${snapshot.failures} " +
                "dropped=${client?.takeDroppedFrameCount() ?: 0} " +
                "ingestConnected=${client?.isConnected ?: false}",
        )
    }

    private companion object {
        /** The Mini 4 Pro's main camera, matching DjiTelemetryMonitor's index usage. */
        val CAMERA_INDEX: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN

        /**
         * NV21 is the format DJI's own frame-listener samples use. The
         * callback still reports what it actually delivered, and the codec
         * handles RGBA_8888 as well, so a different delivery is logged rather
         * than silently dropped.
         */
        val REQUESTED_FORMAT: FrameFormat = FrameFormat.NV21

        const val INGEST_PATH = "/video/ingest?role=drone"

        /** Mirrors config/video.json `source_target_fps`. */
        const val TARGET_FPS = 30
        const val MIN_FRAME_PERIOD_MS = 1000L / TARGET_FPS

        const val ENCODER_WAIT_MS = 250L
        const val CADENCE_LOG_INTERVAL_MS = 5_000L
        const val ENCODE_FAILURE_LOG_INTERVAL_MS = 5_000L
    }
}
