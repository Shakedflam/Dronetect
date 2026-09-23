package com.dronetect.app.vehicle

import android.util.Log
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import okio.ByteString

/**
 * Owns the native `/video/ingest?role=drone` WebSocket.
 *
 * Deliberately scoped to video only: this does NOT open `/ws?role=drone`, so
 * the aircraft still does not advertise itself as a Dronetect mission drone
 * (see DroneRealtimeClient, which stays unimplemented on purpose).
 *
 * Mirrors `adapters/simulator/video.py`'s transport behavior: reconnect with
 * backoff, and latest-frame-wins rather than queuing a backlog -- a stale
 * frame is worth less than the next one, so frames are dropped when the
 * socket is down or the send queue is already backed up.
 */
class VideoIngestClient(private val url: String) {
    private val client = OkHttpClient.Builder()
        // A websocket carries no response body; the read timeout must not
        // close an idle-but-healthy link. Pings keep Cloudflare's tunnel and
        // any intermediary from reaping the connection.
        .readTimeout(0, TimeUnit.MILLISECONDS)
        .pingInterval(PING_INTERVAL_SECONDS, TimeUnit.SECONDS)
        .build()

    private val running = AtomicBoolean(false)
    private val reconnectAttempts = AtomicInteger(0)
    private val droppedFrames = AtomicInteger(0)

    @Volatile private var webSocket: WebSocket? = null
    @Volatile private var connected = false
    private val reconnectLock = Object()

    val isConnected: Boolean get() = connected

    /** Frames dropped since the last call, for throttled cadence logging. */
    fun takeDroppedFrameCount(): Int = droppedFrames.getAndSet(0)

    fun start() {
        if (!running.compareAndSet(false, true)) return
        connect()
    }

    fun stop(reason: String) {
        if (!running.compareAndSet(true, false)) return
        synchronized(reconnectLock) { reconnectLock.notifyAll() }
        val socket = webSocket
        webSocket = null
        connected = false
        try {
            socket?.close(NORMAL_CLOSURE, reason)
        } catch (error: Throwable) {
            socket?.cancel()
        }
        Log.i(DjiSdkRuntime.TAG, "video=ingest_stopped reason=$reason")
    }

    /**
     * Send one encoded bundle. Returns false when the frame was dropped
     * (socket down, or the send queue is already carrying a backlog).
     */
    fun send(bundle: ByteArray): Boolean {
        val socket = webSocket
        if (socket == null || !connected) {
            droppedFrames.incrementAndGet()
            return false
        }
        if (bundle.size > VideoFrameBundleCodec.MAX_PACKET_BYTES) {
            droppedFrames.incrementAndGet()
            Log.w(
                DjiSdkRuntime.TAG,
                "video=send_skipped reason=oversize bytes=${bundle.size}",
            )
            return false
        }
        // OkHttp buffers unsent frames in memory; without this check a slow
        // uplink would grow that buffer without bound while the aircraft
        // keeps producing frames.
        if (socket.queueSize() > MAX_QUEUED_BYTES) {
            droppedFrames.incrementAndGet()
            return false
        }
        val sent = try {
            socket.send(ByteString.of(bundle, 0, bundle.size))
        } catch (error: Throwable) {
            Log.w(DjiSdkRuntime.TAG, "video=send_failed reason=${error.javaClass.simpleName}")
            false
        }
        if (!sent) droppedFrames.incrementAndGet()
        return sent
    }

    private fun connect() {
        if (!running.get()) return
        val httpUrl = toHttpUrl(url)
        Log.i(DjiSdkRuntime.TAG, "video=ingest_connecting url=$url")
        val request = Request.Builder().url(httpUrl).build()
        webSocket = client.newWebSocket(request, Listener())
    }

    private fun scheduleReconnect(cause: String) {
        if (!running.get()) return
        val attempt = reconnectAttempts.incrementAndGet()
        val delayMs = minOf(
            INITIAL_RECONNECT_DELAY_MS * attempt,
            MAX_RECONNECT_DELAY_MS,
        )
        Log.i(
            DjiSdkRuntime.TAG,
            "video=ingest_reconnect attempt=$attempt delayMs=$delayMs cause=$cause",
        )
        val worker = Thread(
            Runnable {
                var interrupted = false
                synchronized(reconnectLock) {
                    if (running.get()) {
                        try {
                            reconnectLock.wait(delayMs)
                        } catch (error: InterruptedException) {
                            Thread.currentThread().interrupt()
                            interrupted = true
                        }
                    }
                }
                if (!interrupted && running.get()) connect()
            },
            "dronetect-video-ingest-reconnect",
        )
        worker.isDaemon = true
        worker.start()
    }

    private inner class Listener : WebSocketListener() {
        override fun onOpen(webSocket: WebSocket, response: Response) {
            connected = true
            reconnectAttempts.set(0)
            Log.i(DjiSdkRuntime.TAG, "video=ingest_connected url=$url")
        }

        override fun onMessage(webSocket: WebSocket, text: String) {
            // realtime_server only answers on this socket to report a
            // protocol error (e.g. video_source_already_connected).
            Log.w(DjiSdkRuntime.TAG, "video=ingest_server_message text=$text")
        }

        override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
            connected = false
            Log.i(DjiSdkRuntime.TAG, "video=ingest_closing code=$code reason=$reason")
            webSocket.close(NORMAL_CLOSURE, null)
        }

        override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
            connected = false
            this@VideoIngestClient.webSocket = null
            Log.i(DjiSdkRuntime.TAG, "video=ingest_disconnected code=$code reason=$reason")
            scheduleReconnect("closed:$code")
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            connected = false
            this@VideoIngestClient.webSocket = null
            Log.w(
                DjiSdkRuntime.TAG,
                "video=ingest_failed error=${t.javaClass.simpleName} message=${t.message} " +
                    "httpCode=${response?.code() ?: -1}",
            )
            scheduleReconnect(t.javaClass.simpleName)
        }
    }

    private companion object {
        const val NORMAL_CLOSURE = 1000
        const val PING_INTERVAL_SECONDS = 20L
        const val INITIAL_RECONNECT_DELAY_MS = 1_000L
        const val MAX_RECONNECT_DELAY_MS = 10_000L

        /** Roughly a few frames of slack before shedding load. */
        const val MAX_QUEUED_BYTES = 2L * 1024 * 1024

        /**
         * OkHttp routes websockets over its HTTP stack; `ws`/`wss` are mapped
         * explicitly here rather than relying on URL-parser coercion.
         */
        fun toHttpUrl(url: String): String = when {
            url.startsWith("wss://", ignoreCase = true) -> "https://" + url.substring(6)
            url.startsWith("ws://", ignoreCase = true) -> "http://" + url.substring(5)
            else -> url
        }
    }
}
