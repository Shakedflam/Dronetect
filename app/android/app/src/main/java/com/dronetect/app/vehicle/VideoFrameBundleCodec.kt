package com.dronetect.app.vehicle

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import dji.v5.manager.interfaces.ICameraStreamManager.FrameFormat
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import org.json.JSONObject

/** One encoded JPEG lane of a video frame bundle. */
data class JpegLane(val width: Int, val height: Int, val bytes: ByteArray) {
    // ByteArray in a data class needs identity-free equals/hashCode; these are
    // only used by tests/logging, never for frame routing.
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is JpegLane) return false
        return width == other.width && height == other.height && bytes.contentEquals(other.bytes)
    }

    override fun hashCode(): Int =
        (width * 31 + height) * 31 + bytes.contentHashCode()
}

/**
 * Binary wire format for `/video/ingest?role=drone`, matching
 * `services/video.py`'s `encode_video_frame_bundle` byte for byte:
 *
 *   [4-byte big-endian header length][UTF-8 JSON header][ui JPEG][vision JPEG]
 *
 * The JSON header carries `type`, `frame_id`, `timestamp`, and a `frames`
 * object with `ui`/`vision` entries of `width`, `height`, `encoding`, and
 * `byte_length`. `realtime_server` rejects the packet if the declared lane
 * byte lengths do not consume the payload exactly ("trailing data"), so the
 * declared lengths must be the real encoded sizes.
 *
 * Lane sizes/qualities mirror `config/video.json`. They are duplicated here
 * because that file is server-side configuration and is not packaged into the
 * APK; `realtime_server` reads each lane's real dimensions out of this header
 * rather than validating them against its own config, so a mismatch changes
 * bandwidth/quality only. Keep them in sync deliberately.
 */
object VideoFrameBundleCodec {
    const val MESSAGE_TYPE = "video_frame_bundle"
    const val JPEG_ENCODING = "image/jpeg"

    /** Mirrors config/video.json `ui_max_width` / `ui_max_height` / `ui_jpeg_quality`. */
    const val UI_MAX_WIDTH = 1280
    const val UI_MAX_HEIGHT = 720
    const val UI_JPEG_QUALITY = 90

    /** Mirrors config/video.json `vision_max_*` / `vision_jpeg_quality`. */
    const val VISION_MAX_WIDTH = 960
    const val VISION_MAX_HEIGHT = 540
    const val VISION_JPEG_QUALITY = 92

    /** Mirrors services/video.py `VIDEO_MAX_PACKET_BYTES`. */
    const val MAX_PACKET_BYTES = 8 * 1024 * 1024

    fun encodeBundle(
        frameId: Long,
        timestampSeconds: Double,
        ui: JpegLane,
        vision: JpegLane,
    ): ByteArray {
        val header = JSONObject().apply {
            put("type", MESSAGE_TYPE)
            put("frame_id", frameId)
            put("timestamp", timestampSeconds)
            put(
                "frames",
                JSONObject().apply {
                    put("ui", laneHeader(ui))
                    put("vision", laneHeader(vision))
                },
            )
        }
        val headerBytes = header.toString().toByteArray(Charsets.UTF_8)
        val lengthPrefix = ByteBuffer.allocate(4)
            .order(ByteOrder.BIG_ENDIAN)
            .putInt(headerBytes.size)
            .array()

        val output = ByteArrayOutputStream(
            lengthPrefix.size + headerBytes.size + ui.bytes.size + vision.bytes.size,
        )
        output.write(lengthPrefix)
        output.write(headerBytes)
        output.write(ui.bytes)
        output.write(vision.bytes)
        return output.toByteArray()
    }

    private fun laneHeader(lane: JpegLane): JSONObject = JSONObject().apply {
        put("width", lane.width)
        put("height", lane.height)
        put("encoding", JPEG_ENCODING)
        put("byte_length", lane.bytes.size)
    }

    /**
     * Convert one DJI camera frame into a Bitmap.
     *
     * `addFrameListener` is asked for NV21, but the callback reports the
     * format it actually delivered, so both the YUV and the packed-RGBA
     * layouts are handled rather than assumed. Returns null for a layout this
     * build does not decode, so the caller can log it once instead of
     * crashing the SDK's frame thread.
     */
    fun decodeFrame(
        data: ByteArray,
        offset: Int,
        length: Int,
        width: Int,
        height: Int,
        format: FrameFormat,
    ): Bitmap? {
        if (width <= 0 || height <= 0 || length <= 0) return null
        return when (format) {
            FrameFormat.NV21 -> decodeNv21(data, offset, length, width, height)
            FrameFormat.RGBA_8888 -> decodeRgba8888(data, offset, length, width, height)
            else -> null
        }
    }

    private fun decodeNv21(
        data: ByteArray,
        offset: Int,
        length: Int,
        width: Int,
        height: Int,
    ): Bitmap? {
        // YuvImage always reads from index 0, so a non-zero offset has to be
        // materialized into its own array first.
        val frame = if (offset == 0 && length == data.size) {
            data
        } else {
            data.copyOfRange(offset, offset + length)
        }
        val jpeg = ByteArrayOutputStream(length / 4)
        val encoded = YuvImage(frame, ImageFormat.NV21, width, height, null)
            .compressToJpeg(Rect(0, 0, width, height), UI_JPEG_QUALITY, jpeg)
        if (!encoded) return null
        val bytes = jpeg.toByteArray()
        return BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
    }

    private fun decodeRgba8888(
        data: ByteArray,
        offset: Int,
        length: Int,
        width: Int,
        height: Int,
    ): Bitmap? {
        val required = width * height * 4
        if (length < required) return null
        // ARGB_8888 is stored as R,G,B,A bytes in memory, which is the layout
        // RGBA_8888 delivers, so the pixels copy straight in.
        val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        bitmap.copyPixelsFromBuffer(ByteBuffer.wrap(data, offset, required))
        return bitmap
    }

    /**
     * Encode one lane, downscaling to fit the lane's bounds while preserving
     * aspect ratio. Mirrors `adapters/simulator/video.py`'s `_resize_to_fit`:
     * a frame already within bounds is never upscaled.
     */
    fun encodeLane(source: Bitmap, maxWidth: Int, maxHeight: Int, quality: Int): JpegLane? {
        val scaled = resizeToFit(source, maxWidth, maxHeight)
        // Read the dimensions before recycling: they must describe the bytes
        // actually encoded, and a recycled Bitmap's accessors are undefined.
        val width = scaled.width
        val height = scaled.height
        val jpeg = ByteArrayOutputStream()
        val compressed = scaled.compress(Bitmap.CompressFormat.JPEG, quality, jpeg)
        if (scaled !== source) scaled.recycle()
        if (!compressed) return null
        return JpegLane(width, height, jpeg.toByteArray())
    }

    private fun resizeToFit(source: Bitmap, maxWidth: Int, maxHeight: Int): Bitmap {
        if (source.width <= maxWidth && source.height <= maxHeight) return source
        val scale = minOf(
            maxWidth.toDouble() / source.width,
            maxHeight.toDouble() / source.height,
        )
        val width = maxOf(1, Math.round(source.width * scale).toInt())
        val height = maxOf(1, Math.round(source.height * scale).toInt())
        return Bitmap.createScaledBitmap(source, width, height, true)
    }
}
