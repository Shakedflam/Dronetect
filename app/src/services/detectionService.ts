import type { DetectedTarget } from '../types/drone';
import { realtimeServerConnection } from './internal/realtimeServerConnection';
import type { Unsubscribe } from './internal/unsubscribe';

/**
 * Abstraction over "what has the vision system detected right now". Emits
 * every currently-detected target (there can be zero, one, or several, e.g.
 * multiple other drones in frame) — choosing which one to act on is a UI
 * concern handled above this layer, not something the detector decides.
 *
 * Detections update at ~10 FPS while video runs at ~30 FPS; there is no
 * interpolation here. The backend may keep lost tracks alive briefly for
 * re-identification, but the operator overlay should only render boxes that
 * came from the latest successful detector match.
 */
export interface DetectionService {
  subscribe(listener: (targets: DetectedTarget[]) => void): Unsubscribe;
}

function toLabel(className: string): string {
  return className.charAt(0).toUpperCase() + className.slice(1);
}

/** Dev-only: how often to log detection age, so ~10fps updates don't flood the console. */
const DEV_LATENCY_LOG_INTERVAL_MS = 1000;

function createRealtimeServerDetectionService(): DetectionService {
  let lastLatencyLogAt = 0;

  return {
    subscribe(listener) {
      return realtimeServerConnection.subscribeDetections((message) => {
        const frameTimestampMs = message.timestampMs ?? Date.now();
        const { frameWidth, frameHeight } = message;
        const activeDetections = message.detections.filter(
          (detection) => detection.trackState !== 'lost'
        );

        if (import.meta.env.DEV) {
          const now = Date.now();
          if (now - lastLatencyLogAt >= DEV_LATENCY_LOG_INTERVAL_MS) {
            lastLatencyLogAt = now;
            // Camera -> YOLO -> tracker -> WebSocket -> tablet delay for this frame.
            console.debug(
              `[detectionService] frame ${message.frameId} age=${now - frameTimestampMs}ms raw=${message.detections.length} active=${activeDetections.length}`
            );
          }
        }

        listener(
          activeDetections.map((detection) => ({
            id: detection.trackId,
            label: toLabel(detection.className),
            confidence: detection.confidence,
            boundingBox: {
              x1: detection.box[0] / frameWidth,
              y1: detection.box[1] / frameHeight,
              x2: detection.box[2] / frameWidth,
              y2: detection.box[3] / frameHeight,
            },
            trackState: detection.trackState,
            lastSeenTimestamp: detection.lastSeenTimestamp,
            velocityPxPerSec: detection.velocityPxPerSec,
            frameTimestampMs,
            velocityNormalizedPerSec: {
              x: detection.velocityPxPerSec.x / frameWidth,
              y: detection.velocityPxPerSec.y / frameHeight,
            },
          }))
        );
      });
    },
  };
}

export const detectionService: DetectionService = createRealtimeServerDetectionService();
