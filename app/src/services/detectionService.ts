import type { DetectedTarget } from '../types/drone';
import { realtimeServerConnection } from './internal/realtimeServerConnection';
import type { Unsubscribe } from './internal/unsubscribe';

/**
 * Abstraction over "what has the vision system detected right now". Emits
 * every currently-detected target (there can be zero, one, or several, e.g.
 * multiple other drones in frame) — choosing which one to act on is a UI
 * concern handled above this layer, not something the detector decides.
 *
 * Detections update at ~6 FPS while video runs at ~30 FPS; there is no
 * interpolation or tracking here — listeners simply keep the last-received
 * boxes until the next message replaces them.
 */
export interface DetectionService {
  subscribe(listener: (targets: DetectedTarget[]) => void): Unsubscribe;
}

function toLabel(className: string): string {
  return className.charAt(0).toUpperCase() + className.slice(1);
}

function createRealtimeServerDetectionService(): DetectionService {
  let lastFrameId = -1;

  return {
    subscribe(listener) {
      return realtimeServerConnection.subscribeDetections((message) => {
        if (message.frameId < lastFrameId) return;
        lastFrameId = message.frameId;

        const { frameWidth, frameHeight } = message;

        listener(
          message.detections.map((detection) => ({
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
          }))
        );
      });
    },
  };
}

export const detectionService: DetectionService = createRealtimeServerDetectionService();
