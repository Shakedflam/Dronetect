import type { DetectedTarget } from '../types/drone';
import type { Unsubscribe } from './internal/unsubscribe';

/**
 * Abstraction over "what has the vision system detected right now". Emits
 * every currently-detected target (there can be zero, one, or several, e.g.
 * multiple other drones in frame) — choosing which one to act on is a UI
 * concern handled above this layer, not something the detector decides.
 *
 * No detection pipeline is wired into the frontend yet, so this always
 * reports zero targets until it's swapped for a real feed.
 */
export interface DetectionService {
  subscribe(listener: (targets: DetectedTarget[]) => void): Unsubscribe;
}

function createDetectionService(): DetectionService {
  return {
    subscribe(listener) {
      listener([]);
      return () => {};
    },
  };
}

export const detectionService: DetectionService = createDetectionService();
