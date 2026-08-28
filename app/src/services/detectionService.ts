import type { DetectedTarget } from '../types/drone';
import { getMockDetections } from '../mock/mockDetections';
import { createPollingSource, type Unsubscribe } from './internal/pollingSource';

/**
 * Abstraction over "what has the vision system detected right now". Emits
 * every currently-detected target (there can be zero, one, or several, e.g.
 * multiple other drones in frame) — choosing which one to act on is a UI
 * concern handled above this layer, not something the detector decides.
 */
export interface DetectionService {
  subscribe(listener: (targets: DetectedTarget[]) => void): Unsubscribe;
}

const POLL_INTERVAL_MS = 400;

function createMockDetectionService(): DetectionService {
  const source = createPollingSource(getMockDetections, POLL_INTERVAL_MS);
  return { subscribe: source.subscribe };
}

export const detectionService: DetectionService = createMockDetectionService();
