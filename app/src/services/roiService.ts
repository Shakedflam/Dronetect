import type { Roi } from '../types/roi';
import type { Unsubscribe } from './internal/unsubscribe';

/**
 * Abstraction over "what is the current region of interest". Frontend-only
 * for now: `setRoi`/`clearRoi` are only ever called from map UI in
 * `components/map/MapView.tsx` (the operator drawing/dragging a circle) —
 * nothing here talks to `realtimeServerConnection`.
 *
 * The service shape is deliberately identical to the other `subscribe`-based
 * services (`locationService`, `detectionService`, ...) so that wiring in a
 * real source later is additive, not a rewrite: a future backend connection
 * would just call `roiService.setRoi(...)` whenever another machine reports
 * an ROI, exactly like map UI does today, and every subscriber (map layer,
 * any future readout) picks it up the same way. No caller needs to simulate
 * a map click.
 */
export interface RoiService {
  subscribe(listener: (roi: Roi | null) => void): Unsubscribe;
  getRoi(): Roi | null;
  setRoi(roi: Roi): void;
  clearRoi(): void;
}

function createInMemoryRoiService(): RoiService {
  const listeners = new Set<(roi: Roi | null) => void>();
  let current: Roi | null = null;

  function notify(): void {
    listeners.forEach((listener) => listener(current));
  }

  return {
    subscribe(listener) {
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
    getRoi() {
      return current;
    },
    setRoi(roi) {
      current = roi;
      notify();
    },
    clearRoi() {
      current = null;
      notify();
    },
  };
}

export const roiService: RoiService = createInMemoryRoiService();
