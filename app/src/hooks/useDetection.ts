import { useEffect, useRef, useState } from 'react';
import type { DetectedTarget } from '../types/drone';
import { detectionService } from '../services/detectionService';

/**
 * How long to keep showing the last known targets after the detector reports
 * none, before clearing the overlay. The detector runs at ~10fps and can
 * intermittently miss a frame or two, so clearing on a single empty update
 * makes boxes blink in and out; a short hold smooths over that gap without
 * hiding a genuine "target lost".
 */
const DETECTION_HOLD_MS = 500;

/** Every target currently detected in the drone's camera frame. */
export function useDetection(): DetectedTarget[] {
  const [targets, setTargets] = useState<DetectedTarget[]>([]);
  const clearTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    const unsubscribe = detectionService.subscribe((nextTargets) => {
      if (nextTargets.length > 0) {
        if (clearTimeoutRef.current !== null) {
          clearTimeout(clearTimeoutRef.current);
          clearTimeoutRef.current = null;
        }
        setTargets(nextTargets);
        return;
      }
      if (clearTimeoutRef.current !== null) return;
      clearTimeoutRef.current = setTimeout(() => {
        clearTimeoutRef.current = null;
        setTargets([]);
      }, DETECTION_HOLD_MS);
    });

    return () => {
      unsubscribe();
      if (clearTimeoutRef.current !== null) {
        clearTimeout(clearTimeoutRef.current);
        clearTimeoutRef.current = null;
      }
    };
  }, []);

  return targets;
}
