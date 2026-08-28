import { useEffect, useState } from 'react';
import type { DetectedTarget } from '../types/drone';
import { detectionService } from '../services/detectionService';

/** Every target currently detected in the drone's camera frame. */
export function useDetection(): DetectedTarget[] {
  const [targets, setTargets] = useState<DetectedTarget[]>([]);

  useEffect(() => {
    return detectionService.subscribe(setTargets);
  }, []);

  return targets;
}
