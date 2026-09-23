import { useEffect, useState } from 'react';
import { locationService } from '../services/locationService';

/**
 * True once it is safe to show real-flight patrol/ROI geometry: always true
 * for simulator/browser mode, false from the moment a real DJI session is
 * detected until the backend confirms its session home. See
 * locationService.ts's subscribeGeometryReady.
 */
export function useGeometryReady(): boolean {
  const [ready, setReady] = useState(true);
  useEffect(() => locationService.subscribeGeometryReady(setReady), []);
  return ready;
}
