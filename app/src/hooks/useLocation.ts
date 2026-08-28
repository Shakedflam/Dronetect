import { useEffect, useState } from 'react';
import type { DronePosition } from '../types/drone';
import { locationService } from '../services/locationService';

/** Latest GPS fix for the drone, or null until the first reading arrives. */
export function useLocation(): DronePosition | null {
  const [position, setPosition] = useState<DronePosition | null>(null);

  useEffect(() => {
    return locationService.subscribe(setPosition);
  }, []);

  return position;
}
