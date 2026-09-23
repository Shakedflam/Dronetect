import { useEffect, useState } from 'react';
import type { Coordinates } from '../types/drone';
import { HOME_REFERENCE } from '../config/mapConfig';
import { locationService } from '../services/locationService';

/**
 * This session's home: the real DJI position captured on its first
 * complete GPS fix, or the static HOME_REFERENCE default until/unless DJI
 * provides one. See locationService.ts's subscribeHome for the capture rule.
 */
export function useHome(): Coordinates {
  const [home, setHome] = useState<Coordinates>(HOME_REFERENCE);
  useEffect(() => locationService.subscribeHome(setHome), []);
  return home;
}
