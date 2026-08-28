import type { Coordinates, DronePosition } from '../types/drone';
import { bearingDeg } from '../utils/geo';

/**
 * Standalone mock position generator. Knows nothing about React or the
 * service layer — it just produces the "next" GPS fix given elapsed time.
 * Swapping this module out for a real GPS/telemetry feed is the only change
 * needed to make LocationService live.
 */

const HOME: Coordinates = { lat: 32.0853, lng: 34.7818 };
const MOCK_DESTINATION: Coordinates = { lat: 32.0975, lng: 34.7995 };

/** One full home -> destination -> home cycle, in milliseconds. */
const CYCLE_DURATION_MS = 90_000;

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

/** Triangle wave in [0, 1]: eases out to the destination, then back home. */
function legProgress(elapsedMs: number): { from: Coordinates; to: Coordinates; t: number } {
  const phase = (elapsedMs % CYCLE_DURATION_MS) / CYCLE_DURATION_MS;
  if (phase < 0.5) {
    return { from: HOME, to: MOCK_DESTINATION, t: phase / 0.5 };
  }
  return { from: MOCK_DESTINATION, to: HOME, t: (phase - 0.5) / 0.5 };
}

export function getMockPosition(elapsedMs: number): DronePosition {
  const { from, to, t } = legProgress(elapsedMs);
  return {
    lat: lerp(from.lat, to.lat, t),
    lng: lerp(from.lng, to.lng, t),
    headingDeg: bearingDeg(from, to),
  };
}

export const mockHome = HOME;
export const mockDestination = MOCK_DESTINATION;
