import type { DronePosition } from '../types/drone';
import { getMockPosition } from '../mock/mockLocation';
import { createPollingSource, type Unsubscribe } from './internal/pollingSource';

/**
 * Abstraction over "where is the drone right now". This is the only piece of
 * the app allowed to know about GPS. When real GPS/telemetry hardware is
 * wired up, only this file's implementation changes — everything else
 * consumes it through `subscribe`.
 */
export interface LocationService {
  subscribe(listener: (position: DronePosition) => void): Unsubscribe;
}

const POLL_INTERVAL_MS = 1000;

function createMockLocationService(): LocationService {
  const source = createPollingSource(getMockPosition, POLL_INTERVAL_MS);
  return { subscribe: source.subscribe };
}

export const locationService: LocationService = createMockLocationService();
