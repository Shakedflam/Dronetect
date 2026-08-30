import type { DronePosition } from '../types/drone';
import { realtimeServerConnection } from './internal/realtimeServerConnection';
import type { Unsubscribe } from './internal/unsubscribe';

/**
 * Abstraction over "where is the drone right now". This is the only piece of
 * the app allowed to know about GPS. Sourced from the shared
 * `realtimeServerConnection`'s live telemetry stream — everything else
 * consumes it through `subscribe`.
 */
export interface LocationService {
  subscribe(listener: (position: DronePosition) => void): Unsubscribe;
}

function createRealtimeServerLocationService(): LocationService {
  return {
    subscribe(listener) {
      return realtimeServerConnection.subscribeTelemetry((message) => {
        listener({
          lat: message.data.gps.lat,
          lng: message.data.gps.lng,
          headingDeg: message.data.headingDeg,
        });
      });
    },
  };
}

export const locationService: LocationService = createRealtimeServerLocationService();
