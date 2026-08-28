import type { DronePosition } from '../types/drone';
import { gatewayConnection } from './internal/gatewayConnection';
import type { Unsubscribe } from './internal/pollingSource';

/**
 * Abstraction over "where is the drone right now". This is the only piece of
 * the app allowed to know about GPS. Sourced from the shared
 * `gatewayConnection`'s live telemetry stream — everything else consumes it
 * through `subscribe`.
 */
export interface LocationService {
  subscribe(listener: (position: DronePosition) => void): Unsubscribe;
}

function createGatewayLocationService(): LocationService {
  return {
    subscribe(listener) {
      return gatewayConnection.subscribeTelemetry((message) => {
        listener({
          lat: message.data.gps.lat,
          lng: message.data.gps.lng,
          headingDeg: message.data.headingDeg,
        });
      });
    },
  };
}

export const locationService: LocationService = createGatewayLocationService();
