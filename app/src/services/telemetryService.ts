import type { TelemetryData } from '../types/drone';
import { gatewayConnection } from './internal/gatewayConnection';
import type { Unsubscribe } from './internal/pollingSource';

/**
 * Abstraction over the flight-instrument readout (speed, altitude, battery,
 * state). Deliberately separate from LocationService — the HUD combines
 * both, but they're conceptually distinct even though the gateway currently
 * happens to send them on the same telemetry message.
 *
 * The gateway's telemetry payload has no destination/ETA concept, so those
 * fields are simply never populated here — see `TelemetryData`.
 */
export interface TelemetryService {
  subscribe(listener: (telemetry: TelemetryData) => void): Unsubscribe;
}

function createGatewayTelemetryService(): TelemetryService {
  return {
    subscribe(listener) {
      return gatewayConnection.subscribeTelemetry((message) => {
        listener({
          speedMps: message.data.speedMps,
          altitudeM: message.data.altitudeM,
          batteryPercent: message.data.batteryPercent,
          state: message.data.state,
        });
      });
    },
  };
}

export const telemetryService: TelemetryService = createGatewayTelemetryService();
