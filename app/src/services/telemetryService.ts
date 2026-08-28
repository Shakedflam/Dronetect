import type { TelemetryData } from '../types/drone';
import { getMockTelemetry } from '../mock/mockTelemetry';
import { createPollingSource, type Unsubscribe } from './internal/pollingSource';

/**
 * Abstraction over the flight-instrument readout (speed, altitude,
 * destination, ETA). Deliberately separate from LocationService — the HUD
 * combines both, but a real backend may well expose them on different
 * channels (e.g. telemetry over MQTT, GPS over its own link).
 */
export interface TelemetryService {
  subscribe(listener: (telemetry: TelemetryData) => void): Unsubscribe;
}

const POLL_INTERVAL_MS = 1000;

function createMockTelemetryService(): TelemetryService {
  const source = createPollingSource(getMockTelemetry, POLL_INTERVAL_MS);
  return { subscribe: source.subscribe };
}

export const telemetryService: TelemetryService = createMockTelemetryService();
