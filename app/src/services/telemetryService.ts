import { Capacitor, registerPlugin, type PluginListenerHandle } from '@capacitor/core';
import type { DjiTelemetryData, TelemetryData } from '../types/drone';
import { realtimeServerConnection } from './internal/realtimeServerConnection';
import type { Unsubscribe } from './internal/unsubscribe';

/**
 * Abstraction over the flight-instrument readout (speed, altitude, battery,
 * state). Deliberately separate from LocationService — the HUD combines
 * both, but they're conceptually distinct even though the realtime server
 * currently happens to send them on the same telemetry message.
 *
 * The realtime server's telemetry payload has no destination/ETA concept, so
 * those fields are simply never populated here — see `TelemetryData`.
 */
export interface TelemetryService {
  subscribe(listener: (telemetry: TelemetryData) => void): Unsubscribe;
  subscribeDji(listener: (telemetry: DjiTelemetryData) => void): Unsubscribe;
}

interface DjiTelemetryPlugin {
  getSnapshot(): Promise<DjiTelemetryData>;
  addListener(eventName: 'snapshot', listener: (snapshot: DjiTelemetryData) => void): Promise<PluginListenerHandle>;
}

const djiTelemetryPlugin = registerPlugin<DjiTelemetryPlugin>('DjiTelemetry');

function createRealtimeServerTelemetryService(): TelemetryService {
  return {
    subscribe(listener) {
      return realtimeServerConnection.subscribeTelemetry((message) => {
        listener({
          source: message.source,
          speedMps: message.data.speedMps,
          altitudeM: message.data.altitudeM,
          batteryPercent: message.data.batteryPercent,
          state: message.data.state,
          targetFollowDiagnostics: message.data.targetFollowDiagnostics ?? null,
        });
      });
    },
    subscribeDji(listener) {
      if (Capacitor.getPlatform() !== 'android') return () => {};

      let disposed = false;
      let latestTimestampMs = -1;
      let receivedEvent = false;
      const deliver = (snapshot: DjiTelemetryData) => {
        if (disposed || snapshot.timestampMs < latestTimestampMs) return;
        latestTimestampMs = snapshot.timestampMs;
        listener(snapshot);
      };
      const handle = djiTelemetryPlugin.addListener('snapshot', (snapshot) => {
        receivedEvent = true;
        deliver(snapshot);
      });
      void handle.then(() => djiTelemetryPlugin.getSnapshot())
        .then((snapshot) => {
          if (!receivedEvent) deliver(snapshot);
        })
        .catch((error: unknown) => console.warn('DJI telemetry bridge unavailable', error));

      return () => {
        disposed = true;
        void handle.then((registered) => registered.remove());
      };
    },
  };
}

export const telemetryService: TelemetryService = createRealtimeServerTelemetryService();
