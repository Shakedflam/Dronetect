import { HOME_REFERENCE } from '../config/mapConfig';
import type { Coordinates, DjiTelemetryData, DronePosition } from '../types/drone';
import { realtimeServerConnection } from './internal/realtimeServerConnection';
import { telemetryService } from './telemetryService';
import type { Unsubscribe } from './internal/unsubscribe';

/**
 * Abstraction over "where is the drone right now" and "where is home this
 * session". This is the only piece of the app allowed to know about GPS.
 * Both merge two sources, preferring the local DJI fix over the
 * cloud-relayed one whenever DJI has one — see `subscribe`/`subscribeHome`
 * below.
 */
export interface LocationService {
  subscribe(listener: (position: DronePosition) => void): Unsubscribe;
  /**
   * This session's home. Once the backend confirms a real-DJI session home
   * (realtime_server's mission_status.sessionHome, captured server-side from
   * the first grounded DJI telemetry sample — see realtime_server/server.py)
   * that value wins, permanently, for the rest of this page session: it is
   * the same home the backend uses for ALL real-flight geometry (patrol
   * polygons, ROI/radius validation, no-fly routing), so this is the only
   * source that can make the map agree with it exactly. Until then, this
   * falls back to a local, unconfirmed guess captured from the first
   * complete on-tablet DJI fix (good enough for basic marker/recenter
   * display), or the static HOME_REFERENCE off Android / without DJI.
   */
  subscribeHome(listener: (home: Coordinates) => void): Unsubscribe;
  /**
   * True once it is safe to show geometry that depends on this session's
   * home: always true for simulator/browser mode (which never reports a
   * real-DJI session), false from the moment a real DJI telemetry source is
   * first seen until the backend confirms sessionHome. Patrol/ROI editing
   * must gate on this — see MapView.tsx — rather than show polygons/ROI
   * geometry anchored to a home that is about to move.
   */
  subscribeGeometryReady(listener: (ready: boolean) => void): Unsubscribe;
}

/** A complete DJI fix as a DronePosition, or null if any field is unavailable. */
function djiPositionFrom(snapshot: DjiTelemetryData): DronePosition | null {
  if (
    snapshot.connectionState !== 'connected' ||
    snapshot.latitude === null ||
    snapshot.longitude === null ||
    snapshot.headingDeg === null
  ) {
    return null;
  }
  return { lat: snapshot.latitude, lng: snapshot.longitude, headingDeg: snapshot.headingDeg };
}

// Module-level, not per-subscription: every capture below has to happen
// exactly once, regardless of which or how many components have called
// subscribe*() so far. A per-subscription capture could instead seed from a
// late-subscriber fallback (the *current* value, not the original one) if a
// component mounted after the first fix/confirmation already fired.
let localGuessHome: Coordinates | null = null;
let confirmedHome: Coordinates | null = null;
let djiSessionDetected = false;
const homeListeners = new Set<(home: Coordinates) => void>();
const geometryReadyListeners = new Set<(ready: boolean) => void>();

function currentHome(): Coordinates {
  return confirmedHome ?? localGuessHome ?? HOME_REFERENCE;
}

function currentGeometryReady(): boolean {
  return !djiSessionDetected || confirmedHome !== null;
}

function publishHome(): void {
  const home = currentHome();
  homeListeners.forEach((listener) => listener(home));
}

function publishGeometryReady(): void {
  const ready = currentGeometryReady();
  geometryReadyListeners.forEach((listener) => listener(ready));
}

// Immediate, unconfirmed local guess — kept only as a display fallback until
// the backend confirms the real one (see the realtime_server subscription
// below), which is why it never overwrites an already-confirmed home.
telemetryService.subscribeDji((snapshot) => {
  if (localGuessHome !== null || confirmedHome !== null) return;
  const position = djiPositionFrom(snapshot);
  if (!position) return;
  localGuessHome = { lat: position.lat, lng: position.lng };
  publishHome();
});

// Backend-confirmed, authoritative home — the same one realtime_server uses
// for patrol/ROI/radius geometry (see realtime_server/server.py's
// _session_home). Sourced from mission_status, not telemetry, since that is
// where the backend reports it.
realtimeServerConnection.subscribeTelemetry((message) => {
  if (message.source !== 'dji' || djiSessionDetected) return;
  djiSessionDetected = true;
  publishGeometryReady();
});

realtimeServerConnection.subscribeMissionStatus((status) => {
  if (confirmedHome !== null || status.sessionHome === null) return;
  confirmedHome = { lat: status.sessionHome.lat, lng: status.sessionHome.lng };
  publishHome();
  publishGeometryReady();
});

function createRealtimeServerLocationService(): LocationService {
  return {
    subscribe(listener) {
      let cloudPosition: DronePosition | null = null;
      let djiPosition: DronePosition | null = null;

      function emit(): void {
        const position = djiPosition ?? cloudPosition;
        if (position) listener(position);
      }

      const unsubscribeCloud = realtimeServerConnection.subscribeTelemetry((message) => {
        cloudPosition = {
          lat: message.data.gps.lat,
          lng: message.data.gps.lng,
          headingDeg: message.data.headingDeg,
        };
        emit();
      });

      // Real DJI GPS, read locally on the tablet, takes precedence over the
      // cloud-relayed position whenever it's a complete fix — the cloud
      // round-trip is unnecessary latency once the phone already knows where
      // the aircraft is. Falls back to the cloud/simulator position whenever
      // DJI has no current fix, so browser dev and simulator testing are
      // unaffected: `subscribeDji` is a no-op off Android.
      const unsubscribeDji = telemetryService.subscribeDji((snapshot) => {
        djiPosition = djiPositionFrom(snapshot);
        emit();
      });

      return () => {
        unsubscribeCloud();
        unsubscribeDji();
      };
    },
    subscribeHome(listener) {
      listener(currentHome());
      homeListeners.add(listener);
      return () => homeListeners.delete(listener);
    },
    subscribeGeometryReady(listener) {
      listener(currentGeometryReady());
      geometryReadyListeners.add(listener);
      return () => geometryReadyListeners.delete(listener);
    },
  };
}

export const locationService: LocationService = createRealtimeServerLocationService();
