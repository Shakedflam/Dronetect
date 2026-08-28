import type { Coordinates } from '../types/drone';

/**
 * Static reference point shown on the map as "HOME". This is not live
 * telemetry — the gateway's telemetry stream never reports a home
 * location — it's a fixed display anchor matching the operational home
 * point the backend uses internally for local -> WGS84 conversion
 * (gateway's DEFAULT_HOME, ~32.0853, 34.7818). If the drone's actual
 * operational home ever becomes configurable or reportable by the
 * gateway, source it from there instead of this constant.
 */
export const HOME_REFERENCE: Coordinates = { lat: 32.0853, lng: 34.7818 };
