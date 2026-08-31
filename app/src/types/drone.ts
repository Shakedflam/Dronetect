/** Shared data contracts for the drone command-center UI. */

/** A geographic position. Altitude lives in TelemetryData, not here — this is a pure GPS fix. */
export interface Coordinates {
  lat: number;
  lng: number;
}

/** GPS-derived state of the drone, as reported by the LocationService. */
export interface DronePosition extends Coordinates {
  /** Compass heading in degrees, 0 = north. */
  headingDeg: number;
}

/** Flight instrument readout, as reported by the TelemetryService. */
export interface TelemetryData {
  speedMps: number;
  altitudeM: number;
  /** Battery charge, 0-100. */
  batteryPercent: number;
  /** Flight-state machine value from the realtime server, e.g. IDLE, FLYING_RANDOMLY, RETURNING_HOME. */
  state: string;
  /** Mock-only field with no realtime server equivalent; absent on live telemetry. */
  destinationName?: string;
  /** Mock-only field with no realtime server equivalent; absent on live telemetry. */
  etaSeconds?: number;
}

/**
 * Bounding box normalized to [0, 1] against the YOLO detection frame's own
 * dimensions (frameWidth/frameHeight from the wire message) — independent of
 * the WebRTC video element's intrinsic resolution, which can differ.
 */
export interface NormalizedBox {
  x1: number;
  y1: number;
  x2: number;
  y2: number;
}

/** A single detected object/target in the drone's camera frame. */
export interface DetectedTarget {
  /** The detector's trackId — stable across frames for the same tracked object. */
  id: string;
  label: string;
  /** Detection confidence in [0, 1]. */
  confidence: number;
  boundingBox: NormalizedBox;
  trackState: string;
  lastSeenTimestamp: number;
  velocityPxPerSec: { x: number; y: number };
}

/** Commands the operator can issue for a specific detected target. */
export type TargetCommand = 'follow' | 'ignore' | 'engage';
