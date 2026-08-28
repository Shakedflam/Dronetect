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
  destinationName: string;
  etaSeconds: number;
}

/** Normalized bounding box, values in [0, 1] relative to the video frame. */
export interface NormalizedBoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

/** A single detected object/target in the drone's camera frame. */
export interface DetectedTarget {
  id: string;
  label: string;
  /** Detection confidence in [0, 1]. */
  confidence: number;
  boundingBox: NormalizedBoundingBox;
}

/** Commands the operator can issue for a specific detected target. */
export type TargetCommand = 'follow' | 'ignore' | 'engage';
