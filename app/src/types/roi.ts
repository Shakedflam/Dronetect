/**
 * A geographic point using `lon` (not `lng`, unlike `Coordinates` elsewhere
 * in the app) to match the field naming future non-UI sources — other
 * machines/services setting the ROI over the wire — are expected to use.
 * See `services/roiService.ts` for the integration point.
 */
export interface RoiCenter {
  lat: number;
  lon: number;
}

/** A single, frontend-only region of interest: a circle on the map. */
export interface Roi {
  center: RoiCenter;
  radiusM: number;
}
