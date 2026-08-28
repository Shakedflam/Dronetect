import type { Coordinates } from '../types/drone';

const METERS_PER_DEG_LAT = 111320;

/** Compass bearing in degrees (0 = north, clockwise) from one point to another. */
export function bearingDeg(from: Coordinates, to: Coordinates): number {
  const dLng = ((to.lng - from.lng) * Math.PI) / 180;
  const lat1 = (from.lat * Math.PI) / 180;
  const lat2 = (to.lat * Math.PI) / 180;
  const y = Math.sin(dLng) * Math.cos(lat2);
  const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLng);
  const deg = (Math.atan2(y, x) * 180) / Math.PI;
  return (deg + 360) % 360;
}

/**
 * Flat-plane Euclidean distance in meters between two coordinates —
 * projects lat/lng onto a local tangent plane rather than following the
 * Earth's curvature. Ignores altitude entirely, unlike a true 3D distance.
 * Accurate enough at the few-kilometer scale this app operates at.
 */
export function euclideanDistanceMeters(from: Coordinates, to: Coordinates): number {
  const metersPerDegLng = METERS_PER_DEG_LAT * Math.cos((from.lat * Math.PI) / 180);
  const dx = (to.lng - from.lng) * metersPerDegLng;
  const dy = (to.lat - from.lat) * METERS_PER_DEG_LAT;
  return Math.sqrt(dx * dx + dy * dy);
}
