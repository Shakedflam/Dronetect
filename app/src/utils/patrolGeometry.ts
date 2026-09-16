import type { Coordinates } from '../types/drone';
import type { PatrolDirection, PatrolRegion } from '../types/patrol';
import { PATROL_BAND_COUNT } from '../config/mapConfig';
import { destinationPoint } from './geo';

/**
 * Same 8 compass points/order as CompassStrip.tsx's CARDINALS, redeclared
 * here because that array is component-local and serves an unrelated
 * heading-tape concern — not imported from there.
 */
export const PATROL_DIRECTIONS: readonly { deg: number; label: PatrolDirection }[] = [
  { deg: 0, label: 'N' },
  { deg: 45, label: 'NE' },
  { deg: 90, label: 'E' },
  { deg: 135, label: 'SE' },
  { deg: 180, label: 'S' },
  { deg: 225, label: 'SW' },
  { deg: 270, label: 'W' },
  { deg: 315, label: 'NW' },
];

/** Sector half-width, degrees: 360 / 8 directions / 2. */
const SECTOR_HALF_WIDTH_DEG = 22.5;

/**
 * Equal-area band outer radii: outerRadius(k) = homeRadiusM * sqrt(k / N)
 * for k = 1..N makes each of the N concentric rings cover exactly 1/N of
 * the Home circle's area (ring area is proportional to outerRadius^2 -
 * innerRadius^2), so every one of the resulting direction x band cells
 * covers roughly the same ground area regardless of how close to Home it
 * sits.
 */
export function patrolBandOuterRadiiMeters(homeRadiusM: number): number[] {
  return Array.from({ length: PATROL_BAND_COUNT }, (_, i) => homeRadiusM * Math.sqrt((i + 1) / PATROL_BAND_COUNT));
}

/** The 8 sector-boundary bearings shared by every band, for drawing radial dividers. */
export function patrolDividerBearingsDeg(): number[] {
  return PATROL_DIRECTIONS.map(({ deg }) => (deg + SECTOR_HALF_WIDTH_DEG) % 360);
}

/** Generates the static 8-direction x N-band patrol regions. Pure/deterministic — call once. */
export function generatePatrolRegions(home: Coordinates, homeRadiusM: number): PatrolRegion[] {
  const outerRadii = patrolBandOuterRadiiMeters(homeRadiusM);
  const regions: PatrolRegion[] = [];

  for (const { deg, label } of PATROL_DIRECTIONS) {
    for (let bandIndex = 1; bandIndex <= PATROL_BAND_COUNT; bandIndex++) {
      const innerRadiusM = bandIndex === 1 ? 0 : outerRadii[bandIndex - 2];
      const outerRadiusM = outerRadii[bandIndex - 1];
      const midRadiusM = (innerRadiusM + outerRadiusM) / 2;
      const centerCoords = destinationPoint(home, midRadiusM, deg);

      regions.push({
        id: `${label}-${bandIndex}`,
        direction: label,
        directionDeg: deg,
        bandIndex,
        innerRadiusM,
        outerRadiusM,
        startBearingDeg: (deg - SECTOR_HALF_WIDTH_DEG + 360) % 360,
        endBearingDeg: (deg + SECTOR_HALF_WIDTH_DEG) % 360,
        center: { lat: centerCoords.lat, lon: centerCoords.lng },
        priorityRank: null,
      });
    }
  }

  return regions;
}

const SECTOR_ARC_STEPS = 8;

/** Points sweeping clockwise from `startBearingDeg` to `endBearingDeg` (the short way) at `radiusM`. */
function sectorArcPoints(
  home: Coordinates,
  radiusM: number,
  startBearingDeg: number,
  endBearingDeg: number
): Coordinates[] {
  const sweepDeg = ((endBearingDeg - startBearingDeg) % 360 + 360) % 360;
  const points: Coordinates[] = [];
  for (let i = 0; i <= SECTOR_ARC_STEPS; i++) {
    const bearing = startBearingDeg + (sweepDeg * i) / SECTOR_ARC_STEPS;
    points.push(destinationPoint(home, radiusM, bearing));
  }
  return points;
}

/**
 * Polygon points for one region's pie-slice shape, bounded by two radii and
 * two bearings — no circle-circle intersection needed, unlike
 * roiClipping.ts's ROI/Home overlap. Band 1 regions have innerRadiusM = 0,
 * which naturally collapses the inner arc to the home point for any
 * bearing, producing a valid pie slice with no special-casing.
 */
export function patrolRegionPolygonPoints(home: Coordinates, region: PatrolRegion): Coordinates[] {
  const outerArc = sectorArcPoints(home, region.outerRadiusM, region.startBearingDeg, region.endBearingDeg);
  // Reverse the resulting ARRAY (not the start/end args) to walk end->start:
  // recomputing with start/end swapped would make sweepDeg wrap the *long*
  // way (315 degrees) instead of retracing the same 45-degree arc backward.
  const innerArc = sectorArcPoints(home, region.innerRadiusM, region.startBearingDeg, region.endBearingDeg).reverse();
  return [...outerArc, ...innerArc];
}
