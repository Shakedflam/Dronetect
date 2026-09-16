import type { Coordinates } from '../types/drone';
import { destinationPoint, euclideanDistanceMeters } from './geo';

/**
 * Computes the polygon (as lat/lng points) of the ROI circle's fill,
 * clipped to the home operational-range circle — the visible/allowed area
 * is only ever the intersection of the two circles. Returns an empty array
 * when the circles don't overlap at all (nothing to fill).
 *
 * Circle-circle intersection is done on a local flat (east/north-meters)
 * plane anchored at `home`, using the same small-scale approximation as
 * `euclideanDistanceMeters`/`destinationPoint` elsewhere in this file's
 * sibling `geo.ts` — accurate enough at the few-km scale a 2-3km ROI/home
 * radius implies, not meant for long distances.
 */
export function clipRoiToHomeRange(
  home: Coordinates,
  homeRadiusM: number,
  roi: { center: Coordinates; radiusM: number }
): Coordinates[] {
  const c1 = { x: 0, y: 0 };
  const r1 = homeRadiusM;
  const c2 = toLocalMeters(home, roi.center);
  const r2 = roi.radiusM;

  const d = Math.hypot(c2.x - c1.x, c2.y - c1.y);

  if (d >= r1 + r2) return []; // no overlap
  if (d <= r1 - r2) return circlePolygonPoints(home, c2, r2); // ROI fully inside home range
  if (d <= r2 - r1) return circlePolygonPoints(home, c1, r1); // home range fully inside ROI

  const intersection = circleIntersectionPoints(c1, r1, c2, r2);
  if (!intersection) return []; // defensive: shouldn't happen once the cases above are excluded
  const [p, q] = intersection;

  const arcOnHome = arcPointsInsideOther(c1, r1, p, q, c2, r2);
  const arcOnRoi = arcPointsInsideOther(c2, r2, q, p, c1, r1);
  const lensLocal = [...arcOnHome, ...arcOnRoi.slice(1)];

  return lensLocal.map((point) => fromLocalMeters(home, point));
}

interface LocalPoint {
  x: number;
  y: number;
}

const ARC_STEPS = 24;
const FULL_CIRCLE_STEPS = 48;

/** Projects `point` onto an east/north-meters plane anchored at `anchor`. */
function toLocalMeters(anchor: Coordinates, point: Coordinates): LocalPoint {
  const eastSign = point.lng >= anchor.lng ? 1 : -1;
  const northSign = point.lat >= anchor.lat ? 1 : -1;
  return {
    x: eastSign * euclideanDistanceMeters(anchor, { lat: anchor.lat, lng: point.lng }),
    y: northSign * euclideanDistanceMeters(anchor, { lat: point.lat, lng: anchor.lng }),
  };
}

/** Inverse of `toLocalMeters`. */
function fromLocalMeters(anchor: Coordinates, point: LocalPoint): Coordinates {
  const northOfAnchor = destinationPoint(anchor, point.y, 0);
  return destinationPoint(northOfAnchor, point.x, 90);
}

function circlePolygonPoints(anchor: Coordinates, center: LocalPoint, radius: number): Coordinates[] {
  const points: Coordinates[] = [];
  for (let i = 0; i <= FULL_CIRCLE_STEPS; i++) {
    const angle = (2 * Math.PI * i) / FULL_CIRCLE_STEPS;
    points.push(
      fromLocalMeters(anchor, {
        x: center.x + radius * Math.sin(angle),
        y: center.y + radius * Math.cos(angle),
      })
    );
  }
  return points;
}

/** The two points where two circles' boundaries cross, or null if they don't (partially) overlap. */
function circleIntersectionPoints(
  c1: LocalPoint,
  r1: number,
  c2: LocalPoint,
  r2: number
): [LocalPoint, LocalPoint] | null {
  const dx = c2.x - c1.x;
  const dy = c2.y - c1.y;
  const d = Math.hypot(dx, dy);
  if (d === 0 || d >= r1 + r2 || d <= Math.abs(r1 - r2)) return null;

  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
  const mx = c1.x + (a * dx) / d;
  const my = c1.y + (a * dy) / d;
  const perpX = -dy / d;
  const perpY = dx / d;

  return [
    { x: mx + h * perpX, y: my + h * perpY },
    { x: mx - h * perpX, y: my - h * perpY },
  ];
}

/**
 * Samples the arc of `(center, radius)` between `from` and `to` — of the two
 * possible arcs, whichever one lies inside `(otherCenter, otherRadius)`,
 * which is the half of this circle's boundary that bounds the lens/overlap
 * region. Starts at `from`, ends at `to`.
 */
function arcPointsInsideOther(
  center: LocalPoint,
  radius: number,
  from: LocalPoint,
  to: LocalPoint,
  otherCenter: LocalPoint,
  otherRadius: number
): LocalPoint[] {
  const angleFrom = Math.atan2(from.y - center.y, from.x - center.x);
  const angleTo = Math.atan2(to.y - center.y, to.x - center.x);

  function sweep(direction: 1 | -1): LocalPoint[] {
    const twoPi = 2 * Math.PI;
    let delta = angleTo - angleFrom;
    if (direction === 1) {
      delta = ((delta % twoPi) + twoPi) % twoPi;
    } else {
      delta = -(((-delta % twoPi) + twoPi) % twoPi);
    }
    const points: LocalPoint[] = [];
    for (let i = 0; i <= ARC_STEPS; i++) {
      const angle = angleFrom + (delta * i) / ARC_STEPS;
      points.push({
        x: center.x + radius * Math.cos(angle),
        y: center.y + radius * Math.sin(angle),
      });
    }
    return points;
  }

  const clockwise = sweep(1);
  const counterClockwise = sweep(-1);

  function isInsideOther(points: LocalPoint[]): boolean {
    const mid = points[Math.floor(points.length / 2)];
    return Math.hypot(mid.x - otherCenter.x, mid.y - otherCenter.y) <= otherRadius + 1e-6;
  }

  return isInsideOther(clockwise) ? clockwise : counterClockwise;
}
