import type { RoiCenter } from './roi';

/** 8-point compass direction a patrol region belongs to. */
export type PatrolDirection = 'N' | 'NE' | 'E' | 'SE' | 'S' | 'SW' | 'W' | 'NW';

/**
 * One of the static cells the Home operational circle is divided into (8
 * directions x 4 equal-area distance bands — see utils/patrolGeometry.ts).
 * Geometry fields are fixed at generation time and never change;
 * `priorityRank` is derived state merged in by patrolService from the
 * current priority order, not part of the region's own persisted geometry.
 */
export interface PatrolRegion {
  /** Stable id: `${direction}-${bandIndex}`, e.g. "N-1", "NW-4". */
  id: string;
  direction: PatrolDirection;
  /** Center bearing of this direction's wedge, degrees (0 = N, clockwise). */
  directionDeg: number;
  /** 1 = innermost band (closest to Home), 4 = outermost. */
  bandIndex: number;
  innerRadiusM: number;
  outerRadiusM: number;
  /** Sector bearing bounds, degrees (may wrap past 360, e.g. N's start is 337.5). */
  startBearingDeg: number;
  endBearingDeg: number;
  /** Representative point for markers / future backend consumption. */
  center: RoiCenter;
  /** 1-based revisit-priority rank, or null if unranked. Several regions may share "unranked". */
  priorityRank: number | null;
}

/**
 * Backend-ready snapshot of the whole patrol system: static region geometry
 * plus the operator-assigned priority order. The single object a future
 * backend integration would send/receive as one payload — see
 * services/patrolService.ts.
 */
export interface PatrolConfig {
  regions: PatrolRegion[];
  /** Region ids in rank order — priorities[i] has rank i+1. */
  priorities: string[];
}
