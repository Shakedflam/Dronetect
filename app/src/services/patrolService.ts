import type { PatrolConfig, PatrolRegion } from '../types/patrol';
import type { Unsubscribe } from './internal/unsubscribe';
import { generatePatrolRegions } from '../utils/patrolGeometry';
import { HOME_RADIUS_METERS, HOME_REFERENCE } from '../config/mapConfig';

/** Static geometry, generated once from the Home reference/radius. Never mutated. */
const PATROL_REGIONS: readonly PatrolRegion[] = generatePatrolRegions(HOME_REFERENCE, HOME_RADIUS_METERS);

/**
 * Abstraction over "how are patrol regions currently prioritized".
 * Frontend-only, same rationale as roiService.ts: a future backend source
 * would call setPriority/clearPriorities the same way the map UI does, and
 * every subscriber picks it up identically. Region geometry is static (see
 * utils/patrolGeometry.ts); the only mutable state here is the priority
 * order.
 */
export interface PatrolService {
  subscribe(listener: (config: PatrolConfig) => void): Unsubscribe;
  getConfig(): PatrolConfig;
  /** Assigns the next rank if `regionId` is unranked; clears its rank (and compacts the rest) if it's already ranked. */
  setPriority(regionId: string): void;
  clearPriorities(): void;
}

function createInMemoryPatrolService(): PatrolService {
  const listeners = new Set<(config: PatrolConfig) => void>();
  // Ids in rank order — index 0 is rank 1. Rank is always derived as
  // indexOf + 1, so removing an id from the middle closes the gap for free
  // instead of needing a separate renumbering step.
  let priorityOrder: string[] = [];

  function buildConfig(): PatrolConfig {
    const regions = PATROL_REGIONS.map((region) => {
      const idx = priorityOrder.indexOf(region.id);
      return { ...region, priorityRank: idx === -1 ? null : idx + 1 };
    });
    return { regions, priorities: [...priorityOrder] };
  }

  function notify(): void {
    const config = buildConfig();
    listeners.forEach((listener) => listener(config));
  }

  return {
    subscribe(listener) {
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
    getConfig: buildConfig,
    setPriority(regionId) {
      const idx = priorityOrder.indexOf(regionId);
      if (idx === -1) {
        priorityOrder = [...priorityOrder, regionId];
      } else {
        priorityOrder = priorityOrder.filter((id) => id !== regionId);
      }
      notify();
    },
    clearPriorities() {
      priorityOrder = [];
      notify();
    },
  };
}

export const patrolService: PatrolService = createInMemoryPatrolService();
