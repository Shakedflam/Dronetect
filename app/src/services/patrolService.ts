import type { PatrolAreaType, PatrolConfig, PatrolRegion } from '../types/patrol';
import type { Unsubscribe } from './internal/unsubscribe';
import { generatePatrolRegions } from '../utils/patrolGeometry';
import { HOME_RADIUS_METERS, HOME_REFERENCE } from '../config/mapConfig';
import { realtimeServerConnection } from './internal/realtimeServerConnection';

/** Static geometry, generated once from the Home reference/radius. Never mutated. */
const PATROL_REGIONS: readonly PatrolRegion[] = generatePatrolRegions(HOME_REFERENCE, HOME_RADIUS_METERS);

/**
 * Abstraction over "how are patrol regions currently prioritized".
 * Region geometry and immediate display state remain local, while each
 * complete priority ordering is sent to the backend patrol planner through
 * the realtime connection.
 */
export interface PatrolService {
  subscribe(listener: (config: PatrolConfig) => void): Unsubscribe;
  getConfig(): PatrolConfig;
  /** Assigns the next rank if `regionId` is unranked; clears its rank (and compacts the rest) if it's already ranked. */
  setPriority(regionId: string): void;
  setAreaType(regionId: string, areaType: PatrolAreaType): void;
  clearPriorities(): void;
}

function createInMemoryPatrolService(): PatrolService {
  const listeners = new Set<(config: PatrolConfig) => void>();
  // Ids in rank order — index 0 is rank 1. Rank is always derived as
  // indexOf + 1, so removing an id from the middle closes the gap for free
  // instead of needing a separate renumbering step.
  let priorityOrder: string[] = [];
  let authoritativePriorityOrder: string[] = [];
  let areaTypes: Record<string, PatrolAreaType> = {};
  let authoritativeAreaTypes: Record<string, PatrolAreaType> = {};

  function buildConfig(): PatrolConfig {
    const regions = PATROL_REGIONS.map((region) => {
      const idx = priorityOrder.indexOf(region.id);
      return {
        ...region,
        areaType: areaTypes[region.id] ?? 'PATROL',
        priorityRank: idx === -1 ? null : idx + 1,
      };
    });
    return { regions, priorities: [...priorityOrder], areaTypes: { ...areaTypes } };
  }

  function publishAreaTypes(): void {
    const overrides = Object.entries(areaTypes)
      .filter(([, areaType]) => areaType !== 'PATROL')
      .map(([regionId, areaType]) => ({ regionId, areaType }));
    realtimeServerConnection
      .sendCommand('set_patrol_area_types', { areaTypes: overrides })
      .catch((error: unknown) => {
        console.error('[patrolService] area type update failed', error);
        areaTypes = { ...authoritativeAreaTypes };
        notify();
      });
  }

  function notify(): void {
    const config = buildConfig();
    listeners.forEach((listener) => listener(config));
  }

  function publishPriorities(): void {
    const priorities = priorityOrder.map((regionId, index) => ({
      regionId,
      rank: index + 1,
    }));
    realtimeServerConnection
      .sendCommand('set_patrol_priorities', { priorities })
      .catch((error: unknown) => {
        console.error('[patrolService] priority update failed', error);
        priorityOrder = [...authoritativePriorityOrder];
        notify();
      });
  }

  realtimeServerConnection.subscribeMissionStatus((status) => {
    authoritativePriorityOrder = status.patrol.priorities
      .slice()
      .sort((left, right) => left.rank - right.rank)
      .map(({ regionId }) => regionId);
    priorityOrder = [...authoritativePriorityOrder];
    authoritativeAreaTypes = Object.fromEntries(
      (status.patrol.areaTypes ?? []).map(({ regionId, areaType }) => [
        regionId,
        areaType,
      ])
    );
    areaTypes = { ...authoritativeAreaTypes };
    notify();
  });

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
      publishPriorities();
    },
    setAreaType(regionId, areaType) {
      areaTypes = { ...areaTypes, [regionId]: areaType };
      if (areaType === 'PATROL') {
        delete areaTypes[regionId];
      } else {
        priorityOrder = priorityOrder.filter((id) => id !== regionId);
      }
      notify();
      publishAreaTypes();
      publishPriorities();
    },
    clearPriorities() {
      priorityOrder = [];
      notify();
      publishPriorities();
    },
  };
}

export const patrolService: PatrolService = createInMemoryPatrolService();
