import type { Coordinates } from '../types/drone';
import type { PatrolAreaType, PatrolConfig, PatrolRegion } from '../types/patrol';
import type { Unsubscribe } from './internal/unsubscribe';
import { generatePatrolRegions } from '../utils/patrolGeometry';
import { HOME_RADIUS_METERS, HOME_REFERENCE } from '../config/mapConfig';
import { locationService } from './locationService';
import { realtimeServerConnection } from './internal/realtimeServerConnection';

// Regenerated atomically whenever either input changes: an accepted
// operating-config radius update (setMaxFlightRadius), or this session's
// home resolving to the backend-confirmed real-DJI one (see
// locationService.ts's subscribeHome) instead of the static HOME_REFERENCE
// fallback — otherwise polygons/region centers would stay anchored to
// HOME_REFERENCE even once the backend confirms a real session home.
let currentHome: Coordinates = HOME_REFERENCE;
let currentRadiusM: number = HOME_RADIUS_METERS;
let patrolRegions: readonly PatrolRegion[] = generatePatrolRegions(currentHome, currentRadiusM);

/**
 * Abstraction over "how are patrol regions currently prioritized".
 * Region geometry and immediate display state remain local, while each
 * complete priority ordering is sent to the backend patrol planner through
 * the realtime connection.
 */
export interface PatrolService {
  subscribe(listener: (config: PatrolConfig) => void): Unsubscribe;
  getConfig(): PatrolConfig;
  /** Assigns the next rank, restoring the region to PATROL first; clears its rank if it is already ranked. */
  setPriority(regionId: string): void;
  setAreaType(regionId: string, areaType: PatrolAreaType): void;
  clearPriorities(): void;
  setMaxFlightRadius(radiusM: number): void;
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
    const regions = patrolRegions.map((region) => {
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

  locationService.subscribeHome((home) => {
    currentHome = home;
    patrolRegions = generatePatrolRegions(currentHome, currentRadiusM);
    notify();
  });

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
      let restoredToPatrol = false;
      if (idx === -1) {
        if ((areaTypes[regionId] ?? 'PATROL') !== 'PATROL') {
          areaTypes = { ...areaTypes };
          delete areaTypes[regionId];
          restoredToPatrol = true;
        }
        priorityOrder = [...priorityOrder, regionId];
      } else {
        priorityOrder = priorityOrder.filter((id) => id !== regionId);
      }
      notify();
      // WebSocket messages are emitted in call order, so the backend restores
      // the region to PATROL before accepting the new priority ordering.
      if (restoredToPatrol) publishAreaTypes();
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
    setMaxFlightRadius(radiusM) {
      if (!Number.isFinite(radiusM) || radiusM <= 0) {
        throw new Error('Maximum flight radius must be a positive number');
      }
      currentRadiusM = radiusM;
      patrolRegions = generatePatrolRegions(currentHome, currentRadiusM);
      notify();
    },
  };
}

export const patrolService: PatrolService = createInMemoryPatrolService();
