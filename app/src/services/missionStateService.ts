import type { MissionState } from '../types/missionState';
import { DEFAULT_MISSION_STATE } from '../types/missionState';
import type { Unsubscribe } from './internal/unsubscribe';

/**
 * Frontend-only mission state, set directly by ActionBar button presses.
 * See types/missionState.ts for why this is not the same thing as
 * telemetry.state. No realtime_server wiring here, by design — this is a
 * plain instant local state write, not a flight command.
 */
export interface MissionStateService {
  subscribe(listener: (state: MissionState) => void): Unsubscribe;
  getMissionState(): MissionState;
  setMissionState(state: MissionState): void;
}

function createInMemoryMissionStateService(): MissionStateService {
  const listeners = new Set<(state: MissionState) => void>();
  let current: MissionState = DEFAULT_MISSION_STATE;

  return {
    subscribe(listener) {
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
    getMissionState() {
      return current;
    },
    setMissionState(state) {
      current = state;
      listeners.forEach((listener) => listener(current));
    },
  };
}

export const missionStateService: MissionStateService = createInMemoryMissionStateService();
