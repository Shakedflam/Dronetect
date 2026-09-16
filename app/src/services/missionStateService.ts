import type { MissionState } from '../types/missionState';
import { DEFAULT_MISSION_STATE } from '../types/missionState';
import { missionStatusService } from './missionStatusService';
import type { Unsubscribe } from './internal/unsubscribe';

/** Read-only UI view of the backend-owned mission state. */
export interface MissionStateService {
  subscribe(listener: (state: MissionState) => void): Unsubscribe;
  getMissionState(): MissionState;
}

const listeners = new Set<(state: MissionState) => void>();
let current = DEFAULT_MISSION_STATE;

export const missionStateService: MissionStateService = {
  subscribe(listener) {
    listeners.add(listener);
    return () => listeners.delete(listener);
  },
  getMissionState() {
    return current;
  },
};

missionStatusService.subscribe(({ state }) => {
  current = state;
  listeners.forEach((listener) => listener(state));
});
