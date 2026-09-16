import {
  realtimeServerConnection,
  type MissionStatusMessage,
} from './internal/realtimeServerConnection';
import type { Unsubscribe } from './internal/unsubscribe';

/** Read-only UI access to the complete backend-owned mission snapshot. */
export interface MissionStatusService {
  subscribe(listener: (status: MissionStatusMessage) => void): Unsubscribe;
  getStatus(): MissionStatusMessage | null;
}

const listeners = new Set<(status: MissionStatusMessage) => void>();
let current: MissionStatusMessage | null = null;

export const missionStatusService: MissionStatusService = {
  subscribe(listener) {
    listeners.add(listener);
    if (current) listener(current);
    return () => listeners.delete(listener);
  },
  getStatus() {
    return current;
  },
};

realtimeServerConnection.subscribeMissionStatus((status) => {
  current = status;
  listeners.forEach((listener) => listener(status));
});
