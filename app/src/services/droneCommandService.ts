import { realtimeServerConnection } from './internal/realtimeServerConnection';
import type { Roi } from '../types/roi';

/**
 * Abstraction over the high-level commands the operator can issue.
 * returnToHome/newMission/cameraTiltUp/cameraTiltDown are real realtime
 * server commands, sent over the shared `realtimeServerConnection`.
 * Approve/Reject act on a specific acquisition candidate. Engage remains mocked.
 */
export interface DroneCommandService {
  startPatrol(): Promise<void>;
  setRoi(roi: Roi): Promise<void>;
  clearRoi(): Promise<void>;
  returnToHome(): Promise<void>;
  newMission(): Promise<void>;
  cameraTiltUp(): Promise<void>;
  cameraTiltDown(): Promise<void>;
  approveTarget(targetId: string): Promise<void>;
  rejectTarget(targetId: string): Promise<void>;
  engageTarget(targetId: string): Promise<void>;
}

const MOCK_LATENCY_MS = 200;

function mockCommand(label: string): Promise<void> {
  console.info(`[droneCommandService:mock] ${label}`);
  return new Promise((resolve) => setTimeout(resolve, MOCK_LATENCY_MS));
}

function createDroneCommandService(): DroneCommandService {
  return {
    startPatrol: () => realtimeServerConnection.sendCommand('start_patrol'),
    setRoi: (roi) =>
      realtimeServerConnection.sendCommand('set_roi', {
        center: roi.center,
        radiusM: roi.radiusM,
      }),
    clearRoi: () => realtimeServerConnection.sendCommand('clear_roi'),
    returnToHome: () => realtimeServerConnection.sendCommand('return_home'),
    newMission: () => realtimeServerConnection.sendCommand('new_mission'),
    cameraTiltUp: () => realtimeServerConnection.sendCommand('camera_tilt_up'),
    cameraTiltDown: () => realtimeServerConnection.sendCommand('camera_tilt_down'),
    approveTarget: (trackId) => realtimeServerConnection.sendCommand('approve_target', { trackId }),
    rejectTarget: (trackId) => realtimeServerConnection.sendCommand('reject_target', { trackId }),
    engageTarget: (targetId) => mockCommand(`ENGAGE target=${targetId}`),
  };
}

export const droneCommandService: DroneCommandService = createDroneCommandService();
