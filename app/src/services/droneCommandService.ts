import { realtimeServerConnection } from './internal/realtimeServerConnection';
import type { Roi } from '../types/roi';

/**
 * Abstraction over the high-level commands the operator can issue.
 * returnToHome/newMission/cameraTiltUp/cameraTiltDown are real realtime
 * server commands, sent over the shared `realtimeServerConnection`.
 * followTarget/ignoreTarget/engageTarget have no realtime server equivalent
 * (the protocol only supports the four commands above) and remain mocked
 * until the backend grows support for them.
 */
export interface DroneCommandService {
  startPatrol(): Promise<void>;
  setRoi(roi: Roi): Promise<void>;
  clearRoi(): Promise<void>;
  returnToHome(): Promise<void>;
  newMission(): Promise<void>;
  cameraTiltUp(): Promise<void>;
  cameraTiltDown(): Promise<void>;
  followTarget(targetId: string): Promise<void>;
  ignoreTarget(targetId: string): Promise<void>;
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
    followTarget: (targetId) => mockCommand(`FOLLOW target=${targetId}`),
    ignoreTarget: (targetId) => mockCommand(`IGNORE target=${targetId}`),
    engageTarget: (targetId) => mockCommand(`ENGAGE target=${targetId}`),
  };
}

export const droneCommandService: DroneCommandService = createDroneCommandService();
