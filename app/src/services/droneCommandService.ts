import { gatewayConnection } from './internal/gatewayConnection';

/**
 * Abstraction over the high-level commands the operator can issue.
 * returnToHome/newMission/cameraTiltUp/cameraTiltDown are real gateway
 * commands, sent over the shared `gatewayConnection`. followTarget/
 * ignoreTarget/engageTarget have no gateway equivalent (the protocol only
 * supports the four commands above) and remain mocked until the backend
 * grows support for them.
 */
export interface DroneCommandService {
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
    returnToHome: () => gatewayConnection.sendCommand('return_home'),
    newMission: () => gatewayConnection.sendCommand('new_mission'),
    cameraTiltUp: () => gatewayConnection.sendCommand('camera_tilt_up'),
    cameraTiltDown: () => gatewayConnection.sendCommand('camera_tilt_down'),
    followTarget: (targetId) => mockCommand(`FOLLOW target=${targetId}`),
    ignoreTarget: (targetId) => mockCommand(`IGNORE target=${targetId}`),
    engageTarget: (targetId) => mockCommand(`ENGAGE target=${targetId}`),
  };
}

export const droneCommandService: DroneCommandService = createDroneCommandService();
