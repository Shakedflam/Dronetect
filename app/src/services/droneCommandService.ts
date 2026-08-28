/**
 * Abstraction over the high-level commands the operator can issue. No real
 * drone link exists yet, so the mock implementation just logs and resolves
 * — but every call site in the UI is already written against this
 * interface, so wiring in a real backend (REST/WebSocket) later is a
 * one-file change.
 */
export interface DroneCommandService {
  returnToHome(): Promise<void>;
  followTarget(targetId: string): Promise<void>;
  ignoreTarget(targetId: string): Promise<void>;
  engageTarget(targetId: string): Promise<void>;
}

const MOCK_LATENCY_MS = 200;

function mockCommand(label: string): Promise<void> {
  console.info(`[droneCommandService:mock] ${label}`);
  return new Promise((resolve) => setTimeout(resolve, MOCK_LATENCY_MS));
}

function createMockDroneCommandService(): DroneCommandService {
  return {
    returnToHome: () => mockCommand('RETURN_TO_HOME'),
    followTarget: (targetId) => mockCommand(`FOLLOW target=${targetId}`),
    ignoreTarget: (targetId) => mockCommand(`IGNORE target=${targetId}`),
    engageTarget: (targetId) => mockCommand(`ENGAGE target=${targetId}`),
  };
}

export const droneCommandService: DroneCommandService = createMockDroneCommandService();
