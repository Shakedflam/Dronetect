import { realtimeServerConnection } from './internal/realtimeServerConnection';
import type { Roi } from '../types/roi';
import type { OperatingConfig } from '../types/operatingConfig';
import type { ManualStick } from '../types/manualControl';

/**
 * Abstraction over the high-level commands the operator can issue.
 * returnToHome/cameraTiltUp/cameraTiltDown are real realtime
 * server commands, sent over the shared `realtimeServerConnection`.
 * Reject acts on a specific acquisition candidate. Pursuit is otherwise
 * autonomous; setPursuitBlock is the operator hold that gates it.
 * setManualControl hands the aircraft to the operator and back; sendManualStick
 * is the high-rate stick stream that only means anything while it is engaged.
 */
export interface DroneCommandService {
  setOperatingConfig(config: OperatingConfig): Promise<void>;
  startPatrol(): Promise<void>;
  setRoi(roi: Roi): Promise<void>;
  clearRoi(): Promise<void>;
  returnToHome(): Promise<void>;
  cameraTiltUp(): Promise<void>;
  cameraTiltDown(): Promise<void>;
  rejectTarget(targetId: string): Promise<void>;
  setPursuitBlock(blocked: boolean): Promise<void>;
  setManualControl(engaged: boolean): Promise<void>;
  sendManualStick(stick: ManualStick): void;
}

function createDroneCommandService(): DroneCommandService {
  return {
    setOperatingConfig: (config) =>
      realtimeServerConnection.sendCommand('set_operating_config', {
        maxFlightRadiusM: config.maxFlightRadiusM,
        cruiseAltitudeM: config.cruiseAltitudeM,
      }),
    startPatrol: () => realtimeServerConnection.sendCommand('start_patrol'),
    setRoi: (roi) =>
      realtimeServerConnection.sendCommand('set_roi', {
        center: roi.center,
        radiusM: roi.radiusM,
      }),
    clearRoi: () => realtimeServerConnection.sendCommand('clear_roi'),
    returnToHome: () => realtimeServerConnection.sendCommand('return_home'),
    cameraTiltUp: () => realtimeServerConnection.sendCommand('camera_tilt_up'),
    cameraTiltDown: () => realtimeServerConnection.sendCommand('camera_tilt_down'),
    rejectTarget: (trackId) => realtimeServerConnection.sendCommand('reject_target', { trackId }),
    setPursuitBlock: (blocked) => realtimeServerConnection.sendCommand('set_pursuit_block', { blocked }),
    setManualControl: (engaged) => realtimeServerConnection.sendCommand('set_manual_control', { engaged }),
    sendManualStick: (stick) =>
      realtimeServerConnection.sendUnacknowledged('set_manual_command', {
        forward: stick.forward,
        right: stick.right,
        up: stick.up,
        yawRate: stick.yawRate,
      }),
  };
}

export const droneCommandService: DroneCommandService = createDroneCommandService();
