/**
 * Backend-owned mission state, distinct from the simulator controller's
 * lower-level telemetry state (for example IDLE or HOLDING).
 */
export type MissionState =
  | 'ON_GROUND'
  | 'TAKING_OFF'
  | 'PATROL'
  | 'TRANSIT_TO_ROI'
  | 'ROI_PATROL'
  | 'RTH'
  | 'LANDING';

export const DEFAULT_MISSION_STATE: MissionState = 'ON_GROUND';
