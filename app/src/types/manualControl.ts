/**
 * Operator stick input, normalized to [-1, 1] in the aircraft's own body
 * frame: forward is wherever the nose and therefore the camera is pointing.
 * All zeros is the resting position and means hover.
 */
export interface ManualStick {
  forward: number;
  right: number;
  up: number;
  yawRate: number;
}

export const CENTERED_STICK: ManualStick = { forward: 0, right: 0, up: 0, yawRate: 0 };

/**
 * Which stick the aircraft is actually flying. A bound DJI remote controller
 * outranks the tablet pad, and the aircraft reports which one won.
 */
export type ManualInputSource = 'tablet' | 'rc';
