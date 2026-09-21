import { useCallback, useEffect, useRef, useState } from 'react';
import { droneCommandService } from '../services/droneCommandService';
import { CENTERED_STICK, type ManualStick } from '../types/manualControl';

/** Axes the pad can drive, each pressed in one direction at a time. */
export type ManualAxis = keyof ManualStick;

/**
 * The aircraft treats silence as a released stick, so a held button has to keep
 * saying so. This also doubles as the link heartbeat: stop sending and the
 * aircraft falls back to a hover within its own command timeout.
 */
const STICK_RESEND_MS = 100;

interface ManualControl {
  stick: ManualStick;
  press(axis: ManualAxis, sign: 1 | -1): void;
  release(axis: ManualAxis): void;
}

/**
 * Owns the live stick and the resend loop while manual control is engaged.
 *
 * Releasing every axis sends one explicit centered stick rather than simply
 * going quiet: a hover the operator asked for should not have to wait out a
 * timeout to happen.
 */
export function useManualControl(engaged: boolean): ManualControl {
  const [stick, setStick] = useState<ManualStick>(CENTERED_STICK);
  const stickRef = useRef<ManualStick>(CENTERED_STICK);

  const apply = useCallback((next: ManualStick) => {
    stickRef.current = next;
    setStick(next);
  }, []);

  const press = useCallback(
    (axis: ManualAxis, sign: 1 | -1) => apply({ ...stickRef.current, [axis]: sign }),
    [apply],
  );

  const release = useCallback(
    (axis: ManualAxis) => apply({ ...stickRef.current, [axis]: 0 }),
    [apply],
  );

  useEffect(() => {
    if (!engaged) return;

    droneCommandService.sendManualStick(CENTERED_STICK);
    const timer = window.setInterval(() => {
      droneCommandService.sendManualStick(stickRef.current);
    }, STICK_RESEND_MS);

    // A finger that slid off the pad, a switched tab or a lost window focus
    // all mean the same thing: nobody is holding this stick any more.
    const center = () => {
      apply(CENTERED_STICK);
      droneCommandService.sendManualStick(CENTERED_STICK);
    };
    window.addEventListener('blur', center);
    window.addEventListener('visibilitychange', center);

    return () => {
      window.clearInterval(timer);
      window.removeEventListener('blur', center);
      window.removeEventListener('visibilitychange', center);
      droneCommandService.sendManualStick(CENTERED_STICK);
      // Whatever was held when control ended must not still look held if the
      // operator takes over again.
      apply(CENTERED_STICK);
    };
  }, [engaged, apply]);

  return { stick, press, release };
}
