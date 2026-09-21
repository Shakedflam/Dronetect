import { useManualControl, type ManualAxis } from '../../hooks/useManualControl';
import type { ManualInputSource, ManualStick } from '../../types/manualControl';
import styles from './ManualControlPad.module.css';

interface ManualControlPadProps {
  engaged: boolean;
  inputSource: ManualInputSource;
}

interface StickButtonProps {
  axis: ManualAxis;
  sign: 1 | -1;
  label: string;
  title: string;
  stick: ManualStick;
  disabled: boolean;
  press(axis: ManualAxis, sign: 1 | -1): void;
  release(axis: ManualAxis): void;
  className?: string;
}

function StickButton({
  axis, sign, label, title, stick, disabled, press, release, className,
}: StickButtonProps) {
  const held = stick[axis] === sign;
  return (
    <button
      type="button"
      className={`${styles.stick} ${held ? styles.stickHeld : ''} ${className ?? ''}`}
      title={title}
      aria-label={title}
      aria-pressed={held}
      disabled={disabled}
      // Pointer events rather than click: the aircraft has to know the moment
      // the finger lifts, and pointercancel/leave cover a finger that slides
      // off the button mid-press without ever firing pointerup on it.
      onPointerDown={(event) => {
        event.preventDefault();
        // Touch captures the pointer implicitly, which would swallow the
        // pointerleave that tells us a finger has slid off the button.
        if (event.currentTarget.hasPointerCapture(event.pointerId)) {
          event.currentTarget.releasePointerCapture(event.pointerId);
        }
        press(axis, sign);
      }}
      onPointerUp={() => release(axis)}
      onPointerCancel={() => release(axis)}
      onPointerLeave={() => release(axis)}
    >
      {label}
    </button>
  );
}

/**
 * The operator's sticks while manual control is engaged.
 *
 * Deliberately momentary throughout: every button moves the aircraft only for
 * as long as it is held, and letting go of everything is a hover. Nothing here
 * latches, so there is no way to walk away from a moving aircraft.
 *
 * A bound remote controller outranks this pad. When the aircraft reports it is
 * flying RC sticks the pad stays visible but inert, so the tablet never looks
 * like it is in command of something it is not.
 */
export function ManualControlPad({ engaged, inputSource }: ManualControlPadProps) {
  const rcInControl = inputSource === 'rc';
  const { stick, press, release } = useManualControl(engaged && !rcInControl);

  if (!engaged) return null;

  const shared = { stick, press, release, disabled: rcInControl };

  return (
    <div className={styles.pad} role="group" aria-label="Manual flight controls">
      <div className={styles.heading}>MANUAL</div>
      {rcInControl && (
        <p className={styles.rcNotice}>Remote controller has the sticks</p>
      )}

      <div className={styles.dpad}>
        <span />
        <StickButton axis="forward" sign={1} label="▲" title="Forward" {...shared} />
        <span />
        <StickButton axis="right" sign={-1} label="◀" title="Left" {...shared} />
        <span className={styles.dpadCenter}>HOVER</span>
        <StickButton axis="right" sign={1} label="▶" title="Right" {...shared} />
        <span />
        <StickButton axis="forward" sign={-1} label="▼" title="Back" {...shared} />
        <span />
      </div>

      <div className={styles.label}>ALT</div>
      <div className={styles.row}>
        <StickButton axis="up" sign={1} label="▲" title="Climb" {...shared} />
        <StickButton axis="up" sign={-1} label="▼" title="Descend" {...shared} />
      </div>

      <div className={styles.label}>YAW</div>
      <div className={styles.row}>
        <StickButton axis="yawRate" sign={-1} label="↺" title="Yaw left" {...shared} />
        <StickButton axis="yawRate" sign={1} label="↻" title="Yaw right" {...shared} />
      </div>
    </div>
  );
}
