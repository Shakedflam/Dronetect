import { useEffect, useRef, useState } from 'react';
import { droneCommandService } from '../../services/droneCommandService';
import { realtimeServerConnection } from '../../services/internal/realtimeServerConnection';
import { useMissionState } from '../../hooks/useMissionState';
import { useMissionStatus } from '../../hooks/useMissionStatus';
import { useRoi } from '../../hooks/useRoi';
import type { Roi } from '../../types/roi';
import styles from './ActionBar.module.css';

interface ActionBarProps {
  selectedTargetId: string | null;
  canReject: boolean;
  onClearSelection: () => void;
  onTargetIgnoreStart: (targetId: string) => void;
  onTargetIgnoreFailed: (targetId: string) => void;
}

function roiKey(roi: Roi | null): string | null {
  if (!roi) return null;
  return `${roi.center.lat}:${roi.center.lon}:${roi.radiusM}`;
}

/**
 * Permanent control bar for drone-wide and mission commands.
 * Mission overrides remain available during manual RTH, but not while a
 * safety-triggered RTH is active.
 * Patrol/ROI/RTH are backend commands, and mission state messages show
 * the authoritative transition in the map/HUD.
 * Reject uses a candidate-specific backend command. Selection survives box
 * disappearance briefly so the operator can still reject a flicker.
 * Pursuit itself is autonomous; the permanent HOLD PURSUIT toggle is the
 * only control that gates it, and its state comes from mission status.
 * MANUAL sits apart on the right edge because it is the one control that takes
 * the aircraft off the computer entirely; like the pursuit hold, whether it is
 * engaged is read back from the aircraft rather than from this button.
 */
export function ActionBar({
  selectedTargetId,
  canReject,
  onClearSelection,
  onTargetIgnoreStart,
  onTargetIgnoreFailed,
}: ActionBarProps) {
  const [targetCommandPending, setTargetCommandPending] = useState(false);
  const [targetCommandError, setTargetCommandError] = useState<string | null>(null);
  const [commandLinkOpen, setCommandLinkOpen] = useState(false);
  const [pursuitBlockPending, setPursuitBlockPending] = useState(false);
  const [manualControlPending, setManualControlPending] = useState(false);
  useEffect(() => realtimeServerConnection.subscribeStatus((status) => setCommandLinkOpen(status === 'open')), []);
  const hasSelection = selectedTargetId !== null;
  const missionState = useMissionState();
  const missionStatus = useMissionStatus();
  const droneConnected = missionStatus?.droneConnected ?? false;
  // Backend-owned: never derive the hold from local button state, so the
  // control always shows what the aircraft is actually doing.
  const pursuitBlocked = missionStatus?.pursuitBlocked ?? false;
  const safetyRthActive = missionStatus?.batteryRth.safetyRthActive ?? false;
  const manualControlActive = missionStatus?.manualControlActive ?? false;
  const manualInputSource = missionStatus?.manualInputSource ?? 'tablet';
  const missionOverrideDisabled = !droneConnected || safetyRthActive;
  const missionOverrideTitle = !droneConnected
    ? 'Drone is not connected'
    : safetyRthActive
      ? 'Safety RTH cannot be overridden'
      : undefined;
  const roi = useRoi();
  const roiMissionActive = missionState === 'TRANSIT_TO_ROI' || missionState === 'ROI_PATROL';
  const lastSentRoiRef = useRef<string | null>(null);

  useEffect(() => {
    if (!roi || !roiMissionActive || missionOverrideDisabled) return;
    const key = roiKey(roi);
    if (key === lastSentRoiRef.current) return;

    // Map dragging emits many intermediate ROI positions. Send the settled
    // location instead of flooding the aircraft command link on every pixel.
    const timer = window.setTimeout(() => {
      lastSentRoiRef.current = key;
      droneCommandService.setRoi(roi).catch((error: unknown) => {
        console.error('[ActionBar] automatic ROI update failed', error);
      });
    }, 200);
    return () => window.clearTimeout(timer);
  }, [roi, roiMissionActive, missionOverrideDisabled]);

  function handleReturnToHome() {
    droneCommandService.returnToHome().catch((error: unknown) => {
      console.error('[ActionBar] returnToHome failed', error);
    });
  }

  function handleCameraTiltUp() {
    droneCommandService.cameraTiltUp().catch((error: unknown) => {
      console.error('[ActionBar] cameraTiltUp failed', error);
    });
  }

  function handleCameraTiltDown() {
    droneCommandService.cameraTiltDown().catch((error: unknown) => {
      console.error('[ActionBar] cameraTiltDown failed', error);
    });
  }

  // Pursuit needs no approval: a confirmed track is followed automatically,
  // so the only target decision an operator makes is the veto.
  async function handleTargetReject() {
    if (!selectedTargetId || targetCommandPending) return;
    const targetId = selectedTargetId;
    setTargetCommandPending(true);
    setTargetCommandError(null);
    onTargetIgnoreStart(targetId);
    try {
      await droneCommandService.rejectTarget(targetId);
      // Acknowledgements mean command delivery; telemetry remains authoritative.
      onClearSelection();
    } catch (error) {
      onTargetIgnoreFailed(targetId);
      setTargetCommandError(error instanceof Error ? error.message : 'Target command failed');
    } finally {
      setTargetCommandPending(false);
    }
  }

  function handleManualControlToggle() {
    if (manualControlPending) return;
    setManualControlPending(true);
    droneCommandService
      // Send the desired value, never a local flip: what the aircraft reports
      // back is what the button shows.
      .setManualControl(!manualControlActive)
      .catch((error: unknown) => {
        console.error('[ActionBar] setManualControl failed', error);
      })
      .finally(() => setManualControlPending(false));
  }

  function handlePursuitBlockToggle() {
    if (pursuitBlockPending) return;
    setPursuitBlockPending(true);
    droneCommandService
      .setPursuitBlock(!pursuitBlocked)
      .catch((error: unknown) => {
        console.error('[ActionBar] setPursuitBlock failed', error);
      })
      .finally(() => setPursuitBlockPending(false));
  }

  function handleStartPatrol() {
    droneCommandService.startPatrol().catch((error: unknown) => {
      console.error('[ActionBar] startPatrol failed', error);
    });
  }

  function handleTransitToRoi() {
    if (!roi) return;
    lastSentRoiRef.current = roiKey(roi);
    droneCommandService.setRoi(roi).catch((error: unknown) => {
      console.error('[ActionBar] setRoi failed', error);
    });
  }

  return (
    <div className={styles.bar}>
      <div className={styles.primaryGroup}>
        <button
          type="button"
          className={`${styles.button} ${styles.rth}`}
          onClick={handleReturnToHome}
          disabled={!droneConnected}
          title={droneConnected ? undefined : 'Drone is not connected'}
        >
          RTH
        </button>
        <button
          type="button"
          className={`${styles.button} ${styles.tilt}`}
          onClick={handleCameraTiltUp}
        >
          TILT UP
        </button>
        <button
          type="button"
          className={`${styles.button} ${styles.tilt}`}
          onClick={handleCameraTiltDown}
        >
          TILT DOWN
        </button>
        <button
          type="button"
          className={`${styles.button} ${styles.patrol} ${missionState === 'PATROL' ? styles.patrolActive : ''}`}
          onClick={handleStartPatrol}
          disabled={missionOverrideDisabled}
          title={missionOverrideTitle}
          aria-pressed={missionState === 'PATROL'}
        >
          PATROL
        </button>
        <button
          type="button"
          className={`${styles.button} ${styles.roi} ${roiMissionActive ? styles.roiActive : ''}`}
          onClick={handleTransitToRoi}
          disabled={!roi || missionOverrideDisabled}
          title={missionOverrideTitle}
          aria-pressed={roiMissionActive}
        >
          ROI
        </button>
        <button
          type="button"
          className={`${styles.button} ${styles.pursuitHold} ${pursuitBlocked ? styles.pursuitHeld : ''}`}
          onClick={handlePursuitBlockToggle}
          disabled={!commandLinkOpen || pursuitBlockPending}
          aria-pressed={pursuitBlocked}
          title={
            pursuitBlocked
              ? 'Pursuit is held: targets are reported but not chased'
              : 'Pursuit is autonomous. Press to hold.'
          }
        >
          {pursuitBlocked ? 'PURSUIT HELD' : 'HOLD PURSUIT'}
        </button>
      </div>

      <div className={styles.contextualGroup}>
        {hasSelection && (
          <button type="button" className={`${styles.button} ${styles.ignore}`} onClick={() => void handleTargetReject()} disabled={missionOverrideDisabled || !commandLinkOpen || targetCommandPending || !canReject}>
            REJECT
          </button>
        )}
        <button
          type="button"
          className={`${styles.button} ${styles.manual} ${manualControlActive ? styles.manualActive : ''}`}
          onClick={handleManualControlToggle}
          disabled={
            !droneConnected ||
            !commandLinkOpen ||
            manualControlPending ||
            (safetyRthActive && !manualControlActive)
          }
          aria-pressed={manualControlActive}
          title={
            !droneConnected
              ? 'Drone is not connected'
              : safetyRthActive && !manualControlActive
                ? 'Safety RTH cannot be overridden'
                : manualControlActive
                  ? manualInputSource === 'rc'
                    ? 'Remote controller has the sticks. Press to give the aircraft back.'
                    : 'You are flying. Press to return the aircraft to the computer.'
                  : 'Press to fly the aircraft yourself.'
          }
        >
          {manualControlActive ? 'MANUAL ACTIVE' : 'MANUAL'}
        </button>
      </div>
      {targetCommandError && <span role="alert">{targetCommandError}</span>}
    </div>
  );
}
