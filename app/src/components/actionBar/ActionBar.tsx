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
  canApprove: boolean;
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
 * Approve/Reject use candidate-specific backend commands. Selection survives
 * box disappearance briefly so the operator can still reject a flicker.
 */
export function ActionBar({
  selectedTargetId,
  canApprove,
  canReject,
  onClearSelection,
  onTargetIgnoreStart,
  onTargetIgnoreFailed,
}: ActionBarProps) {
  const [targetCommandPending, setTargetCommandPending] = useState(false);
  const [targetCommandError, setTargetCommandError] = useState<string | null>(null);
  const [commandLinkOpen, setCommandLinkOpen] = useState(false);
  useEffect(() => realtimeServerConnection.subscribeStatus((status) => setCommandLinkOpen(status === 'open')), []);
  const hasSelection = selectedTargetId !== null;
  const missionState = useMissionState();
  const missionStatus = useMissionStatus();
  const droneConnected = missionStatus?.droneConnected ?? false;
  const safetyRthActive = missionStatus?.batteryRth.safetyRthActive ?? false;
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

  function handleNewMission() {
    droneCommandService.newMission().catch((error: unknown) => {
      console.error('[ActionBar] newMission failed', error);
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

  async function handleTargetDecision(approve: boolean) {
    if (!selectedTargetId || targetCommandPending) return;
    const targetId = selectedTargetId;
    setTargetCommandPending(true);
    setTargetCommandError(null);
    if (!approve) onTargetIgnoreStart(targetId);
    try {
      if (approve) await droneCommandService.approveTarget(targetId);
      else await droneCommandService.rejectTarget(targetId);
      // Acknowledgements mean command delivery; telemetry remains authoritative.
      if (!approve) onClearSelection();
    } catch (error) {
      if (!approve) onTargetIgnoreFailed(targetId);
      setTargetCommandError(error instanceof Error ? error.message : 'Target command failed');
    } finally {
      setTargetCommandPending(false);
    }
  }

  function handleEngage() {
    if (!selectedTargetId) return;
    void droneCommandService.engageTarget(selectedTargetId);
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
          className={`${styles.button} ${styles.mission}`}
          onClick={handleNewMission}
          disabled={missionOverrideDisabled}
          title={missionOverrideTitle}
        >
          NEW MISSION
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
      </div>

      {hasSelection && (
        <div className={styles.contextualGroup}>
          <button type="button" className={`${styles.button} ${styles.follow}`} onClick={() => void handleTargetDecision(true)} disabled={missionOverrideDisabled || !commandLinkOpen || targetCommandPending || !canApprove}>
            APPROVE
          </button>
          <button type="button" className={`${styles.button} ${styles.ignore}`} onClick={() => void handleTargetDecision(false)} disabled={missionOverrideDisabled || !commandLinkOpen || targetCommandPending || !canReject}>
            REJECT
          </button>
          <button type="button" className={`${styles.button} ${styles.engage}`} onClick={handleEngage}>
            ENGAGE
          </button>
        </div>
      )}
      {targetCommandError && <span role="alert">{targetCommandError}</span>}
    </div>
  );
}
