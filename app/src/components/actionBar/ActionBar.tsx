import { droneCommandService } from '../../services/droneCommandService';
import { useMissionState } from '../../hooks/useMissionState';
import { useMissionStatus } from '../../hooks/useMissionStatus';
import { useRoi } from '../../hooks/useRoi';
import styles from './ActionBar.module.css';

interface ActionBarProps {
  selectedTargetId: string | null;
  onClearSelection: () => void;
}

/**
 * Permanent control bar. RTH/New Mission/camera tilt are always available —
 * they're the drone-wide commands the realtime server currently supports.
 * Patrol/ROI/RTH are backend commands, and mission state messages show
 * the authoritative transition in the map/HUD.
 * Follow/Ignore/Engage only appear once the
 * operator has selected a detected target (App owns that selection state);
 * they have no realtime server equivalent yet and remain mocked.
 */
export function ActionBar({ selectedTargetId, onClearSelection }: ActionBarProps) {
  const hasSelection = selectedTargetId !== null;
  const missionState = useMissionState();
  const missionStatus = useMissionStatus();
  const droneConnected = missionStatus?.droneConnected ?? false;
  const roi = useRoi();
  const roiMissionActive = missionState === 'TRANSIT_TO_ROI' || missionState === 'ROI_PATROL';

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

  function handleFollow() {
    if (!selectedTargetId) return;
    void droneCommandService.followTarget(selectedTargetId);
  }

  function handleIgnore() {
    if (!selectedTargetId) return;
    void droneCommandService.ignoreTarget(selectedTargetId);
    onClearSelection();
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
          disabled={!droneConnected}
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
          disabled={!droneConnected}
          aria-pressed={missionState === 'PATROL'}
        >
          PATROL
        </button>
        <button
          type="button"
          className={`${styles.button} ${styles.roi} ${roiMissionActive ? styles.roiActive : ''}`}
          onClick={handleTransitToRoi}
          disabled={!roi || !droneConnected}
          title={droneConnected ? undefined : 'Drone is not connected'}
          aria-pressed={roiMissionActive}
        >
          ROI
        </button>
      </div>

      {hasSelection && (
        <div className={styles.contextualGroup}>
          <button type="button" className={`${styles.button} ${styles.follow}`} onClick={handleFollow}>
            FOLLOW
          </button>
          <button type="button" className={`${styles.button} ${styles.ignore}`} onClick={handleIgnore}>
            IGNORE
          </button>
          <button type="button" className={`${styles.button} ${styles.engage}`} onClick={handleEngage}>
            ENGAGE
          </button>
        </div>
      )}
    </div>
  );
}
