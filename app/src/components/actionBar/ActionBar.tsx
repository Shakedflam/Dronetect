import { droneCommandService } from '../../services/droneCommandService';
import { missionStateService } from '../../services/missionStateService';
import { useMissionState } from '../../hooks/useMissionState';
import { useRoi } from '../../hooks/useRoi';
import styles from './ActionBar.module.css';

interface ActionBarProps {
  selectedTargetId: string | null;
  onClearSelection: () => void;
}

/**
 * Permanent control bar. RTH/New Mission/camera tilt are always available —
 * they're the drone-wide commands the realtime server currently supports.
 * Patrol/ROI/RTH additionally write a shared frontend-only
 * missionStateService (see types/missionState.ts) so the map/HUD can show
 * what the operator last commanded — decoupled from TelemetryData.state,
 * the backend's own flight-state-machine string. ROI_PATROL is declared in
 * that state but not reachable from any button yet (no automatic
 * transitions are implemented). Follow/Ignore/Engage only appear once the
 * operator has selected a detected target (App owns that selection state);
 * they have no realtime server equivalent yet and remain mocked.
 */
export function ActionBar({ selectedTargetId, onClearSelection }: ActionBarProps) {
  const hasSelection = selectedTargetId !== null;
  const missionState = useMissionState();
  const roi = useRoi();
  const roiMissionActive = missionState === 'TRANSIT_TO_ROI' || missionState === 'ROI_PATROL';

  function handleReturnToHome() {
    droneCommandService.returnToHome().catch((error: unknown) => {
      console.error('[ActionBar] returnToHome failed', error);
    });
    missionStateService.setMissionState('RTH');
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

  function handleTogglePatrol() {
    missionStateService.setMissionState(missionState === 'PATROL' ? 'ON_GROUND' : 'PATROL');
  }

  function handleTransitToRoi() {
    if (!roi) return;
    missionStateService.setMissionState('TRANSIT_TO_ROI');
  }

  return (
    <div className={styles.bar}>
      <div className={styles.primaryGroup}>
        <button
          type="button"
          className={`${styles.button} ${styles.rth}`}
          onClick={handleReturnToHome}
        >
          RTH
        </button>
        <button
          type="button"
          className={`${styles.button} ${styles.mission}`}
          onClick={handleNewMission}
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
          onClick={handleTogglePatrol}
          aria-pressed={missionState === 'PATROL'}
        >
          PATROL
        </button>
        <button
          type="button"
          className={`${styles.button} ${styles.roi} ${roiMissionActive ? styles.roiActive : ''}`}
          onClick={handleTransitToRoi}
          disabled={!roi}
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
