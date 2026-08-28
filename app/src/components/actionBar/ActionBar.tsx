import { droneCommandService } from '../../services/droneCommandService';
import styles from './ActionBar.module.css';

interface ActionBarProps {
  selectedTargetId: string | null;
  onClearSelection: () => void;
}

/**
 * Permanent control bar. RTH/New Mission/camera tilt are always available —
 * these are the drone-wide commands the realtime server currently supports.
 * Follow/Ignore/Engage only appear once the operator has selected a
 * detected target (App owns that selection state); they have no realtime
 * server equivalent yet and remain mocked.
 */
export function ActionBar({ selectedTargetId, onClearSelection }: ActionBarProps) {
  const hasSelection = selectedTargetId !== null;

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
