import { droneCommandService } from '../../services/droneCommandService';
import styles from './ActionBar.module.css';

interface ActionBarProps {
  selectedTargetId: string | null;
  onClearSelection: () => void;
}

/**
 * Permanent control bar. RTH is always available. Follow/Ignore/Engage only
 * appear once the operator has selected a detected target (App owns that
 * selection state) — they act on whichever target is currently selected.
 */
export function ActionBar({ selectedTargetId, onClearSelection }: ActionBarProps) {
  const hasSelection = selectedTargetId !== null;

  function handleReturnToHome() {
    void droneCommandService.returnToHome();
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
      <button
        type="button"
        className={`${styles.button} ${styles.rth}`}
        onClick={handleReturnToHome}
      >
        RTH
      </button>

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
