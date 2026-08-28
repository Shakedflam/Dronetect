import type { DetectedTarget } from '../../types/drone';
import { useLocation } from '../../hooks/useLocation';
import { DetectionOverlay } from './DetectionOverlay';
import { CompassStrip } from './CompassStrip';
import styles from './DroneVideo.module.css';

interface DroneVideoProps {
  targets: DetectedTarget[];
  selectedTargetId: string | null;
  onSelectTarget: (targetId: string) => void;
}

/**
 * Camera feed area. Today it's a placeholder — later this swaps for a
 * <video>/<canvas> element streaming the real feed. The detection overlay
 * is already layered on top via absolute positioning so that swap won't
 * require restructuring this component.
 */
export function DroneVideo({ targets, selectedTargetId, onSelectTarget }: DroneVideoProps) {
  const position = useLocation();

  return (
    <div className={styles.wrapper}>
      <div className={styles.feedPlaceholder}>
        <div className={styles.scanlines} />
        <span className={styles.feedLabel}>MOCK FEED — CAMERA NOT CONNECTED</span>
      </div>
      <DetectionOverlay
        targets={targets}
        selectedTargetId={selectedTargetId}
        onSelectTarget={onSelectTarget}
      />
      {position && <CompassStrip headingDeg={position.headingDeg} />}
    </div>
  );
}
