import type { DetectedTarget } from '../../types/drone';
import styles from './DetectionOverlay.module.css';

interface DetectionOverlayProps {
  targets: DetectedTarget[];
  selectedTargetId: string | null;
  onSelectTarget: (targetId: string) => void;
}

function toPercent(value: number): string {
  return `${(value * 100).toFixed(2)}%`;
}

/**
 * Draws one bounding box per detected target on top of the video feed.
 * Boxes are positioned/sized from normalized (0..1) coordinates, so this
 * has no dependency on the underlying feed's pixel resolution. Tapping a
 * box selects it — with multiple targets in frame, the operator has to pick
 * one before Follow/Ignore/Engage become available.
 */
export function DetectionOverlay({ targets, selectedTargetId, onSelectTarget }: DetectionOverlayProps) {
  return (
    <div className={styles.overlay}>
      {targets.map((target) => {
        const isSelected = target.id === selectedTargetId;
        return (
          <button
            key={target.id}
            type="button"
            className={`${styles.box} ${isSelected ? styles.boxSelected : ''}`}
            style={{
              left: toPercent(target.boundingBox.x),
              top: toPercent(target.boundingBox.y),
              width: toPercent(target.boundingBox.width),
              height: toPercent(target.boundingBox.height),
            }}
            onClick={() => onSelectTarget(target.id)}
          >
            <span className={styles.tag}>
              {target.label} · {Math.round(target.confidence * 100)}%
              {isSelected ? ' · SELECTED' : ''}
            </span>
          </button>
        );
      })}
    </div>
  );
}
