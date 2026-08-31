import type { DetectedTarget, NormalizedBox } from '../../types/drone';
import type { CoverTransform } from './useVideoCoverTransform';
import styles from './DetectionOverlay.module.css';

interface DetectionOverlayProps {
  targets: DetectedTarget[];
  selectedTargetId: string | null;
  onSelectTarget: (targetId: string) => void;
  transform: CoverTransform | null;
}

function toDisplayRect(box: NormalizedBox, transform: CoverTransform) {
  // Re-project the box (normalized against the detector's own frame size)
  // into the video's intrinsic pixel space before applying the cover crop.
  const left = box.x1 * transform.videoWidth * transform.scale - transform.offsetX;
  const top = box.y1 * transform.videoHeight * transform.scale - transform.offsetY;
  const right = box.x2 * transform.videoWidth * transform.scale - transform.offsetX;
  const bottom = box.y2 * transform.videoHeight * transform.scale - transform.offsetY;

  return {
    left: `${(left / transform.containerWidth) * 100}%`,
    top: `${(top / transform.containerHeight) * 100}%`,
    width: `${((right - left) / transform.containerWidth) * 100}%`,
    height: `${((bottom - top) / transform.containerHeight) * 100}%`,
  };
}

/**
 * Draws one bounding box per detected target on top of the video feed.
 * Boxes are normalized against the detector's own frame size (which can
 * differ from the video's intrinsic resolution); `transform` re-projects
 * them into the video's displayed area, accounting for object-fit: cover,
 * so they stay aligned as the element is resized. Tapping a box selects it
 * — with multiple targets in frame, the operator has to pick one before
 * Follow/Ignore/Engage become available.
 */
export function DetectionOverlay({ targets, selectedTargetId, onSelectTarget, transform }: DetectionOverlayProps) {
  if (!transform) return <div className={styles.overlay} />;

  return (
    <div className={styles.overlay}>
      {targets.map((target) => {
        const isSelected = target.id === selectedTargetId;
        return (
          <button
            key={target.id}
            type="button"
            className={`${styles.box} ${isSelected ? styles.boxSelected : ''}`}
            style={toDisplayRect(target.boundingBox, transform)}
            onClick={() => onSelectTarget(target.id)}
          >
            <span className={styles.tag}>
              {target.label} {Math.round(target.confidence * 100)}%
              {isSelected ? ' · SELECTED' : ''}
            </span>
          </button>
        );
      })}
    </div>
  );
}
