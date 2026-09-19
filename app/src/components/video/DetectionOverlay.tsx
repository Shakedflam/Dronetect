import type { CSSProperties } from 'react';
import type { DetectedTarget, NormalizedBox } from '../../types/drone';
import type { FitTransform } from './useVideoFitTransform';
import { useSmoothedTargets } from './useSmoothedTargets';
import styles from './DetectionOverlay.module.css';

interface DetectionOverlayProps {
  targets: DetectedTarget[];
  selectedTargetId: string | null;
  onSelectTarget: (targetId: string) => void;
  transform: FitTransform | null;
}

const MIN_TARGET_WIDTH_PX = 48;
const MIN_TARGET_HEIGHT_PX = 36;

function toDisplayRect(box: NormalizedBox, transform: FitTransform): CSSProperties {
  // Re-project the box (normalized against the detector's own frame size)
  // into the video's intrinsic pixel space before applying the fit/zoom transform.
  const left = box.x1 * transform.videoWidth * transform.scale - transform.offsetX;
  const top = box.y1 * transform.videoHeight * transform.scale - transform.offsetY;
  const right = box.x2 * transform.videoWidth * transform.scale - transform.offsetX;
  const bottom = box.y2 * transform.videoHeight * transform.scale - transform.offsetY;
  const width = right - left;
  const height = bottom - top;
  const displayWidth = Math.max(width, MIN_TARGET_WIDTH_PX);
  const displayHeight = Math.max(height, MIN_TARGET_HEIGHT_PX);
  const displayLeft = left + width / 2 - displayWidth / 2;
  const displayTop = top + height / 2 - displayHeight / 2;

  return {
    left: `${(displayLeft / transform.containerWidth) * 100}%`,
    top: `${(displayTop / transform.containerHeight) * 100}%`,
    width: `${(displayWidth / transform.containerWidth) * 100}%`,
    height: `${(displayHeight / transform.containerHeight) * 100}%`,
  };
}

/** Persistent per-track identity for display, e.g. trackId "drone-3" -> "#3". */
function formatTrackNumber(trackId: string): string {
  const match = trackId.match(/(\d+)\s*$/);
  return match ? `#${match[1]}` : `#${trackId}`;
}

/**
 * Draws one bounding box per detected target on top of the video feed.
 * Boxes are normalized against the detector's own frame size (which can
 * differ from the video's intrinsic resolution); `transform` re-projects
 * them into the video's displayed area, accounting for the active fit mode
 * (contain/cover) and digital zoom, so they stay aligned as the element is
 * resized or the operator zooms in. Tapping a box selects it
 * — with multiple targets in frame, the operator has to pick one before
 * Approve/Reject/Engage become available.
 */
export function DetectionOverlay({ targets, selectedTargetId, onSelectTarget, transform }: DetectionOverlayProps) {
  const smoothedTargets = useSmoothedTargets(targets);

  if (!transform) return <div className={styles.overlay} />;

  return (
    <div className={styles.overlay}>
      {smoothedTargets.map((target) => {
        const isSelected = target.id === selectedTargetId;
        const isTentative = target.trackState === 'tentative';
        return (
          <button
            key={target.id}
            type="button"
            className={`${styles.box} ${isTentative ? styles.boxTentative : ''} ${isSelected ? styles.boxSelected : ''}`}
            style={toDisplayRect(target.boundingBox, transform)}
            onClick={() => onSelectTarget(target.id)}
          >
            <span className={styles.tag}>
              {target.label} {formatTrackNumber(target.id)} {Math.round(target.confidence * 100)}%
              {isSelected ? ' · SELECTED' : ''}
            </span>
          </button>
        );
      })}
    </div>
  );
}
