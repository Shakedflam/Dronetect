import { useEffect, useState } from 'react';
import type { DetectedTarget } from '../../types/drone';
import { useLocation } from '../../hooks/useLocation';
import { useVideoStream } from '../../hooks/useVideoStream';
import { DetectionOverlay } from './DetectionOverlay';
import { CompassStrip } from './CompassStrip';
import { useVideoCoverTransform } from './useVideoCoverTransform';
import styles from './DroneVideo.module.css';

interface DroneVideoProps {
  targets: DetectedTarget[];
  selectedTargetId: string | null;
  onSelectTarget: (targetId: string) => void;
}

/**
 * Camera feed area. Renders the live WebRTC feed once useVideoStream()
 * has a track; falls back to a placeholder until then. The detection
 * overlay stays layered on top via absolute positioning either way.
 */
export function DroneVideo({ targets, selectedTargetId, onSelectTarget }: DroneVideoProps) {
  const position = useLocation();
  const stream = useVideoStream();
  const [videoElement, setVideoElement] = useState<HTMLVideoElement | null>(null);
  const transform = useVideoCoverTransform(videoElement);

  useEffect(() => {
    if (!videoElement) return;
    videoElement.srcObject = stream;
    return () => {
      videoElement.srcObject = null;
    };
  }, [stream, videoElement]);

  return (
    <div className={styles.wrapper}>
      {stream ? (
        <video ref={setVideoElement} className={styles.videoFeed} autoPlay playsInline muted />
      ) : (
        <div className={styles.feedPlaceholder}>
          <div className={styles.scanlines} />
          <span className={styles.feedLabel}>CONNECTING TO CAMERA FEED…</span>
        </div>
      )}
      <DetectionOverlay
        targets={targets}
        selectedTargetId={selectedTargetId}
        onSelectTarget={onSelectTarget}
        transform={transform}
      />
      {position && <CompassStrip headingDeg={position.headingDeg} />}
    </div>
  );
}
