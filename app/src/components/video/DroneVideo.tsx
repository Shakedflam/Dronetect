import { useEffect, useRef } from 'react';
import type { DetectedTarget } from '../../types/drone';
import { useLocation } from '../../hooks/useLocation';
import { useVideoStream } from '../../hooks/useVideoStream';
import { DetectionOverlay } from './DetectionOverlay';
import { CompassStrip } from './CompassStrip';
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
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    const video = videoRef.current;
    if (!video) return;
    video.srcObject = stream;
    return () => {
      video.srcObject = null;
    };
  }, [stream]);

  return (
    <div className={styles.wrapper}>
      {stream ? (
        <video ref={videoRef} className={styles.videoFeed} autoPlay playsInline muted />
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
      />
      {position && <CompassStrip headingDeg={position.headingDeg} />}
    </div>
  );
}
