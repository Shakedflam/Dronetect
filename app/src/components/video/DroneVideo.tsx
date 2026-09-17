import { useEffect, useState } from 'react';
import type { DetectedTarget } from '../../types/drone';
import { useLocation } from '../../hooks/useLocation';
import { useVideoStream } from '../../hooks/useVideoStream';
import { DetectionOverlay } from './DetectionOverlay';
import { CompassStrip } from './CompassStrip';
import { useVideoFitTransform, type VideoFitMode } from './useVideoFitTransform';
import styles from './DroneVideo.module.css';

interface DroneVideoProps {
  targets: DetectedTarget[];
  selectedTargetId: string | null;
  onSelectTarget: (targetId: string) => void;
}

const MIN_ZOOM = 1;
const MAX_ZOOM = 3;
const ZOOM_STEP = 0.25;

/**
 * Camera feed area. Renders the live WebRTC feed once useVideoStream()
 * has a track; falls back to a placeholder until then. The detection
 * overlay stays layered on top via absolute positioning either way.
 *
 * Defaults to `contain` so the operator sees the camera's whole frame
 * (nothing cropped off the edges); a toolbar toggle switches to `cover`
 * for an edge-to-edge fill instead, and zoom controls apply a digital
 * zoom on top of either mode for a closer look at a target.
 */
export function DroneVideo({ targets, selectedTargetId, onSelectTarget }: DroneVideoProps) {
  const position = useLocation();
  const stream = useVideoStream();
  const [videoElement, setVideoElement] = useState<HTMLVideoElement | null>(null);
  const [fitMode, setFitMode] = useState<VideoFitMode>('contain');
  const [zoom, setZoom] = useState(1);
  const transform = useVideoFitTransform(videoElement, fitMode, zoom);

  useEffect(() => {
    if (!videoElement) return;
    videoElement.srcObject = stream;
    return () => {
      videoElement.srcObject = null;
    };
  }, [stream, videoElement]);

  function zoomBy(delta: number): void {
    setZoom((current) => Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, round2(current + delta))));
  }

  return (
    <div className={styles.wrapper}>
      {stream ? (
        <video
          ref={setVideoElement}
          className={styles.videoFeed}
          style={{ objectFit: fitMode, transform: `scale(${zoom})` }}
          autoPlay
          playsInline
          muted
        />
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
      {stream && (
        <div className={styles.videoControls}>
          <button
            type="button"
            className={styles.controlButton}
            onClick={() => setFitMode((mode) => (mode === 'contain' ? 'cover' : 'contain'))}
            title={fitMode === 'contain' ? 'Showing full camera view — click to fill frame' : 'Filling frame — click to show full camera view'}
          >
            {fitMode === 'contain' ? 'FULL VIEW' : 'FILL FRAME'}
          </button>
          <div className={styles.zoomGroup}>
            <button
              type="button"
              className={styles.controlButton}
              onClick={() => zoomBy(-ZOOM_STEP)}
              disabled={zoom <= MIN_ZOOM}
              aria-label="Zoom out"
            >
              −
            </button>
            <span className={styles.zoomReadout}>{formatZoom(zoom)}×</span>
            <button
              type="button"
              className={styles.controlButton}
              onClick={() => zoomBy(ZOOM_STEP)}
              disabled={zoom >= MAX_ZOOM}
              aria-label="Zoom in"
            >
              +
            </button>
            {zoom !== 1 && (
              <button type="button" className={styles.controlButton} onClick={() => setZoom(1)}>
                RESET
              </button>
            )}
          </div>
        </div>
      )}
    </div>
  );
}

function round2(value: number): number {
  return Math.round(value * 100) / 100;
}

/** "1", "1.25", "1.5" — trims trailing zeros without dropping a whole integer. */
function formatZoom(zoom: number): string {
  return zoom.toFixed(2).replace(/\.?0+$/, '');
}
