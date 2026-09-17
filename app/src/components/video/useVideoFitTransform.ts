import { useEffect, useState } from 'react';

export type VideoFitMode = 'contain' | 'cover';

/**
 * Maps the video's intrinsic pixel space onto its displayed box, matching
 * whichever `object-fit` mode is active (`contain`: whole frame visible,
 * letterboxed; `cover`: fills the box, center-cropped) plus an optional
 * digital zoom applied on top via `transform: scale()`. Exposes the video's
 * own intrinsic size too, since detection boxes are normalized against a
 * possibly-different detector frame size and must be re-projected into the
 * video's intrinsic space before the fit scale/offset apply.
 *
 * The zoom factor folds cleanly into `scale`/`offsetX`/`offsetY` because a
 * CSS `transform: scale(zoom)` around the box's own center is algebraically
 * equivalent to just multiplying the fit scale by `zoom` before computing
 * the centered offsets below — no separate zoom-aware branch needed.
 */
export interface FitTransform {
  videoWidth: number;
  videoHeight: number;
  scale: number;
  offsetX: number;
  offsetY: number;
  containerWidth: number;
  containerHeight: number;
}

export function useVideoFitTransform(
  video: HTMLVideoElement | null,
  fitMode: VideoFitMode,
  zoom: number,
): FitTransform | null {
  const [transform, setTransform] = useState<FitTransform | null>(null);

  useEffect(() => {
    if (!video) {
      setTransform(null);
      return;
    }

    function recompute() {
      if (!video) return;
      const { videoWidth, videoHeight, clientWidth, clientHeight } = video;
      if (!videoWidth || !videoHeight || !clientWidth || !clientHeight) return;

      const fitScale =
        fitMode === 'cover'
          ? Math.max(clientWidth / videoWidth, clientHeight / videoHeight)
          : Math.min(clientWidth / videoWidth, clientHeight / videoHeight);
      const scale = fitScale * zoom;
      setTransform({
        videoWidth,
        videoHeight,
        scale,
        offsetX: (videoWidth * scale - clientWidth) / 2,
        offsetY: (videoHeight * scale - clientHeight) / 2,
        containerWidth: clientWidth,
        containerHeight: clientHeight,
      });
    }

    recompute();
    video.addEventListener('loadedmetadata', recompute);
    const resizeObserver = new ResizeObserver(recompute);
    resizeObserver.observe(video);

    return () => {
      video.removeEventListener('loadedmetadata', recompute);
      resizeObserver.disconnect();
    };
  }, [video, fitMode, zoom]);

  return transform;
}
