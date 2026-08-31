import { useEffect, useState } from 'react';

/**
 * Maps the video's intrinsic pixel space onto its displayed box, matching
 * `object-fit: cover` (center-cropped, uniform scale-to-fill). Exposes the
 * video's own intrinsic size too, since detection boxes are normalized
 * against a possibly-different detector frame size and must be re-projected
 * into the video's intrinsic space before the cover scale/offset apply.
 */
export interface CoverTransform {
  videoWidth: number;
  videoHeight: number;
  scale: number;
  offsetX: number;
  offsetY: number;
  containerWidth: number;
  containerHeight: number;
}

export function useVideoCoverTransform(video: HTMLVideoElement | null): CoverTransform | null {
  const [transform, setTransform] = useState<CoverTransform | null>(null);

  useEffect(() => {
    if (!video) {
      setTransform(null);
      return;
    }

    function recompute() {
      if (!video) return;
      const { videoWidth, videoHeight, clientWidth, clientHeight } = video;
      if (!videoWidth || !videoHeight || !clientWidth || !clientHeight) return;

      const scale = Math.max(clientWidth / videoWidth, clientHeight / videoHeight);
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
  }, [video]);

  return transform;
}
