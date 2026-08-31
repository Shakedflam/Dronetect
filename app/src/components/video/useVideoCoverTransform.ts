import { useEffect, useState } from 'react';

/**
 * Maps the video's intrinsic pixel space onto its displayed box, matching
 * `object-fit: cover` (center-cropped, uniform scale-to-fill). Used to
 * convert YOLO pixel boxes — which are in the video's intrinsic frame — into
 * on-screen percentages that stay aligned as the element is resized.
 */
export interface CoverTransform {
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
