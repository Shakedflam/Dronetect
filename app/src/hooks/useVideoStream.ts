import { useEffect, useState } from 'react';
import { videoStreamService } from '../services/videoStreamService';

/** Latest live video stream from the drone's camera, or null until connected. */
export function useVideoStream(): MediaStream | null {
  const [stream, setStream] = useState<MediaStream | null>(null);

  useEffect(() => {
    return videoStreamService.subscribe(setStream);
  }, []);

  return stream;
}
