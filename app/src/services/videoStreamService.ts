import { webrtcConnection } from './internal/webrtcConnection';
import type { Unsubscribe } from './internal/unsubscribe';

/**
 * Abstraction over "what does the drone's camera currently see". This is
 * the only piece of the app allowed to know about the WebRTC signaling
 * connection — sourced from the shared `webrtcConnection`, everything else
 * consumes it through `subscribe`.
 */
export interface VideoStreamService {
  subscribe(listener: (stream: MediaStream | null) => void): Unsubscribe;
}

function createWebrtcVideoStreamService(): VideoStreamService {
  return {
    subscribe(listener) {
      return webrtcConnection.subscribeStream(listener);
    },
  };
}

export const videoStreamService: VideoStreamService = createWebrtcVideoStreamService();
