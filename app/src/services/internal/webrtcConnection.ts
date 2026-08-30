import type { Unsubscribe } from './unsubscribe';

/**
 * Owns the single live WebRTC link to the edge realtime server's video
 * pipeline (`/video/webrtc?role=viewer`). Receive-only: the app never sends
 * a track, only a recvonly video transceiver. Like `realtimeServerConnection.ts`,
 * connects eagerly on module load and stays connected for the app's
 * lifetime.
 *
 * The server (aiortc-based) reads exactly one SDP offer message, replies
 * with exactly one SDP answer, then ignores anything else sent on the
 * signaling socket — it does not support trickle ICE. So the offer is only
 * sent once local ICE gathering has finished.
 */

export type WebrtcConnectionStatus = 'connecting' | 'open' | 'reconnecting' | 'closed';

const RECONNECT_DELAY_MS = 1000;
const ICE_GATHERING_TIMEOUT_MS = 2000;
const DEFAULT_REALTIME_SERVER_ORIGIN = 'ws://127.0.0.1:8765';

function resolveWebrtcSignalingUrl(): string {
  const origin = import.meta.env.VITE_REALTIME_SERVER_WS_URL || DEFAULT_REALTIME_SERVER_ORIGIN;
  return `${origin}/video/webrtc?role=viewer`;
}

function waitForIceGatheringComplete(peer: RTCPeerConnection): Promise<void> {
  if (peer.iceGatheringState === 'complete') return Promise.resolve();
  return new Promise((resolve) => {
    const timeoutHandle = setTimeout(() => {
      peer.removeEventListener('icegatheringstatechange', onChange);
      resolve();
    }, ICE_GATHERING_TIMEOUT_MS);

    function onChange(): void {
      if (peer.iceGatheringState !== 'complete') return;
      clearTimeout(timeoutHandle);
      peer.removeEventListener('icegatheringstatechange', onChange);
      resolve();
    }

    peer.addEventListener('icegatheringstatechange', onChange);
  });
}

function createWebrtcConnection() {
  const streamListeners = new Set<(stream: MediaStream | null) => void>();
  const statusListeners = new Set<(status: WebrtcConnectionStatus) => void>();

  let peer: RTCPeerConnection | null = null;
  let socket: WebSocket | null = null;
  let status: WebrtcConnectionStatus = 'connecting';
  let hasConnectedOnce = false;

  function setStatus(next: WebrtcConnectionStatus): void {
    status = next;
    statusListeners.forEach((listener) => listener(next));
  }

  function setStream(stream: MediaStream | null): void {
    streamListeners.forEach((listener) => listener(stream));
  }

  function teardownAndReconnect(): void {
    if (peer) {
      peer.onconnectionstatechange = null;
      peer.ontrack = null;
      peer.close();
      peer = null;
    }
    if (socket) {
      socket.onopen = null;
      socket.onmessage = null;
      socket.onerror = null;
      socket.onclose = null;
      socket.close();
      socket = null;
    }
    setStream(null);
    setStatus(hasConnectedOnce ? 'reconnecting' : 'connecting');
    setTimeout(connect, RECONNECT_DELAY_MS);
  }

  async function sendOffer(): Promise<void> {
    if (!peer || !socket) return;
    const offer = await peer.createOffer();
    await peer.setLocalDescription(offer);
    await waitForIceGatheringComplete(peer);
    if (!peer.localDescription || socket.readyState !== WebSocket.OPEN) return;
    socket.send(JSON.stringify({ type: 'offer', sdp: peer.localDescription.sdp }));
  }

  function handleMessage(event: MessageEvent<string>): void {
    let parsed: unknown;
    try {
      parsed = JSON.parse(event.data);
    } catch {
      console.warn('[webrtcConnection] received malformed JSON, ignoring', event.data);
      return;
    }

    if (typeof parsed !== 'object' || parsed === null) return;
    const message = parsed as Record<string, unknown>;

    if (message.type === 'answer' && typeof message.sdp === 'string') {
      peer?.setRemoteDescription({ type: 'answer', sdp: message.sdp }).catch((error: unknown) => {
        console.warn('[webrtcConnection] failed to set remote description', error);
      });
      return;
    }

    if (message.type === 'error') {
      console.error('[webrtcConnection] realtime server error', message);
    }
  }

  function connect(): void {
    peer = new RTCPeerConnection();
    peer.addTransceiver('video', { direction: 'recvonly' });

    peer.ontrack = (event) => {
      setStream(event.streams[0] ?? new MediaStream([event.track]));
    };

    peer.onconnectionstatechange = () => {
      if (
        peer &&
        (peer.connectionState === 'failed' ||
          peer.connectionState === 'disconnected' ||
          peer.connectionState === 'closed')
      ) {
        socket?.close();
      }
    };

    try {
      socket = new WebSocket(resolveWebrtcSignalingUrl());
    } catch (error) {
      console.warn('[webrtcConnection] failed to open signaling WebSocket, retrying', error);
      teardownAndReconnect();
      return;
    }

    socket.onopen = () => {
      hasConnectedOnce = true;
      setStatus('open');
      sendOffer().catch((error: unknown) => {
        console.warn('[webrtcConnection] failed to negotiate offer', error);
        socket?.close();
      });
    };

    socket.onmessage = handleMessage;

    socket.onclose = () => {
      teardownAndReconnect();
    };

    socket.onerror = () => {
      socket?.close();
    };
  }

  connect();

  return {
    subscribeStream(listener: (stream: MediaStream | null) => void): Unsubscribe {
      streamListeners.add(listener);
      return () => streamListeners.delete(listener);
    },
    subscribeStatus(listener: (status: WebrtcConnectionStatus) => void): Unsubscribe {
      statusListeners.add(listener);
      listener(status);
      return () => statusListeners.delete(listener);
    },
  };
}

export const webrtcConnection = createWebrtcConnection();
