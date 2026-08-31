import type { Unsubscribe } from './unsubscribe';

/**
 * Owns the single live WebSocket link to the edge realtime server
 * (`/ws?role=tablet`). Unlike `pollingSource.ts`'s per-metric lazy-start
 * timers, this connects eagerly on module load and stays connected for the
 * app's lifetime — telemetry and command sending both need the same live
 * socket, and there's no polling cost to defer.
 */

export type ConnectionStatus = 'connecting' | 'open' | 'reconnecting' | 'closed';

export interface TelemetryMessage {
  timestamp: number;
  source: string;
  data: {
    gps: { lat: number; lng: number };
    altitudeM: number;
    headingDeg: number;
    speedMps: number;
    batteryPercent: number;
    state: string;
  };
}

export interface DetectionsMessage {
  type: 'detections';
  frameId: number;
  /** Pixel dimensions of the frame the detector ran on — the coordinate space `box` is in. */
  frameWidth: number;
  frameHeight: number;
  detections: Array<{
    className: string;
    confidence: number;
    box: [number, number, number, number];
    trackId: string;
    trackState: string;
    lastSeenTimestamp: number;
    velocityPxPerSec: { x: number; y: number };
  }>;
}

/** Exactly the command names the realtime server/simulator currently support. */
export type RealtimeServerCommandName = 'return_home' | 'new_mission' | 'camera_tilt_up' | 'camera_tilt_down';

export interface RealtimeServerConnection {
  subscribeTelemetry(listener: (message: TelemetryMessage) => void): Unsubscribe;
  subscribeDetections(listener: (message: DetectionsMessage) => void): Unsubscribe;
  subscribeStatus(listener: (status: ConnectionStatus) => void): Unsubscribe;
  sendCommand(command: RealtimeServerCommandName, params?: Record<string, unknown>): Promise<void>;
}

const RECONNECT_DELAY_MS = 1000;
const COMMAND_TIMEOUT_MS = 5000;
const DEFAULT_REALTIME_SERVER_ORIGIN = 'ws://127.0.0.1:8765';

function resolveRealtimeServerUrl(): string {
  const origin = import.meta.env.VITE_REALTIME_SERVER_WS_URL || DEFAULT_REALTIME_SERVER_ORIGIN;
  return `${origin}/ws?role=tablet`;
}

interface PendingCommand {
  resolve: () => void;
  reject: (error: Error) => void;
  timeoutHandle: ReturnType<typeof setTimeout>;
}

function isTelemetryMessage(value: unknown): value is TelemetryMessage {
  if (typeof value !== 'object' || value === null) return false;
  const message = value as Record<string, unknown>;
  if (message.type !== 'telemetry') return false;

  const data = message.data as Record<string, unknown> | undefined;
  if (typeof data !== 'object' || data === null) return false;

  const gps = data.gps as Record<string, unknown> | undefined;
  return (
    typeof gps === 'object' &&
    gps !== null &&
    typeof gps.lat === 'number' &&
    typeof gps.lng === 'number' &&
    typeof data.altitudeM === 'number' &&
    typeof data.headingDeg === 'number' &&
    typeof data.speedMps === 'number' &&
    typeof data.batteryPercent === 'number' &&
    typeof data.state === 'string'
  );
}

function isDetectionsMessage(value: unknown): value is DetectionsMessage {
  if (typeof value !== 'object' || value === null) return false;
  const message = value as Record<string, unknown>;
  if (message.type !== 'detections') return false;
  if (typeof message.frameId !== 'number') return false;
  if (typeof message.frameWidth !== 'number' || typeof message.frameHeight !== 'number') return false;
  if (!Array.isArray(message.detections)) return false;

  return message.detections.every((detection) => {
    if (typeof detection !== 'object' || detection === null) return false;
    const d = detection as Record<string, unknown>;
    const velocity = d.velocityPxPerSec as Record<string, unknown> | undefined;
    return (
      typeof d.className === 'string' &&
      typeof d.confidence === 'number' &&
      Array.isArray(d.box) &&
      d.box.length === 4 &&
      d.box.every((coordinate) => typeof coordinate === 'number') &&
      typeof d.trackId === 'string' &&
      typeof d.trackState === 'string' &&
      typeof d.lastSeenTimestamp === 'number' &&
      typeof velocity === 'object' &&
      velocity !== null &&
      typeof velocity.x === 'number' &&
      typeof velocity.y === 'number'
    );
  });
}

function createRealtimeServerConnection(): RealtimeServerConnection {
  const telemetryListeners = new Set<(message: TelemetryMessage) => void>();
  const detectionListeners = new Set<(message: DetectionsMessage) => void>();
  const statusListeners = new Set<(status: ConnectionStatus) => void>();
  const pendingCommands = new Map<string, PendingCommand>();

  let socket: WebSocket | null = null;
  let status: ConnectionStatus = 'connecting';
  let hasConnectedOnce = false;

  function setStatus(next: ConnectionStatus): void {
    status = next;
    statusListeners.forEach((listener) => listener(next));
  }

  function rejectAllPending(message: string): void {
    pendingCommands.forEach(({ reject, timeoutHandle }) => {
      clearTimeout(timeoutHandle);
      reject(new Error(message));
    });
    pendingCommands.clear();
  }

  function handleMessage(event: MessageEvent<string>): void {
    let parsed: unknown;
    try {
      parsed = JSON.parse(event.data);
    } catch {
      console.warn('[realtimeServerConnection] received malformed JSON, ignoring', event.data);
      return;
    }

    if (typeof parsed !== 'object' || parsed === null || !('type' in parsed)) {
      console.warn('[realtimeServerConnection] received message with no type, ignoring', parsed);
      return;
    }

    const message = parsed as Record<string, unknown>;

    switch (message.type) {
      case 'telemetry': {
        if (!isTelemetryMessage(message)) {
          console.warn('[realtimeServerConnection] telemetry message failed shape check, ignoring', message);
          return;
        }
        telemetryListeners.forEach((listener) => listener(message));
        return;
      }
      case 'detections': {
        if (!isDetectionsMessage(message)) {
          console.warn('[realtimeServerConnection] detections message failed shape check, ignoring', message);
          return;
        }
        detectionListeners.forEach((listener) => listener(message));
        return;
      }
      case 'command_ack': {
        const commandId = message.commandId;
        if (typeof commandId !== 'string') return;
        const pending = pendingCommands.get(commandId);
        if (!pending) return;
        clearTimeout(pending.timeoutHandle);
        pendingCommands.delete(commandId);
        pending.resolve();
        return;
      }
      case 'error': {
        const commandId = message.commandId;
        const errorText = typeof message.message === 'string' ? message.message : 'Realtime server error';
        const errorCode = typeof message.code === 'string' ? message.code : undefined;
        const fullMessage = errorCode ? `${errorText} (${errorCode})` : errorText;

        if (typeof commandId === 'string') {
          const pending = pendingCommands.get(commandId);
          if (pending) {
            clearTimeout(pending.timeoutHandle);
            pendingCommands.delete(commandId);
            pending.reject(new Error(fullMessage));
            return;
          }
        }
        console.error('[realtimeServerConnection] realtime server error', message);
        return;
      }
      default:
        console.warn('[realtimeServerConnection] unknown message type, ignoring', message);
    }
  }

  function scheduleReconnect(): void {
    setStatus(hasConnectedOnce ? 'reconnecting' : 'connecting');
    setTimeout(connect, RECONNECT_DELAY_MS);
  }

  function connect(): void {
    try {
      socket = new WebSocket(resolveRealtimeServerUrl());
    } catch (error) {
      console.warn('[realtimeServerConnection] failed to open WebSocket, retrying', error);
      scheduleReconnect();
      return;
    }

    socket.onopen = () => {
      hasConnectedOnce = true;
      setStatus('open');
    };

    socket.onmessage = handleMessage;

    socket.onclose = () => {
      rejectAllPending('Realtime server connection closed');
      socket = null;
      scheduleReconnect();
    };

    socket.onerror = () => {
      socket?.close();
    };
  }

  connect();

  return {
    subscribeTelemetry(listener) {
      telemetryListeners.add(listener);
      return () => telemetryListeners.delete(listener);
    },
    subscribeDetections(listener) {
      detectionListeners.add(listener);
      return () => detectionListeners.delete(listener);
    },
    subscribeStatus(listener) {
      statusListeners.add(listener);
      listener(status);
      return () => statusListeners.delete(listener);
    },
    sendCommand(command, params = {}) {
      return new Promise((resolve, reject) => {
        if (!socket || socket.readyState !== WebSocket.OPEN) {
          reject(new Error('Realtime server not connected'));
          return;
        }

        const commandId = crypto.randomUUID();
        const timeoutHandle = setTimeout(() => {
          pendingCommands.delete(commandId);
          reject(new Error(`Command "${command}" timed out waiting for realtime server response`));
        }, COMMAND_TIMEOUT_MS);

        pendingCommands.set(commandId, { resolve, reject, timeoutHandle });

        socket.send(JSON.stringify({ type: 'command', commandId, command, params }));
      });
    },
  };
}

export const realtimeServerConnection: RealtimeServerConnection = createRealtimeServerConnection();
