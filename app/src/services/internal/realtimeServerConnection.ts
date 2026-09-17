import type { MissionState } from '../../types/missionState';
import type { TargetFollowDiagnostics } from '../../types/drone';
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
    targetPatrolRegionId: string | null;
    targetFollowDiagnostics?: TargetFollowDiagnostics | null;
  };
}

export interface DetectionsMessage {
  type: 'detections';
  frameId: number;
  /** Source video frame timestamp, carried through detection unchanged, in Unix epoch milliseconds. */
  timestampMs?: number;
  /** Capture-to-realtime-server delay, measured on the backend clock. */
  detectionAgeMs: number;
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

/** Complete backend-owned mission snapshot. Frontend code must not derive transitions. */
export interface MissionStatusMessage {
  type: 'mission_status';
  state: MissionState;
  droneConnected: boolean;
  activeRoi: { center: { lat: number; lon: number }; radiusM: number } | null;
  patrol: {
    priorities: Array<{ regionId: string; rank: number }>;
    areaTypes?: Array<{ regionId: string; areaType: 'PATROL' | 'IGNORE' | 'NO_FLY' }>;
    currentRegionId: string | null;
    targetRegionId: string | null;
  };
  batteryRth: {
    batteryPercent: number | null;
    lowBattery: boolean;
    rthActive: boolean;
    safetyRthActive: boolean;
    rthReason: string | null;
  };
}

/** Exactly the command names the realtime server/simulator currently support. */
export type RealtimeServerCommandName =
  | 'start_patrol'
  | 'set_patrol_priorities'
  | 'set_patrol_area_types'
  | 'set_roi'
  | 'clear_roi'
  | 'return_home'
  | 'new_mission'
  | 'camera_tilt_up'
  | 'camera_tilt_down';

export interface RealtimeServerConnection {
  subscribeTelemetry(listener: (message: TelemetryMessage) => void): Unsubscribe;
  subscribeDetections(listener: (message: DetectionsMessage) => void): Unsubscribe;
  subscribeMissionState(listener: (state: MissionState) => void): Unsubscribe;
  subscribeMissionStatus(listener: (status: MissionStatusMessage) => void): Unsubscribe;
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

/**
 * `crypto.randomUUID` only exists in secure contexts (https, or literal
 * `localhost`) — this tablet UI is typically served plain-http to a device
 * on the same local network as the drone, which is not a secure context.
 * Command IDs are just a local correlation key for matching `command_ack`
 * responses, not a security boundary, so a non-cryptographic fallback is
 * fine when the real API is unavailable.
 */
function generateCommandId(): string {
  if (typeof crypto !== 'undefined' && typeof crypto.randomUUID === 'function') {
    return crypto.randomUUID();
  }
  return `cmd-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
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
  const diagnostics = data.targetFollowDiagnostics;
  return (
    typeof gps === 'object' &&
    gps !== null &&
    typeof gps.lat === 'number' &&
    typeof gps.lng === 'number' &&
    typeof data.altitudeM === 'number' &&
    typeof data.headingDeg === 'number' &&
    typeof data.speedMps === 'number' &&
    typeof data.batteryPercent === 'number' &&
    typeof data.state === 'string' &&
    (data.targetPatrolRegionId === null || typeof data.targetPatrolRegionId === 'string') &&
    (diagnostics === undefined || diagnostics === null || isTargetFollowDiagnostics(diagnostics))
  );
}

function isTargetFollowDiagnostics(value: unknown): value is TargetFollowDiagnostics {
  if (typeof value !== 'object' || value === null) return false;
  const diagnostics = value as Record<string, unknown>;
  const error = diagnostics.imageCenterError as Record<string, unknown> | undefined;
  const confirmation = diagnostics.confirmation as Record<string, unknown> | undefined;
  const velocity = diagnostics.estimatedImageVelocity as Record<string, unknown> | undefined;
  const pursuit = diagnostics.pursuit as Record<string, unknown> | undefined;
  const validVector = (value: unknown) => {
    if (typeof value !== 'object' || value === null) return false;
    const vector = value as Record<string, unknown>;
    return ['x', 'y', 'z'].every((axis) => typeof vector[axis] === 'number' && Number.isFinite(vector[axis]));
  };
  return (
    (pursuit === undefined || (typeof pursuit === 'object' && pursuit !== null &&
      typeof pursuit.state === 'string' &&
      (pursuit.constraint === null || typeof pursuit.constraint === 'string') &&
      validVector(pursuit.velocityCommand) && validVector(pursuit.estimatedMotion))) &&
    typeof diagnostics.flightState === 'string' &&
    (diagnostics.trackerId === null || typeof diagnostics.trackerId === 'string') &&
    (diagnostics.detectionConfidence === null || typeof diagnostics.detectionConfidence === 'number') &&
    typeof error === 'object' && error !== null &&
    typeof error.x === 'number' && typeof error.y === 'number' &&
    typeof confirmation === 'object' && confirmation !== null &&
    typeof confirmation.hits === 'number' &&
    typeof confirmation.requiredHits === 'number' &&
    typeof confirmation.progress === 'number' &&
    typeof velocity === 'object' && velocity !== null &&
    typeof velocity.x === 'number' && typeof velocity.y === 'number' &&
    typeof velocity.magnitude === 'number' &&
    (velocity.directionDeg === null || typeof velocity.directionDeg === 'number') &&
    typeof diagnostics.boundingBoxScale === 'number' &&
    typeof diagnostics.boundingBoxAreaRatio === 'number' &&
    typeof diagnostics.standoffError === 'number' &&
    typeof diagnostics.targetLostDurationS === 'number' &&
    typeof diagnostics.patrolInterrupted === 'boolean' &&
    (diagnostics.patrolResumeState === null || typeof diagnostics.patrolResumeState === 'string') &&
    typeof diagnostics.patrolResumeStatus === 'string'
  );
}

function isDetectionsMessage(value: unknown): value is DetectionsMessage {
  if (typeof value !== 'object' || value === null) return false;
  const message = value as Record<string, unknown>;
  if (message.type !== 'detections') return false;
  if (typeof message.frameId !== 'number') return false;
  if (
    message.timestampMs !== undefined &&
    (typeof message.timestampMs !== 'number' || !Number.isFinite(message.timestampMs))
  ) {
    return false;
  }
  if (
    typeof message.detectionAgeMs !== 'number' ||
    !Number.isFinite(message.detectionAgeMs) ||
    message.detectionAgeMs < 0
  ) {
    return false;
  }
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

const MISSION_STATES = new Set<MissionState>([
  'ON_GROUND',
  'TAKING_OFF',
  'PATROL',
  'TRANSIT_TO_ROI',
  'ROI_PATROL',
  'RTH',
  'LANDING',
]);

function isMissionState(value: unknown): value is MissionState {
  return typeof value === 'string' && MISSION_STATES.has(value as MissionState);
}

function isMissionStatusMessage(value: unknown): value is MissionStatusMessage {
  if (typeof value !== 'object' || value === null) return false;
  const message = value as Record<string, unknown>;
  if (
    message.type !== 'mission_status' ||
    !isMissionState(message.state) ||
    typeof message.droneConnected !== 'boolean'
  ) return false;
  if (message.activeRoi !== null) {
    if (typeof message.activeRoi !== 'object') return false;
    const roi = message.activeRoi as Record<string, unknown>;
    const center = roi.center as Record<string, unknown> | undefined;
    if (
      typeof center !== 'object' || center === null ||
      typeof center.lat !== 'number' || typeof center.lon !== 'number' ||
      typeof roi.radiusM !== 'number'
    ) return false;
  }
  const patrol = message.patrol as Record<string, unknown> | undefined;
  const batteryRth = message.batteryRth as Record<string, unknown> | undefined;
  const areaTypes = patrol?.areaTypes;
  const hasValidAreaTypes =
    areaTypes === undefined ||
    (Array.isArray(areaTypes) && areaTypes.every((entry) => {
      if (typeof entry !== 'object' || entry === null) return false;
      const typed = entry as Record<string, unknown>;
      return (
        typeof typed.regionId === 'string' &&
        (typed.areaType === 'PATROL' || typed.areaType === 'IGNORE' || typed.areaType === 'NO_FLY')
      );
    }));
  return (
    typeof patrol === 'object' && patrol !== null && Array.isArray(patrol.priorities) &&
    hasValidAreaTypes &&
    typeof batteryRth === 'object' && batteryRth !== null &&
    (batteryRth.batteryPercent === null || typeof batteryRth.batteryPercent === 'number') &&
    typeof batteryRth.lowBattery === 'boolean' &&
    typeof batteryRth.rthActive === 'boolean' &&
    typeof batteryRth.safetyRthActive === 'boolean' &&
    (batteryRth.rthReason === null || typeof batteryRth.rthReason === 'string')
  );
}

function createRealtimeServerConnection(): RealtimeServerConnection {
  const telemetryListeners = new Set<(message: TelemetryMessage) => void>();
  const detectionListeners = new Set<(message: DetectionsMessage) => void>();
  const missionStateListeners = new Set<(state: MissionState) => void>();
  const missionStatusListeners = new Set<(status: MissionStatusMessage) => void>();
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
      case 'mission_state': {
        const state = message.state;
        if (!isMissionState(state)) {
          console.warn('[realtimeServerConnection] mission state failed shape check, ignoring', message);
          return;
        }
        missionStateListeners.forEach((listener) => listener(state));
        return;
      }
      case 'mission_status': {
        if (!isMissionStatusMessage(message)) {
          console.warn('[realtimeServerConnection] mission status failed shape check, ignoring', message);
          return;
        }
        missionStatusListeners.forEach((listener) => listener(message));
        missionStateListeners.forEach((listener) => listener(message.state));
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
    subscribeMissionState(listener) {
      missionStateListeners.add(listener);
      return () => missionStateListeners.delete(listener);
    },
    subscribeMissionStatus(listener) {
      missionStatusListeners.add(listener);
      return () => missionStatusListeners.delete(listener);
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

        const commandId = generateCommandId();
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
