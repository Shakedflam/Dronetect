const DEFAULT_REALTIME_SERVER_ORIGIN = 'ws://127.0.0.1:8765';

/**
 * Resolve the shared realtime origin once for browser and packaged builds.
 * The value is an origin (not a path); individual transports append their
 * existing protocol endpoints below.
 */
export function realtimeServerOrigin(): string {
  const configured = import.meta.env.VITE_REALTIME_SERVER_WS_URL?.trim();
  const origin = configured || DEFAULT_REALTIME_SERVER_ORIGIN;

  if (!origin.startsWith('ws://') && !origin.startsWith('wss://')) {
    throw new Error('VITE_REALTIME_SERVER_WS_URL must start with ws:// or wss://');
  }

  return origin.replace(/\/+$/, '');
}

export function realtimeServerUrl(path: `/${string}`, role: string): string {
  return `${realtimeServerOrigin()}${path}?role=${encodeURIComponent(role)}`;
}
