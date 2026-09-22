/// <reference types="vite/client" />

interface ImportMetaEnv {
  /** WebSocket origin of the Drontect edge realtime server, e.g. "ws://127.0.0.1:8765". */
  readonly VITE_REALTIME_SERVER_WS_URL: string;
  /** Comma-separated STUN/TURN URLs for the WebRTC video viewer's ICE config, e.g. "stun:stun.l.google.com:19302". Optional -- falls back to a public STUN server. */
  readonly VITE_WEBRTC_STUN_URLS?: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
