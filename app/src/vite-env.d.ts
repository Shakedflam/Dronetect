/// <reference types="vite/client" />

interface ImportMetaEnv {
  /** WebSocket origin of the Drontect edge gateway, e.g. "ws://127.0.0.1:8765". */
  readonly VITE_GATEWAY_WS_URL: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
