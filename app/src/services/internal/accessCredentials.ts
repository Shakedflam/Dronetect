import { Capacitor, registerPlugin } from '@capacitor/core';
import { realtimeServerOrigin } from './runtimeConfig';

export interface AccessKeys { operator: string; viewer: string }
const STORAGE_KEY = `dronetect.sessionAccessKeys:${realtimeServerOrigin()}`;
const MANUAL_KEY = 'dronetect.manualAccessKeys';
const VALID_KEY = /^[a-f0-9]{64}$/;
let keys: AccessKeys | null = null;

export function localDevelopmentAccess(): boolean {
  const host = new URL(realtimeServerOrigin()).hostname;
  return import.meta.env.VITE_ALLOW_INSECURE_LOCAL === '1' &&
    ['127.0.0.1', '[::1]'].includes(host);
}

export function validAccessKeys(value: AccessKeys): boolean {
  return VALID_KEY.test(value.operator) && VALID_KEY.test(value.viewer) && value.operator !== value.viewer;
}

export async function restoreAccessKeys(): Promise<boolean> {
  if (localDevelopmentAccess()) return true;
  try {
    const value = Capacitor.isNativePlatform() && sessionStorage.getItem(MANUAL_KEY) !== '1'
      ? await registerPlugin<{ getServerCredentials(): Promise<AccessKeys> }>('DjiTelemetry').getServerCredentials()
      : JSON.parse(sessionStorage.getItem(STORAGE_KEY) || 'null') as AccessKeys | null;
    if (value && validAccessKeys(value)) { keys = value; return true; }
  } catch { /* Missing credentials leave the access form visible. */ }
  return false;
}

export function setAccessKeys(value: AccessKeys): void {
  if (!validAccessKeys(value)) throw new Error('Enter two different 64-character access keys.');
  keys = value;
  sessionStorage.setItem(STORAGE_KEY, JSON.stringify(value));
}

export function clearAccessKeys(): void {
  keys = null;
  sessionStorage.removeItem(STORAGE_KEY);
  sessionStorage.setItem(MANUAL_KEY, '1');
}

export function accessProtocols(role: keyof AccessKeys): string[] {
  const url = new URL(realtimeServerOrigin());
  if (url.protocol !== 'wss:' && !['127.0.0.1', '[::1]'].includes(url.hostname)) {
    throw new Error('Remote connections require WSS.');
  }
  if (localDevelopmentAccess()) return [];
  if (!keys) throw new Error('Server access keys are required.');
  // Browsers cannot set Authorization headers. A handshake header carries the
  // credential without putting it in the URL, bundle, or application messages.
  return [`dronetect-auth.${keys[role]}`];
}
