import type { CapacitorConfig } from '@capacitor/cli';

const config: CapacitorConfig = {
  appId: 'com.dronetect.app',
  appName: 'Dronetect',
  webDir: 'dist',
  android: {
    // Required only for debug APKs that target a ws:// LAN endpoint. Android's
    // release manifest still denies cleartext traffic, so production remains
    // WSS-only.
    allowMixedContent: true,
  },
};

export default config;
