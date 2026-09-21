import { useEffect, useState } from 'react';
import startupLogoUrl from '../../../../logos/main_icon.png';
import styles from './StartupSplash.module.css';

interface StartupSplashProps {
  ready: boolean;
}

type SplashPhase = 'visible' | 'fading' | 'hidden';

const MINIMUM_VISIBLE_MS = 800;

/** Startup-only connection gate. Once dismissed, reconnects stay in the app. */
export function StartupSplash({ ready }: StartupSplashProps) {
  const [phase, setPhase] = useState<SplashPhase>('visible');
  const [minimumVisibleElapsed, setMinimumVisibleElapsed] = useState(false);

  useEffect(() => {
    const timeout = window.setTimeout(
      () => setMinimumVisibleElapsed(true),
      MINIMUM_VISIBLE_MS,
    );
    return () => window.clearTimeout(timeout);
  }, []);

  useEffect(() => {
    if (!ready || !minimumVisibleElapsed || phase !== 'visible') return;

    const frame = window.requestAnimationFrame(
      () => setPhase('fading'),
    );
    return () => window.cancelAnimationFrame(frame);
  }, [ready, minimumVisibleElapsed, phase]);

  if (phase === 'hidden') return null;

  return (
    <div
      className={`${styles.splash} ${phase === 'fading' ? styles.fading : ''}`}
      role="status"
      aria-live="polite"
      aria-label={ready ? 'Dronetect is ready' : 'Connecting to Dronetect services'}
      onAnimationEnd={(event) => {
        if (event.target === event.currentTarget && phase === 'fading') {
          setPhase('hidden');
        }
      }}
    >
      <div className={styles.glow} aria-hidden="true" />
      <img
        className={styles.logo}
        src={startupLogoUrl}
        alt="Dronetect"
        draggable={false}
      />
      <div className={styles.connectionState}>
        <span className={ready ? styles.readyDot : styles.connectingDot} />
        <span>{ready ? 'System ready' : 'Connecting to system'}</span>
      </div>
    </div>
  );
}
