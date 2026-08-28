import styles from './CompassStrip.module.css';

const CARDINALS = [
  { deg: 0, label: 'N' },
  { deg: 45, label: 'NE' },
  { deg: 90, label: 'E' },
  { deg: 135, label: 'SE' },
  { deg: 180, label: 'S' },
  { deg: 225, label: 'SW' },
  { deg: 270, label: 'W' },
  { deg: 315, label: 'NW' },
] as const;

const TICK_STEP_DEG = 15;
const PX_PER_DEG = 6;

interface Tick {
  deg: number;
  label: string | null;
}

/**
 * Three copies of the 0-360 tick set, offset by a full turn each way, so the
 * tape has ticks to slide in from both directions as heading wraps past
 * north instead of running out at the edges.
 */
function buildTicks(): Tick[] {
  const ticks: Tick[] = [];
  for (let cycle = -1; cycle <= 1; cycle++) {
    for (let deg = 0; deg < 360; deg += TICK_STEP_DEG) {
      const cardinal = CARDINALS.find((c) => c.deg === deg);
      ticks.push({ deg: deg + cycle * 360, label: cardinal ? cardinal.label : null });
    }
  }
  return ticks;
}

const TICKS = buildTicks();

function nearestCardinal(normalizedDeg: number): string {
  const index = Math.round(normalizedDeg / 45) % 8;
  return CARDINALS[index].label;
}

interface CompassStripProps {
  headingDeg: number;
}

/**
 * Heading tape across the top of the camera feed. The tape slides under a
 * fixed center marker so the drone/camera's current heading is always
 * legible at a glance, the way a real FPV/GCS HUD compass works.
 */
export function CompassStrip({ headingDeg }: CompassStripProps) {
  const normalized = ((headingDeg % 360) + 360) % 360;

  return (
    <div className={styles.wrapper}>
      <div className={styles.viewport}>
        <div
          className={styles.tape}
          style={{ transform: `translateX(${-normalized * PX_PER_DEG}px)` }}
        >
          {TICKS.map((tick) => (
            <div
              key={tick.deg}
              className={tick.label ? styles.majorTick : styles.minorTick}
              style={{ left: `${tick.deg * PX_PER_DEG}px` }}
            >
              {tick.label && <span className={styles.tickLabel}>{tick.label}</span>}
            </div>
          ))}
        </div>
        <div className={styles.centerMarker} />
      </div>
      <div className={styles.readout}>
        {nearestCardinal(normalized)} {Math.round(normalized).toString().padStart(3, '0')}°
      </div>
    </div>
  );
}
