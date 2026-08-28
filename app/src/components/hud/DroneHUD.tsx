import { useLocation } from '../../hooks/useLocation';
import { useTelemetry } from '../../hooks/useTelemetry';
import styles from './DroneHUD.module.css';

function formatCoordinate(value: number): string {
  return value.toFixed(5);
}

function formatSpeed(speedMps: number): string {
  return `${speedMps.toFixed(1)} m/s`;
}

function formatAltitude(altitudeM: number): string {
  return `${altitudeM.toFixed(0)} m`;
}

function formatBattery(percent: number): string {
  return `${Math.round(percent)}%`;
}

function formatState(state: string): string {
  return state.replace(/_/g, ' ');
}

function formatEta(etaSeconds: number): string {
  const minutes = Math.floor(etaSeconds / 60);
  const seconds = etaSeconds % 60;
  return `${minutes}:${seconds.toString().padStart(2, '0')}`;
}

interface HudRowProps {
  label: string;
  value: string;
}

function HudRow({ label, value }: HudRowProps) {
  return (
    <div className={styles.row}>
      <span className={styles.label}>{label}</span>
      <span className={styles.value}>{value}</span>
    </div>
  );
}

/**
 * Compact telemetry readout overlaid on the map's top-right corner.
 * Position comes from useLocation() (GPS), everything else from
 * useTelemetry() — both are backed by the live realtime server connection.
 */
export function DroneHUD() {
  const position = useLocation();
  const telemetry = useTelemetry();

  const isReady = position !== null && telemetry !== null;

  return (
    <div className={styles.panel}>
      {isReady ? (
        <>
          <HudRow label="SPEED" value={formatSpeed(telemetry.speedMps)} />
          <HudRow label="ALTITUDE" value={formatAltitude(telemetry.altitudeM)} />
          <HudRow label="BATTERY" value={formatBattery(telemetry.batteryPercent)} />
          <HudRow label="STATE" value={formatState(telemetry.state)} />
          <HudRow label="LATITUDE" value={formatCoordinate(position.lat)} />
          <HudRow label="LONGITUDE" value={formatCoordinate(position.lng)} />
          {telemetry.destinationName !== undefined && (
            <HudRow label="DESTINATION" value={telemetry.destinationName} />
          )}
          {telemetry.etaSeconds !== undefined && (
            <HudRow label="ETA" value={formatEta(telemetry.etaSeconds)} />
          )}
        </>
      ) : (
        <div className={styles.connecting}>CONNECTING…</div>
      )}
    </div>
  );
}
