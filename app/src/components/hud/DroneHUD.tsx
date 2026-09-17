import { useLocation } from '../../hooks/useLocation';
import { useTelemetry } from '../../hooks/useTelemetry';
import { useMissionState } from '../../hooks/useMissionState';
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

/** Same underscore-to-space formatting as formatState, applied to mission state. */
function formatMissionState(state: string): string {
  return state.replace(/_/g, ' ');
}

function formatEta(etaSeconds: number): string {
  const minutes = Math.floor(etaSeconds / 60);
  const seconds = etaSeconds % 60;
  return `${minutes}:${seconds.toString().padStart(2, '0')}`;
}

function formatSigned(value: number, digits = 3): string {
  return `${value >= 0 ? '+' : ''}${value.toFixed(digits)}`;
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
  const missionState = useMissionState();

  const isReady = position !== null && telemetry !== null;
  const diagnostics = telemetry?.targetFollowDiagnostics ?? null;

  return (
    <div className={styles.panel}>
      {isReady ? (
        <>
          <HudRow label="SPEED" value={formatSpeed(telemetry.speedMps)} />
          <HudRow label="ALTITUDE" value={formatAltitude(telemetry.altitudeM)} />
          <HudRow label="BATTERY" value={formatBattery(telemetry.batteryPercent)} />
          <HudRow
            label="STATE"
            value={formatState(diagnostics?.flightState ?? telemetry.state)}
          />
          <HudRow label="MISSION" value={formatMissionState(missionState)} />
          <HudRow label="LATITUDE" value={formatCoordinate(position.lat)} />
          <HudRow label="LONGITUDE" value={formatCoordinate(position.lng)} />
          {telemetry.destinationName !== undefined && (
            <HudRow label="DESTINATION" value={telemetry.destinationName} />
          )}
          {telemetry.etaSeconds !== undefined && (
            <HudRow label="ETA" value={formatEta(telemetry.etaSeconds)} />
          )}
          {diagnostics !== null && diagnostics.trackerId !== null && (
            <div className={styles.diagnostics}>
              <HudRow label="TRACK" value={diagnostics.trackerId} />
              {diagnostics.pursuit && <>
                <HudRow label="PURSUIT" value={diagnostics.pursuit.state} />
                <HudRow label="CMD XYZ m/s" value={[diagnostics.pursuit.velocityCommand.x, diagnostics.pursuit.velocityCommand.y, diagnostics.pursuit.velocityCommand.z].map((v) => formatSigned(v, 2)).join(' / ')} />
                <HudRow label="MOTION CUE XY" value={`${formatSigned(diagnostics.pursuit.estimatedMotion.x)} / ${formatSigned(diagnostics.pursuit.estimatedMotion.y)}`} />
                {diagnostics.pursuit.constraint && <HudRow label="CONSTRAINT" value={diagnostics.pursuit.constraint} />}
              </>}
              <HudRow
                label="CONFIDENCE"
                value={diagnostics.detectionConfidence === null
                  ? '—'
                  : `${(diagnostics.detectionConfidence * 100).toFixed(1)}%`}
              />
              <HudRow
                label="CENTER ERR"
                value={`${formatSigned(diagnostics.imageCenterError.x)} / ${formatSigned(diagnostics.imageCenterError.y)}`}
              />
              <HudRow
                label="CONFIRM"
                value={`${diagnostics.confirmation.hits}/${diagnostics.confirmation.requiredHits} (${(diagnostics.confirmation.progress * 100).toFixed(0)}%)`}
              />
              <HudRow
                label="IMG VELOCITY"
                value={`${formatSigned(diagnostics.estimatedImageVelocity.x)} / ${formatSigned(diagnostics.estimatedImageVelocity.y)} · ${diagnostics.estimatedImageVelocity.directionDeg === null ? '—' : `${diagnostics.estimatedImageVelocity.directionDeg.toFixed(0)}°`}`}
              />
              <HudRow
                label="BOX / STANDOFF"
                value={`${diagnostics.boundingBoxScale.toFixed(3)} / ${formatSigned(diagnostics.standoffError, 4)}`}
              />
              <HudRow
                label="TARGET LOST"
                value={`${diagnostics.targetLostDurationS.toFixed(2)} s`}
              />
              <HudRow
                label="PATROL"
                value={diagnostics.patrolInterrupted
                  ? `INTERRUPTED → ${diagnostics.patrolResumeState ?? '—'}`
                  : diagnostics.patrolResumeStatus.toUpperCase()}
              />
            </div>
          )}
        </>
      ) : (
        <div className={styles.connecting}>CONNECTING…</div>
      )}
    </div>
  );
}
