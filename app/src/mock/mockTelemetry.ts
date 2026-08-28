import type { TelemetryData } from '../types/drone';

/** Same cycle length used by mockLocation, so the ETA countdown feels consistent. */
const CYCLE_DURATION_MS = 90_000;
const CRUISE_SPEED_MPS = 12;
const CRUISE_ALTITUDE_M = 45;
const DESTINATION_NAME = 'Waypoint Alpha';
const HOME_NAME = 'Home';

export function getMockTelemetry(elapsedMs: number): TelemetryData {
  const phase = (elapsedMs % CYCLE_DURATION_MS) / CYCLE_DURATION_MS;
  const outbound = phase < 0.5;
  const legPhase = outbound ? phase / 0.5 : (phase - 0.5) / 0.5;

  const speedJitter = Math.sin(elapsedMs / 1500) * 1.2;
  const altitudeJitter = Math.sin(elapsedMs / 2200) * 2;
  const legRemainingMs = (1 - legPhase) * (CYCLE_DURATION_MS / 2);

  return {
    speedMps: Math.max(0, CRUISE_SPEED_MPS + speedJitter),
    altitudeM: CRUISE_ALTITUDE_M + altitudeJitter,
    destinationName: outbound ? DESTINATION_NAME : HOME_NAME,
    etaSeconds: Math.round(legRemainingMs / 1000),
  };
}
