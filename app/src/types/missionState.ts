/**
 * Frontend-only "what is the operator currently commanding" state — distinct
 * from TelemetryData.state (the backend's own flight-state-machine string,
 * e.g. IDLE / FLYING_RANDOMLY / RETURNING_HOME). This is never read from or
 * written to the realtime server; it only reflects which ActionBar button
 * was last pressed. ROI_PATROL is declared for future/backend use but isn't
 * reachable from any button yet — no automatic transitions are implemented.
 */
export type MissionState = 'ON_GROUND' | 'PATROL' | 'TRANSIT_TO_ROI' | 'ROI_PATROL' | 'RTH';

export const DEFAULT_MISSION_STATE: MissionState = 'ON_GROUND';
