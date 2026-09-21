import { useEffect, useRef, useState, type FormEvent } from 'react';
import L from 'leaflet';
import type { OperatingConfig } from '../../types/operatingConfig';
import { parsePositiveDecimal } from '../../utils/parsePositiveDecimal';
import { droneCommandService } from '../../services/droneCommandService';
import { patrolService } from '../../services/patrolService';
import { LEGAL_MAX_CRUISE_ALTITUDE_M, LEGAL_MAX_FLIGHT_RADIUS_M } from '../../config/mapConfig';
import styles from './FlightParametersLayer.module.css';

interface FlightParametersLayerProps {
  config: OperatingConfig;
  pending: boolean;
  /** Height in px of whatever sits above this panel (the ROI panel), so the two never overlap. */
  topOffsetPx: number;
}

/**
 * Always-available top-left control for changing max flight radius and
 * cruise altitude mid-mission. Stacks directly beneath the ROI panel via
 * `topOffsetPx`, reported by RoiLayer through a ResizeObserver.
 */
export function FlightParametersLayer({ config, pending, topOffsetPx }: FlightParametersLayerProps) {
  const [expanded, setExpanded] = useState(false);
  const [radius, setRadius] = useState(String(config.maxFlightRadiusM));
  const [altitude, setAltitude] = useState(String(config.cruiseAltitudeM));
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [cappedNote, setCappedNote] = useState<string | null>(null);
  const [confirmationNote, setConfirmationNote] = useState<string | null>(null);
  const panelRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const panel = panelRef.current;
    if (!panel) return;
    L.DomEvent.disableClickPropagation(panel);
    L.DomEvent.disableScrollPropagation(panel);
  }, []);

  // Only re-seed from the authoritative config while collapsed, so an
  // in-progress edit is never clobbered by a status update arriving mid-type.
  useEffect(() => {
    if (expanded) return;
    setRadius(String(config.maxFlightRadiusM));
    setAltitude(String(config.cruiseAltitudeM));
    setCappedNote(null);
    setConfirmationNote(null);
    setError(null);
  }, [config, expanded]);

  async function handleSubmit(event: FormEvent<HTMLFormElement>): Promise<void> {
    event.preventDefault();
    const parsedRadius = parsePositiveDecimal(radius);
    const parsedAltitude = parsePositiveDecimal(altitude);
    if (parsedRadius === null || parsedAltitude === null) {
      setError('Enter positive numbers in meters for both fields.');
      setCappedNote(null);
      setConfirmationNote(null);
      return;
    }

    const maxFlightRadiusM = Math.min(parsedRadius, LEGAL_MAX_FLIGHT_RADIUS_M);
    const cruiseAltitudeM = Math.min(parsedAltitude, LEGAL_MAX_CRUISE_ALTITUDE_M);
    const cappedFields = [
      ...(parsedRadius > LEGAL_MAX_FLIGHT_RADIUS_M ? [`radius to ${LEGAL_MAX_FLIGHT_RADIUS_M} m`] : []),
      ...(parsedAltitude > LEGAL_MAX_CRUISE_ALTITUDE_M ? [`altitude to ${LEGAL_MAX_CRUISE_ALTITUDE_M} m`] : []),
    ];
    if (cappedFields.length > 0) {
      setRadius(String(maxFlightRadiusM));
      setAltitude(String(cruiseAltitudeM));
      setCappedNote(`Legal maximum reached — capped ${cappedFields.join(' and ')}.`);
    } else {
      setCappedNote(null);
    }

    setSubmitting(true);
    setError(null);
    setConfirmationNote(null);
    try {
      await droneCommandService.setOperatingConfig({ maxFlightRadiusM, cruiseAltitudeM });
      patrolService.setMaxFlightRadius(maxFlightRadiusM);
      setConfirmationNote(`Radius and altitude are now set to ${maxFlightRadiusM} m and ${cruiseAltitudeM} m.`);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : 'Configuration was rejected.');
    } finally {
      setSubmitting(false);
    }
  }

  return (
    <div
      ref={panelRef}
      className={styles.panel}
      style={{ top: `calc(var(--space-md) + ${topOffsetPx}px)` }}
    >
      <button
        type="button"
        className={`${styles.toggleButton} ${expanded ? styles.toggleButtonActive : ''}`}
        onClick={() => setExpanded((value) => !value)}
      >
        Flight Parameters
      </button>
      {expanded && (
        <form className={styles.form} onSubmit={(event) => void handleSubmit(event)}>
          <label className={styles.field}>
            Radius (m)
            <input
              inputMode="decimal"
              value={radius}
              onChange={(event) => setRadius(event.target.value)}
              disabled={submitting}
              aria-invalid={error !== null}
            />
          </label>
          <label className={styles.field}>
            Altitude (m)
            <input
              inputMode="decimal"
              value={altitude}
              onChange={(event) => setAltitude(event.target.value)}
              disabled={submitting}
              aria-invalid={error !== null}
            />
          </label>
          <button type="submit" className={styles.applyButton} disabled={submitting}>
            {submitting ? 'Applying…' : 'Apply'}
          </button>
          {pending && (
            <div className={styles.pendingNote}>
              Queued — will apply once the current maneuver settles.
            </div>
          )}
          {cappedNote && <div className={styles.cappedNote}>{cappedNote}</div>}
          {!pending && confirmationNote && (
            <div className={styles.confirmationNote}>{confirmationNote}</div>
          )}
          {error && <div className={styles.error} role="alert">{error}</div>}
        </form>
      )}
    </div>
  );
}
