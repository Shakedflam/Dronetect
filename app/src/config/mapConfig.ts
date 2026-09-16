import type { PointTuple } from 'leaflet';
import type { Coordinates } from '../types/drone';

/**
 * Static reference point shown on the map as "HOME". This is not live
 * telemetry — the realtime server's telemetry stream never reports a home
 * location — it's a fixed display anchor matching the operational home
 * point the backend uses internally for local -> WGS84 conversion
 * (realtime server's DEFAULT_HOME, ~32.0853, 34.7818). If the drone's actual
 * operational home ever becomes configurable or reportable by the
 * realtime server, source it from there instead of this constant.
 */
export const HOME_REFERENCE: Coordinates = { lat: 32.0853, lng: 34.7818 };

/**
 * Radius, in meters, of the purely-visual reference ring drawn around Home
 * on the map. Display-only for now — no mission logic reads or enforces it.
 */
export const HOME_RADIUS_METERS = 2500;

/** Initial map zoom level on first render, before any auto-follow occurs. */
export const DEFAULT_ZOOM = 15;

/**
 * Zoom cap for auto-follow. Deliberately higher than DEFAULT_ZOOM so that
 * when the drone is close to home the view zooms in and keeps the two
 * markers visually separated, instead of sitting at a fixed zoom that
 * crowds them together.
 */
export const MAX_FOLLOW_ZOOM = 18;

/** Padding, in pixels, kept between the drone/home bounds and the map edge. */
export const FOLLOW_PADDING: PointTuple = [64, 64];

/** Radius, in meters, a newly-placed ROI starts with before it's resized. */
export const DEFAULT_ROI_RADIUS_METERS = 300;

/** Smallest radius, in meters, the ROI drag handle / numeric input allow. */
export const MIN_ROI_RADIUS_METERS = 25;

/** Largest radius, in meters, the ROI drag handle / numeric input allow. */
export const MAX_ROI_RADIUS_METERS = 5000;
