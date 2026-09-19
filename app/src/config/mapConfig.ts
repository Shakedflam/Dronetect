import type { PointTuple } from 'leaflet';
import type { Coordinates } from '../types/drone';
import sharedConfig from '../../../config/shared.json';

/**
 * Static reference point shown on the map as "HOME". This is not live
 * telemetry — the realtime server's telemetry stream never reports a home
 * location — it's a fixed display anchor matching the operational home
 * point the backend uses internally for local -> WGS84 conversion
 * (services/gps.py's DEFAULT_HOME). Both are read from the same
 * config/shared.json at the repo root (see config/shared_config.py for why:
 * only region IDs and coordinates cross the wire, never geometry, so the two
 * sides must compute identical geometry from identical inputs). If the
 * drone's actual operational home ever becomes configurable or reportable by
 * the realtime server, source it from there instead of this constant.
 */
export const HOME_REFERENCE: Coordinates = { lat: sharedConfig.home.lat, lng: sharedConfig.home.lng };

/**
 * Radius, in meters, of the Home operational range: the ring drawn around
 * Home on the map, the range ROI fill is clipped to, and — via
 * patrolService.ts's `generatePatrolRegions` call — the geometry backing the
 * patrol grid's polygons and priority badges. Read from config/shared.json,
 * the same file services/patrol_regions.py's HOME_RADIUS_M reads, since that
 * value is what the backend patrol planner and pursuit geofence actually fly
 * against: only region IDs (e.g. "SW-1") are sent over the wire, never
 * geometry, so a mismatch here makes the map's polygons/badges point at
 * different real-world places than where the drone actually treats that
 * region ID as being.
 */
export const HOME_RADIUS_METERS = sharedConfig.homeRadiusM;

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

/**
 * Number of concentric equal-area distance bands each patrol direction is
 * divided into. Read from config/shared.json, matching services/
 * patrol_regions.py's PATROL_BAND_COUNT for the same reason as
 * HOME_RADIUS_METERS above.
 */
export const PATROL_BAND_COUNT = sharedConfig.patrolBandCount;
