import { useEffect, useRef, useState, type ChangeEvent } from 'react';
import {
  Circle,
  MapContainer,
  Marker,
  Polygon,
  Polyline,
  Tooltip,
  TileLayer,
  useMap,
  useMapEvents,
} from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import type { Coordinates, DronePosition } from '../../types/drone';
import type { RoiCenter } from '../../types/roi';
import { useLocation } from '../../hooks/useLocation';
import { useMissionState } from '../../hooks/useMissionState';
import { useRoi } from '../../hooks/useRoi';
import { missionStateService } from '../../services/missionStateService';
import { roiService } from '../../services/roiService';
import {
  DEFAULT_ROI_RADIUS_METERS,
  DEFAULT_ZOOM,
  FOLLOW_PADDING,
  HOME_RADIUS_METERS,
  HOME_REFERENCE,
  MAX_FOLLOW_ZOOM,
  MAX_ROI_RADIUS_METERS,
  MIN_ROI_RADIUS_METERS,
} from '../../config/mapConfig';
import { bearingDeg, euclideanDistanceMeters } from '../../utils/geo';
import { clipRoiToHomeRange } from '../../utils/roiClipping';
import { PatrolLayer } from './PatrolLayer';
import styles from './MapView.module.css';

const droneIcon = L.divIcon({
  className: styles.droneIcon,
  html: '<span class="' + styles.droneIconPulse + '"></span><span class="' + styles.droneIconDot + '"></span>',
  iconSize: [22, 22],
  iconAnchor: [11, 11],
});

const homeIcon = L.divIcon({
  className: styles.homeIcon,
  html: '<span class="' + styles.homeIconDot + '">⌂</span><span class="' + styles.homeIconLabel + '">HOME</span>',
  iconSize: [22, 22],
  iconAnchor: [11, 11],
});

/** Fraction along the drone->home line where the direction arrowhead sits. */
const HOME_ARROW_T = 0.5;

function createHomeArrowIcon(bearing: number): L.DivIcon {
  return L.divIcon({
    className: styles.homeArrowIcon,
    html: '<span class="' + styles.homeArrowGlyph + '" style="transform: rotate(' + bearing + 'deg)">&#10148;</span>',
    iconSize: [18, 18],
    iconAnchor: [9, 9],
  });
}

function formatDistance(meters: number): string {
  if (meters >= 1000) return `${(meters / 1000).toFixed(2)} km`;
  return `${Math.round(meters)} m`;
}

/** Duration of each auto-follow flight, in seconds. */
const FOLLOW_FLY_DURATION_S = 0.8;
/**
 * Minimum time between flyToBounds calls, in ms. Telemetry (and therefore
 * `position`) can update far faster than the flight animation takes to
 * play out — without this, every update would restart the in-flight
 * animation, so the map would visibly judder instead of smoothly tracking.
 * Kept just under the flight duration so a flight mostly finishes before
 * the next one starts.
 */
const FOLLOW_REFLY_MIN_INTERVAL_MS = FOLLOW_FLY_DURATION_S * 1000 - 100;

/**
 * Keeps both the drone and home in view as the drone moves, zooming out as
 * needed rather than just centering on the drone — otherwise the drone
 * quickly pans home (and the home-direction line) off-screen. Backs off
 * once the operator manually drags the map; a "Recenter" button hands
 * control back.
 */
function LiveTrackingControl({ position, home }: { position: DronePosition; home: Coordinates }) {
  const map = useMap();
  const [autoFollow, setAutoFollow] = useState(true);
  const autoFollowRef = useRef(true);
  const hasCenteredOnce = useRef(false);
  const lastFlyAt = useRef(0);

  useEffect(() => {
    const stopFollowing = () => {
      // Cancel any in-flight recenter animation immediately, rather than
      // waiting for it to finish — otherwise the constant pane movement
      // from a just-started flyToBounds can eat into the few pixels of
      // mouse movement Leaflet needs to ever recognize a drag as a drag
      // (dragstart wouldn't fire at all), which is what makes panning
      // while the drone is actively moving feel like the map "fights
      // back" instead of staying where it's dragged. Listening on
      // mousedown/touchstart — the instant the user touches the map,
      // before that threshold is even evaluated — instead of only
      // dragstart closes that race.
      map.stop();
      autoFollowRef.current = false;
      setAutoFollow(false);
    };
    map.on('mousedown', stopFollowing);
    map.on('touchstart', stopFollowing);
    map.on('dragstart', stopFollowing);
    map.on('patrolfocus', stopFollowing);
    return () => {
      map.off('mousedown', stopFollowing);
      map.off('touchstart', stopFollowing);
      map.off('dragstart', stopFollowing);
      map.off('patrolfocus', stopFollowing);
    };
  }, [map]);

  useEffect(() => {
    const bounds = L.latLngBounds([position.lat, position.lng], [home.lat, home.lng]);
    if (!hasCenteredOnce.current) {
      map.fitBounds(bounds, { padding: FOLLOW_PADDING, maxZoom: MAX_FOLLOW_ZOOM, animate: false });
      hasCenteredOnce.current = true;
      lastFlyAt.current = performance.now();
      return;
    }
    if (!autoFollowRef.current) return;
    const now = performance.now();
    if (now - lastFlyAt.current < FOLLOW_REFLY_MIN_INTERVAL_MS) return;
    lastFlyAt.current = now;
    map.flyToBounds(bounds, { padding: FOLLOW_PADDING, maxZoom: MAX_FOLLOW_ZOOM, duration: FOLLOW_FLY_DURATION_S });
  }, [position, home, autoFollow, map]);

  if (autoFollow) return null;

  return (
    <button
      type="button"
      className={styles.recenterButton}
        onClick={() => {
          // Bypass the throttle so Recenter reacts immediately.
          lastFlyAt.current = 0;
          autoFollowRef.current = true;
          setAutoFollow(true);
        }}
    >
      Recenter
    </button>
  );
}

/** Clearly distinct from the blue home range, amber home line, and red/green markers. */
const ROI_COLOR = '#c65ce0';
/**
 * Stroke weight of the invisible circle stacked on the visible ROI outline,
 * purely to widen the draggable hit area along the boundary — the visible
 * outline itself stays thin. A near-zero (not exactly zero) opacity keeps
 * it invisible while still being "painted" enough for pointer-events hit
 * testing in every browser.
 */
const ROI_BOUNDARY_HIT_WEIGHT = 28;
const ROI_BOUNDARY_HIT_OPACITY = 0.001;
/** Native DOM click-count threshold (`MouseEvent.detail`) treated as a triple-tap/triple-click. */
const ROI_TRIPLE_CLICK_DETAIL = 3;

const roiCenterIcon = L.divIcon({
  className: styles.roiCenterIcon,
  html: '<span class="' + styles.roiCenterDot + '"></span>',
  iconSize: [14, 14],
  iconAnchor: [7, 7],
});

function clampRoiRadius(radiusM: number): number {
  return Math.min(MAX_ROI_RADIUS_METERS, Math.max(MIN_ROI_RADIUS_METERS, radiusM));
}

function toCoordinates(center: RoiCenter): Coordinates {
  return { lat: center.lat, lng: center.lon };
}

/**
 * Frontend-only "define a region of interest" tool: the operator places and
 * resizes a circle on the map. Purely visual for now — no mission logic
 * reads it. State lives in `roiService`, not here, so a future non-UI
 * source (another machine reporting an ROI over some future connection)
 * can call `roiService.setRoi(...)` directly and every subscriber —
 * including this layer — picks it up the same way a click here does; this
 * component never simulates or requires a map click to update the ROI.
 */
function RoiLayer({ home, homeRadiusM }: { home: Coordinates; homeRadiusM: number }) {
  const roi = useRoi();
  const missionState = useMissionState();
  const [defining, setDefining] = useState(false);
  const panelRef = useRef<HTMLDivElement>(null);
  // Shared by both the boundary (resize) and interior (move) drag gestures
  // below — only one can ever be active at a time, so one ref is enough.
  const stopDragRef = useRef<(() => void) | null>(null);

  useEffect(() => {
    const panel = panelRef.current;
    if (!panel) return;
    // Otherwise clicks/scrolls on the panel (buttons, radius input) would
    // bubble to the map and be treated as "place the ROI here".
    L.DomEvent.disableClickPropagation(panel);
    L.DomEvent.disableScrollPropagation(panel);
    return () => stopDragRef.current?.();
  }, []);

  const map = useMapEvents({
    click(event) {
      // A triple tap/click places (or moves) the ROI center regardless of
      // "Define ROI" mode — an always-available shortcut alongside it.
      const isTripleClick = event.originalEvent.detail >= ROI_TRIPLE_CLICK_DETAIL;
      if (!isTripleClick && !defining) return;
      roiService.setRoi({
        center: { lat: event.latlng.lat, lon: event.latlng.lng },
        radiusM: roi?.radiusM ?? DEFAULT_ROI_RADIUS_METERS,
      });
      if (defining) setDefining(false);
    },
  });

  function handleBoundaryMouseDown(event: L.LeafletMouseEvent): void {
    if (!roi) return;
    const roiCenter = roi.center;
    const centerCoords = toCoordinates(roiCenter);
    L.DomEvent.stopPropagation(event.originalEvent);
    stopDragRef.current?.();
    map.dragging.disable();
    let active = true;

    function onMove(moveEvent: L.LeafletMouseEvent): void {
      const radiusM = clampRoiRadius(euclideanDistanceMeters(centerCoords, moveEvent.latlng));
      roiService.setRoi({ center: roiCenter, radiusM });
    }

    function stopDragging(): void {
      if (!active) return;
      active = false;
      map.dragging.enable();
      map.off('mousemove', onMove);
      map.off('mouseup', stopDragging);
      document.removeEventListener('mouseup', stopDragging);
      window.removeEventListener('blur', stopDragging);
      if (stopDragRef.current === stopDragging) {
        stopDragRef.current = null;
      }
    }

    map.on('mousemove', onMove);
    map.on('mouseup', stopDragging);
    document.addEventListener('mouseup', stopDragging);
    window.addEventListener('blur', stopDragging);
    stopDragRef.current = stopDragging;
  }

  /**
   * Drag-to-move: grabbing anywhere inside the circle (not on the boundary
   * ring, which resizes instead — see the interactive circle's render order
   * below) translates the ROI by the same lat/lng delta as the cursor, so
   * the circle tracks the pointer exactly regardless of map projection,
   * the same way dragging any other map object would.
   */
  function handleInteriorMouseDown(event: L.LeafletMouseEvent): void {
    if (!roi) return;
    const roiCenter = roi.center;
    const radiusM = roi.radiusM;
    const grabLatLng = event.latlng;
    L.DomEvent.stopPropagation(event.originalEvent);
    stopDragRef.current?.();
    map.dragging.disable();
    let active = true;

    function onMove(moveEvent: L.LeafletMouseEvent): void {
      const deltaLat = moveEvent.latlng.lat - grabLatLng.lat;
      const deltaLng = moveEvent.latlng.lng - grabLatLng.lng;
      roiService.setRoi({
        center: { lat: roiCenter.lat + deltaLat, lon: roiCenter.lon + deltaLng },
        radiusM,
      });
    }

    function stopDragging(): void {
      if (!active) return;
      active = false;
      map.dragging.enable();
      map.off('mousemove', onMove);
      map.off('mouseup', stopDragging);
      document.removeEventListener('mouseup', stopDragging);
      window.removeEventListener('blur', stopDragging);
      if (stopDragRef.current === stopDragging) {
        stopDragRef.current = null;
      }
    }

    map.on('mousemove', onMove);
    map.on('mouseup', stopDragging);
    document.addEventListener('mouseup', stopDragging);
    window.addEventListener('blur', stopDragging);
    stopDragRef.current = stopDragging;
  }

  function handleRadiusInputChange(event: ChangeEvent<HTMLInputElement>): void {
    if (!roi) return;
    const value = Number(event.target.value);
    if (!Number.isFinite(value)) return;
    roiService.setRoi({ center: roi.center, radiusM: clampRoiRadius(value) });
  }

  function handleClear(): void {
    roiService.clearRoi();
    if (missionState === 'TRANSIT_TO_ROI' || missionState === 'ROI_PATROL') {
      missionStateService.setMissionState('ON_GROUND');
    }
    setDefining(false);
  }

  const roiCenterCoords = roi ? toCoordinates(roi.center) : null;
  const clippedFill =
    roi && roiCenterCoords
      ? clipRoiToHomeRange(home, homeRadiusM, { center: roiCenterCoords, radiusM: roi.radiusM })
      : [];

  return (
    <>
      {roi && roiCenterCoords && (
        <>
          {clippedFill.length > 0 && (
            <Polygon
              positions={clippedFill.map((point): [number, number] => [point.lat, point.lng])}
              interactive={false}
              pathOptions={{ stroke: false, fillColor: ROI_COLOR, fillOpacity: 0.22 }}
            />
          )}
          <Circle
            center={[roiCenterCoords.lat, roiCenterCoords.lng]}
            radius={roi.radiusM}
            interactive={false}
            pathOptions={{ color: ROI_COLOR, weight: 2, opacity: 0.9, fill: false, dashArray: '4 4' }}
          />
          {/* Invisible fill covering the circle's interior: grabbing anywhere
              in here (that isn't the boundary ring on top of it, below)
              drags the whole ROI to a new location. Rendered before the
              boundary-hit circle so that one — a thin ring hugging the
              actual edge — wins the hit test near the boundary, leaving
              this one to catch everything strictly inside it. */}
          <Circle
            center={[roiCenterCoords.lat, roiCenterCoords.lng]}
            radius={roi.radiusM}
            interactive
            eventHandlers={{ mousedown: handleInteriorMouseDown }}
            pathOptions={{
              stroke: false,
              fill: true,
              fillOpacity: ROI_BOUNDARY_HIT_OPACITY,
              className: styles.roiMoveHandle,
            }}
          />
          {/* Invisible, wider-stroke twin of the outline circle: any point
              along the circumference can be grabbed and dragged to resize. */}
          <Circle
            center={[roiCenterCoords.lat, roiCenterCoords.lng]}
            radius={roi.radiusM}
            interactive
            eventHandlers={{ mousedown: handleBoundaryMouseDown }}
            pathOptions={{
              color: ROI_COLOR,
              weight: ROI_BOUNDARY_HIT_WEIGHT,
              opacity: ROI_BOUNDARY_HIT_OPACITY,
              fill: false,
              className: styles.roiBoundaryHandle,
            }}
          />
          <Marker position={[roiCenterCoords.lat, roiCenterCoords.lng]} icon={roiCenterIcon} interactive={false} />
        </>
      )}

      <div ref={panelRef} className={styles.roiPanel}>
        <div className={styles.roiButtons}>
          <button
            type="button"
            className={`${styles.roiButton} ${defining ? styles.roiButtonActive : ''}`}
            onClick={() => setDefining((value) => !value)}
          >
            {defining ? 'Click Map to Place…' : roi ? 'Redefine ROI' : 'Define ROI'}
          </button>
          {roi && (
            <button type="button" className={styles.roiButton} onClick={handleClear}>
              Clear ROI
            </button>
          )}
        </div>
        {roi && (
          <div className={styles.roiInfo}>
            <span>LAT {roi.center.lat.toFixed(5)}</span>
            <span>LON {roi.center.lon.toFixed(5)}</span>
            <label className={styles.roiRadiusLabel}>
              RADIUS
              <input
                type="number"
                className={styles.roiRadiusInput}
                min={MIN_ROI_RADIUS_METERS}
                max={MAX_ROI_RADIUS_METERS}
                value={Math.round(roi.radiusM)}
                onChange={handleRadiusInputChange}
              />
              m
            </label>
          </div>
        )}
      </div>
    </>
  );
}

/**
 * Renders the interactive map and the drone's live position. Position comes
 * exclusively from useLocation()/LocationService, which is backed by the
 * live realtime server connection — this component never touches mock data
 * directly. The home marker is a static display anchor (HOME_REFERENCE),
 * not live telemetry — see config/mapConfig.ts.
 */
export function MapView() {
  const position = useLocation();

  if (!position) {
    return (
      <div className={styles.wrapper}>
        <div className={styles.acquiring}>Acquiring GPS fix…</div>
      </div>
    );
  }

  const homeBearing = bearingDeg(position, HOME_REFERENCE);
  const homeDistanceM = euclideanDistanceMeters(position, HOME_REFERENCE);
  const arrowPosition: [number, number] = [
    position.lat + (HOME_REFERENCE.lat - position.lat) * HOME_ARROW_T,
    position.lng + (HOME_REFERENCE.lng - position.lng) * HOME_ARROW_T,
  ];

  return (
    <div className={styles.wrapper}>
      <MapContainer
        center={[position.lat, position.lng]}
        zoom={DEFAULT_ZOOM}
        className={styles.map}
        // Off so a rapid double-click (the first two taps of an ROI
        // triple-tap) doesn't zoom the map out from under the gesture.
        doubleClickZoom={false}
      >
        <TileLayer
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        />
        <Circle
          center={[HOME_REFERENCE.lat, HOME_REFERENCE.lng]}
          radius={HOME_RADIUS_METERS}
          interactive={false}
          pathOptions={{ color: '#3a6fd8', weight: 2, opacity: 0.55, fill: true, fillColor: '#3a6fd8', fillOpacity: 0.08 }}
        />
        <Polyline
          positions={[
            [position.lat, position.lng],
            [HOME_REFERENCE.lat, HOME_REFERENCE.lng],
          ]}
          pathOptions={{ color: '#ffb020', weight: 2, dashArray: '6 6', opacity: 0.85 }}
        />
        <Marker position={arrowPosition} icon={createHomeArrowIcon(homeBearing)} interactive={false}>
          <Tooltip permanent direction="top" offset={[0, -6]} className={styles.homeDistanceTooltip}>
            {formatDistance(homeDistanceM)}
          </Tooltip>
        </Marker>
        <Marker position={[HOME_REFERENCE.lat, HOME_REFERENCE.lng]} icon={homeIcon} />
        <Marker position={[position.lat, position.lng]} icon={droneIcon} />
        <LiveTrackingControl position={position} home={HOME_REFERENCE} />
        <RoiLayer home={HOME_REFERENCE} homeRadiusM={HOME_RADIUS_METERS} />
        <PatrolLayer home={HOME_REFERENCE} homeRadiusM={HOME_RADIUS_METERS} />
      </MapContainer>
    </div>
  );
}
