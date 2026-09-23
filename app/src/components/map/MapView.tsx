import { useCallback, useEffect, useRef, useState, type ChangeEvent } from 'react';
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
import type {
  Coordinates,
  DetectedTarget,
  DronePosition,
  TargetFollowDiagnostics,
} from '../../types/drone';
import type { RoiCenter } from '../../types/roi';
import { useLocation } from '../../hooks/useLocation';
import { useRoi } from '../../hooks/useRoi';
import { droneCommandService } from '../../services/droneCommandService';
import { roiService } from '../../services/roiService';
import {
  DEFAULT_ROI_RADIUS_METERS,
  DEFAULT_ZOOM,
  FOLLOW_PADDING,
  HOME_REFERENCE,
  MAX_FOLLOW_ZOOM,
  MAX_ROI_RADIUS_METERS,
  MIN_ROI_RADIUS_METERS,
} from '../../config/mapConfig';
import { bearingDeg, destinationPoint, euclideanDistanceMeters } from '../../utils/geo';
import { clipRoiToHomeRange } from '../../utils/roiClipping';
import { PatrolLayer } from './PatrolLayer';
import { FlightParametersLayer } from './FlightParametersLayer';
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
const TARGET_CAMERA_HORIZONTAL_FOV_RAD = 1.0;
const TARGET_MAX_DISTANCE_M = 100;
const TARGET_REFERENCE_DISTANCE_M = 2;
const TARGET_REFERENCE_BOX_AREA_RATIO = 0.025;
const TARGET_DOT_SIZE_PX = 10;

interface TargetMapEstimate {
  center: Coordinates;
}

interface MapViewProps {
  targets: DetectedTarget[];
  diagnostics: TargetFollowDiagnostics | null;
  ignoredTargetId: string | null;
  maxFlightRadiusM: number;
  cruiseAltitudeM: number;
  operatingConfigPending: boolean;
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}

function estimateRangeFromBox(target: DetectedTarget): number | null {
  const boxWidth = Math.max(0, target.boundingBox.x2 - target.boundingBox.x1);
  const boxHeight = Math.max(0, target.boundingBox.y2 - target.boundingBox.y1);
  const areaRatio = boxWidth * boxHeight;
  if (!Number.isFinite(areaRatio) || areaRatio <= 0) return null;

  const rangeM = TARGET_REFERENCE_DISTANCE_M * Math.sqrt(
    TARGET_REFERENCE_BOX_AREA_RATIO / areaRatio,
  );
  return Number.isFinite(rangeM) && rangeM > 0
    ? clamp(rangeM, TARGET_REFERENCE_DISTANCE_M, TARGET_MAX_DISTANCE_M)
    : null;
}

const targetDotIcon = L.divIcon({
  className: styles.targetIcon,
  html: `<span class="${styles.targetDot}"></span>`,
  iconSize: [TARGET_DOT_SIZE_PX, TARGET_DOT_SIZE_PX],
  iconAnchor: [TARGET_DOT_SIZE_PX / 2, TARGET_DOT_SIZE_PX / 2],
});

/** Approximate map position from the followed box and camera-only range cue. */
function estimateTargetMapPosition(
  position: DronePosition,
  targets: DetectedTarget[],
  diagnostics: TargetFollowDiagnostics | null,
  ignoredTargetId: string | null,
): TargetMapEstimate | null {
  const trackerId = diagnostics?.trackerId;
  if (
    !trackerId ||
    trackerId === ignoredTargetId ||
    diagnostics.targetLostDurationS > 0
  ) {
    return null;
  }

  const target = targets.find(
    (candidate) => candidate.id === trackerId && candidate.trackState !== 'lost',
  );
  if (!target) return null;

  const diagnosticRangeM = diagnostics.estimatedTargetDistanceM;
  const slantRangeM = diagnosticRangeM !== null && Number.isFinite(diagnosticRangeM)
    && diagnosticRangeM > 0
    ? clamp(diagnosticRangeM, TARGET_REFERENCE_DISTANCE_M, TARGET_MAX_DISTANCE_M)
    : estimateRangeFromBox(target);
  if (slantRangeM === null) return null;

  const centerX = clamp((target.boundingBox.x1 + target.boundingBox.x2) / 2, 0, 1);
  const imageRight = (2 * centerX - 1) * Math.tan(TARGET_CAMERA_HORIZONTAL_FOV_RAD / 2);
  const horizontalOffsetDeg = Math.atan(imageRight) * 180 / Math.PI;
  const targetBearingDeg = (position.headingDeg + horizontalOffsetDeg + 360) % 360;

  const heightDifferenceM = diagnostics.estimatedTargetHeightDifferenceM;
  const horizontalDistanceM = heightDifferenceM !== null && Number.isFinite(heightDifferenceM)
    ? Math.sqrt(Math.max(0, slantRangeM ** 2 - heightDifferenceM ** 2))
    : slantRangeM;

  return {
    center: destinationPoint(position, horizontalDistanceM, targetBearingDeg),
  };
}

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
function LiveTrackingControl({
  position,
  home,
  target,
  maxFlightRadiusM,
}: {
  position: DronePosition;
  home: Coordinates;
  target: Coordinates | null;
  maxFlightRadiusM: number;
}) {
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
    bounds.extend(L.latLng(home.lat, home.lng).toBounds(maxFlightRadiusM * 2));
    if (target) bounds.extend([target.lat, target.lng]);
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
  }, [position, home, target, maxFlightRadiusM, autoFollow, map]);

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
function RoiLayer({
  home,
  homeRadiusM,
  onPanelHeightChange,
}: {
  home: Coordinates;
  homeRadiusM: number;
  onPanelHeightChange?: (heightPx: number) => void;
}) {
  const roi = useRoi();
  const [defining, setDefining] = useState(false);
  const panelRef = useRef<HTMLDivElement>(null);
  // Shared by both the boundary (resize) and interior (move) drag gestures
  // below — only one can ever be active at a time, so one ref is enough.
  const stopDragRef = useRef<(() => void) | null>(null);
  const interiorHandleRef = useRef<L.Circle | null>(null);
  const boundaryHandleRef = useRef<L.Circle | null>(null);
  // The drag handlers are bound to the SVG paths once and then read the ROI
  // through this ref, so a mid-drag update (every pointermove calls setRoi)
  // never has to tear the listeners down and rebind them. Written in an
  // effect rather than during render; only pointerdown reads it, which
  // cannot run before the commit that set it.
  const roiRef = useRef(roi);
  useEffect(() => {
    roiRef.current = roi;
  }, [roi]);
  const hasRoi = roi !== null;

  useEffect(() => {
    const panel = panelRef.current;
    if (!panel) return;
    // Otherwise clicks/scrolls on the panel (buttons, radius input) would
    // bubble to the map and be treated as "place the ROI here".
    L.DomEvent.disableClickPropagation(panel);
    L.DomEvent.disableScrollPropagation(panel);
    return () => stopDragRef.current?.();
  }, []);

  useEffect(() => {
    const panel = panelRef.current;
    if (!panel || !onPanelHeightChange) return;
    const observer = new ResizeObserver((entries) => {
      const entry = entries[0];
      if (entry) onPanelHeightChange(entry.target.getBoundingClientRect().height);
    });
    observer.observe(panel);
    onPanelHeightChange(panel.getBoundingClientRect().height);
    return () => observer.disconnect();
  }, [onPanelHeightChange]);

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

  /**
   * Shared drag driver for both ROI gestures.
   *
   * Pointer events rather than mouse events, because these handles have to
   * work under a finger on the tablet as well as a mouse on the desktop. A
   * touch delivers `mousedown` only as a compatibility event *after* the
   * gesture finishes, and never delivers `mousemove` during it, so the
   * previous mouse-only listeners could not drive a touch drag at all.
   * `pointer*` is emitted uniformly for mouse, touch and stylus, and
   * PointerEvent extends MouseEvent so Leaflet's coordinate conversion still
   * accepts it directly.
   */
  const beginPointerDrag = useCallback((
    event: PointerEvent,
    onMove: (latlng: L.LatLng) => void,
  ): void => {
    // Keeps the browser from claiming the gesture as a map pan/scroll, and
    // keeps Leaflet from also starting its own drag from the same press.
    event.preventDefault();
    L.DomEvent.stopPropagation(event);
    stopDragRef.current?.();
    map.dragging.disable();
    let active = true;

    function handleMove(moveEvent: PointerEvent): void {
      // Ignore a second finger mid-drag; the first one owns the gesture.
      if (moveEvent.pointerId !== event.pointerId) return;
      onMove(map.mouseEventToLatLng(moveEvent));
    }

    function stopDragging(): void {
      if (!active) return;
      active = false;
      map.dragging.enable();
      document.removeEventListener('pointermove', handleMove);
      document.removeEventListener('pointerup', stopDragging);
      document.removeEventListener('pointercancel', stopDragging);
      window.removeEventListener('blur', stopDragging);
      if (stopDragRef.current === stopDragging) {
        stopDragRef.current = null;
      }
    }

    document.addEventListener('pointermove', handleMove);
    document.addEventListener('pointerup', stopDragging);
    document.addEventListener('pointercancel', stopDragging);
    window.addEventListener('blur', stopDragging);
    stopDragRef.current = stopDragging;
  }, [map]);

  /** Drag the boundary ring to resize. */
  const handleBoundaryPointerDown = useCallback((event: PointerEvent): void => {
    const current = roiRef.current;
    if (!current) return;
    const roiCenter = current.center;
    const centerCoords = toCoordinates(roiCenter);
    beginPointerDrag(event, (latlng) => {
      roiService.setRoi({
        center: roiCenter,
        radiusM: clampRoiRadius(euclideanDistanceMeters(centerCoords, latlng)),
      });
    });
  }, [beginPointerDrag]);

  /**
   * Drag-to-move: grabbing anywhere inside the circle (not on the boundary
   * ring, which resizes instead — see the interactive circle's render order
   * below) translates the ROI by the same lat/lng delta as the pointer, so
   * the circle tracks it exactly regardless of map projection, the same way
   * dragging any other map object would.
   */
  const handleInteriorPointerDown = useCallback((event: PointerEvent): void => {
    const current = roiRef.current;
    if (!current) return;
    const roiCenter = current.center;
    const radiusM = current.radiusM;
    const grabLatLng = map.mouseEventToLatLng(event);
    beginPointerDrag(event, (latlng) => {
      roiService.setRoi({
        center: {
          lat: roiCenter.lat + (latlng.lat - grabLatLng.lat),
          lon: roiCenter.lon + (latlng.lng - grabLatLng.lng),
        },
        radiusM,
      });
    });
  }, [beginPointerDrag, map]);

  // Bound straight to the rendered SVG paths: react-leaflet's `eventHandlers`
  // only exposes Leaflet's own (mouse) event names, which is exactly what
  // did not reach touch. Leaflet mutates a path's geometry in place rather
  // than recreating the element, so re-binding only has to track the
  // circles appearing/disappearing with the ROI itself.
  useEffect(() => {
    // Leaflet types getElement() as the generic Element, whose event map has
    // no pointer events; the rendered handle is always an SVG path.
    const interior = interiorHandleRef.current?.getElement() as SVGElement | undefined;
    const boundary = boundaryHandleRef.current?.getElement() as SVGElement | undefined;
    interior?.addEventListener('pointerdown', handleInteriorPointerDown);
    boundary?.addEventListener('pointerdown', handleBoundaryPointerDown);
    return () => {
      interior?.removeEventListener('pointerdown', handleInteriorPointerDown);
      boundary?.removeEventListener('pointerdown', handleBoundaryPointerDown);
    };
  }, [hasRoi, handleInteriorPointerDown, handleBoundaryPointerDown]);

  function handleRadiusInputChange(event: ChangeEvent<HTMLInputElement>): void {
    if (!roi) return;
    const value = Number(event.target.value);
    if (!Number.isFinite(value)) return;
    roiService.setRoi({ center: roi.center, radiusM: clampRoiRadius(value) });
  }

  function handleClear(): void {
    roiService.clearRoi();
    droneCommandService.clearRoi().catch((error: unknown) => {
      console.error('[MapView] clearRoi failed', error);
    });
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
            ref={interiorHandleRef}
            center={[roiCenterCoords.lat, roiCenterCoords.lng]}
            radius={roi.radiusM}
            interactive
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
            ref={boundaryHandleRef}
            center={[roiCenterCoords.lat, roiCenterCoords.lng]}
            radius={roi.radiusM}
            interactive
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
export function MapView({
  targets,
  diagnostics,
  ignoredTargetId,
  maxFlightRadiusM,
  cruiseAltitudeM,
  operatingConfigPending,
}: MapViewProps) {
  const position = useLocation();
  const [roiPanelHeight, setRoiPanelHeight] = useState(0);

  // ROI and flight radius/altitude configure the *mission*, not the aircraft,
  // so they have to stay reachable before the first fix arrives. Previously
  // this component returned an "Acquiring GPS fix" placeholder instead, which
  // meant those controls were never mounted at all whenever no drone-role
  // client was feeding telemetry to the realtime server. The map now falls
  // back to the home reference and only the aircraft-derived layers are
  // withheld.
  const targetEstimate = position
    ? estimateTargetMapPosition(position, targets, diagnostics, ignoredTargetId)
    : null;
  const homeBearing = position ? bearingDeg(position, HOME_REFERENCE) : 0;
  const homeDistanceM = position ? euclideanDistanceMeters(position, HOME_REFERENCE) : 0;
  const arrowPosition: [number, number] | null = position
    ? [
        position.lat + (HOME_REFERENCE.lat - position.lat) * HOME_ARROW_T,
        position.lng + (HOME_REFERENCE.lng - position.lng) * HOME_ARROW_T,
      ]
    : null;

  return (
    <div className={styles.wrapper}>
      <MapContainer
        center={position ? [position.lat, position.lng] : [HOME_REFERENCE.lat, HOME_REFERENCE.lng]}
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
          radius={maxFlightRadiusM}
          interactive={false}
          pathOptions={{ color: '#3a6fd8', weight: 2, opacity: 0.55, fill: true, fillColor: '#3a6fd8', fillOpacity: 0.08 }}
        />
        {position && arrowPosition && (
          <>
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
          </>
        )}
        <Marker position={[HOME_REFERENCE.lat, HOME_REFERENCE.lng]} icon={homeIcon} />
        {position && <Marker position={[position.lat, position.lng]} icon={droneIcon} />}
        {targetEstimate && (
          <Marker
            position={[targetEstimate.center.lat, targetEstimate.center.lng]}
            icon={targetDotIcon}
            interactive={false}
            zIndexOffset={1000}
          />
        )}
        {position && (
          <LiveTrackingControl
            position={position}
            home={HOME_REFERENCE}
            target={targetEstimate?.center ?? null}
            maxFlightRadiusM={maxFlightRadiusM}
          />
        )}
        <RoiLayer
          home={HOME_REFERENCE}
          homeRadiusM={maxFlightRadiusM}
          onPanelHeightChange={setRoiPanelHeight}
        />
        <PatrolLayer home={HOME_REFERENCE} homeRadiusM={maxFlightRadiusM} />
        <FlightParametersLayer
          config={{ maxFlightRadiusM, cruiseAltitudeM }}
          pending={operatingConfigPending}
          topOffsetPx={roiPanelHeight}
        />
        {!position && <div className={styles.awaitingFix}>Acquiring GPS fix…</div>}
      </MapContainer>
    </div>
  );
}
