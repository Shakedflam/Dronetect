import { useEffect, useRef, useState } from 'react';
import { MapContainer, Marker, Polyline, Tooltip, TileLayer, useMap } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import type { Coordinates, DronePosition } from '../../types/drone';
import { useLocation } from '../../hooks/useLocation';
import { HOME_REFERENCE } from '../../config/homeReference';
import { bearingDeg, euclideanDistanceMeters } from '../../utils/geo';
import styles from './MapView.module.css';

const DEFAULT_ZOOM = 15;

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

/** Padding, in pixels, kept between the drone/home bounds and the map edge. */
const FOLLOW_PADDING: L.PointTuple = [64, 64];
/**
 * Zoom cap for auto-follow. Deliberately higher than DEFAULT_ZOOM so that
 * when the drone is close to home the view zooms in and keeps the two
 * markers visually separated, instead of sitting at a fixed zoom that
 * crowds them together.
 */
const MAX_FOLLOW_ZOOM = 18;

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
  const hasCenteredOnce = useRef(false);

  useEffect(() => {
    const stopFollowing = () => setAutoFollow(false);
    map.on('dragstart', stopFollowing);
    return () => {
      map.off('dragstart', stopFollowing);
    };
  }, [map]);

  useEffect(() => {
    const bounds = L.latLngBounds([position.lat, position.lng], [home.lat, home.lng]);
    if (!hasCenteredOnce.current) {
      map.fitBounds(bounds, { padding: FOLLOW_PADDING, maxZoom: MAX_FOLLOW_ZOOM, animate: false });
      hasCenteredOnce.current = true;
      return;
    }
    if (autoFollow) {
      map.flyToBounds(bounds, { padding: FOLLOW_PADDING, maxZoom: MAX_FOLLOW_ZOOM, duration: 0.8 });
    }
  }, [position, home, autoFollow, map]);

  if (autoFollow) return null;

  return (
    <button
      type="button"
      className={styles.recenterButton}
      onClick={() => setAutoFollow(true)}
    >
      Recenter
    </button>
  );
}

/**
 * Renders the interactive map and the drone's live position. Position comes
 * exclusively from useLocation()/LocationService, which is backed by the
 * live gateway connection — this component never touches mock data
 * directly. The home marker is a static display anchor (HOME_REFERENCE),
 * not live telemetry — see config/homeReference.ts.
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
      >
        <TileLayer
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
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
      </MapContainer>
    </div>
  );
}
