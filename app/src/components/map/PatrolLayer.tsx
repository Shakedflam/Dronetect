import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Circle, Marker, Polygon, Polyline, useMap } from 'react-leaflet';
import L from 'leaflet';
import type { Coordinates } from '../../types/drone';
import { useMissionState } from '../../hooks/useMissionState';
import { usePatrolConfig } from '../../hooks/usePatrolConfig';
import { patrolService } from '../../services/patrolService';
import { patrolBandOuterRadiiMeters, patrolDividerBearingsDeg, patrolRegionPolygonPoints } from '../../utils/patrolGeometry';
import { destinationPoint } from '../../utils/geo';
import styles from './PatrolLayer.module.css';

/** Thin so the grid divides the map without hiding it, but bright/opaque enough to actually read against map tiles. */
const GRID_COLOR = '#4aa3ff';
const GRID_PATH_OPTIONS = { color: GRID_COLOR, weight: 1.5, opacity: 0.75 };

/** Darker/more muted than the radial dividers, so the band circles read as a subtler background grid. */
const BAND_CIRCLE_COLOR = '#1f4a75';
const BAND_CIRCLE_PATH_OPTIONS = { color: BAND_CIRCLE_COLOR, weight: 1.5, opacity: 0.75 };

/** Fill used for a region's invisible-but-hittable tap target when unranked. */
const HIT_FILL_OPACITY = 0.001;
/** Fill used for a region's tap target once it has a priority rank — a subtle highlight, still no stroke. */
const RANKED_FILL_OPACITY = 0.12;

function toLatLngTuples(points: Coordinates[]): [number, number][] {
  return points.map((point): [number, number] => [point.lat, point.lng]);
}

function rankBadgeIcon(rank: number): L.DivIcon {
  return L.divIcon({
    className: styles.rankIcon,
    html: `<span class="${styles.rankBadge}">${rank}</span>`,
    iconSize: [18, 18],
    iconAnchor: [9, 9],
  });
}

/**
 * Frontend-only "Persistent Coverage with Dynamic Priorities" patrol grid:
 * divides the Home operational circle into 8 directions x 4 equal-area
 * distance bands, and lets the operator rank regions by revisit priority.
 * Purely visual/data-structure — no flight-planning logic reads it. State
 * lives in patrolService, not here, mirroring RoiLayer/roiService in
 * MapView.tsx, which this component otherwise never touches.
 */
export function PatrolLayer({ home, homeRadiusM }: { home: Coordinates; homeRadiusM: number }) {
  const map = useMap();
  const missionState = useMissionState();
  const config = usePatrolConfig();
  const [settingPriority, setSettingPriority] = useState(false);
  const panelRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const panel = panelRef.current;
    if (!panel) return;
    // Otherwise clicks on the panel (buttons) would bubble to the map and
    // be treated as a click on whatever's underneath it.
    L.DomEvent.disableClickPropagation(panel);
    L.DomEvent.disableScrollPropagation(panel);
  }, []);

  const regionPolygons = useMemo(
    () => config.regions.map((region) => ({ region, points: toLatLngTuples(patrolRegionPolygonPoints(home, region)) })),
    [config.regions, home]
  );

  // The outermost band boundary coincides exactly with the Home circle
  // already drawn in MapView.tsx — skip it to avoid a redundant overlapping line.
  const bandBoundaryRadii = useMemo(() => patrolBandOuterRadiiMeters(homeRadiusM).slice(0, -1), [homeRadiusM]);

  const dividerLines = useMemo(
    () =>
      patrolDividerBearingsDeg().map((bearing): [number, number][] => {
        const edge = destinationPoint(home, homeRadiusM, bearing);
        return [
          [home.lat, home.lng],
          [edge.lat, edge.lng],
        ];
      }),
    [home, homeRadiusM]
  );

  const rankedRegions = config.regions.filter((region) => region.priorityRank !== null);

  const focusPatrolArea = useCallback(() => {
    // Stop live tracking before changing the viewport; otherwise the next
    // telemetry update immediately zooms back to the drone/home bounds.
    map.fire('patrolfocus');
    const patrolBounds = L.latLng(home.lat, home.lng).toBounds(homeRadiusM);
    map.fitBounds(patrolBounds, { padding: [48, 48], animate: false });
  }, [home.lat, home.lng, homeRadiusM, map]);

  useEffect(() => {
    if (missionState === 'PATROL') focusPatrolArea();
  }, [focusPatrolArea, missionState]);

  return (
    <>
      {bandBoundaryRadii.map((radiusM) => (
        <Circle
          key={radiusM}
          center={[home.lat, home.lng]}
          radius={radiusM}
          interactive={false}
          pathOptions={{ ...BAND_CIRCLE_PATH_OPTIONS, fill: false, dashArray: '4 5' }}
        />
      ))}

      {dividerLines.map((positions, index) => (
        <Polyline key={index} positions={positions} interactive={false} pathOptions={GRID_PATH_OPTIONS} />
      ))}

      {regionPolygons.map(({ region, points }) => (
        // react-leaflet only applies `interactive` once, at layer creation
        // (@react-leaflet/core's usePathOptions syncs pathOptions via
        // setStyle() but never touches the interactive flag), so toggling
        // interactive={settingPriority} here would silently stop working
        // after the first render. Instead these stay interactive={true} for
        // their whole lifetime, and the click handler itself — a fresh
        // closure every render, which react-leaflet's useEventHandlers does
        // rebind on each change — is what actually gates whether a tap does
        // anything.
        <Polygon
          key={region.id}
          positions={points}
          interactive
          eventHandlers={{
            click: () => {
              if (!settingPriority) return;
              patrolService.setPriority(region.id);
            },
          }}
          pathOptions={{
            stroke: false,
            fill: true,
            fillColor: GRID_COLOR,
            fillOpacity: region.priorityRank !== null ? RANKED_FILL_OPACITY : HIT_FILL_OPACITY,
          }}
        />
      ))}

      {rankedRegions.map((region) => (
        <Marker
          key={region.id}
          position={[region.center.lat, region.center.lon]}
          icon={rankBadgeIcon(region.priorityRank as number)}
          interactive={false}
        />
      ))}

      <div ref={panelRef} className={styles.patrolPanel}>
        <div className={styles.patrolButtons}>
          <button
            type="button"
            className={`${styles.patrolButton} ${settingPriority ? styles.patrolButtonActive : ''}`}
            onClick={() => {
              const nextSettingPriority = !settingPriority;
              setSettingPriority(nextSettingPriority);
              if (nextSettingPriority) focusPatrolArea();
            }}
            aria-pressed={settingPriority}
          >
            {settingPriority ? 'Tap Regions to Rank…' : 'Set Priority'}
          </button>
          {rankedRegions.length > 0 && (
            <button type="button" className={styles.patrolButton} onClick={() => patrolService.clearPriorities()}>
              Clear Priorities
            </button>
          )}
        </div>
      </div>
    </>
  );
}
