import { useState } from 'react';
import { ActionBar } from './components/actionBar/ActionBar';
import { ResizableSplitView } from './components/layout/ResizableSplitView';
import { DroneHUD } from './components/hud/DroneHUD';
import { MapView } from './components/map/MapView';
import { DroneVideo } from './components/video/DroneVideo';
import { ManualControlPad } from './components/manualControl/ManualControlPad';
import { StartupSplash } from './components/startup/StartupSplash';
import { useDetection } from './hooks/useDetection';
import { useMissionStatus } from './hooks/useMissionStatus';
import { useTelemetry } from './hooks/useTelemetry';
import { useTargetSelection } from './hooks/useTargetSelection';
import styles from './App.module.css';
import { DEFAULT_OPERATING_CONFIG } from './config/mapConfig';

/**
 * Top-level layout. Owns target selection plus the short-lived optimistic
 * suppression used to hide a rejected target from the map immediately.
 * Everything else lives behind the hooks/services each child pulls from.
 */
function App() {
  const missionStatus = useMissionStatus();
  const operatingConfig = missionStatus?.operatingConfig ?? DEFAULT_OPERATING_CONFIG;
  const targets = useDetection();
  const telemetry = useTelemetry();
  const diagnostics = telemetry?.targetFollowDiagnostics ?? null;
  const decision = diagnostics?.targetDecision;
  const [ignoredMapTargetId, setIgnoredMapTargetId] = useState<string | null>(null);
  const [selectedTargetId, setSelectedTargetId] = useTargetSelection(
    targets.map((target) => target.id), decision?.trackId ?? null);

  // Once telemetry stops tracking the ignored ID, forget the local suppression
  // so a later legitimate reacquisition of the same tracker can be displayed.
  if (ignoredMapTargetId !== null && diagnostics?.trackerId !== ignoredMapTargetId) {
    setIgnoredMapTargetId(null);
  }

  return (
    <>
      <div className={styles.app}>
        <ResizableSplitView
          top={
            <div className={styles.mapPane}>
              <MapView
                targets={targets}
                diagnostics={diagnostics}
                ignoredTargetId={ignoredMapTargetId}
                maxFlightRadiusM={operatingConfig.maxFlightRadiusM}
                cruiseAltitudeM={operatingConfig.cruiseAltitudeM}
                operatingConfigPending={missionStatus?.operatingConfigPending ?? false}
              />
              <DroneHUD />
            </div>
          }
          bottom={
            <DroneVideo
              targets={targets}
              selectedTargetId={selectedTargetId}
              onSelectTarget={setSelectedTargetId}
            />
          }
        />
        <ActionBar
          selectedTargetId={selectedTargetId}
          canReject={Boolean(selectedTargetId && decision?.rejectableTrackIds.includes(selectedTargetId))}
          onClearSelection={() => setSelectedTargetId(null)}
          onTargetIgnoreStart={setIgnoredMapTargetId}
          onTargetIgnoreFailed={(targetId) => {
            setIgnoredMapTargetId((ignoredId) => ignoredId === targetId ? null : ignoredId);
          }}
        />
      </div>
      <ManualControlPad
        engaged={missionStatus?.manualControlActive ?? false}
        inputSource={missionStatus?.manualInputSource ?? 'tablet'}
      />
      <StartupSplash ready={missionStatus !== null} />
    </>
  );
}

export default App;
