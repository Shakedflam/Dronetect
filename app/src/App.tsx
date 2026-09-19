import { ActionBar } from './components/actionBar/ActionBar';
import { ResizableSplitView } from './components/layout/ResizableSplitView';
import { DroneHUD } from './components/hud/DroneHUD';
import { MapView } from './components/map/MapView';
import { DroneVideo } from './components/video/DroneVideo';
import { useDetection } from './hooks/useDetection';
import { useTelemetry } from './hooks/useTelemetry';
import { useTargetSelection } from './hooks/useTargetSelection';
import styles from './App.module.css';

/**
 * Top-level layout. Owns only one piece of state — which detected target is
 * currently selected — since that's shared between the video overlay (where
 * the operator picks a target) and the action bar (which acts on it).
 * Everything else lives behind the hooks/services each child pulls from.
 */
function App() {
  const targets = useDetection();
  const telemetry = useTelemetry();
  const decision = telemetry?.targetFollowDiagnostics?.targetDecision;
  const [selectedTargetId, setSelectedTargetId] = useTargetSelection(
    targets.map((target) => target.id), decision?.trackId ?? null);

  return (
    <div className={styles.app}>
      <ResizableSplitView
        top={
          <div className={styles.mapPane}>
            <MapView />
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
        canApprove={Boolean(decision?.canApprove && decision.trackId === selectedTargetId && targets.some((target) => target.id === selectedTargetId))}
        canReject={Boolean(selectedTargetId && decision?.rejectableTrackIds.includes(selectedTargetId))}
        onClearSelection={() => setSelectedTargetId(null)}
      />
    </div>
  );
}

export default App;
