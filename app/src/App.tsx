import { useEffect, useState } from 'react';
import { ActionBar } from './components/actionBar/ActionBar';
import { ResizableSplitView } from './components/layout/ResizableSplitView';
import { DroneHUD } from './components/hud/DroneHUD';
import { MapView } from './components/map/MapView';
import { DroneVideo } from './components/video/DroneVideo';
import { useDetection } from './hooks/useDetection';
import styles from './App.module.css';

/**
 * Top-level layout. Owns only one piece of state — which detected target is
 * currently selected — since that's shared between the video overlay (where
 * the operator picks a target) and the action bar (which acts on it).
 * Everything else lives behind the hooks/services each child pulls from.
 */
function App() {
  const targets = useDetection();
  const [selectedTargetId, setSelectedTargetId] = useState<string | null>(null);

  useEffect(() => {
    if (selectedTargetId === null) return;
    const stillPresent = targets.some((target) => target.id === selectedTargetId);
    if (!stillPresent) {
      setSelectedTargetId(null);
    }
  }, [targets, selectedTargetId]);

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
        onClearSelection={() => setSelectedTargetId(null)}
      />
    </div>
  );
}

export default App;
