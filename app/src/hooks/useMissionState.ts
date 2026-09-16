import { useEffect, useState } from 'react';
import type { MissionState } from '../types/missionState';
import { missionStateService } from '../services/missionStateService';

/** The current frontend-only mission state (ON_GROUND/PATROL/TRANSIT_TO_ROI/ROI_PATROL/RTH). */
export function useMissionState(): MissionState {
  const [state, setState] = useState<MissionState>(() => missionStateService.getMissionState());

  useEffect(() => {
    return missionStateService.subscribe(setState);
  }, []);

  return state;
}
