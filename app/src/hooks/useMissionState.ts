import { useEffect, useState } from 'react';
import type { MissionState } from '../types/missionState';
import { missionStateService } from '../services/missionStateService';

/** The current backend-owned mission state. */
export function useMissionState(): MissionState {
  const [state, setState] = useState<MissionState>(() => missionStateService.getMissionState());

  useEffect(() => {
    return missionStateService.subscribe(setState);
  }, []);

  return state;
}
