import { useEffect, useState } from 'react';
import {
  missionStatusService,
  type MissionStatusService,
} from '../services/missionStatusService';

type MissionStatus = ReturnType<MissionStatusService['getStatus']>;

/** Complete backend-owned mission status, including drone-link health. */
export function useMissionStatus(): MissionStatus {
  const [status, setStatus] = useState<MissionStatus>(() =>
    missionStatusService.getStatus()
  );

  useEffect(() => missionStatusService.subscribe(setStatus), []);
  return status;
}
