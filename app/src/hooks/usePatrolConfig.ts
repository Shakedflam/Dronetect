import { useEffect, useState } from 'react';
import type { PatrolConfig } from '../types/patrol';
import { patrolService } from '../services/patrolService';

/** The current patrol regions with their priority ranks. */
export function usePatrolConfig(): PatrolConfig {
  const [config, setConfig] = useState<PatrolConfig>(() => patrolService.getConfig());

  useEffect(() => {
    return patrolService.subscribe(setConfig);
  }, []);

  return config;
}
