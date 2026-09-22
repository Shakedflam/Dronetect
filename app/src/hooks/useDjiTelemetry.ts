import { useEffect, useState } from 'react';
import type { DjiTelemetryData } from '../types/drone';
import { telemetryService } from '../services/telemetryService';

export function useDjiTelemetry(): DjiTelemetryData | null {
  const [snapshot, setSnapshot] = useState<DjiTelemetryData | null>(null);
  useEffect(() => telemetryService.subscribeDji(setSnapshot), []);
  return snapshot;
}
