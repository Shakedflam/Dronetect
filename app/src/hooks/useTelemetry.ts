import { useEffect, useState } from 'react';
import type { TelemetryData } from '../types/drone';
import { telemetryService } from '../services/telemetryService';

/** Latest flight-instrument readout, or null until the first reading arrives. */
export function useTelemetry(): TelemetryData | null {
  const [telemetry, setTelemetry] = useState<TelemetryData | null>(null);

  useEffect(() => {
    return telemetryService.subscribe(setTelemetry);
  }, []);

  return telemetry;
}
