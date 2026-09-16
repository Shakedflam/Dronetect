import { useEffect, useState } from 'react';
import type { Roi } from '../types/roi';
import { roiService } from '../services/roiService';

/** The current region of interest, or null if none has been defined. */
export function useRoi(): Roi | null {
  const [roi, setRoi] = useState<Roi | null>(() => roiService.getRoi());

  useEffect(() => {
    return roiService.subscribe(setRoi);
  }, []);

  return roi;
}
