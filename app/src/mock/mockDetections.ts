import type { DetectedTarget } from '../types/drone';

/**
 * Simulates several independent detections (e.g. other drones) entering and
 * leaving the frame over time. Each "slot" below is a pure function of
 * elapsed time, so the whole module stays deterministic and side-effect free
 * — the same shape a real detector's output would take once this is swapped
 * for a live computer-vision feed.
 */

interface DetectionSlot {
  id: string;
  label: string;
  /** How long this slot's spawn/hide cycle repeats, in ms. */
  cycleDurationMs: number;
  /** Fraction of the cycle during which the target is visible, in (0, 1]. */
  visibleFraction: number;
  /** Offset so slots don't all spawn in lockstep. */
  phaseOffsetMs: number;
  /** Base confidence, gently jittered while visible. */
  baseConfidence: number;
  /** Center of the drift path, in normalized [0, 1] video coordinates. */
  center: { x: number; y: number };
  boxSize: { width: number; height: number };
}

const SLOTS: DetectionSlot[] = [
  {
    id: 'target-1',
    label: 'Unidentified Drone',
    cycleDurationMs: 16_000,
    visibleFraction: 0.6,
    phaseOffsetMs: 0,
    baseConfidence: 0.87,
    center: { x: 0.28, y: 0.35 },
    boxSize: { width: 0.16, height: 0.14 },
  },
  {
    id: 'target-2',
    label: 'Unidentified Drone',
    cycleDurationMs: 22_000,
    visibleFraction: 0.45,
    phaseOffsetMs: 6_000,
    baseConfidence: 0.74,
    center: { x: 0.68, y: 0.55 },
    boxSize: { width: 0.14, height: 0.12 },
  },
  {
    id: 'target-3',
    label: 'Unidentified Aircraft',
    cycleDurationMs: 27_000,
    visibleFraction: 0.35,
    phaseOffsetMs: 14_000,
    baseConfidence: 0.63,
    center: { x: 0.5, y: 0.22 },
    boxSize: { width: 0.12, height: 0.1 },
  },
];

function clamp01(value: number): number {
  return Math.min(1, Math.max(0, value));
}

function resolveSlot(slot: DetectionSlot, elapsedMs: number): DetectedTarget | null {
  const t = (elapsedMs + slot.phaseOffsetMs) % slot.cycleDurationMs;
  const isVisible = t < slot.cycleDurationMs * slot.visibleFraction;
  if (!isVisible) return null;

  const driftX = Math.sin(elapsedMs / 2600 + slot.phaseOffsetMs) * 0.05;
  const driftY = Math.cos(elapsedMs / 3100 + slot.phaseOffsetMs) * 0.04;
  const confidenceJitter = Math.sin(elapsedMs / 900 + slot.phaseOffsetMs) * 0.05;

  return {
    id: slot.id,
    label: slot.label,
    confidence: clamp01(slot.baseConfidence + confidenceJitter),
    boundingBox: {
      x: clamp01(slot.center.x + driftX - slot.boxSize.width / 2),
      y: clamp01(slot.center.y + driftY - slot.boxSize.height / 2),
      width: slot.boxSize.width,
      height: slot.boxSize.height,
    },
  };
}

export function getMockDetections(elapsedMs: number): DetectedTarget[] {
  return SLOTS.map((slot) => resolveSlot(slot, elapsedMs)).filter(
    (target): target is DetectedTarget => target !== null,
  );
}
