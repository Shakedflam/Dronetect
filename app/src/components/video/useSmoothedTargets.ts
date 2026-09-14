import { useEffect, useRef, useState } from 'react';
import type { DetectedTarget, NormalizedBox } from '../../types/drone';

/**
 * Eases each target's `boundingBox` toward the latest backend value instead
 * of snapping to it, to hide jitter between the detector's ~10fps updates.
 * Everything else on the target (id, label, confidence, trackState, ...)
 * passes through untouched from the latest backend message — the backend
 * tracker remains the sole source of truth for identity and track state,
 * this only interpolates the four box numbers already provided.
 *
 * The easing target itself is the backend box nudged forward by
 * `velocityNormalizedPerSec * age`, where age is how long ago the backend
 * captured that box. The backend supplies capture-to-server age, and the UI
 * adds monotonic time elapsed since receipt. That keeps the box "coasting" in the
 * right direction between updates instead of visibly lagging behind a
 * moving target. Age is clamped to MAX_EXTRAPOLATION_MS so a stale or lost
 * track never keeps flying across the screen on old velocity alone.
 */

const SMOOTHING_TAU_SECONDS = 0.085;
const CONVERGED_EPSILON = 0.0015;
/** Small sensible window: backend updates land every ~100ms, so this covers roughly one missed update before extrapolation freezes. */
const MAX_EXTRAPOLATION_MS = 200;

function lerp(from: number, to: number, factor: number): number {
  return from + (to - from) * factor;
}

function lerpBox(from: NormalizedBox, to: NormalizedBox, factor: number): NormalizedBox {
  return {
    x1: lerp(from.x1, to.x1, factor),
    y1: lerp(from.y1, to.y1, factor),
    x2: lerp(from.x2, to.x2, factor),
    y2: lerp(from.y2, to.y2, factor),
  };
}

/** Shifts a box by `velocity * ageSeconds`, keeping its size fixed. */
function extrapolateBox(
  box: NormalizedBox,
  velocityNormalizedPerSec: { x: number; y: number },
  ageSeconds: number
): NormalizedBox {
  const dx = velocityNormalizedPerSec.x * ageSeconds;
  const dy = velocityNormalizedPerSec.y * ageSeconds;
  return { x1: box.x1 + dx, y1: box.y1 + dy, x2: box.x2 + dx, y2: box.y2 + dy };
}

/** How long ago the backend captured this box, without comparing clocks across devices. */
function clampedAgeMs(target: DetectedTarget, now: number): number {
  const elapsedSinceReceiptMs = Math.max(0, now - target.receivedAtPerformanceMs);
  const ageMs = target.detectionAgeMs + elapsedSinceReceiptMs;
  if (!Number.isFinite(ageMs)) return 0;
  return Math.min(Math.max(ageMs, 0), MAX_EXTRAPOLATION_MS);
}

function boxDistance(a: NormalizedBox, b: NormalizedBox): number {
  return Math.max(
    Math.abs(a.x1 - b.x1),
    Math.abs(a.y1 - b.y1),
    Math.abs(a.x2 - b.x2),
    Math.abs(a.y2 - b.y2)
  );
}

export function useSmoothedTargets(targets: DetectedTarget[]): DetectedTarget[] {
  const [smoothed, setSmoothed] = useState<DetectedTarget[]>(targets);
  const targetsRef = useRef(targets);
  const currentBoxes = useRef(new Map<string, NormalizedBox>());
  const frameHandle = useRef<number | null>(null);
  const lastTime = useRef<number | null>(null);

  useEffect(() => {
    targetsRef.current = targets;

    function stepOnce(): boolean {
      const latest = targetsRef.current;
      const activeIds = new Set(latest.map((target) => target.id));
      for (const id of currentBoxes.current.keys()) {
        if (!activeIds.has(id)) currentBoxes.current.delete(id);
      }

      const now = performance.now();
      const dt = lastTime.current === null ? 0 : (now - lastTime.current) / 1000;
      lastTime.current = now;
      const factor = dt <= 0 ? 1 : 1 - Math.exp(-dt / SMOOTHING_TAU_SECONDS);

      let allConverged = true;
      const next = latest.map((target) => {
        const ageMs = clampedAgeMs(target, now);
        const shouldKeepExtrapolating =
          ageMs < MAX_EXTRAPOLATION_MS &&
          (target.velocityNormalizedPerSec.x !== 0 || target.velocityNormalizedPerSec.y !== 0);
        if (shouldKeepExtrapolating) allConverged = false;
        const predictedBox = extrapolateBox(
          target.boundingBox,
          target.velocityNormalizedPerSec,
          ageMs / 1000
        );

        const previousBox = currentBoxes.current.get(target.id);
        if (!previousBox) {
          currentBoxes.current.set(target.id, predictedBox);
          return { ...target, boundingBox: predictedBox };
        }

        if (boxDistance(previousBox, predictedBox) <= CONVERGED_EPSILON) {
          currentBoxes.current.set(target.id, predictedBox);
          return { ...target, boundingBox: predictedBox };
        }

        allConverged = false;
        const eased = lerpBox(previousBox, predictedBox, factor);
        currentBoxes.current.set(target.id, eased);
        return { ...target, boundingBox: eased };
      });

      setSmoothed(next);
      return allConverged;
    }

    function tick() {
      const converged = stepOnce();
      if (converged) {
        frameHandle.current = null;
        lastTime.current = null;
        return;
      }
      frameHandle.current = requestAnimationFrame(tick);
    }

    if (frameHandle.current === null) {
      frameHandle.current = requestAnimationFrame(tick);
    }
  }, [targets]);

  useEffect(() => {
    return () => {
      if (frameHandle.current !== null) {
        cancelAnimationFrame(frameHandle.current);
        frameHandle.current = null;
      }
      lastTime.current = null;
    };
  }, []);

  return smoothed;
}
