import {
  useCallback,
  useRef,
  useState,
  type KeyboardEvent as ReactKeyboardEvent,
  type PointerEvent as ReactPointerEvent,
  type ReactNode,
} from 'react';
import styles from './ResizableSplitView.module.css';

interface ResizableSplitViewProps {
  top: ReactNode;
  bottom: ReactNode;
  /** Fraction of the available height given to the top pane, 0..1. */
  initialSplit?: number;
  minTopPx?: number;
  minBottomPx?: number;
}

const DIVIDER_HEIGHT_PX = 16;
const KEYBOARD_STEP_PX = 24;

/**
 * Two vertically stacked panes with a draggable horizontal divider between
 * them. Panes are sized with flex-grow ratios (not fixed pixels) so the
 * split stays proportional across window/orientation changes, while
 * min-height keeps either pane from being fully dragged away.
 */
export function ResizableSplitView({
  top,
  bottom,
  initialSplit = 0.5,
  minTopPx = 160,
  minBottomPx = 160,
}: ResizableSplitViewProps) {
  const [topRatio, setTopRatio] = useState(initialSplit);
  const [isDragging, setIsDragging] = useState(false);

  const containerRef = useRef<HTMLDivElement>(null);
  const topPaneRef = useRef<HTMLDivElement>(null);
  const dragStartRef = useRef({ pointerY: 0, topHeightPx: 0, availableHeightPx: 0 });

  const applyRatioFromTopHeight = useCallback((topHeightPx: number, availableHeightPx: number) => {
    const minTop = minTopPx;
    const maxTop = availableHeightPx - minBottomPx;
    const clampedTopHeightPx = Math.min(Math.max(topHeightPx, minTop), Math.max(maxTop, minTop));
    setTopRatio(clampedTopHeightPx / availableHeightPx);
  }, [minTopPx, minBottomPx]);

  const handlePointerDown = useCallback((event: ReactPointerEvent<HTMLDivElement>) => {
    const container = containerRef.current;
    const topPane = topPaneRef.current;
    if (!container || !topPane) return;

    dragStartRef.current = {
      pointerY: event.clientY,
      topHeightPx: topPane.getBoundingClientRect().height,
      availableHeightPx: container.getBoundingClientRect().height - DIVIDER_HEIGHT_PX,
    };
    event.currentTarget.setPointerCapture(event.pointerId);
    setIsDragging(true);
  }, []);

  const handlePointerMove = useCallback((event: ReactPointerEvent<HTMLDivElement>) => {
    if (!isDragging) return;
    const { pointerY, topHeightPx, availableHeightPx } = dragStartRef.current;
    const deltaY = event.clientY - pointerY;
    applyRatioFromTopHeight(topHeightPx + deltaY, availableHeightPx);
  }, [isDragging, applyRatioFromTopHeight]);

  const handlePointerUp = useCallback((event: ReactPointerEvent<HTMLDivElement>) => {
    event.currentTarget.releasePointerCapture(event.pointerId);
    setIsDragging(false);
  }, []);

  const handleKeyDown = useCallback((event: ReactKeyboardEvent<HTMLDivElement>) => {
    const container = containerRef.current;
    const topPane = topPaneRef.current;
    if (!container || !topPane) return;

    const step = event.key === 'ArrowUp' ? -KEYBOARD_STEP_PX : event.key === 'ArrowDown' ? KEYBOARD_STEP_PX : 0;
    if (step === 0) return;

    event.preventDefault();
    const availableHeightPx = container.getBoundingClientRect().height - DIVIDER_HEIGHT_PX;
    applyRatioFromTopHeight(topPane.getBoundingClientRect().height + step, availableHeightPx);
  }, [applyRatioFromTopHeight]);

  return (
    <div ref={containerRef} className={styles.container}>
      <div
        ref={topPaneRef}
        className={styles.pane}
        style={{ flexGrow: topRatio, flexBasis: 0, minHeight: minTopPx }}
      >
        {top}
      </div>

      <div
        className={`${styles.divider} ${isDragging ? styles.dividerActive : ''}`}
        style={{ height: DIVIDER_HEIGHT_PX }}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onKeyDown={handleKeyDown}
        role="separator"
        aria-orientation="horizontal"
        aria-label="Resize map and camera panels"
        tabIndex={0}
      >
        <div className={styles.dividerGrip} />
      </div>

      <div
        className={styles.pane}
        style={{ flexGrow: 1 - topRatio, flexBasis: 0, minHeight: minBottomPx }}
      >
        {bottom}
      </div>
    </div>
  );
}
