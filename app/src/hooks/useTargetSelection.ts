import { useEffect, useState } from 'react';

/** Keep operator selection after disappearance, without retaining a video box. */
export function useTargetSelection(visibleIds: string[], activeId: string | null) {
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const activeVisible = activeId !== null && visibleIds.includes(activeId);
  const selectedVisible = selectedId !== null && visibleIds.includes(selectedId);

  // Adjusting state during render (not in an effect) per React's guidance for
  // deriving state from props: the guard makes this a one-shot correction,
  // not a loop, and it avoids an extra commit-then-effect render pass.
  if (selectedId === null && activeVisible) {
    setSelectedId(activeId);
  }

  useEffect(() => {
    if (selectedId === null || selectedVisible) return;
    const timer = window.setTimeout(() => setSelectedId(null), 1500);
    return () => window.clearTimeout(timer);
  }, [selectedId, selectedVisible]);

  return [selectedId, setSelectedId] as const;
}
