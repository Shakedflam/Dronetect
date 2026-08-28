export type Unsubscribe = () => void;

/**
 * Shared plumbing behind the mock services: turns a pure
 * `(elapsedMs) => T` function into a subscribe/notify source, polling on an
 * interval only while at least one listener is attached. Each mock service
 * (location, telemetry, detection) wraps one of these around its own mock
 * data function so the timer/listener bookkeeping isn't repeated three
 * times, and none of it survives the swap to a real WebSocket/REST feed.
 */
export function createPollingSource<T>(getValue: (elapsedMs: number) => T, intervalMs: number) {
  const listeners = new Set<(value: T) => void>();
  const startedAt = Date.now();
  let timer: ReturnType<typeof setInterval> | null = null;

  function elapsed(): number {
    return Date.now() - startedAt;
  }

  function notifyAll(): void {
    const value = getValue(elapsed());
    listeners.forEach((listener) => listener(value));
  }

  function subscribe(listener: (value: T) => void): Unsubscribe {
    listeners.add(listener);
    if (timer === null) {
      timer = setInterval(notifyAll, intervalMs);
    }
    listener(getValue(elapsed()));

    return () => {
      listeners.delete(listener);
      if (listeners.size === 0 && timer !== null) {
        clearInterval(timer);
        timer = null;
      }
    };
  }

  return { subscribe };
}
