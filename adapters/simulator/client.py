"""WebSocket client used by the Webots simulator controller."""

from __future__ import annotations

import asyncio
import os
from queue import Empty, Queue
from threading import Event, Lock, Thread
import time
from typing import Any, Callable

from realtime_server.protocol import (
    MESSAGE_TYPE_DETECTIONS,
    encode_message,
    parse_json_message,
    validate_command_message,
    validate_detection_message,
)

try:
    from websockets.asyncio.client import connect
except ImportError:  # The simulator should still fly if server deps are absent.
    connect = None


DEFAULT_SERVER_URL = "ws://127.0.0.1:8765/ws?role=drone"
RECONNECT_DELAY_S = 1.0
QUEUE_TIMEOUT_S = 0.1


class SimulatorRealtimeClient:
    """Non-blocking realtime server client for Webots' synchronous loop."""

    def __init__(
        self,
        url: str | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.url = url or os.getenv("DRONTECT_SIMULATOR_WS_URL", DEFAULT_SERVER_URL)
        self._outgoing: Queue[dict[str, Any]] = Queue(maxsize=10)
        self._commands: Queue[dict[str, Any]] = Queue()
        self._detections: Queue[dict[str, Any]] = Queue(maxsize=2)
        self._stop = Event()
        self._thread: Thread | None = None
        self._clock = clock
        self._connected = Event()
        self._ever_connected = Event()
        self._connection_state_lock = Lock()
        self._disconnected_at: float | None = None

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def connection_lost_for(self, timeout_s: float) -> bool:
        """Return true after an established command link exceeds its grace period."""
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if self.connected or not self._ever_connected.is_set():
            return False
        with self._connection_state_lock:
            disconnected_at = self._disconnected_at
        return (
            disconnected_at is not None
            and self._clock() - disconnected_at >= timeout_s
        )

    def start(self) -> None:
        if connect is None or self._thread is not None:
            return

        self._thread = Thread(
            target=self._run_loop,
            name="drontect-simulator-realtime-client",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def publish_telemetry(self, telemetry: dict[str, Any]) -> None:
        if connect is None:
            return
        if self._outgoing.full():
            try:
                self._outgoing.get_nowait()
            except Empty:
                pass
        self._outgoing.put_nowait(telemetry)

    def drain_commands(self) -> list[dict[str, Any]]:
        commands: list[dict[str, Any]] = []
        while True:
            try:
                commands.append(self._commands.get_nowait())
            except Empty:
                return commands

    def drain_detections(self) -> list[dict[str, Any]]:
        """Return pending vision frames, dropping old backlog under load."""
        detections: list[dict[str, Any]] = []
        while True:
            try:
                detections.append(self._detections.get_nowait())
            except Empty:
                return detections

    def _run_loop(self) -> None:
        asyncio.run(self._connect_forever())

    async def _connect_forever(self) -> None:
        while not self._stop.is_set():
            try:
                assert connect is not None
                async with connect(self.url) as websocket:
                    self._mark_connected()
                    try:
                        print(f"Realtime command link connected: {self.url}")
                        receiver = asyncio.create_task(self._receive_commands(websocket))
                        sender = asyncio.create_task(self._send_telemetry(websocket))
                        done, pending = await asyncio.wait(
                            {receiver, sender},
                            return_when=asyncio.FIRST_COMPLETED,
                        )
                        for task in pending:
                            task.cancel()
                        for task in done:
                            task.result()
                    finally:
                        self._mark_disconnected()
            except Exception as exc:
                if not self._stop.is_set():
                    print(f"Realtime command link unavailable: {exc}")
                    await asyncio.sleep(RECONNECT_DELAY_S)

    def _mark_connected(self) -> None:
        with self._connection_state_lock:
            self._disconnected_at = None
        self._ever_connected.set()
        self._connected.set()

    def _mark_disconnected(self) -> None:
        self._connected.clear()
        if not self._ever_connected.is_set():
            return
        with self._connection_state_lock:
            self._disconnected_at = self._clock()

    async def _receive_commands(self, websocket) -> None:
        async for raw in websocket:
            try:
                message = parse_json_message(str(raw))
                if message.get("type") == MESSAGE_TYPE_DETECTIONS:
                    detection = validate_detection_message(message)
                    if self._detections.full():
                        try:
                            self._detections.get_nowait()
                        except Empty:
                            pass
                    self._detections.put_nowait(detection)
                    continue
                command = validate_command_message(message)
            except Exception:
                continue
            self._commands.put(command)

    async def _send_telemetry(self, websocket) -> None:
        while not self._stop.is_set():
            try:
                telemetry = await asyncio.to_thread(
                    self._outgoing.get,
                    True,
                    QUEUE_TIMEOUT_S,
                )
            except Empty:
                continue
            await websocket.send(encode_message(telemetry))
