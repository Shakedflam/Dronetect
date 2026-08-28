"""WebSocket client used by the Webots simulator controller."""

from __future__ import annotations

import asyncio
import os
from queue import Empty, Queue
from threading import Event, Thread
from typing import Any

from realtime_server.protocol import (
    encode_message,
    parse_json_message,
    validate_command_message,
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

    def __init__(self, url: str | None = None) -> None:
        self.url = url or os.getenv("DRONTECT_SIMULATOR_WS_URL", DEFAULT_SERVER_URL)
        self._outgoing: Queue[dict[str, Any]] = Queue(maxsize=10)
        self._commands: Queue[dict[str, Any]] = Queue()
        self._stop = Event()
        self._thread: Thread | None = None

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

    def _run_loop(self) -> None:
        asyncio.run(self._connect_forever())

    async def _connect_forever(self) -> None:
        while not self._stop.is_set():
            try:
                assert connect is not None
                async with connect(self.url) as websocket:
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
            except Exception:
                if not self._stop.is_set():
                    await asyncio.sleep(RECONNECT_DELAY_S)

    async def _receive_commands(self, websocket) -> None:
        async for raw in websocket:
            try:
                command = validate_command_message(parse_json_message(str(raw)))
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
