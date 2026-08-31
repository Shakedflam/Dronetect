"""Print scripted tracker lifecycle messages from the real vision WebSocket path."""

from __future__ import annotations

import asyncio
from io import BytesIO
import json
from pathlib import Path
import sys

from PIL import Image
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from config.video import VideoConfig
from services.video import JPEG_ENCODING, VideoFrame, encode_video_frame
from vision_server.detector import Detection
from vision_server.server import VisionServer


class ScriptedDetector:
    def __init__(self) -> None:
        self._steps = [
            [Detection("drone", 0.88, (10.0, 20.0, 110.0, 120.0))],
            [Detection("drone", 0.90, (16.0, 22.0, 116.0, 122.0))],
            [Detection("drone", 0.86, (112.0, 24.0, 212.0, 124.0))],
            [],
            [Detection("drone", 0.91, (118.0, 26.0, 218.0, 126.0))],
            [],
            [],
            [],
            [],
            [],
        ]

    def detect(self, frame: VideoFrame) -> list[Detection]:
        index = min(frame.frame_id - 1, len(self._steps) - 1)
        return self._steps[index]


async def main() -> None:
    vision_server = VisionServer(VideoConfig(), detector=ScriptedDetector())
    async with serve(vision_server.frame_handler, "127.0.0.1", 0) as server:
        port = server.sockets[0].getsockname()[1]
        async with connect(f"ws://127.0.0.1:{port}/frames") as websocket:
            for frame_id, note in enumerate(_notes(), start=1):
                await websocket.send(encode_video_frame(_frame(frame_id)))
                message = json.loads(await websocket.recv())
                print(f"{note}:")
                print(json.dumps(message, indent=2))
                print()


def _notes() -> list[str]:
    return [
        "First detection -> drone-1 / tentative",
        "Second detection -> drone-1 / confirmed",
        "Moves across screen -> still drone-1",
        "One YOLO miss -> drone-1 / lost",
        "Seen again -> drone-1 / confirmed",
        "Long disappearance miss 1 -> drone-1 / lost",
        "Long disappearance miss 2 -> drone-1 / lost",
        "Long disappearance miss 3 -> drone-1 / lost",
        "Long disappearance miss 4 -> drone-1 / lost",
        "Long disappearance miss 5 -> removed",
    ]


def _frame(frame_id: int) -> VideoFrame:
    return VideoFrame(
        frame_id=frame_id,
        timestamp=float(frame_id),
        width=640,
        height=360,
        encoding=JPEG_ENCODING,
        data=_jpeg_bytes(),
    )


def _jpeg_bytes() -> bytes:
    image = Image.new("RGB", (640, 360), color=(12, 18, 24))
    buffer = BytesIO()
    image.save(buffer, format="JPEG", quality=80)
    return buffer.getvalue()


if __name__ == "__main__":
    asyncio.run(main())
