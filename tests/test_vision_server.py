import asyncio
import json

from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from config.video import VideoConfig
from services.video import JPEG_ENCODING, VideoFrame, encode_video_frame
from vision_server.server import VisionServer


def test_vision_server_receives_frames_and_updates_diagnostics():
    async def scenario():
        vision_server = VisionServer(VideoConfig())
        async with serve(vision_server.frame_handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            async with connect(f"ws://127.0.0.1:{port}/frames") as websocket:
                await websocket.send(encode_video_frame(_frame(10)))
                await websocket.send(encode_video_frame(_frame(12)))
                await asyncio.wait_for(
                    _wait_for(lambda: vision_server.diagnostics.received_count == 2),
                    1.0,
                )

        diagnostics = vision_server.diagnostics.to_dict()
        assert diagnostics["received_count"] == 2
        assert diagnostics["latest_frame_id"] == 12
        assert diagnostics["fps"] >= 0.0

    asyncio.run(scenario())


def test_vision_server_exposes_http_diagnostics():
    async def scenario():
        vision_server = VisionServer(VideoConfig())
        vision_server.diagnostics.record(_frame(8))

        server = await asyncio.start_server(
            vision_server.diagnostics_handler,
            "127.0.0.1",
            0,
        )
        async with server:
            port = server.sockets[0].getsockname()[1]
            reader, writer = await asyncio.open_connection("127.0.0.1", port)
            writer.write(b"GET /diagnostics HTTP/1.1\r\nHost: localhost\r\n\r\n")
            await writer.drain()
            response = await reader.read()
            writer.close()
            await writer.wait_closed()

        body = response.split(b"\r\n\r\n", 1)[1]
        parsed = json.loads(body.decode("utf-8"))

        assert parsed["received_count"] == 1
        assert parsed["latest_frame_id"] == 8
        assert "fps" in parsed

    asyncio.run(scenario())


async def _wait_for(predicate):
    while not predicate():
        await asyncio.sleep(0.01)


def _frame(frame_id: int) -> VideoFrame:
    return VideoFrame(
        frame_id=frame_id,
        timestamp=float(frame_id),
        width=640,
        height=360,
        encoding=JPEG_ENCODING,
        data=b"jpeg",
    )
