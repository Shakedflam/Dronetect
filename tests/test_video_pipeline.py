import asyncio
import json

from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from config.video import VideoConfig
from realtime_server.protocol import ROLE_DRONE, encode_message
from realtime_server.server import RealtimeServer
from realtime_server.video_pipeline import ROLE_VIEWER, VideoPipeline, validate_video_role
from services.video import (
    JPEG_ENCODING,
    VideoFrame,
    VideoFrameBundle,
    decode_video_frame,
    encode_video_frame_bundle,
)


def test_video_roles_are_validated_separately():
    assert validate_video_role("drone", expected=ROLE_DRONE) == ROLE_DRONE
    assert validate_video_role("simulator", expected=ROLE_DRONE) == ROLE_DRONE
    assert validate_video_role("viewer", expected=ROLE_VIEWER) == ROLE_VIEWER


def test_video_pipeline_uses_configured_vision_frame_interval():
    async def scenario():
        pipeline = VideoPipeline(
            VideoConfig(
                vision_frame_interval=3,
                vision_server_frames_url="ws://127.0.0.1:9/frames",
            )
        )

        for frame_id in range(1, 7):
            await pipeline.publish(_bundle(frame_id))

        latest = pipeline.vision._queue.get_nowait()
        _, ui_frame = await asyncio.wait_for(pipeline.hub._frames.wait_next(0), 1.0)

        assert ui_frame.frame_id == 6
        assert ui_frame.width == 960
        assert ui_frame.data == b"ui-jpeg-6"
        assert latest.frame_id == 6
        assert latest.width == 1280
        assert latest.data == b"vision-jpeg-6"

    asyncio.run(scenario())


def test_vision_forwarder_keeps_only_latest_frame():
    async def scenario():
        pipeline = VideoPipeline(VideoConfig())
        sent_packets = []

        class FakeWebSocket:
            def __init__(self) -> None:
                self._received = asyncio.Event()

            async def send(self, packet):
                sent_packets.append(packet)
                self._received.set()

            async def recv(self):
                await self._received.wait()
                frame = decode_video_frame(sent_packets[-1])
                return encode_message(
                    {
                        "type": "detections",
                        "frameId": frame.frame_id,
                        "timestampMs": int(frame.timestamp * 1000),
                        "frameWidth": frame.width,
                        "frameHeight": frame.height,
                        "detections": [],
                    }
                )

        pipeline.vision.publish(_vision_frame(1))
        pipeline.vision.publish(_vision_frame(2))
        pipeline.vision.publish(_vision_frame(3))

        sender = asyncio.create_task(pipeline.vision._exchange_frames_and_detections(FakeWebSocket()))
        await asyncio.sleep(0.05)
        sender.cancel()
        try:
            await sender
        except asyncio.CancelledError:
            pass

        assert len(sent_packets) == 1
        assert decode_video_frame(sent_packets[0]).frame_id == 3

    asyncio.run(scenario())


def test_vision_forwarder_waits_for_detection_before_sending_next_frame():
    async def scenario():
        pipeline = VideoPipeline(VideoConfig())
        sent_packets = []

        class SlowVisionWebSocket:
            async def send(self, packet):
                sent_packets.append(packet)

            async def recv(self):
                await asyncio.Future()

        pipeline.vision.publish(_vision_frame(1))
        exchange = asyncio.create_task(
            pipeline.vision._exchange_frames_and_detections(SlowVisionWebSocket())
        )
        await asyncio.sleep(0.05)

        pipeline.vision.publish(_vision_frame(2))
        pipeline.vision.publish(_vision_frame(3))
        await asyncio.sleep(0.05)

        exchange.cancel()
        try:
            await exchange
        except asyncio.CancelledError:
            pass

        assert len(sent_packets) == 1
        assert decode_video_frame(sent_packets[0]).frame_id == 1

    asyncio.run(scenario())


def test_realtime_video_ingest_forwards_even_frames_to_vision_server():
    async def scenario():
        received_frame_ids = []

        async def vision_handler(websocket):
            async for packet in websocket:
                frame = decode_video_frame(bytes(packet))
                received_frame_ids.append(frame.frame_id)
                await websocket.send(
                    encode_message(
                        {
                            "type": "detections",
                            "frameId": frame.frame_id,
                            "timestampMs": int(frame.timestamp * 1000),
                            "frameWidth": frame.width,
                            "frameHeight": frame.height,
                            "detections": [],
                        }
                    )
                )
                if len(received_frame_ids) >= 2:
                    await websocket.close()
                    return

        async with serve(vision_handler, "127.0.0.1", 0) as vision:
            vision_port = vision.sockets[0].getsockname()[1]
            realtime_server = RealtimeServer(
                VideoConfig(
                    vision_server_frames_url=f"ws://127.0.0.1:{vision_port}/frames",
                )
            )
            realtime_server.start_background_tasks()
            try:
                async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
                    port = server.sockets[0].getsockname()[1]
                    await asyncio.sleep(0.2)
                    async with connect(
                        f"ws://127.0.0.1:{port}/video/ingest?role=drone"
                    ) as source:
                        for frame_id in range(1, 5):
                            await source.send(encode_video_frame_bundle(_bundle(frame_id)))
                            await asyncio.sleep(0.05)

                    await asyncio.wait_for(
                        _wait_for(lambda: len(received_frame_ids) >= 2),
                        2.0,
                    )
            finally:
                await realtime_server.video_pipeline.vision.stop()

        assert received_frame_ids == [2, 4]

    asyncio.run(scenario())


def test_realtime_video_ingest_forwards_vision_detections_to_tablets():
    async def scenario():
        async def vision_handler(websocket):
            async for packet in websocket:
                frame = decode_video_frame(bytes(packet))
                await websocket.send(
                    encode_message(
                        {
                            "type": "detections",
                            "frameId": frame.frame_id,
                            "timestampMs": int(frame.timestamp * 1000),
                            "frameWidth": frame.width,
                            "frameHeight": frame.height,
                            "detections": [
                                {
                                    "className": "drone",
                                    "confidence": 0.87,
                                    "box": [10.0, 20.0, 110.0, 120.0],
                                    "trackId": "drone-1",
                                    "trackState": "confirmed",
                                    "lastSeenTimestamp": frame.timestamp,
                                    "velocityPxPerSec": {"x": 12.0, "y": -4.0},
                                }
                            ],
                        }
                    )
                )

        async with serve(vision_handler, "127.0.0.1", 0) as vision:
            vision_port = vision.sockets[0].getsockname()[1]
            realtime_server = RealtimeServer(
                VideoConfig(
                    vision_frame_interval=2,
                    vision_server_frames_url=f"ws://127.0.0.1:{vision_port}/frames",
                )
            )
            realtime_server.start_background_tasks()
            try:
                async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
                    port = server.sockets[0].getsockname()[1]
                    await asyncio.sleep(0.2)
                    async with (
                        connect(f"ws://127.0.0.1:{port}/ws?role=tablet") as tablet,
                        connect(f"ws://127.0.0.1:{port}/video/ingest?role=drone") as source,
                    ):
                        await source.send(encode_video_frame_bundle(_bundle(1)))
                        await source.send(encode_video_frame_bundle(_bundle(2)))
                        raw = await asyncio.wait_for(tablet.recv(), 2.0)
            finally:
                await realtime_server.video_pipeline.vision.stop()

        return json.loads(raw)

    received = asyncio.run(scenario())

    assert received == {
        "type": "detections",
        "frameId": 2,
        "timestampMs": 2000,
        "frameWidth": 1280,
        "frameHeight": 720,
        "detections": [
            {
                "className": "drone",
                "confidence": 0.87,
                "box": [10.0, 20.0, 110.0, 120.0],
                "trackId": "drone-1",
                "trackState": "confirmed",
                "lastSeenTimestamp": 2.0,
                "velocityPxPerSec": {"x": 12.0, "y": -4.0},
            }
        ],
    }


def test_video_ingest_continues_when_vision_server_is_offline():
    async def scenario():
        realtime_server = RealtimeServer(
            VideoConfig(vision_server_frames_url="ws://127.0.0.1:9/frames")
        )
        realtime_server.start_background_tasks()
        try:
            async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
                port = server.sockets[0].getsockname()[1]
                async with connect(f"ws://127.0.0.1:{port}/video/ingest?role=drone") as source:
                    for frame_id in range(1, 6):
                        await source.send(encode_video_frame_bundle(_bundle(frame_id)))
        finally:
            await realtime_server.video_pipeline.vision.stop()

    asyncio.run(scenario())


async def _wait_for(predicate):
    while not predicate():
        await asyncio.sleep(0.01)


def _bundle(frame_id: int) -> VideoFrameBundle:
    return VideoFrameBundle(
        ui=_ui_frame(frame_id),
        vision=_vision_frame(frame_id),
    )


def _ui_frame(frame_id: int) -> VideoFrame:
    return VideoFrame(
        frame_id=frame_id,
        timestamp=float(frame_id),
        width=960,
        height=540,
        encoding=JPEG_ENCODING,
        data=f"ui-jpeg-{frame_id}".encode("ascii"),
    )


def _vision_frame(frame_id: int) -> VideoFrame:
    return VideoFrame(
        frame_id=frame_id,
        timestamp=float(frame_id),
        width=1280,
        height=720,
        encoding=JPEG_ENCODING,
        data=f"vision-jpeg-{frame_id}".encode("ascii"),
    )
