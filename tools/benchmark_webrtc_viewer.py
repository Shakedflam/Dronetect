"""Measure the UI WebRTC video lane from the realtime server."""

from __future__ import annotations

import argparse
import asyncio
import json
import statistics
import time

from aiortc import RTCPeerConnection, RTCSessionDescription
from websockets.asyncio.client import connect


async def _wait_for_ice_gathering_complete(peer: RTCPeerConnection) -> None:
    if peer.iceGatheringState == "complete":
        return

    done = asyncio.Event()

    @peer.on("icegatheringstatechange")
    def on_ice_gathering_state_change() -> None:
        if peer.iceGatheringState == "complete":
            done.set()

    try:
        await asyncio.wait_for(done.wait(), 2.0)
    except asyncio.TimeoutError:
        pass


async def benchmark(url: str, duration_s: float) -> dict[str, object]:
    peer = RTCPeerConnection()
    peer.addTransceiver("video", direction="recvonly")
    track_ready: asyncio.Future = asyncio.get_running_loop().create_future()

    @peer.on("track")
    def on_track(track) -> None:
        if track.kind == "video" and not track_ready.done():
            track_ready.set_result(track)

    try:
        async with connect(url) as websocket:
            offer = await peer.createOffer()
            await peer.setLocalDescription(offer)
            await _wait_for_ice_gathering_complete(peer)
            assert peer.localDescription is not None
            await websocket.send(
                json.dumps(
                    {
                        "type": "offer",
                        "sdp": peer.localDescription.sdp,
                    },
                    separators=(",", ":"),
                )
            )

            raw_answer = await asyncio.wait_for(websocket.recv(), 5.0)
            answer = json.loads(str(raw_answer))
            await peer.setRemoteDescription(
                RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
            )

            track = await asyncio.wait_for(track_ready, 5.0)
            started = time.perf_counter()
            previous = started
            gaps_ms: list[float] = []
            widths: set[int] = set()
            heights: set[int] = set()
            frames = 0

            while time.perf_counter() - started < duration_s:
                frame = await asyncio.wait_for(track.recv(), 2.0)
                now = time.perf_counter()
                gaps_ms.append((now - previous) * 1000.0)
                previous = now
                widths.add(frame.width)
                heights.add(frame.height)
                frames += 1

            elapsed = time.perf_counter() - started
            sorted_gaps = sorted(gaps_ms[1:] or gaps_ms)
            p95_gap_ms = (
                sorted_gaps[int(0.95 * (len(sorted_gaps) - 1))]
                if sorted_gaps
                else 0.0
            )
            return {
                "frames": frames,
                "elapsed_s": round(elapsed, 3),
                "fps": round(frames / elapsed, 2) if elapsed > 0 else 0.0,
                "widths": sorted(widths),
                "heights": sorted(heights),
                "avg_gap_ms": round(statistics.mean(gaps_ms[1:] or gaps_ms), 2)
                if gaps_ms
                else 0.0,
                "p95_gap_ms": round(p95_gap_ms, 2),
                "max_gap_ms": round(max(gaps_ms[1:] or gaps_ms), 2)
                if gaps_ms
                else 0.0,
            }
    finally:
        await peer.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--url",
        default="ws://127.0.0.1:8765/video/webrtc?role=viewer",
    )
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()
    print(json.dumps(asyncio.run(benchmark(args.url, args.duration)), indent=2))


if __name__ == "__main__":
    main()
