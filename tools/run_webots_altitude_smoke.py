"""Run the real Webots/video/vision pursuit path and validate altitude convergence."""

from __future__ import annotations

import argparse
import asyncio
from datetime import datetime
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

from websockets.asyncio.client import connect


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEBOTS = Path(
    r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
)
def _wait_for_port(port: int, process: subprocess.Popen, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"process exited before port {port} became ready")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.25):
                return
        except OSError:
            time.sleep(0.1)
    raise TimeoutError(f"port {port} did not become ready in {timeout_s:g}s")


async def _start_patrol_when_drone_is_ready(
    port: int,
    timeout_s: float = 60.0,
) -> None:
    async with connect(f"ws://127.0.0.1:{port}/ws?role=tablet") as tablet:
        async def send_command(command_id: str, command: str, params: dict) -> None:
            await tablet.send(json.dumps({
                "type": "command", "commandId": command_id,
                "command": command, "params": params,
            }))
            while asyncio.get_running_loop().time() < deadline:
                reply = json.loads(await asyncio.wait_for(tablet.recv(), 5.0))
                if reply.get("commandId") != command_id:
                    continue
                if reply.get("type") != "command_ack":
                    raise RuntimeError(f"{command} was rejected: {reply}")
                return
            raise TimeoutError(f"{command} acknowledgement timed out")

        deadline = asyncio.get_running_loop().time() + timeout_s
        while asyncio.get_running_loop().time() < deadline:
            try:
                message = json.loads(await asyncio.wait_for(tablet.recv(), 5.0))
            except TimeoutError:
                continue
            if message.get("type") != "telemetry":
                continue
            await send_command(
                "webots-altitude-priority", "set_patrol_priorities",
                {"priorities": [{"regionId": "SW-1", "rank": 1}]},
            )
            await send_command("webots-altitude-smoke", "start_patrol", {})
            shared = json.loads(
                (REPO_ROOT / "config" / "shared.json").read_text(encoding="utf-8")
            )
            home = shared["home"]
            route_center_x, route_center_y = -2.0, 0.0
            route_center = {
                "lat": home["lat"] + route_center_y / 111_320.0,
                "lon": home["lng"] + route_center_x / (
                    111_320.0 * math.cos(math.radians(home["lat"]))
                ),
            }
            await send_command(
                "webots-altitude-roi", "set_roi",
                {"center": route_center, "radiusM": 3.0},
            )
            return
        raise TimeoutError("drone telemetry did not arrive before command timeout")


def _terminate(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=subprocess.CREATE_NO_WINDOW,
            check=False,
        )
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def _verify_scenario() -> None:
    scenario = json.loads(
        (REPO_ROOT / "config" / "simulation_scenario.json").read_text(encoding="utf-8")
    )
    traffic = scenario.get("traffic_drones", {}).get("count")
    birds = scenario.get("birds", {}).get("count")
    if traffic != 1 or birds != 0:
        raise RuntimeError(
            "altitude smoke check requires exactly one traffic drone and no birds "
            f"(found traffic_drones={traffic!r}, birds={birds!r})"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--webots", type=Path, default=DEFAULT_WEBOTS)
    parser.add_argument("--wall-timeout", type=float, default=180.0)
    args = parser.parse_args()
    _verify_scenario()
    if not args.webots.is_file():
        raise FileNotFoundError(f"Webots executable not found: {args.webots}")
    model = REPO_ROOT / "models" / "drone.pt"
    if not model.is_file():
        raise FileNotFoundError(f"YOLO model not found: {model}")

    output_dir = REPO_ROOT / "artifacts" / "webots_altitude_smoke" / datetime.now().strftime("%Y%m%d-%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=True)
    result_path = output_dir / "result.json"
    environment = os.environ.copy()
    realtime_port = 18765
    environment.update({
        "PYTHONUNBUFFERED": "1",
        "DRONTECT_WEBOTS_ALTITUDE_SMOKE": "1",
        "DRONTECT_YOLO_MODEL_PATH": str(model),
        "DRONTECT_YOLO_CONFIDENCE": "0.10",
        "DRONTECT_TARGET_MIN_CONFIDENCE": "0.10",
        "DRONTECT_TARGET_OBSERVATION_S": "0",
        "DRONTECT_PURSUIT_MAX_SPEED_MPS": "0.01",
        "DRONTECT_PURSUIT_ACCELERATION_MPS2": "0.5",
        "DRONTECT_REALTIME_PORT": str(realtime_port),
        "DRONTECT_SIMULATOR_WS_URL": f"ws://127.0.0.1:{realtime_port}/ws?role=drone",
        "DRONTECT_SIMULATOR_VIDEO_WS_URL": f"ws://127.0.0.1:{realtime_port}/video/ingest?role=drone",
        "DRONTECT_ALTITUDE_SMOKE_RESULT_PATH": str(result_path),
    })
    python = REPO_ROOT / ".venv" / "Scripts" / "python.exe"
    processes: list[subprocess.Popen] = []
    log_handles = []
    webots_process: subprocess.Popen | None = None
    try:
        for name, command in (
            ("realtime", [str(python), "-m", "realtime_server.server"]),
            ("vision", [str(python), "-m", "vision_server.server"]),
        ):
            handle = (output_dir / f"{name}.log").open("w", encoding="utf-8")
            log_handles.append(handle)
            process = subprocess.Popen(
                command, cwd=REPO_ROOT, env=environment,
                stdout=handle, stderr=subprocess.STDOUT, text=True,
            )
            processes.append(process)
        _wait_for_port(realtime_port, processes[0], 30.0)
        _wait_for_port(8870, processes[1], 60.0)

        webots_log = (output_dir / "webots.log").open("w", encoding="utf-8")
        log_handles.append(webots_log)
        webots_process = subprocess.Popen(
            [
                str(args.webots), "--batch", "--mode=realtime", "--no-rendering",
                "--stdout", "--stderr", "--port=1239",
                str(REPO_ROOT / "worlds" / "drone_poc.wbt"),
            ],
            cwd=REPO_ROOT, env=environment,
            stdout=webots_log, stderr=subprocess.STDOUT, text=True,
        )
        asyncio.run(_start_patrol_when_drone_is_ready(realtime_port))
        try:
            webots_process.wait(timeout=args.wall_timeout)
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError(
                f"Webots did not finish within {args.wall_timeout:g} wall seconds"
            ) from exc
    finally:
        _terminate(webots_process)
        for process in reversed(processes):
            _terminate(process)
        for handle in log_handles:
            handle.close()

    if not result_path.is_file():
        print(f"Altitude smoke check produced no result. Logs: {output_dir}", file=sys.stderr)
        return 1
    result = json.loads(result_path.read_text(encoding="utf-8"))
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"Logs: {output_dir}")
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
