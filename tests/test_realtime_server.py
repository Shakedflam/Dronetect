import asyncio
import json

from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from realtime_server.protocol import encode_message
from realtime_server.server import RealtimeServer
from services.gps import LocalPosition
from services.telemetry import build_simulator_telemetry


def test_simulator_telemetry_is_broadcast_to_tablets():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                telemetry = build_simulator_telemetry(
                    position_local=LocalPosition(1.0, 2.0, 3.0),
                    heading_deg=45.0,
                    speed_mps=6.0,
                    state="TAKING_OFF",
                    timestamp=1.5,
                )
                await simulator.send(encode_message(telemetry))

                received = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert received["type"] == "telemetry"
                assert "positionLocal" not in received["data"]
                assert received["data"]["altitudeM"] == 3.0
                assert received["data"]["state"] == "TAKING_OFF"

    asyncio.run(scenario())


def test_tablet_command_is_forwarded_to_simulator_and_acknowledged():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                command = {
                    "type": "command",
                    "commandId": "cmd-1",
                    "command": "return_home",
                    "params": {},
                }
                await tablet.send(encode_message(command))

                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                ack = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert forwarded == command
                assert ack == {
                    "type": "command_ack",
                    "commandId": "cmd-1",
                    "status": "accepted",
                }

    asyncio.run(scenario())


def test_tablet_command_errors_when_no_drone_is_connected():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            async with connect(f"ws://127.0.0.1:{port}/ws?role=tablet") as tablet:
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "cmd-2",
                            "command": "new_mission",
                            "params": {},
                        }
                    )
                )

                error = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert error["type"] == "error"
                assert error["code"] == "drone_unavailable"
                assert error["commandId"] == "cmd-2"

    asyncio.run(scenario())


def test_second_tablet_is_rejected_and_reconnect_succeeds_after_first_closes():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            url = f"ws://127.0.0.1:{port}/ws?role=tablet"

            first = await connect(url)
            try:
                async with connect(url) as second:
                    error = json.loads(await asyncio.wait_for(second.recv(), 1.0))
                    assert error["code"] == "tablet_already_connected"
            finally:
                await first.close()

            await asyncio.wait_for(_wait_for(lambda: realtime_server.tablet is None), 1.0)
            async with connect(url) as replacement:
                await asyncio.wait_for(
                    _wait_for(lambda: realtime_server.tablet is not None),
                    1.0,
                )
                await realtime_server._route_drone_telemetry(
                    build_simulator_telemetry(
                        position_local=LocalPosition(1.0, 2.0, 3.0),
                        heading_deg=45.0,
                        speed_mps=6.0,
                        state="TAKING_OFF",
                        timestamp=1.5,
                    )
                )
                received = json.loads(await asyncio.wait_for(replacement.recv(), 1.0))
                assert received["type"] == "telemetry"

    asyncio.run(scenario())


def test_legacy_simulator_role_is_accepted_as_drone_source():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=simulator") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                telemetry = build_simulator_telemetry(
                    position_local=LocalPosition(1.0, 2.0, 3.0),
                    heading_deg=45.0,
                    speed_mps=6.0,
                    state="TAKING_OFF",
                    timestamp=1.5,
                )
                await simulator.send(encode_message(telemetry))

                received = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert received["type"] == "telemetry"
                assert received["source"] == "simulator"

    asyncio.run(scenario())


async def _wait_for(predicate):
    while not predicate():
        await asyncio.sleep(0.01)
