import asyncio
import json

from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from realtime_server.protocol import encode_message, operating_config_result
from realtime_server.server import RealtimeServer
from services.gps import LocalPosition
from services.gps import DEFAULT_HOME, destination_point
from services.commands import CommandName
from services.flight_safety import SafetyRthSignal
from services.mission_state import MissionState
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
                await _receive_initial_mission_state(tablet)
                telemetry = build_simulator_telemetry(
                    position_local=LocalPosition(1.0, 2.0, 3.0),
                    heading_deg=45.0,
                    speed_mps=6.0,
                    state="TAKING_OFF",
                    target_patrol_region_id="NE-3",
                    timestamp=1.5,
                )
                await simulator.send(encode_message(telemetry))

                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                received = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert status["type"] == "mission_status"
                assert status["batteryRth"] == {
                    "batteryPercent": 100.0,
                    "lowBattery": False,
                    "rthActive": False,
                    "safetyRthActive": False,
                    "rthReason": None,
                }
                assert status["patrol"]["targetRegionId"] == "NE-3"
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
                await _receive_initial_mission_state(tablet)
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
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "cmd-2",
                            "command": "return_home",
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
                await _receive_initial_mission_state(replacement)
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
                status = json.loads(await asyncio.wait_for(replacement.recv(), 1.0))
                received = json.loads(await asyncio.wait_for(replacement.recv(), 1.0))
                assert status["type"] == "mission_status"
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
                await _receive_initial_mission_state(tablet)
                telemetry = build_simulator_telemetry(
                    position_local=LocalPosition(1.0, 2.0, 3.0),
                    heading_deg=45.0,
                    speed_mps=6.0,
                    state="TAKING_OFF",
                    timestamp=1.5,
                )
                await simulator.send(encode_message(telemetry))

                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                received = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert status["type"] == "mission_status"
                assert received["type"] == "telemetry"
                assert received["source"] == "simulator"

    asyncio.run(scenario())


def test_tablet_receives_initial_and_updated_backend_mission_state():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                initial = await _receive_initial_mission_state(tablet)
                assert initial["state"] == "ON_GROUND"

                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "patrol-1",
                            "command": "start_patrol",
                            "params": {},
                        }
                    )
                )

                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                ack = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                state = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert ack == {
                    "type": "command_ack",
                    "commandId": "patrol-1",
                    "status": "accepted",
                }
                assert forwarded["command"] == "start_patrol"
                assert state["type"] == "mission_status"
                assert state["state"] == "TAKING_OFF"

    asyncio.run(scenario())


def test_set_and_clear_roi_are_forwarded_before_backend_state_changes():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "roi-1",
                            "command": "set_roi",
                            "params": {
                                "center": {
                                    "lat": DEFAULT_HOME.lat,
                                    "lon": DEFAULT_HOME.lng,
                                },
                                "radiusM": 0,
                            },
                        }
                    )
                )
                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                await asyncio.wait_for(tablet.recv(), 1.0)
                state = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert state["state"] == "TAKING_OFF"
                assert forwarded["command"] == "set_roi"
                assert state["activeRoi"] == {
                    "center": {
                        "lat": DEFAULT_HOME.lat,
                        "lon": DEFAULT_HOME.lng,
                    },
                    "radiusM": 0.0,
                }
                assert realtime_server.mission.roi is not None

                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "roi-2",
                            "command": "clear_roi",
                            "params": {},
                        }
                    )
                )
                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                await asyncio.wait_for(tablet.recv(), 1.0)
                cleared = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert cleared["activeRoi"] is None
                assert forwarded["command"] == "clear_roi"
                assert realtime_server.mission.roi is None

    asyncio.run(scenario())


def test_mission_command_is_rejected_without_drone_and_does_not_mutate_state():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            async with connect(
                f"ws://127.0.0.1:{port}/ws?role=tablet"
            ) as tablet:
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "patrol-without-drone",
                            "command": "start_patrol",
                            "params": {},
                        }
                    )
                )

                error = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert error["code"] == "drone_unavailable"
                assert realtime_server.mission.state is MissionState.ON_GROUND

    asyncio.run(scenario())


def test_start_patrol_is_forwarded_and_altitude_changes_backend_state():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                command = {
                    "type": "command",
                    "commandId": "patrol-flight-1",
                    "command": "start_patrol",
                    "params": {},
                }
                await tablet.send(encode_message(command))

                assert json.loads(await asyncio.wait_for(simulator.recv(), 1.0)) == command
                await asyncio.wait_for(tablet.recv(), 1.0)  # acknowledgement
                taking_off = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert taking_off["type"] == "mission_status"
                assert taking_off["state"] == "TAKING_OFF"

                telemetry = build_simulator_telemetry(
                    position_local=LocalPosition(0.0, 0.0, 15.0),
                    heading_deg=0.0,
                    speed_mps=0.0,
                    state="HOLDING",
                )
                await simulator.send(encode_message(telemetry))

                patrol = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert patrol["type"] == "mission_status"
                assert patrol["state"] == "PATROL"
                await asyncio.wait_for(tablet.recv(), 1.0)  # telemetry

    asyncio.run(scenario())


def test_priority_updates_apply_during_patrol_and_empty_set_clears_them():
    async def scenario():
        realtime_server = RealtimeServer()
        realtime_server.mission.apply(CommandName.START_PATROL, {})
        realtime_server.mission.observe_altitude(15.0)
        assert realtime_server.mission.state is MissionState.PATROL

        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            async with connect(f"ws://127.0.0.1:{port}/ws?role=tablet") as tablet:
                await _receive_initial_mission_state(tablet)
                now = realtime_server.patrol_planner.started_at + 1000.0
                before = realtime_server.patrol_planner.choose_next(
                    DEFAULT_HOME, now=now
                )
                assert before.region.band_index == 1

                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "priorities-1",
                            "command": "set_patrol_priorities",
                            "params": {
                                "priorities": [{"regionId": "N-4", "rank": 1}]
                            },
                        }
                    )
                )
                ack = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert ack["status"] == "accepted"
                assert status["patrol"]["priorities"] == [
                    {"regionId": "N-4", "rank": 1}
                ]
                assert realtime_server.mission.state is MissionState.PATROL
                assert realtime_server.patrol_planner.choose_next(
                    DEFAULT_HOME, now=now
                ).region.id == "N-4"

                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "priorities-2",
                            "command": "set_patrol_priorities",
                            "params": {
                                "priorities": [{"regionId": "NE-4", "rank": 1}]
                            },
                        }
                    )
                )
                await asyncio.wait_for(tablet.recv(), 1.0)
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert status["patrol"]["priorities"] == [
                    {"regionId": "NE-4", "rank": 1}
                ]
                assert realtime_server.patrol_regions.get("N-4").priority is None
                assert realtime_server.patrol_regions.get("NE-4").priority == 1

                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "priorities-3",
                            "command": "set_patrol_priorities",
                            "params": {"priorities": []},
                        }
                    )
                )
                await asyncio.wait_for(tablet.recv(), 1.0)
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert status["patrol"]["priorities"] == []
                assert all(
                    region.priority is None
                    for region in realtime_server.patrol_regions.regions
                )

    asyncio.run(scenario())


def test_priority_update_is_forwarded_to_connected_drone():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                command = {
                    "type": "command",
                    "commandId": "priority-forward-1",
                    "command": "set_patrol_priorities",
                    "params": {
                        "priorities": [
                            {"regionId": "N-4", "rank": 1},
                            {"regionId": "NE-4", "rank": 2},
                        ]
                    },
                }
                await tablet.send(encode_message(command))

                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                ack = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert forwarded == command
                assert ack["status"] == "accepted"
                assert status["patrol"]["priorities"] == command["params"]["priorities"]

    asyncio.run(scenario())


def test_area_type_update_is_authoritative_and_forwarded_to_connected_drone():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                command = {
                    "type": "command",
                    "commandId": "area-types-forward-1",
                    "command": "set_patrol_area_types",
                    "params": {
                        "areaTypes": [
                            {"regionId": "N-1", "areaType": "IGNORE"},
                            {"regionId": "N-2", "areaType": "NO_FLY"},
                        ]
                    },
                }
                await tablet.send(encode_message(command))

                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                ack = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert forwarded == command
                assert ack["status"] == "accepted"
                assert status["patrol"]["areaTypes"] == command["params"]["areaTypes"]
                assert realtime_server.patrol_regions.get("N-1").area_type == "IGNORE"
                assert realtime_server.patrol_regions.get("N-2").area_type == "NO_FLY"
                assert realtime_server.patrol_regions.get("N-3").area_type == "PATROL"

    asyncio.run(scenario())


def test_area_type_update_removes_priority_and_compacts_remaining_ranks():
    async def scenario():
        realtime_server = RealtimeServer()
        realtime_server.patrol_regions.replace_priorities(
            [("N-1", 1), ("N-2", 2), ("N-3", 3), ("N-4", 4)]
        )
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            async with connect(f"ws://127.0.0.1:{port}/ws?role=tablet") as tablet:
                initial = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert initial["patrol"]["priorities"] == [
                    {"regionId": "N-1", "rank": 1},
                    {"regionId": "N-2", "rank": 2},
                    {"regionId": "N-3", "rank": 3},
                    {"regionId": "N-4", "rank": 4},
                ]
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "area-types-compact",
                            "command": "set_patrol_area_types",
                            "params": {
                                "areaTypes": [
                                    {"regionId": "N-2", "areaType": "IGNORE"}
                                ]
                            },
                        }
                    )
                )

                ack = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert ack["status"] == "accepted"
                assert status["patrol"]["priorities"] == [
                    {"regionId": "N-1", "rank": 1},
                    {"regionId": "N-3", "rank": 2},
                    {"regionId": "N-4", "rank": 3},
                ]

    asyncio.run(scenario())


def test_roi_inside_no_fly_temporarily_overrides_patrol_avoidance():
    async def scenario():
        realtime_server = RealtimeServer()
        realtime_server.patrol_regions.set_area_type("N-1", "NO_FLY")
        roi_center = destination_point(DEFAULT_HOME, 300.0, 0.0)
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                synced_type = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                assert synced_type["command"] == "set_patrol_area_types"
                initial = await _receive_initial_mission_state(tablet)
                assert initial["patrol"]["areaTypes"] == [
                    {"regionId": "N-1", "areaType": "NO_FLY"}
                ]
                command = {
                    "type": "command",
                    "commandId": "roi-overrides-no-fly",
                    "command": "set_roi",
                    "params": {
                        "center": {
                            "lat": roi_center.lat,
                            "lon": roi_center.lng,
                        },
                        "radiusM": 20.0,
                    },
                }
                await tablet.send(encode_message(command))

                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                ack = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert forwarded == command
                assert ack["status"] == "accepted"
                assert status["activeRoi"] == command["params"]
                assert status["patrol"]["areaTypes"] == [
                    {"regionId": "N-1", "areaType": "NO_FLY"}
                ]

    asyncio.run(scenario())


def test_new_no_fly_area_does_not_cancel_active_roi():
    async def scenario():
        realtime_server = RealtimeServer()
        roi_center = destination_point(DEFAULT_HOME, 300.0, 0.0)
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "active-roi",
                            "command": "set_roi",
                            "params": {
                                "center": {
                                    "lat": roi_center.lat,
                                    "lon": roi_center.lng,
                                },
                                "radiusM": 20.0,
                            },
                        }
                    )
                )
                await asyncio.wait_for(simulator.recv(), 1.0)
                await asyncio.wait_for(tablet.recv(), 1.0)
                await asyncio.wait_for(tablet.recv(), 1.0)

                area_command = {
                    "type": "command",
                    "commandId": "no-fly-during-roi",
                    "command": "set_patrol_area_types",
                    "params": {
                        "areaTypes": [
                            {"regionId": "N-1", "areaType": "NO_FLY"}
                        ]
                    },
                }
                await tablet.send(encode_message(area_command))

                assert json.loads(await asyncio.wait_for(simulator.recv(), 1.0)) == area_command
                await asyncio.wait_for(tablet.recv(), 1.0)
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert status["activeRoi"] is not None
                assert realtime_server.mission.roi is not None

    asyncio.run(scenario())


def test_roi_arrival_updates_state_without_touching_normal_patrol_ages():
    async def scenario():
        realtime_server = RealtimeServer()
        realtime_server.mission.apply(CommandName.START_PATROL, {})
        realtime_server.mission.observe_altitude(15.0)
        realtime_server.patrol_regions.record_observation("N-1", observed_at=42.0)

        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                command = {
                    "type": "command",
                    "commandId": "roi-transit-1",
                    "command": "set_roi",
                    "params": {
                        "center": {
                            "lat": DEFAULT_HOME.lat,
                            "lon": DEFAULT_HOME.lng,
                        },
                        "radiusM": 0,
                    },
                }
                await tablet.send(encode_message(command))

                assert json.loads(await asyncio.wait_for(simulator.recv(), 1.0)) == command
                await asyncio.wait_for(tablet.recv(), 1.0)  # acknowledgement
                transit = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert transit["type"] == "mission_status"
                assert transit["state"] == "TRANSIT_TO_ROI"
                assert transit["activeRoi"] == command["params"]

                telemetry = build_simulator_telemetry(
                    position_local=LocalPosition(0.0, 0.0, 15.0),
                    heading_deg=0.0,
                    speed_mps=0.0,
                    state="HOLDING",
                )
                await simulator.send(encode_message(telemetry))

                arrived = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert arrived["type"] == "mission_status"
                assert arrived["state"] == "ROI_PATROL"
                await asyncio.wait_for(tablet.recv(), 1.0)  # telemetry
                assert (
                    realtime_server.patrol_regions.get("N-1").last_observed
                    == 42.0
                )

    asyncio.run(scenario())


def test_vision_detections_are_forwarded_to_connected_drone():
    class RecordingDrone:
        def __init__(self):
            self.messages = []

        async def send_command(self, message):
            self.messages.append(message)

    async def scenario():
        realtime_server = RealtimeServer()
        drone = RecordingDrone()
        realtime_server.drone = drone
        detections = {
            "type": "detections",
            "frameId": 5,
            "timestampMs": 1000,
            "frameWidth": 640,
            "frameHeight": 480,
            "detections": [],
        }

        await realtime_server._route_vision_detections(detections)

        assert len(drone.messages) == 1
        assert drone.messages[0]["type"] == "detections"
        assert drone.messages[0]["frameId"] == 5
        assert "detectionAgeMs" in drone.messages[0]

    asyncio.run(scenario())


def test_low_battery_telemetry_starts_safety_rth_and_commands_drone():
    class RecordingDrone:
        def __init__(self):
            self.commands = []

        async def send_command(self, command):
            self.commands.append(command)

    async def scenario():
        realtime_server = RealtimeServer()
        realtime_server.mission.apply(CommandName.START_PATROL, {})
        realtime_server.mission.observe_altitude(15.0)
        drone = RecordingDrone()
        realtime_server.drone = drone

        await realtime_server._route_drone_telemetry(
            build_simulator_telemetry(
                position_local=LocalPosition(0.0, 0.0, 15.0),
                heading_deg=0.0,
                speed_mps=0.0,
                state="HOLDING",
                battery_percent=20.0,
            )
        )

        assert realtime_server.mission.state is MissionState.RTH
        assert realtime_server.mission.safety_rth_active is True
        assert realtime_server._mission_status()["batteryRth"] == {
            "batteryPercent": 20.0,
            "lowBattery": True,
            "rthActive": True,
            "safetyRthActive": True,
            "rthReason": "low_battery",
        }
        assert len(drone.commands) == 1
        assert drone.commands[0]["command"] == CommandName.RETURN_HOME.value
        assert drone.commands[0]["commandId"].startswith("safety-rth-")
        assert drone.commands[0]["safetyReason"] == "low_battery"

        await realtime_server._route_drone_telemetry(
            build_simulator_telemetry(
                position_local=LocalPosition(0.0, 0.0, 15.0),
                heading_deg=0.0,
                speed_mps=0.0,
                state="LANDING",
                battery_percent=19.0,
            )
        )
        assert realtime_server.mission.state is MissionState.LANDING
        assert len(drone.commands) == 1

        await realtime_server._route_drone_telemetry(
            build_simulator_telemetry(
                position_local=LocalPosition(0.0, 0.0, 0.1),
                heading_deg=0.0,
                speed_mps=0.0,
                state="LANDED",
                battery_percent=19.0,
            )
        )
        assert realtime_server.mission.state is MissionState.ON_GROUND
        assert realtime_server.mission.safety_rth_active is False

    asyncio.run(scenario())


def test_aircraft_connection_loss_rth_is_latched_by_backend():
    class RecordingDrone:
        def __init__(self):
            self.commands = []

        async def send_command(self, command):
            self.commands.append(command)

    async def scenario():
        realtime_server = RealtimeServer()
        realtime_server.mission.apply(CommandName.START_PATROL, {})
        realtime_server.mission.observe_altitude(15.0)
        drone = RecordingDrone()
        realtime_server.drone = drone

        await realtime_server._route_drone_telemetry(
            build_simulator_telemetry(
                position_local=LocalPosition(0.0, 0.0, 15.0),
                heading_deg=0.0,
                speed_mps=0.0,
                state="RETURNING_HOME",
                aircraft_safety_rth_active=True,
                aircraft_safety_rth_reason="connection_lost",
            )
        )

        assert realtime_server.mission.state is MissionState.RTH
        assert realtime_server.mission.safety_rth_active is True
        assert realtime_server.mission.safety_rth_reason == "connection_lost"
        assert drone.commands[0]["command"] == CommandName.RETURN_HOME.value
        assert drone.commands[0]["safetyReason"] == "connection_lost"

    asyncio.run(scenario())


def test_approve_target_command_cannot_override_active_safety_rth():
    async def scenario():
        realtime_server = RealtimeServer()
        realtime_server.mission.apply(CommandName.START_PATROL, {})
        realtime_server.mission.observe_safety(
            SafetyRthSignal(required=True, reason="low_battery")
        )

        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            async with connect(
                f"ws://127.0.0.1:{port}/ws?role=tablet"
            ) as tablet:
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "blocked-approve-target",
                            "command": "approve_target",
                            "params": {"trackId": "one"},
                        }
                    )
                )

                error = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))

                assert error["type"] == "error"
                assert error["code"] == "invalid_mission_transition"
                assert error["details"]["state"] == "RTH"
                assert realtime_server.mission.state is MissionState.RTH

    asyncio.run(scenario())


def test_operating_config_commits_both_values_after_controller_accepts():
    async def scenario():
        realtime_server = RealtimeServer()
        previous = realtime_server.operating_config
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "config-accepted",
                            "command": "set_operating_config",
                            "params": {
                                "maxFlightRadiusM": 750,
                                "cruiseAltitudeM": 15,
                            },
                        }
                    )
                )

                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                assert realtime_server.operating_config == previous
                await simulator.send(
                    encode_message(
                        operating_config_result(
                            forwarded["commandId"], accepted=True
                        )
                    )
                )

                ack = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert ack["status"] == "accepted"
                assert status["operatingConfig"] == {
                    "maxFlightRadiusM": 750.0,
                    "cruiseAltitudeM": 15.0,
                }
                assert realtime_server.mission.config.takeoff_altitude_m == 15.0
                assert max(
                    region.outer_radius_m
                    for region in realtime_server.patrol_regions.regions
                ) == 750.0

    asyncio.run(scenario())


def test_operating_config_controller_rejection_keeps_previous_pair():
    async def scenario():
        realtime_server = RealtimeServer()
        previous = realtime_server.operating_config
        previous_store = realtime_server.patrol_regions
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "config-rejected",
                            "command": "set_operating_config",
                            "params": {
                                "maxFlightRadiusM": 750,
                                "cruiseAltitudeM": 15,
                            },
                        }
                    )
                )
                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                await simulator.send(
                    encode_message(
                        operating_config_result(
                            forwarded["commandId"],
                            accepted=False,
                            message="grounded only",
                        )
                    )
                )

                error = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert error["type"] == "error"
                assert realtime_server.operating_config == previous
                assert realtime_server.patrol_regions is previous_store

    asyncio.run(scenario())


def test_operating_config_without_drone_syncs_before_later_connection_is_ready():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with connect(f"{base_url}?role=tablet") as tablet:
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "config-offline",
                            "command": "set_operating_config",
                            "params": {
                                "maxFlightRadiusM": 750,
                                "cruiseAltitudeM": 15,
                            },
                        }
                    )
                )
                await asyncio.wait_for(tablet.recv(), 1.0)
                await asyncio.wait_for(tablet.recv(), 1.0)

                async with connect(f"{base_url}?role=drone") as simulator:
                    syncing = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                    not_ready = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                    assert not_ready["droneConnected"] is False
                    await simulator.send(
                        encode_message(
                            operating_config_result(
                                syncing["commandId"], accepted=True
                            )
                        )
                    )
                    ready = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                    assert ready["droneConnected"] is True

    asyncio.run(scenario())


def test_operating_config_is_forwarded_even_when_mission_is_not_on_ground():
    """Reload-lockout fix: the server no longer rejects set_operating_config
    just because the mission isn't ON_GROUND -- it forwards to the drone,
    which is the sole authority on accept/defer/reject."""

    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "patrol-1",
                            "command": "start_patrol",
                            "params": {},
                        }
                    )
                )
                await asyncio.wait_for(simulator.recv(), 1.0)
                await asyncio.wait_for(tablet.recv(), 1.0)
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert status["state"] == "TAKING_OFF"

                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "config-airborne",
                            "command": "set_operating_config",
                            "params": {"maxFlightRadiusM": 750, "cruiseAltitudeM": 15},
                        }
                    )
                )

                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                assert forwarded["command"] == "set_operating_config"
                await simulator.send(
                    encode_message(
                        operating_config_result(forwarded["commandId"], accepted=True)
                    )
                )
                ack = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert ack == {
                    "type": "command_ack",
                    "commandId": "config-airborne",
                    "status": "accepted",
                }

    asyncio.run(scenario())


def test_operating_config_pending_ack_surfaces_in_mission_status_and_clears_on_telemetry():
    async def scenario():
        realtime_server = RealtimeServer()
        previous_config = realtime_server.operating_config
        previous_store = realtime_server.patrol_regions
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with (
                connect(f"{base_url}?role=drone") as simulator,
                connect(f"{base_url}?role=tablet") as tablet,
            ):
                await _receive_initial_mission_state(tablet)
                await tablet.send(
                    encode_message(
                        {
                            "type": "command",
                            "commandId": "config-pending",
                            "command": "set_operating_config",
                            "params": {"maxFlightRadiusM": 750, "cruiseAltitudeM": 15},
                        }
                    )
                )
                forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                await simulator.send(
                    encode_message(
                        operating_config_result(
                            forwarded["commandId"], accepted=True, pending=True
                        )
                    )
                )

                await asyncio.wait_for(tablet.recv(), 1.0)  # command_ack
                status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert status["operatingConfigPending"] is True
                # The drone deferred actually applying this -- it's still
                # flying under the old radius/altitude, so the published
                # config (and anything derived from it, like the map's
                # boundary circle and patrol grid) must not jump to the new
                # values yet. Regression check for the bug where the server
                # swapped these in on acceptance alone, ahead of the drone
                # ever enforcing them.
                assert status["operatingConfig"] == {
                    "maxFlightRadiusM": previous_config.max_flight_radius_m,
                    "cruiseAltitudeM": previous_config.cruise_altitude_m,
                }
                assert realtime_server.operating_config == previous_config
                assert realtime_server.patrol_regions is previous_store

                await simulator.send(
                    encode_message(
                        build_simulator_telemetry(
                            position_local=LocalPosition(0.0, 0.0, 15.0),
                            heading_deg=0.0,
                            speed_mps=0.0,
                            state="PATROLLING",
                            operating_config_pending=False,
                        )
                    )
                )
                cleared = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert cleared["type"] == "mission_status"
                assert cleared["operatingConfigPending"] is False
                # Only now -- once telemetry reports the drone has actually
                # applied it -- does the published config catch up.
                assert cleared["operatingConfig"] == {
                    "maxFlightRadiusM": 750.0,
                    "cruiseAltitudeM": 15.0,
                }
                assert realtime_server.operating_config.max_flight_radius_m == 750.0
                assert max(
                    region.outer_radius_m
                    for region in realtime_server.patrol_regions.regions
                ) == 750.0

    asyncio.run(scenario())


def test_operating_config_pending_resets_on_drone_disconnect():
    async def scenario():
        realtime_server = RealtimeServer()
        async with serve(realtime_server.handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            base_url = f"ws://127.0.0.1:{port}/ws"
            async with connect(f"{base_url}?role=tablet") as tablet:
                await _receive_initial_mission_state(tablet)

                async with connect(f"{base_url}?role=drone") as simulator:
                    await asyncio.wait_for(tablet.recv(), 1.0)  # drone-connected mission_status
                    await tablet.send(
                        encode_message(
                            {
                                "type": "command",
                                "commandId": "config-pending-2",
                                "command": "set_operating_config",
                                "params": {"maxFlightRadiusM": 750, "cruiseAltitudeM": 15},
                            }
                        )
                    )
                    forwarded = json.loads(await asyncio.wait_for(simulator.recv(), 1.0))
                    await simulator.send(
                        encode_message(
                            operating_config_result(
                                forwarded["commandId"], accepted=True, pending=True
                            )
                        )
                    )
                    await asyncio.wait_for(tablet.recv(), 1.0)  # command_ack
                    status = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                    assert status["operatingConfigPending"] is True

                disconnected = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
                assert disconnected["operatingConfigPending"] is False

    asyncio.run(scenario())


async def _wait_for(predicate):
    while not predicate():
        await asyncio.sleep(0.01)


async def _receive_initial_mission_state(tablet):
    message = json.loads(await asyncio.wait_for(tablet.recv(), 1.0))
    assert message["type"] == "mission_status"
    assert isinstance(message["droneConnected"], bool)
    assert message["activeRoi"] is None
    assert message["patrol"]["priorities"] == []
    assert message["batteryRth"]["batteryPercent"] is None
    return message
