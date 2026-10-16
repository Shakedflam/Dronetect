import asyncio
import json

import pytest
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve
from websockets.exceptions import InvalidStatus
from websockets.http11 import Request
from websockets.datastructures import Headers

from realtime_server.server import RealtimeServer
from services.access_control import AccessControl, TOKEN_ENV, client_auth_headers, request_role


TOKENS = {role: str(index) * 64 for index, role in enumerate(TOKEN_ENV, 1)}


def test_production_requires_distinct_well_formed_role_keys(monkeypatch):
    monkeypatch.delenv("DRONTECT_ALLOW_INSECURE_LOCAL")
    for name in TOKEN_ENV.values():
        monkeypatch.delenv(name, raising=False)
    with pytest.raises(ValueError, match="refusing to start"):
        AccessControl.from_env().validate_listener("0.0.0.0", ("operator",))
    monkeypatch.setenv("DRONTECT_OPERATOR_TOKEN", "bad-key")
    with pytest.raises(ValueError, match="64 lowercase hex"):
        AccessControl.from_env()
    monkeypatch.setenv("DRONTECT_OPERATOR_TOKEN", TOKENS["operator"])
    monkeypatch.setenv("DRONTECT_VIEWER_TOKEN", TOKENS["operator"])
    with pytest.raises(ValueError, match="different"):
        AccessControl.from_env()


def test_insecure_mode_cannot_bind_public_interfaces_or_trust_forwarded_ip():
    access = AccessControl(allow_insecure_local=True)
    with pytest.raises(ValueError, match="loopback"):
        access.validate_listener("0.0.0.0", ("operator",))
    access.validate_listener("127.0.0.1", ("operator",))
    request = Request("/ws?role=tablet", Headers({"X-Forwarded-For": "127.0.0.1"}))
    assert not access.authorized(request, "operator", "192.0.2.1")


def test_query_tokens_duplicate_credentials_and_role_escalation_are_rejected():
    access = AccessControl(TOKENS)
    assert not access.authorized(Request("/ws?role=tablet&token=" + TOKENS["operator"], Headers()), "operator")
    for role in ("viewer", "drone", "ingest", "vision"):
        request = Request("/ws?role=tablet", Headers({"Authorization": "Bearer " + TOKENS[role]}))
        assert not access.authorized(request, "operator")
    request = Request("/ws?role=tablet", Headers([
        ("Authorization", "Bearer " + TOKENS["operator"]),
        ("Sec-WebSocket-Protocol", "dronetect-auth." + TOKENS["operator"]),
    ]))
    assert not access.authorized(request, "operator")
    assert request_role("/ws?role=tablet&role=drone") is None
    assert request_role("/video/webrtc?role=tablet") is None


@pytest.mark.parametrize("path,role", [
    ("/ws?role=tablet", "operator"), ("/ws?role=drone", "drone"),
    ("/video/ingest?role=drone", "ingest"), ("/video/webrtc?role=viewer", "viewer"),
])
def test_unauthenticated_and_wrong_role_connections_are_rejected_before_upgrade(path, role):
    async def scenario():
        server = RealtimeServer(access=AccessControl(TOKENS))
        async with serve(server.handler, "127.0.0.1", 0, process_request=server.process_request) as listener:
            url = f"ws://127.0.0.1:{listener.sockets[0].getsockname()[1]}{path}"
            wrong = TOKENS["viewer" if role != "viewer" else "operator"]
            for headers in ({}, {"Authorization": "Bearer " + wrong}):
                with pytest.raises(InvalidStatus) as exc:
                    async with connect(url, additional_headers=headers):
                        pytest.fail("Unauthorized upgrade succeeded")
                assert exc.value.response.status_code == 401
            assert server.tablet is None and server.drone is None and server.video_source is None
            assert not server._video_pipeline_started
    asyncio.run(scenario())


def test_authorized_browser_operator_and_drone_can_exchange_commands():
    async def scenario():
        server = RealtimeServer(access=AccessControl(TOKENS))
        async with serve(server.handler, "127.0.0.1", 0, process_request=server.process_request) as listener:
            url = f"ws://127.0.0.1:{listener.sockets[0].getsockname()[1]}/ws"
            async with connect(url + "?role=drone", additional_headers={"Authorization": "Bearer " + TOKENS["drone"]}) as drone:
                # Initial synchronization messages precede mission commands.
                async with connect(url + "?role=tablet", subprotocols=["dronetect-auth." + TOKENS["operator"]]) as operator:
                    assert json.loads(await operator.recv())["type"] == "mission_status"
                    command = {"type": "command", "commandId": "auth-test", "command": "return_home", "params": {}}
                    await operator.send(json.dumps(command))
                    for _ in range(10):
                        forwarded = json.loads(await asyncio.wait_for(drone.recv(), 1))
                        if forwarded.get("commandId") == "auth-test":
                            break
                    assert forwarded == command
                    assert json.loads(await operator.recv())["status"] == "accepted"
    asyncio.run(scenario())


@pytest.mark.parametrize("path,role", [
    ("/video/ingest?role=drone", "ingest"), ("/video/webrtc?role=viewer", "viewer"),
])
def test_correct_video_credentials_are_accepted(path, role):
    async def scenario():
        server = RealtimeServer(access=AccessControl(TOKENS))
        try:
            async with serve(server.handler, "127.0.0.1", 0, process_request=server.process_request) as listener:
                url = f"ws://127.0.0.1:{listener.sockets[0].getsockname()[1]}{path}"
                async with connect(url, subprotocols=["dronetect-auth." + TOKENS[role]]) as ws:
                    assert ws.response.status_code == 101
                    # No credentials are echoed in a negotiated subprotocol.
                    assert ws.subprotocol is None
        finally:
            await server.video_pipeline.vision.stop()
    asyncio.run(scenario())


def test_credentials_are_not_sent_over_public_plaintext_websockets(monkeypatch):
    for role, name in TOKEN_ENV.items():
        monkeypatch.setenv(name, TOKENS[role])
    with pytest.raises(ValueError, match="WSS"):
        client_auth_headers("drone", "ws://example.com/ws")
    assert client_auth_headers("drone", "ws://127.0.0.1:8765/ws")
    assert client_auth_headers("vision", "ws://vision_server:8870/frames")
    assert client_auth_headers("drone", "wss://example.com/ws")


def test_vision_requires_its_own_key_before_consuming_frames():
    from vision_server.server import VisionServer
    from vision_server.detector import NoopDroneDetector
    async def scenario():
        server = VisionServer(detector=NoopDroneDetector(), access=AccessControl(TOKENS))
        async with serve(server.frame_handler, "127.0.0.1", 0) as listener:
            url = f"ws://127.0.0.1:{listener.sockets[0].getsockname()[1]}/frames"
            for role in (None, "operator", "ingest", "vision"):
                headers = {"Authorization": "Bearer " + TOKENS[role]} if role else {}
                async with connect(url, additional_headers=headers) as ws:
                    if role == "vision":
                        await ws.ping()
                    else:
                        await ws.wait_closed()
                        assert ws.close_code == 1008
    asyncio.run(scenario())
