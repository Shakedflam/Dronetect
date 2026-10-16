"""Role-scoped WebSocket credentials. Tokens never belong in URLs or logs."""

from dataclasses import dataclass, field
import hmac
import ipaddress
import os
import re
from urllib.parse import parse_qs, urlsplit


TOKEN_ENV = {
    "operator": "DRONTECT_OPERATOR_TOKEN",
    "drone": "DRONTECT_DRONE_TOKEN",
    "ingest": "DRONTECT_VIDEO_INGEST_TOKEN",
    "viewer": "DRONTECT_VIEWER_TOKEN",
    "vision": "DRONTECT_VISION_TOKEN",
}
TOKEN_PATTERN = re.compile(r"[a-f0-9]{64}\Z")
PROTOCOL_PREFIX = "dronetect-auth."


def is_loopback(host: str) -> bool:
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def request_role(path: str) -> str | None:
    parsed = urlsplit(path)
    query = parse_qs(parsed.query)
    roles = query.get("role", [])
    if len(roles) != 1:
        return None
    role = roles[0]
    if parsed.path == "/ws":
        return {"tablet": "operator", "drone": "drone", "simulator": "drone"}.get(role)
    if parsed.path == "/video/ingest" and role == "drone":
        return "ingest"
    if parsed.path == "/video/webrtc" and role == "viewer":
        return "viewer"
    return None


@dataclass(frozen=True)
class AccessControl:
    # repr=False prevents accidental credential disclosure in diagnostics.
    tokens: dict[str, str] = field(default_factory=dict, repr=False)
    allow_insecure_local: bool = False

    @classmethod
    def from_env(cls) -> "AccessControl":
        tokens = {role: os.getenv(name, "").strip() for role, name in TOKEN_ENV.items()}
        for role, token in tokens.items():
            if token and not TOKEN_PATTERN.fullmatch(token):
                raise ValueError(f"{TOKEN_ENV[role]} must contain 64 lowercase hex characters")
        populated = [token for token in tokens.values() if token]
        if len(populated) != len(set(populated)):
            raise ValueError("Access keys must be different for every role")
        return cls(tokens, os.getenv("DRONTECT_ALLOW_INSECURE_LOCAL") == "1")

    def validate_listener(self, host: str, roles: tuple[str, ...]) -> None:
        if self.allow_insecure_local:
            if not is_loopback(host):
                raise ValueError("Insecure development mode requires a literal loopback bind address")
            return
        for role in roles:
            if not self.tokens.get(role):
                raise ValueError(f"Missing {TOKEN_ENV[role]}; refusing to start without authentication")

    def authorized(self, request, role: str | None, peer_host: str = "") -> bool:
        if request is None or role is None:
            return False
        if self.allow_insecure_local and is_loopback(peer_host):
            return True
        expected = self.tokens.get(role, "")
        if not expected:
            return False
        supplied: list[str] = []
        for header in request.headers.get_all("Authorization"):
            if header.startswith("Bearer "):
                supplied.append(header[7:])
        for header in request.headers.get_all("Sec-WebSocket-Protocol"):
            for item in header.split(","):
                item = item.strip()
                if item.startswith(PROTOCOL_PREFIX):
                    supplied.append(item[len(PROTOCOL_PREFIX):])
        return (len(supplied) == 1 and bool(TOKEN_PATTERN.fullmatch(supplied[0]))
                and hmac.compare_digest(supplied[0], expected))


def client_auth_headers(role: str, url: str = "") -> dict[str, str]:
    token = AccessControl.from_env().tokens.get(role)
    if token and url:
        parsed = urlsplit(url)
        internal_vision = role == "vision" and parsed.hostname == "vision_server"
        if parsed.scheme != "wss" and not is_loopback(parsed.hostname or "") and not internal_vision:
            raise ValueError("Credentials require WSS outside loopback or the internal vision network")
    return {"Authorization": f"Bearer {token}"} if token else {}
