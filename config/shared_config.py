"""Parameters shared between the Python backend and the React frontend.

`shared.json` is the single source of truth for values both sides must agree
on bit-for-bit. This matters because only opaque identifiers (patrol region
IDs like "SW-1", WGS84 coordinates, ...) cross the wire between the realtime
server/drone controller and the map UI -- never geometry. If the backend and
frontend independently computed the patrol grid from different home
coordinates, radius, or band count, the same region ID would refer to two
different real-world places on each side, and the map would show the drone
patrolling/avoiding areas that don't match what it actually flies.

The frontend counterpart is app/src/config/mapConfig.ts, which imports this
same shared.json directly rather than duplicating its values.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


DEFAULT_SHARED_CONFIG_PATH = Path(__file__).with_name("shared.json")


@dataclass(frozen=True, slots=True)
class HomeReference:
    lat: float
    lng: float


@dataclass(frozen=True, slots=True)
class SharedConfig:
    home: HomeReference = HomeReference(lat=32.0853, lng=34.7818)
    home_radius_m: float = 300.0
    patrol_band_count: int = 4


def load_shared_config(path: str | Path | None = None) -> SharedConfig:
    config_path = Path(path) if path is not None else DEFAULT_SHARED_CONFIG_PATH
    if not config_path.exists():
        return SharedConfig()

    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return SharedConfig()

    if not isinstance(raw, dict):
        return SharedConfig()

    defaults = SharedConfig()
    return SharedConfig(
        home=_home_reference(raw.get("home"), defaults.home),
        home_radius_m=_positive_number(raw.get("homeRadiusM"), defaults.home_radius_m),
        patrol_band_count=_positive_int(raw.get("patrolBandCount"), defaults.patrol_band_count),
    )


def _home_reference(value: Any, fallback: HomeReference) -> HomeReference:
    if not isinstance(value, dict):
        return fallback
    lat, lng = value.get("lat"), value.get("lng")
    if (
        isinstance(lat, bool) or isinstance(lng, bool)
        or not isinstance(lat, (int, float)) or not isinstance(lng, (int, float))
    ):
        return fallback
    return HomeReference(lat=float(lat), lng=float(lng))


def _positive_number(value: Any, fallback: float) -> float:
    if isinstance(value, bool):
        return fallback
    if isinstance(value, (int, float)) and value > 0:
        return float(value)
    return fallback


def _positive_int(value: Any, fallback: int) -> int:
    if isinstance(value, bool):
        return fallback
    if isinstance(value, int) and value > 0:
        return value
    return fallback


shared_config = load_shared_config()
