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
import math
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
    cruise_altitude_m: float = 10.0
    patrol_band_count: int = 4
    # Regulatory ceiling on operator-chosen values -- kept a small margin
    # below the actual legal limits (4000 m / 50 m) rather than exactly at
    # them.
    legal_max_flight_radius_m: float = 3900.0
    legal_max_cruise_altitude_m: float = 48.0


@dataclass(frozen=True, slots=True)
class OperatingConfig:
    """The two operator-selected values that must change as one unit."""

    max_flight_radius_m: float = 300.0
    cruise_altitude_m: float = 10.0

    def __post_init__(self) -> None:
        for name in ("max_flight_radius_m", "cruise_altitude_m"):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be a finite positive number")
        if self.max_flight_radius_m > shared_config.legal_max_flight_radius_m:
            raise ValueError(
                "max_flight_radius_m exceeds the legal maximum of "
                f"{shared_config.legal_max_flight_radius_m:.0f} m"
            )
        if self.cruise_altitude_m > shared_config.legal_max_cruise_altitude_m:
            raise ValueError(
                "cruise_altitude_m exceeds the legal maximum of "
                f"{shared_config.legal_max_cruise_altitude_m:.0f} m"
            )


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
        cruise_altitude_m=_positive_number(
            raw.get("cruiseAltitudeM"), defaults.cruise_altitude_m
        ),
        patrol_band_count=_positive_int(raw.get("patrolBandCount"), defaults.patrol_band_count),
        legal_max_flight_radius_m=_positive_number(
            raw.get("legalMaxFlightRadiusM"), defaults.legal_max_flight_radius_m
        ),
        legal_max_cruise_altitude_m=_positive_number(
            raw.get("legalMaxCruiseAltitudeM"), defaults.legal_max_cruise_altitude_m
        ),
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
default_operating_config = OperatingConfig(
    max_flight_radius_m=shared_config.home_radius_m,
    cruise_altitude_m=shared_config.cruise_altitude_m,
)
