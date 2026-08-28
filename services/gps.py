"""Convert Webots local meter coordinates to WGS84 coordinates."""

from __future__ import annotations

from dataclasses import dataclass
import math


METERS_PER_DEG_LAT = 111_320.0


@dataclass(frozen=True, slots=True)
class Wgs84Point:
    lat: float
    lng: float


@dataclass(frozen=True, slots=True)
class LocalPosition:
    x: float
    y: float
    z: float


DEFAULT_HOME = Wgs84Point(lat=32.085300, lng=34.781800)


def local_meters_to_wgs84(
    position: LocalPosition,
    home: Wgs84Point = DEFAULT_HOME,
) -> Wgs84Point:
    """Map Webots local X/Y meter displacement from home into WGS84.

    Webots X is treated as east/west displacement and Webots Y as north/south
    displacement. Altitude remains separate telemetry and is not folded into
    the returned geographic point.
    """
    meters_per_deg_lng = METERS_PER_DEG_LAT * math.cos(math.radians(home.lat))
    return Wgs84Point(
        lat=home.lat + position.y / METERS_PER_DEG_LAT,
        lng=home.lng + position.x / meters_per_deg_lng,
    )
