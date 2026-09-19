"""Convert Webots local meter coordinates to WGS84 coordinates."""

from __future__ import annotations

from dataclasses import dataclass
import math

from config.shared_config import shared_config


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


# Sourced from config/shared.json, the single value both the backend and the
# frontend (app/src/config/mapConfig.ts) build their home-relative geometry
# from -- see config/shared_config.py for why this must not be a local literal.
DEFAULT_HOME = Wgs84Point(lat=shared_config.home.lat, lng=shared_config.home.lng)


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


def wgs84_to_local_meters(
    point: Wgs84Point,
    home: Wgs84Point = DEFAULT_HOME,
    *,
    altitude_m: float = 0.0,
) -> LocalPosition:
    """Map a nearby WGS84 point into local east/north meter coordinates."""
    mean_latitude_rad = math.radians((point.lat + home.lat) / 2.0)
    return LocalPosition(
        x=(point.lng - home.lng) * METERS_PER_DEG_LAT * math.cos(mean_latitude_rad),
        y=(point.lat - home.lat) * METERS_PER_DEG_LAT,
        z=altitude_m,
    )


def destination_point(
    home: Wgs84Point,
    distance_m: float,
    bearing_deg: float,
) -> Wgs84Point:
    """Walk distance_m from home along bearing_deg (0 = north, clockwise).

    Uses the same flat-plane approximation as local_meters_to_wgs84 —
    accurate enough at the few-kilometer scale this app operates at, not for
    long distances or polar latitudes.
    """
    bearing_rad = math.radians(bearing_deg)
    offset = LocalPosition(
        x=distance_m * math.sin(bearing_rad),
        y=distance_m * math.cos(bearing_rad),
        z=0.0,
    )
    return local_meters_to_wgs84(offset, home)


def wgs84_distance_m(first: Wgs84Point, second: Wgs84Point) -> float:
    """Approximate short-range ground distance between two WGS84 points."""
    mean_latitude_rad = math.radians((first.lat + second.lat) / 2.0)
    north_m = (second.lat - first.lat) * METERS_PER_DEG_LAT
    east_m = (
        (second.lng - first.lng)
        * METERS_PER_DEG_LAT
        * math.cos(mean_latitude_rad)
    )
    return math.hypot(east_m, north_m)
