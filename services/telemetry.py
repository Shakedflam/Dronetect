"""Normalized telemetry payloads for the realtime server."""

from __future__ import annotations

from dataclasses import dataclass
import time

from services.gps import LocalPosition, local_meters_to_wgs84


SIMULATOR_SOURCE = "simulator"
SIMULATOR_BATTERY_PERCENT = 100


@dataclass(frozen=True, slots=True)
class TelemetryPayload:
    position_local: LocalPosition
    altitude_m: float
    heading_deg: float
    speed_mps: float
    battery_percent: int
    state: str


def build_simulator_telemetry(
    *,
    position_local: LocalPosition,
    heading_deg: float,
    speed_mps: float,
    state: str,
    timestamp: float | None = None,
) -> dict:
    """Build the JSON-compatible simulator telemetry message."""
    gps = local_meters_to_wgs84(position_local)
    return {
        # "type": "telemetry" tells the receiver what kind of message this is
        "type": "telemetry",
        "timestamp": time.time() if timestamp is None else timestamp,
        "source": SIMULATOR_SOURCE,
        "data": {
            "gps": {
                "lat": gps.lat,
                "lng": gps.lng,
            },
            "altitudeM": position_local.z,
            "headingDeg": heading_deg,
            "speedMps": speed_mps,
            "batteryPercent": SIMULATOR_BATTERY_PERCENT,
            "state": state,
        },
    }
