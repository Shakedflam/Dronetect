"""Normalized telemetry payloads for the realtime server."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

from services.gps import LocalPosition, local_meters_to_wgs84


SIMULATOR_SOURCE = "simulator"
SIMULATOR_BATTERY_PERCENT = 100


@dataclass(frozen=True, slots=True)
class TelemetryPayload:
    position_local: LocalPosition
    altitude_m: float
    heading_deg: float
    speed_mps: float
    battery_percent: float
    state: str


def build_simulator_telemetry(
    *,
    position_local: LocalPosition,
    heading_deg: float,
    speed_mps: float,
    state: str,
    battery_percent: float = SIMULATOR_BATTERY_PERCENT,
    target_patrol_region_id: str | None = None,
    aircraft_safety_rth_active: bool = False,
    aircraft_safety_rth_reason: str | None = None,
    target_follow_diagnostics: dict[str, Any] | None = None,
    operating_config_pending: bool = False,
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
            "batteryPercent": battery_percent,
            "state": state,
            "targetPatrolRegionId": target_patrol_region_id,
            "aircraftSafetyRthActive": aircraft_safety_rth_active,
            "aircraftSafetyRthReason": aircraft_safety_rth_reason,
            "targetFollowDiagnostics": target_follow_diagnostics,
            "operatingConfigPending": operating_config_pending,
        },
    }
