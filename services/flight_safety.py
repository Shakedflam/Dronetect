"""Adapter-neutral safety inputs that can request mission-level RTH."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol


LOW_BATTERY_REASON = "low_battery"
AIRCRAFT_RTH_REASON = "aircraft_safety_rth"


@dataclass(frozen=True, slots=True)
class FlightSafetySnapshot:
    """Normalized safety facts supplied by a simulator or aircraft adapter."""

    battery_percent: float | None = None
    aircraft_rth_active: bool = False
    aircraft_rth_reason: str | None = None


@dataclass(frozen=True, slots=True)
class SafetyRthSignal:
    required: bool
    reason: str | None = None


class SafetyRthPolicy(Protocol):
    """Boundary allowing DJI safety data to replace simulator inputs later."""

    def evaluate(self, snapshot: FlightSafetySnapshot) -> SafetyRthSignal: ...


@dataclass(frozen=True, slots=True)
class DefaultSafetyRthPolicy:
    low_battery_threshold_percent: float = 20.0

    def __post_init__(self) -> None:
        threshold = self.low_battery_threshold_percent
        if (
            isinstance(threshold, bool)
            or not math.isfinite(threshold)
            or not 0.0 <= threshold <= 100.0
        ):
            raise ValueError(
                "low_battery_threshold_percent must be finite and between 0 and 100"
            )

    def evaluate(self, snapshot: FlightSafetySnapshot) -> SafetyRthSignal:
        if snapshot.aircraft_rth_active:
            return SafetyRthSignal(
                required=True,
                reason=snapshot.aircraft_rth_reason or AIRCRAFT_RTH_REASON,
            )

        battery = snapshot.battery_percent
        if battery is not None:
            if isinstance(battery, bool) or not math.isfinite(battery):
                raise ValueError("battery_percent must be finite or None")
            if battery <= self.low_battery_threshold_percent:
                return SafetyRthSignal(required=True, reason=LOW_BATTERY_REASON)

        return SafetyRthSignal(required=False)
