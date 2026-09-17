"""Pure backend mission-state transitions with no flight side effects."""

from __future__ import annotations

from enum import StrEnum


class MissionState(StrEnum):
    ON_GROUND = "ON_GROUND"
    TAKING_OFF = "TAKING_OFF"
    PATROL = "PATROL"
    TRANSIT_TO_ROI = "TRANSIT_TO_ROI"
    ROI_PATROL = "ROI_PATROL"
    RTH = "RTH"
    LANDING = "LANDING"


VALID_MISSION_TRANSITIONS: dict[MissionState, frozenset[MissionState]] = {
    MissionState.ON_GROUND: frozenset({MissionState.TAKING_OFF}),
    MissionState.TAKING_OFF: frozenset(
        {
            MissionState.PATROL,
            MissionState.TRANSIT_TO_ROI,
            MissionState.RTH,
        }
    ),
    MissionState.PATROL: frozenset(
        {
            MissionState.TRANSIT_TO_ROI,
            MissionState.RTH,
        }
    ),
    MissionState.TRANSIT_TO_ROI: frozenset(
        {
            MissionState.ROI_PATROL,
            MissionState.PATROL,
            MissionState.RTH,
        }
    ),
    MissionState.ROI_PATROL: frozenset(
        {
            MissionState.PATROL,
            MissionState.TRANSIT_TO_ROI,
            MissionState.RTH,
        }
    ),
    MissionState.RTH: frozenset(
        {
            MissionState.PATROL,
            MissionState.TRANSIT_TO_ROI,
            MissionState.LANDING,
        }
    ),
    MissionState.LANDING: frozenset({MissionState.ON_GROUND}),
}


class InvalidMissionTransition(ValueError):
    """Raised when a requested mission-state transition is not allowed."""

    def __init__(self, current: MissionState, requested: MissionState) -> None:
        self.current = current
        self.requested = requested
        super().__init__(
            f"Invalid mission transition: {current.value} -> {requested.value}"
        )


class MissionStateMachine:
    """State-only mission lifecycle, deliberately isolated from flight control.

    Constructing or inspecting the machine never advances it. A caller must
    explicitly request every transition, so startup cannot trigger takeoff or
    movement on its own.
    """

    def __init__(self) -> None:
        self._state = MissionState.ON_GROUND

    @property
    def state(self) -> MissionState:
        return self._state

    def can_transition_to(self, requested: MissionState) -> bool:
        return requested in VALID_MISSION_TRANSITIONS[self._state]

    def transition_to(self, requested: MissionState) -> MissionState:
        if not self.can_transition_to(requested):
            raise InvalidMissionTransition(self._state, requested)
        self._state = requested
        return self._state
