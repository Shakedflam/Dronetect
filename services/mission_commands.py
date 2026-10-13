"""Map validated backend commands to mission-state intent only."""

from __future__ import annotations

from dataclasses import dataclass

from services.commands import CommandName
from services.flight_safety import SafetyRthSignal
from services.gps import Wgs84Point, wgs84_distance_m
from services.mission_state import MissionState, MissionStateMachine


@dataclass(frozen=True, slots=True)
class MissionRoi:
    lat: float
    lon: float
    radius_m: float


@dataclass(frozen=True, slots=True)
class MissionCommandConfig:
    takeoff_altitude_m: float = 15.0
    altitude_tolerance_m: float = 0.5
    roi_arrival_tolerance_m: float = 10.0


class InvalidMissionCommand(ValueError):
    def __init__(self, command: CommandName, state: MissionState) -> None:
        self.command = command
        self.state = state
        super().__init__(f"Command {command.value} is invalid while in {state.value}")


class MissionCommandService:
    """Own mission intent without issuing navigation or motor commands."""

    def __init__(
        self,
        state_machine: MissionStateMachine | None = None,
        config: MissionCommandConfig | None = None,
    ) -> None:
        self._state_machine = state_machine or MissionStateMachine()
        self.config = config or MissionCommandConfig()
        self._roi: MissionRoi | None = None
        self._post_takeoff_intent: MissionState | None = None
        self._safety_rth_reason: str | None = None

    @property
    def state(self) -> MissionState:
        return self._state_machine.state

    @property
    def roi(self) -> MissionRoi | None:
        return self._roi

    @property
    def post_takeoff_intent(self) -> MissionState | None:
        return self._post_takeoff_intent

    @property
    def safety_rth_active(self) -> bool:
        return self._safety_rth_reason is not None

    @property
    def safety_rth_reason(self) -> str | None:
        return self._safety_rth_reason

    def apply(self, command: CommandName, params: dict) -> MissionState:
        self.validate(command)
        if command is CommandName.START_PATROL:
            self._request_operating_state(command, MissionState.PATROL)
            self._roi = None
        elif command is CommandName.SET_ROI:
            center = params["center"]
            roi = MissionRoi(
                lat=float(center["lat"]),
                lon=float(center["lon"]),
                radius_m=float(params["radiusM"]),
            )
            self._request_operating_state(command, MissionState.TRANSIT_TO_ROI)
            self._roi = roi
        elif command is CommandName.CLEAR_ROI:
            self._clear_roi()
        elif command is CommandName.RETURN_HOME:
            self._return_home()
        else:
            raise ValueError(f"Not a mission command: {command.value}")
        return self.state

    def validate(self, command: CommandName) -> None:
        """Check a command without mutating mission state."""
        if self.safety_rth_active and command is not CommandName.RETURN_HOME:
            raise InvalidMissionCommand(command, self.state)
        if command not in {
            CommandName.START_PATROL,
            CommandName.SET_ROI,
            CommandName.CLEAR_ROI,
            CommandName.RETURN_HOME,
        }:
            raise ValueError(f"Not a mission command: {command.value}")

    def observe_altitude(self, altitude_m: float) -> MissionState:
        """Complete a commanded takeoff after telemetry reaches its target."""
        if (
            self.state is MissionState.TAKING_OFF
            and altitude_m
            >= self.config.takeoff_altitude_m - self.config.altitude_tolerance_m
        ):
            destination = self._post_takeoff_intent
            if destination is not None:
                self._state_machine.transition_to(destination)
                self._post_takeoff_intent = None
        return self.state

    def observe_position(self, position: Wgs84Point) -> MissionState:
        """Complete ROI transit once telemetry reaches the accepted boundary."""
        if self.state is MissionState.TRANSIT_TO_ROI and self._roi is not None:
            center = Wgs84Point(lat=self._roi.lat, lng=self._roi.lon)
            accepted_radius_m = (
                self._roi.radius_m + self.config.roi_arrival_tolerance_m
            )
            if wgs84_distance_m(position, center) <= accepted_radius_m:
                self._state_machine.transition_to(MissionState.ROI_PATROL)
        return self.state

    def observe_safety(self, signal: SafetyRthSignal) -> MissionState:
        """Latch an adapter-neutral safety RTH request until touchdown."""
        if not signal.required:
            return self.state
        if not signal.reason:
            raise ValueError("A required safety RTH signal must include a reason")

        if self.state in {
            MissionState.TAKING_OFF,
            MissionState.PATROL,
            MissionState.TRANSIT_TO_ROI,
            MissionState.ROI_PATROL,
        }:
            self._post_takeoff_intent = None
            self._safety_rth_reason = signal.reason
            self._state_machine.transition_to(MissionState.RTH)
        elif self.state in {MissionState.RTH, MissionState.LANDING}:
            self._safety_rth_reason = signal.reason
        return self.state

    def observe_touchdown(self) -> MissionState:
        """The aircraft reported an airborne -> grounded edge: end the mission.

        A pilot can land mid-mission (or after an RTH that our autonomy does
        not finish itself); the next mission must start from ON_GROUND, not
        inherit the old intent, ROI or a safety latch that would refuse it.
        Walks the existing transitions rather than adding shortcut edges.
        """
        self._post_takeoff_intent = None
        self._roi = None
        self._safety_rth_reason = None
        if self.state in {
            MissionState.TAKING_OFF,
            MissionState.PATROL,
            MissionState.TRANSIT_TO_ROI,
            MissionState.ROI_PATROL,
        }:
            self._state_machine.transition_to(MissionState.RTH)
        if self.state is MissionState.RTH:
            self._state_machine.transition_to(MissionState.LANDING)
        if self.state is MissionState.LANDING:
            self._state_machine.transition_to(MissionState.ON_GROUND)
        return self.state

    def observe_flight_state(self, flight_state: str) -> MissionState:
        """Advance RTH completion from normalized aircraft flight state."""
        if self.state is MissionState.RTH and flight_state in {"LANDING", "LANDED"}:
            self._state_machine.transition_to(MissionState.LANDING)
        if self.state is MissionState.LANDING and flight_state == "LANDED":
            self._state_machine.transition_to(MissionState.ON_GROUND)
            self._safety_rth_reason = None
        return self.state

    def adopt_aircraft_state(self, observed: MissionState) -> MissionState:
        """Follow the aircraft when it is demonstrably doing something else.

        A forwarded command can still be refused on the aircraft (DJI owns
        the landing, a latch the server cannot see, an unconfigured bridge),
        and the aircraft can change mission on its own (a pilot hand-back
        resuming a mission). The caller only adopts a persistent disagreement.
        A safety RTH latch is never adopted away from RTH/LANDING.
        """
        if observed is self.state:
            return self.state
        if self.safety_rth_active and observed not in {MissionState.RTH, MissionState.LANDING}:
            return self.state
        self._post_takeoff_intent = None
        if observed in {MissionState.PATROL, MissionState.RTH, MissionState.LANDING}:
            self._roi = None
        return self._state_machine.adopt(observed)

    def _request_operating_state(
        self,
        command: CommandName,
        requested: MissionState,
    ) -> None:
        current = self.state
        if current is MissionState.ON_GROUND:
            self._post_takeoff_intent = requested
            self._state_machine.transition_to(MissionState.TAKING_OFF)
        elif current is MissionState.TAKING_OFF:
            self._post_takeoff_intent = requested
        elif current is requested:
            self._post_takeoff_intent = None
        elif current in {
            MissionState.PATROL,
            MissionState.TRANSIT_TO_ROI,
            MissionState.ROI_PATROL,
            MissionState.RTH,
            MissionState.LANDING,
        }:
            self._post_takeoff_intent = None
            self._state_machine.transition_to(requested)
        else:
            raise InvalidMissionCommand(command, current)

    def _clear_roi(self) -> None:
        self._roi = None
        if (
            self.state is MissionState.TAKING_OFF
            and self._post_takeoff_intent is MissionState.TRANSIT_TO_ROI
        ):
            self._post_takeoff_intent = MissionState.PATROL
        elif self.state in {
            MissionState.TRANSIT_TO_ROI,
            MissionState.ROI_PATROL,
        }:
            self._state_machine.transition_to(MissionState.PATROL)

    def _return_home(self) -> None:
        self._post_takeoff_intent = None
        if self.state in {
            MissionState.TAKING_OFF,
            MissionState.PATROL,
            MissionState.TRANSIT_TO_ROI,
            MissionState.ROI_PATROL,
        }:
            self._state_machine.transition_to(MissionState.RTH)
        elif self.state not in {
            MissionState.ON_GROUND,
            MissionState.RTH,
            MissionState.LANDING,
        }:
            raise InvalidMissionCommand(CommandName.RETURN_HOME, self.state)
