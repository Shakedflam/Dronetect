import pytest

from services.commands import CommandName
from services.flight_safety import SafetyRthSignal
from services.mission_commands import (
    InvalidMissionCommand,
    MissionCommandConfig,
    MissionCommandService,
    MissionRoi,
)
from services.mission_state import MissionState
from services.gps import DEFAULT_HOME, destination_point


def test_start_patrol_records_takeoff_intent_without_movement():
    missions = MissionCommandService()

    state = missions.apply(CommandName.START_PATROL, {})

    assert state is MissionState.TAKING_OFF
    assert missions.post_takeoff_intent is MissionState.PATROL


def test_set_roi_stores_validated_intent_without_flight_behavior():
    missions = MissionCommandService()

    state = missions.apply(
        CommandName.SET_ROI,
        {"center": {"lat": 0.0, "lon": 0.0}, "radiusM": 0.0},
    )

    assert state is MissionState.TAKING_OFF
    assert missions.post_takeoff_intent is MissionState.TRANSIT_TO_ROI
    assert missions.roi == MissionRoi(lat=0.0, lon=0.0, radius_m=0.0)


@pytest.mark.parametrize(
    ("command", "params", "expected"),
    [
        (CommandName.START_PATROL, {}, MissionState.PATROL),
        (
            CommandName.SET_ROI,
            {"center": {"lat": 0.0, "lon": 0.0}, "radiusM": 0.0},
            MissionState.TRANSIT_TO_ROI,
        ),
    ],
)
def test_reaching_configured_takeoff_altitude_completes_pending_intent(
    command, params, expected
):
    missions = MissionCommandService(
        config=MissionCommandConfig(
            takeoff_altitude_m=12.0,
            altitude_tolerance_m=0.25,
        )
    )
    missions.apply(command, params)

    assert missions.observe_altitude(11.74) is MissionState.TAKING_OFF
    assert missions.observe_altitude(11.75) is expected
    assert missions.post_takeoff_intent is None


def test_airborne_mission_switch_skips_taking_off():
    missions = MissionCommandService()
    missions.apply(CommandName.START_PATROL, {})
    missions.observe_altitude(15.0)

    state = missions.apply(
        CommandName.SET_ROI,
        {"center": {"lat": 0.0, "lon": 0.0}, "radiusM": 10.0},
    )

    assert state is MissionState.TRANSIT_TO_ROI


def test_roi_from_patrol_transitions_to_roi_patrol_at_arrival_boundary():
    missions = MissionCommandService(
        config=MissionCommandConfig(roi_arrival_tolerance_m=10.0)
    )
    missions.apply(CommandName.START_PATROL, {})
    missions.observe_altitude(15.0)
    roi_center = destination_point(DEFAULT_HOME, 1000.0, 90.0)

    state = missions.apply(
        CommandName.SET_ROI,
        {
            "center": {"lat": roi_center.lat, "lon": roi_center.lng},
            "radiusM": 100.0,
        },
    )

    assert state is MissionState.TRANSIT_TO_ROI
    outside = destination_point(roi_center, 110.1, 270.0)
    assert missions.observe_position(outside) is MissionState.TRANSIT_TO_ROI
    boundary = destination_point(roi_center, 110.0, 270.0)
    assert missions.observe_position(boundary) is MissionState.ROI_PATROL


def test_roi_from_ground_waits_for_takeoff_before_arrival_transition():
    missions = MissionCommandService()
    missions.apply(
        CommandName.SET_ROI,
        {
            "center": {"lat": DEFAULT_HOME.lat, "lon": DEFAULT_HOME.lng},
            "radiusM": 50.0,
        },
    )

    assert missions.observe_position(DEFAULT_HOME) is MissionState.TAKING_OFF
    assert missions.observe_altitude(15.0) is MissionState.TRANSIT_TO_ROI
    assert missions.observe_position(DEFAULT_HOME) is MissionState.ROI_PATROL


def test_new_roi_during_roi_patrol_transitions_back_to_transit():
    missions = MissionCommandService()
    missions.apply(CommandName.START_PATROL, {})
    missions.observe_altitude(15.0)
    missions.apply(
        CommandName.SET_ROI,
        {
            "center": {"lat": DEFAULT_HOME.lat, "lon": DEFAULT_HOME.lng},
            "radiusM": 50.0,
        },
    )
    missions.observe_position(DEFAULT_HOME)
    assert missions.state is MissionState.ROI_PATROL

    state = missions.apply(
        CommandName.SET_ROI,
        {"center": {"lat": 32.0, "lon": 34.0}, "radiusM": 100.0},
    )

    assert state is MissionState.TRANSIT_TO_ROI
    assert missions.roi == MissionRoi(lat=32.0, lon=34.0, radius_m=100.0)


def test_clear_roi_removes_roi_and_changes_pending_intent_to_patrol():
    missions = MissionCommandService()
    missions.apply(
        CommandName.SET_ROI,
        {"center": {"lat": 32.0, "lon": 34.0}, "radiusM": 300.0},
    )

    state = missions.apply(CommandName.CLEAR_ROI, {})

    assert state is MissionState.TAKING_OFF
    assert missions.roi is None
    assert missions.post_takeoff_intent is MissionState.PATROL


def test_start_patrol_ends_roi_override_and_clears_active_roi():
    missions = MissionCommandService()
    missions.apply(CommandName.START_PATROL, {})
    missions.observe_altitude(15.0)
    missions.apply(
        CommandName.SET_ROI,
        {"center": {"lat": 32.0, "lon": 34.0}, "radiusM": 100.0},
    )

    state = missions.apply(CommandName.START_PATROL, {})

    assert state is MissionState.PATROL
    assert missions.roi is None


def test_return_home_replaces_active_mission_intent():
    missions = MissionCommandService()
    missions.apply(CommandName.START_PATROL, {})

    state = missions.apply(CommandName.RETURN_HOME, {})

    assert state is MissionState.RTH
    assert missions.post_takeoff_intent is None


def test_new_mission_intent_is_rejected_during_return_home():
    missions = MissionCommandService()
    missions.apply(CommandName.START_PATROL, {})
    missions.apply(CommandName.RETURN_HOME, {})

    with pytest.raises(InvalidMissionCommand):
        missions.apply(CommandName.SET_ROI, {"center": {"lat": 0, "lon": 0}, "radiusM": 0})


def _missions_in_flying_state(state: MissionState) -> MissionCommandService:
    missions = MissionCommandService()
    missions.apply(CommandName.START_PATROL, {})
    if state is MissionState.TAKING_OFF:
        return missions

    missions.observe_altitude(15.0)
    if state is MissionState.PATROL:
        return missions

    missions.apply(
        CommandName.SET_ROI,
        {
            "center": {"lat": DEFAULT_HOME.lat, "lon": DEFAULT_HOME.lng},
            "radiusM": 50.0,
        },
    )
    if state is MissionState.TRANSIT_TO_ROI:
        return missions

    missions.observe_position(DEFAULT_HOME)
    assert state is MissionState.ROI_PATROL
    return missions


FLYING_MISSION_STATES = (
    MissionState.TAKING_OFF,
    MissionState.PATROL,
    MissionState.TRANSIT_TO_ROI,
    MissionState.ROI_PATROL,
)


@pytest.mark.parametrize("starting_state", FLYING_MISSION_STATES)
def test_return_home_transitions_every_flying_mission_state_to_rth(starting_state):
    missions = _missions_in_flying_state(starting_state)

    assert missions.apply(CommandName.RETURN_HOME, {}) is MissionState.RTH
    assert missions.post_takeoff_intent is None


@pytest.mark.parametrize("starting_state", FLYING_MISSION_STATES)
def test_safety_trigger_transitions_every_flying_mission_state_to_rth(
    starting_state,
):
    missions = _missions_in_flying_state(starting_state)

    state = missions.observe_safety(
        SafetyRthSignal(required=True, reason="low_battery")
    )

    assert state is MissionState.RTH
    assert missions.safety_rth_active is True
    assert missions.safety_rth_reason == "low_battery"
    assert missions.post_takeoff_intent is None


@pytest.mark.parametrize("safety_triggered", [False, True])
def test_rth_flight_states_complete_landing_and_return_on_ground(safety_triggered):
    missions = _missions_in_flying_state(MissionState.PATROL)
    if safety_triggered:
        missions.observe_safety(
            SafetyRthSignal(required=True, reason="low_battery")
        )
    else:
        missions.apply(CommandName.RETURN_HOME, {})

    assert missions.observe_flight_state("RETURNING_HOME") is MissionState.RTH
    assert missions.observe_flight_state("LANDING") is MissionState.LANDING
    assert missions.observe_flight_state("LANDED") is MissionState.ON_GROUND
    assert missions.safety_rth_active is False


@pytest.mark.parametrize(
    ("command", "params"),
    [
        (CommandName.START_PATROL, {}),
        (
            CommandName.SET_ROI,
            {"center": {"lat": 0.0, "lon": 0.0}, "radiusM": 10.0},
        ),
        (CommandName.CLEAR_ROI, {}),
    ],
)
@pytest.mark.parametrize("during_landing", [False, True])
def test_mission_commands_cannot_override_active_safety_rth(
    command,
    params,
    during_landing,
):
    missions = _missions_in_flying_state(MissionState.PATROL)
    missions.observe_safety(SafetyRthSignal(required=True, reason="low_battery"))
    if during_landing:
        missions.observe_flight_state("LANDING")

    with pytest.raises(InvalidMissionCommand):
        missions.apply(command, params)

    expected = MissionState.LANDING if during_landing else MissionState.RTH
    assert missions.state is expected
    assert missions.safety_rth_active is True


def test_return_home_is_idempotent_during_active_safety_rth():
    missions = _missions_in_flying_state(MissionState.PATROL)
    missions.observe_safety(SafetyRthSignal(required=True, reason="low_battery"))

    assert missions.apply(CommandName.RETURN_HOME, {}) is MissionState.RTH
    assert missions.safety_rth_active is True


def test_safety_trigger_does_not_launch_rth_while_on_ground():
    missions = MissionCommandService()

    state = missions.observe_safety(
        SafetyRthSignal(required=True, reason="low_battery")
    )

    assert state is MissionState.ON_GROUND
    assert missions.safety_rth_active is False
