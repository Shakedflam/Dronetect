import pytest

from services.mission_state import (
    InvalidMissionTransition,
    MissionState,
    MissionStateMachine,
)


def machine_in(state: MissionState) -> MissionStateMachine:
    machine = MissionStateMachine()
    paths = {
        MissionState.ON_GROUND: (),
        MissionState.TAKING_OFF: (MissionState.TAKING_OFF,),
        MissionState.PATROL: (MissionState.TAKING_OFF, MissionState.PATROL),
        MissionState.TRANSIT_TO_ROI: (
            MissionState.TAKING_OFF,
            MissionState.TRANSIT_TO_ROI,
        ),
        MissionState.ROI_PATROL: (
            MissionState.TAKING_OFF,
            MissionState.TRANSIT_TO_ROI,
            MissionState.ROI_PATROL,
        ),
        MissionState.RTH: (MissionState.TAKING_OFF, MissionState.RTH),
        MissionState.LANDING: (
            MissionState.TAKING_OFF,
            MissionState.RTH,
            MissionState.LANDING,
        ),
    }
    for next_state in paths[state]:
        machine.transition_to(next_state)
    return machine


def test_starts_on_ground_and_does_not_advance_by_itself():
    machine = MissionStateMachine()

    assert machine.state is MissionState.ON_GROUND
    assert machine.can_transition_to(MissionState.TAKING_OFF) is True
    assert machine.state is MissionState.ON_GROUND


def test_declares_the_complete_mission_state_set():
    assert tuple(MissionState) == (
        MissionState.ON_GROUND,
        MissionState.TAKING_OFF,
        MissionState.PATROL,
        MissionState.TRANSIT_TO_ROI,
        MissionState.ROI_PATROL,
        MissionState.RTH,
        MissionState.LANDING,
    )


@pytest.mark.parametrize(
    ("current", "requested"),
    [
        (MissionState.ON_GROUND, MissionState.TAKING_OFF),
        (MissionState.TAKING_OFF, MissionState.PATROL),
        (MissionState.TAKING_OFF, MissionState.TRANSIT_TO_ROI),
        (MissionState.TAKING_OFF, MissionState.RTH),
        (MissionState.PATROL, MissionState.TRANSIT_TO_ROI),
        (MissionState.PATROL, MissionState.RTH),
        (MissionState.TRANSIT_TO_ROI, MissionState.ROI_PATROL),
        (MissionState.TRANSIT_TO_ROI, MissionState.PATROL),
        (MissionState.TRANSIT_TO_ROI, MissionState.RTH),
        (MissionState.ROI_PATROL, MissionState.PATROL),
        (MissionState.ROI_PATROL, MissionState.TRANSIT_TO_ROI),
        (MissionState.ROI_PATROL, MissionState.RTH),
        (MissionState.RTH, MissionState.PATROL),
        (MissionState.RTH, MissionState.TRANSIT_TO_ROI),
        (MissionState.RTH, MissionState.LANDING),
        (MissionState.LANDING, MissionState.ON_GROUND),
    ],
)
def test_accepts_valid_transition(current, requested):
    machine = machine_in(current)

    assert machine.transition_to(requested) is requested
    assert machine.state is requested


@pytest.mark.parametrize(
    ("current", "requested"),
    [
        (MissionState.ON_GROUND, MissionState.PATROL),
        (MissionState.ON_GROUND, MissionState.TRANSIT_TO_ROI),
        (MissionState.ON_GROUND, MissionState.RTH),
        (MissionState.PATROL, MissionState.LANDING),
        (MissionState.TRANSIT_TO_ROI, MissionState.LANDING),
        (MissionState.ROI_PATROL, MissionState.LANDING),
        (MissionState.RTH, MissionState.ON_GROUND),
        (MissionState.LANDING, MissionState.RTH),
        (MissionState.PATROL, MissionState.PATROL),
    ],
)
def test_rejects_invalid_transition_without_changing_state(current, requested):
    machine = machine_in(current)

    with pytest.raises(InvalidMissionTransition) as error:
        machine.transition_to(requested)

    assert error.value.current is current
    assert error.value.requested is requested
    assert machine.state is current
