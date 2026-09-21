"""Command names shared by tablet clients, realtime server, and drone adapters."""

from __future__ import annotations

from enum import StrEnum


class CommandName(StrEnum):
    SET_OPERATING_CONFIG = "set_operating_config"
    START_PATROL = "start_patrol"
    SET_PATROL_PRIORITIES = "set_patrol_priorities"
    SET_PATROL_AREA_TYPES = "set_patrol_area_types"
    SET_ROI = "set_roi"
    CLEAR_ROI = "clear_roi"
    RETURN_HOME = "return_home"
    CAMERA_TILT_UP = "camera_tilt_up"
    CAMERA_TILT_DOWN = "camera_tilt_down"
    APPROVE_TARGET = "approve_target"
    REJECT_TARGET = "reject_target"


SUPPORTED_COMMANDS = frozenset(command.value for command in CommandName)
MISSION_COMMANDS = frozenset(
    {
        CommandName.START_PATROL,
        CommandName.SET_ROI,
        CommandName.CLEAR_ROI,
        CommandName.RETURN_HOME,
    }
)
