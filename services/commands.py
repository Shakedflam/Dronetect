"""Command names shared by tablet clients, realtime server, and drone adapters."""

from __future__ import annotations

from enum import StrEnum


class CommandName(StrEnum):
    RETURN_HOME = "return_home"
    NEW_MISSION = "new_mission"
    CAMERA_TILT_UP = "camera_tilt_up"
    CAMERA_TILT_DOWN = "camera_tilt_down"


SUPPORTED_COMMANDS = frozenset(command.value for command in CommandName)
