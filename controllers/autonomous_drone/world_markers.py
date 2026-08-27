"""Webots marker management for home and target visualization."""

from __future__ import annotations

from controller import Field, Supervisor

from flight_logic import FlightState, Position


class WorldMarkers:
    """Move the colored home and target objects in the Webots world."""

    def __init__(self, supervisor: Supervisor) -> None:
        self.home_translation = self._marker_translation(supervisor, "HOME_MARKER")
        self.target_translation = self._marker_translation(
            supervisor,
            "TARGET_MARKER",
        )
        self.target_ground_translation = self._marker_translation(
            supervisor,
            "TARGET_GROUND_MARKER",
        )

    def update(
        self,
        home: Position | None,
        target: Position | None,
        state: FlightState,
    ) -> None:
        if home is None:
            return

        ground_height = max(0.03, home[2] - 0.07)
        self.home_translation.setSFVec3f([home[0], home[1], ground_height])

        if target is None or state in {FlightState.IDLE, FlightState.LANDED}:
            self.hide_target_near(home)
            return

        self.target_translation.setSFVec3f(list(target))
        self.target_ground_translation.setSFVec3f(
            [target[0], target[1], ground_height]
        )

    def hide_target_near(self, home: Position) -> None:
        hidden = [home[0], home[1], -5.0]
        self.target_translation.setSFVec3f(hidden)
        self.target_ground_translation.setSFVec3f(hidden)

    @staticmethod
    def _marker_translation(supervisor: Supervisor, def_name: str) -> Field:
        node = supervisor.getFromDef(def_name)
        if node is None:
            raise RuntimeError(f"World marker DEF {def_name!r} was not found.")
        field = node.getField("translation")
        if field is None:
            raise RuntimeError(f"World marker {def_name!r} has no translation field.")
        return field
