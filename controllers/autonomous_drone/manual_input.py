"""Where the manual sticks come from, and who wins when there are two.

The tablet's on-screen pad is the fallback. A real DJI remote controller, when
one is bound to the aircraft, always outranks it: the person holding a physical
transmitter has the aircraft in sight and expects their sticks to be the ones
being flown, and two operators fighting over the same airframe through two
different links is not a situation to resolve mid-flight.
"""

from __future__ import annotations

from typing import Protocol

try:
    from .flight_logic import ManualStick
except ImportError:  # Webots loads this directory as top-level modules.
    from flight_logic import ManualStick


MANUAL_INPUT_TABLET = "tablet"
MANUAL_INPUT_RC = "rc"


class RemoteController(Protocol):
    """A physical transmitter bound to the aircraft.

    Implemented by a real drone adapter (DJI and friends); the simulator has
    none, so the tablet pad is the only source there.
    """

    @property
    def connected(self) -> bool:
        """Whether a transmitter is currently bound and reporting."""

    def read_stick(self) -> ManualStick | None:
        """The stick right now, or None when the transmitter has gone quiet."""


class ManualInputRouter:
    """Pick the stick the aircraft should fly this tick, and say whose it is."""

    def __init__(
        self,
        *,
        command_timeout_s: float,
        remote_controller: RemoteController | None = None,
    ) -> None:
        self.remote_controller = remote_controller
        self._command_timeout_s = command_timeout_s
        self._tablet_stick = ManualStick()
        self._tablet_received_at: float | None = None

    @property
    def source(self) -> str:
        controller = self.remote_controller
        if controller is not None and controller.connected:
            return MANUAL_INPUT_RC
        return MANUAL_INPUT_TABLET

    def accept_tablet_stick(self, stick: ManualStick, now: float) -> None:
        """Record a stick from the tablet pad, timestamped for the timeout."""
        self._tablet_stick = stick
        self._tablet_received_at = now

    def reset(self) -> None:
        """Forget every stick, so a new takeover never inherits an old one."""
        self._tablet_stick = ManualStick()
        self._tablet_received_at = None

    def stick(self, now: float) -> ManualStick:
        """The stick to fly, hovering when nothing current is being held.

        A tablet that stops sending is treated as a released stick rather than
        a held one: the pad resends while a button is down, so silence means
        the operator has let go, or the link has -- and both want a hover.
        """
        controller = self.remote_controller
        if controller is not None and controller.connected:
            return controller.read_stick() or ManualStick()
        if self._tablet_received_at is None:
            return ManualStick()
        if now - self._tablet_received_at > self._command_timeout_s:
            return ManualStick()
        return self._tablet_stick
