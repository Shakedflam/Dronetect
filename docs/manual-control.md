# Manual control

Every other control on the bar tells the computer what mission to fly. **MANUAL**
is the one that takes the aircraft off the computer entirely: while it is
engaged the autopilot selects no waypoints at all, and the only thing moving
the airframe is the operator's stick.

It sits apart from the rest, on the right edge of the action bar, because it is
not a mission override — it is a change of who is flying.

## Taking control

The button is a latching toggle, not a dead-man switch. Holding a button down
with one thumb while flying with the other is not a control anyone can use, and
a takeover that ends the moment a finger slips is worse than one that has to be
ended deliberately.

Pressing it sends `set_manual_control` with `params: {"engaged": true}`. The
button does not light up on the click: like **HOLD PURSUIT**, it shows what the
aircraft reports back in `mission_status.manualControlActive`, so a takeover the
aircraft never received never looks engaged.

Control may be taken while the aircraft is airborne and under control — patrol,
ROI, a target chase, an operator-initiated return home. It is refused on the
ground, during takeoff and during landing: there is nothing useful to fly and no
mission to hand back to. Taking control mid-chase ends the chase, for the same
reason the pursuit hold does.

## Flying

While engaged, a pad appears on the right edge of the screen: a four-way pad for
horizontal movement, plus climb/descend and yaw. Every button is momentary — the
aircraft moves only for as long as one is held.

**Releasing everything is a hover.** It is not "no command"; it is the command
to hold position, and the aircraft stops where it is.

Sticks travel as `set_manual_command` with four axes, each normalized to
`[-1, 1]`:

| Axis | Meaning |
|---|---|
| `forward` | +1 toward the nose, and therefore toward whatever the camera is showing |
| `right` | +1 to starboard |
| `up` | +1 climb |
| `yawRate` | +1 turn right |

The sticks are read in the aircraft's body frame, because the operator is flying
from the video feed and "forward" has to mean what they see. A value outside
`[-1, 1]` is refused rather than clamped: a client sending 50 is a bug, and
quietly flying its nearest legal value would hide it.

The pad resends the held stick about ten times a second, which doubles as a
heartbeat. Unlike the takeover latch, sticks are forwarded and forgotten — the
server stores none of them, and nothing is ever re-asserted to a reconnecting
controller. A stale stick replayed later would fly the aircraft on an input
nobody is holding.

Manual flight is deliberately slower than autonomous cruise (defaults: 6 m/s
horizontal, 2 m/s vertical, 60°/s yaw) and stays inside the same limits the
computer does. Reaching the flight-radius boundary zeroes only the outward part
of the stick, so the aircraft can still be flown along the fence instead of the
control dying in the operator's hand. Descent stops above the ground: manual may
fly low, but landing stays a computer job.

## Giving control back

Pressing MANUAL again sends `{"engaged": false}` and the computer takes the
aircraft back where the operator left it:

| Mission when control was taken | On release |
|---|---|
| ROI | re-enters the ROI and replans the sweep from the aircraft's current position |
| Patrol | discards patrol progress and plans a **fresh** patrol |
| Operator return home | carries on flying home |

The ROI sweep and the patrol route are both rebuilt from where the aircraft
actually is, not resumed from a waypoint it may now be a long way from. If the
operator flew somewhere the ROI can no longer be reached from, it falls back to
a patrol rather than leaving the aircraft stuck.

## Safety

Safety RTH always wins. While one is active the MANUAL button is disabled and
the command is refused, by the server and again by the controller — a
connection-loss RTH is latched on the aircraft before the server hears about it.
Releasing manual is never refused.

Anything that puts the computer back in charge ends the takeover, not just the
button: the cockpit RTH key, a PATROL or ROI command from the tablet, a safety
return. A latch that outlived the takeover would be worse than none at all --
the computer would be flying while every readout still said the operator was,
target acquisition would stay suppressed for the rest of the sortie, and the
next press of the button would resume a mission the aircraft left long ago.
Clearing the ROI mid-takeover hands back a patrol rather than flying back to an
area the operator has just finished with.

If a safety RTH begins while the operator is flying — low battery, or the link
to the tablet dying — the takeover is dropped, the aircraft flies home, and the
button clears. Nothing is handed back on the way: there is no mission to resume,
the aircraft is going home.

A tablet that stops sending sticks for longer than the command timeout (1 s by
default) leaves the aircraft hovering rather than drifting on the last stick it
heard. If the link stays down, the normal connection-loss RTH takes over from
there.

## Remote controllers

A bound physical remote controller outranks the tablet pad. Whoever is holding a
transmitter has the aircraft in sight, and two operators flying one airframe
through two links is not a situation to resolve mid-flight, so the pad yields
rather than summing into a third input.

The aircraft reports which sticks it is actually flying as
`manualInputSource: "tablet" | "rc"`. When it reports `rc`, the pad stays visible
but inert and says so, so the tablet never looks like it is in command of
something it is not. The simulator has no transmitter, so the pad always flies
there; a real drone adapter supplies one through `RemoteController` in
`controllers/autonomous_drone/manual_input.py`.

## Tuning

| Variable | Default |
|---|---|
| `DRONTECT_MANUAL_MAX_SPEED_MPS` | `6.0` |
| `DRONTECT_MANUAL_MAX_VERTICAL_SPEED_MPS` | `2.0` |
| `DRONTECT_MANUAL_MAX_YAW_RATE_DEG_S` | `60.0` |
| `DRONTECT_MANUAL_MIN_ALTITUDE_M` | `2.0` |
| `DRONTECT_MANUAL_COMMAND_TIMEOUT_SECONDS` | `1.0` |
