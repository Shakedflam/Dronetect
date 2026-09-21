# Target acquisition and false positives

Acquisition holds for 1.7 simulation seconds before normal tracker confirmation
can enter follow. Only fresh `lastSeenTimestamp` values count as observations;
cached predictions cannot complete the check. Takeoff continues its existing
vertical climb, and normal pursuit still requires its existing stability and
altitude checks.

Automatic rejection requires at least three fresh observations spanning the
window, a nearly unchanged image center, a nearly unchanged box area,
negligible observer/camera motion, and low or unstable confidence. A
high-confidence hovering target is accepted. If motion or sparse observations
make the test inconclusive, normal tracker confirmation decides after the
window. These thresholds are heuristics, not a measurement of target world
velocity.

The box-area condition is what confines the filter to genuinely static
objects. A target closing on the aircraft has no lateral image motion while
the aircraft hovers, so image position alone cannot tell it from a fixed
false positive, and on weak confidence the window used to discard it -- the
most dangerous target being the one most likely to be abandoned. Apparent
size is the discriminator: an antenna or a mark on a building holds its size,
anything approaching or receding does not.

Pursuit requires no operator authorization. A confirmed track is followed
automatically, so the only target decision an operator makes is the veto: the
tablet's Reject button sends `reject_target` with `params: {"trackId": "..."}`.
Reject is available while a candidate box is visible and for 1.5 seconds after
disappearance. Rejection of a previous candidate during that grace period
cannot abort a different current candidate. A command ACK confirms delivery;
flight telemetry remains authoritative. Safety return blocks the command.
Engage retains its existing mock behavior.

There was previously an Approve button beside it. It never gated pursuit --
tracker confirmation already started the follow on its own -- so all it did
was cancel the pending automatic rejection. A control that implies
authorization it does not provide is worse than no control, and it was
removed rather than left to mislead.

The gate that does exist is the permanent **HOLD PURSUIT** toggle, sent as
`set_pursuit_block` with `params: {"blocked": true|false}`. It defaults to
released, so pursuit is autonomous. Engaged, it suppresses acquisition,
follow and pursuit entirely, and disengages a chase already in progress --
a hold that applied only to targets not yet acquired would leave the operator
watching the very pursuit they asked to stop. Detections still reach the
operator throughout: only the aircraft's reaction to them is suppressed.

Because it is a safety-increasing control it is never refused, including
during a safety return. The drone reports the hold in telemetry and the
server re-asserts an engaged hold to any drone that connects, so a controller
that restarts mid-hold cannot come back pursuing.

## Height above the target

How far above or below the target the aircraft is, is measured rather than
assumed, from four things it already has:

- the **slant range**, from box area against the range calibration below
- the **camera pitch** it is actually looking down at
- **where the box sits vertically in frame**, which gives the ray's elevation
  once the pitch is taken out of it
- the aircraft's **own altitude**, which the answer is relative to

`estimate_target_altitude` combines them as
`height_error_m = slant_range * sin(elevation)`, positive when the target is
above. The stern chase holds `altitude_offset_m` above that.

Because height is slant range times a sine, **any error in range is an error
in height by the same factor**. That is what used to leave the aircraft well
above its target: with range short by 2.3x, a target 10 m below read as 4.4 m
below, and the aircraft settled about 5.6 m high with every gain and band in
the loop working exactly as intended. Fixing the range calibration fixes the
height with it; nothing in the vertical law needed to change for that.

What did change is how closely the station height is then held.
`target_altitude_target_band_m` is 0.3 m (was 0.5) and
`target_altitude_gain_per_s` is 1.5 (was 1.0), so the aircraft settles within
about 0.3 m of the intended standoff height instead of 0.5 m, and gets there
sooner. The band is there to absorb box jitter -- the estimate is recomputed
from a fresh box every frame -- and not to licence drifting high, so it is
kept just wide enough to do that job. Tune with
`DRONTECT_TARGET_ALTITUDE_TARGET_BAND_M` and
`DRONTECT_TARGET_ALTITUDE_GAIN_PER_S`.

## Chasing fast enough to get there

A stern chase needs a **speed margin** over the target, not parity. The
simulated traffic drones cruise at 6.0-6.6 m/s and the pursuit was capped at
6.0, so the gap could only ever hold or grow: an aircraft starting 34 m behind
stayed 34 m behind indefinitely, and no amount of station-law tuning moved it,
because there was no closing speed to tune. `DRONTECT_PURSUIT_MAX_SPEED_MPS`
is now 10.0, and the same chase closes to the standoff in about fifteen
seconds and holds it.

The margin is spent only where it helps. Closure stays capped by
`approach_time_to_contact_s`, so the aircraft does not arrive at the target
faster than it can stop being on a collision course with it, and the height
clearance bound (`v^2 / 2a` against the floor) still applies -- at a typical
chase altitude that leaves the full 10 m/s available, and lower down it
tapers on its own.

## Coming back from a chase

When a target interrupts a mission, the aircraft flies back to the point it
was interrupted at before picking the mission up again. That return needs a
route clear of NO_FLY sectors, and pursuit deliberately ignores NO_FLY
routing -- so a chase can leave the aircraft somewhere no safe route home
exists from.

The return used to hover and retry for one every tick. That wait never ends
on its own: hovering cannot change the aircraft's position, and its position
is the one thing the route depends on, so a single refusal repeated
indefinitely leaves the aircraft holding station until the battery decides
for it. Holding pursuit during a lost-target search makes it easy to hit,
because the return then begins immediately from wherever the chase left the
aircraft rather than after the search has coasted it clear.

The wait is now bounded by `DRONTECT_MISSION_RETURN_TIMEOUT_S` (10 s). Past
it, the aircraft restores the interrupted mission's context -- the ROI sweep,
the patrol route -- and resumes it from where it actually is. The exact point
the mission was interrupted at is worth less than the mission.

## Not breaking the simulation

The aircraft is flown by a supervisor writing poses directly, so it never
bounces off anything: driving it into another airframe is an interpenetration
the physics engine has to resolve, and the whole world pays for that, not just
this aircraft. Once the pursuit was given a real speed margin it could reach
the traffic drones for the first time, which made that failure reachable too.

Braking distance grows as the square of speed: at 10 m/s with 5 m/s2 the
aircraft needs 10 m to stop, and the standoff is 5. The tapering closure cap
normally covers that, but it is gated on a valid range estimate that a single
bad box removes, and the station point it flies to is only as good as the
world estimate behind it. So there is now a floor that depends on none of
them: `minimum_separation_m` (2 m, beneath the 2.5 m dodge threshold) removes
the part of the commanded velocity that would close inside it this tick,
leaving everything crossways untouched so the aircraft keeps manoeuvring
rather than stopping dead.

Separately, the controller tick is wrapped. A Webots controller that raises
stops calling `step()`, and Webots waits on it -- so one exception freezes
every robot in the world, with the cause buried. A failed tick now prints its
traceback, latches a safety RTH, and keeps stepping; after
`MAX_TICK_FAILURES` it stops commanding the aircraft but still steps, so the
simulation stays alive and the error stays visible.

## How hard to yaw

Keeping the target near frame centre is not cosmetic: it is where the tracker
holds identity and where the box distorts the range estimate least. Three
things make a turn urgent, and the rate limit reads all three:

- **How fast it is crossing.** Holding a target centred takes exactly
  `v_perp / range` rad/s, so that is a feed-forward term rather than
  something a proportional loop has to discover. Its weight is
  `target_yaw_rate_margin`, now 2.0 (was 1.5).
- **How close it is.** The same lateral motion sweeps across frame faster the
  nearer the target, so the bearing time constant is shortened below
  `DRONTECT_TARGET_YAW_RANGE_REFERENCE_M` (30 m) -- a correction that is
  unhurried at thirty metres is already too slow at ten.
  `DRONTECT_TARGET_YAW_RANGE_MIN_FACTOR` floors that shortening, because range
  is in a denominator and a target a metre away would otherwise demand a rate
  set by nothing but how small that number got.
- **How near the frame edge it is.** A target about to leave frame is a track
  about to be lost, so the whole rate is scaled by
  `1 + DRONTECT_TARGET_YAW_EDGE_GAIN * edge^2`. Squared, so it stays out of
  the way through the comfortable middle of the frame and bites only when the
  last of it is being defended.

The resulting ceiling, in deg/s, for a stationary target:

| off centre | 40 m | 20 m | 10 m | 5 m |
|---|---|---|---|---|
| 10% | 20 | 20 | 20 | 20 |
| 30% | 20 | 28 | 56 | 62 |
| 60% | 53 | 80 | 120 | 120 |
| 90% | 119 | 120 | 120 | 120 |

The floor (`target_min_yaw_rate_rad_s`, 20 deg/s) keeps a distant, centred
target from being chased with twitchy corrections. The ceiling
(`DRONTECT_TARGET_MAX_YAW_RATE_DEG_S`, now 120 deg/s, was 70) protects the
detector from motion blur and bounds the own-yaw term that
`_pursuit_image_motion` subtracts out, which is only a first-order
correction -- yaw hard enough and the aircraft loses the target to its own
turn.

## Holding the six, and losing it

The stern chase holds a station one standoff behind the target along its own
heading and one offset above it. Closing the gap is capped by
`approach_time_to_contact_s`, so the part that actually takes time is swinging
around behind a target the aircraft is already keeping pace with. That is
governed by `station_keeping_gain_per_s`, raised from 0.8 to 1.6
(`DRONTECT_PURSUIT_STATION_KEEPING_GAIN`) so the aircraft gets onto the six
sooner without changing how fast it is allowed to close.

When the target goes out of sight, two things now happen that did not before:

- **The search gains a metre of height**, measured from the altitude the
  target was lost at, braking into it so the metre is actually gained inside
  the search window rather than still converging when the window ends. A
  target behind a roofline is reacquired by seeing over it, not by staring at
  where it was. The climb is one-way -- descending during a search would put
  the aircraft back behind the same obstruction -- and it is measured per
  loss, so a target behind a row of trees cannot ratchet the aircraft upward
  one metre per occlusion. Tune with `DRONTECT_TARGET_LOST_SEARCH_CLIMB_M`.
- **The sweep is faster, and faster still if it went out sideways.** Where a
  target vanished says how far away from the camera's aim it already is. One
  lost in the middle of frame went behind something and will come back out
  near where it was, so a slow sweep keeps it in view. One lost at the edge
  left sideways and is already somewhere the camera is not pointing, so the
  sweep has to catch up to it rather than wait for it. The base rate
  (`DRONTECT_TARGET_SEARCH_YAW_RATE_DEG_S`) is 25 deg/s, was 10, scaled by the
  same edge urgency the tracking yaw uses:

  | last seen | sweep |
  |---|---|
  | dead centre | 25 deg/s |
  | 40% out | 33 deg/s |
  | 70% out | 50 deg/s |
  | at the edge | 75 deg/s |

  The yaw rate limit is refreshed during the search too. It is otherwise only
  updated while the target is visible, so a target that drifted out slowly
  would leave the limit at its floor and the widened sweep would never
  actually be flown.
- **The search runs a second longer**, three seconds instead of two
  (`DRONTECT_TARGET_LOST_SEARCH_TIMEOUT_S`). Two seconds was ending searches
  while the target was still behind something, which cost the track and sent
  the aircraft back to its patrol. The window also sets the decay on the
  dead-reckoned drift, so a longer one keeps pace with the target's last known
  velocity for longer as well.

## Range calibration

Everything the aircraft believes about absolute distance comes from box area:
area falls off as the square of range, so one box at a known range fixes the
whole scale. That scale used to be taken from the pursuit standoff pair
(`target_standoff_distance_m` / `target_standoff_box_area_ratio`), which is a
*setpoint* -- when to stop closing -- and not a measurement of anything. It
described a 0.19 m target, so a 0.45 m Mavic read as 2.3x closer than it was.

That was harmless while pursuit was purely image-space: a law driven by box
area error and image velocity never needs to know the distance. It stopped
being harmless when the stern chase began steering to world coordinates and
the pass-under logic began gating on absolute range -- a target at 48 m read
as 21 m, inside the 25 m dodge window, and the aircraft stepped around
something that was never close to it.

The calibration now has its own knobs, deliberately separate from the
setpoint:

| Variable | Default | Meaning |
|---|---|---|
| `DRONTECT_TARGET_REFERENCE_SPAN_M` | `0.45` | Expected target span, in metres |
| `DRONTECT_TARGET_REFERENCE_DISTANCE_M` | `2.0` | Anchor distance for the ratio |
| `DRONTECT_TARGET_REFERENCE_AREA_RATIO` | *(derived)* | A directly measured ratio, which wins when set |

Left unset, the ratio is derived from the span and the *real* lens and frame
via pinhole projection, so it tracks `camera.getFov()` instead of being a
constant that silently assumes one. Because the projection is exact, the
anchor distance is arbitrary -- moving it does not change any reported range.

Measure it rather than trust the span when you can: a trained detector draws
boxes around rotor blur and landing gear, not around the airframe. In the
simulator, set `DRONTECT_TARGET_RANGE_CALIBRATION` to the DEF name of the
target node (for example `TRAFFIC_DRONE_2`) and fly a pass. Each tracked
second prints the true range read straight out of the world and the ratio it
implies:

```text
DRONTECT_TARGET_RANGE_CALIBRATION {"boxAreaRatio":0.000241,"node":"TRAFFIC_DRONE_2",
"referenceDistanceM":2.0,"reportedRangeM":45.8,"suggestedReferenceAreaRatio":0.126,
"trueRangeM":48.1}
```

Put `suggestedReferenceAreaRatio` into `DRONTECT_TARGET_REFERENCE_AREA_RATIO`
and `reportedRangeM` should land on `trueRangeM` on the next pass.

One consequence to expect: reported ranges are now ~2.3x larger, so
`target_range_estimate_max_m` (100 m) now cuts off at a true 100 m rather than
a true 230 m. A target parked beyond that gets no world fix at all -- correct,
but it means a distant traffic drone that used to be localised no longer is.
Raise it only knowingly: the area-derived estimate degrades as the square of
distance, so it is weak well before that limit.

When a confirmed target is lost, the search follows where it was going rather
than stopping where it vanished. A target does not disappear at rest -- it
goes behind a tree, a mast or a roofline while still moving -- so holding
station and sweeping from a standstill points the camera at the one place it
will not reappear. The bearing is dead-reckoned at the rate the target was
last crossing frame, the sweep widens in that same direction rather than
defaulting to a side, and the aircraft keeps pace with the target's last known
world velocity. All three decay to nothing across the lost-search window, so a
target that is genuinely gone leaves the aircraft stopped and pointing
sensibly instead of flying off after a stale guess. The drift uses its own
slow smoothing, not the alpha-beta velocity the stern chase steers by: that
one is deliberately responsive, and a single box-size step on it reads as
metres per second of motion that never happened. The geofence is rechecked on
this path because it moves without a fix to steer by.

Rejection remembers the track ID and, where possible, an approximate ground
location for twelve simulation seconds. The location is a box-center camera-ray
intersection with the flat plane at home altitude. It is **not measured target
GPS**, and can be inaccurate for airborne objects, buildings, terrain, and delayed
frames. Rays within 5 degrees of the horizon, above it, or projecting beyond
250 m use track-ID suppression only. Replacement track IDs within 5 m of a
valid stored ground point are also ignored. Visibility and repeated commands
do not extend the timeout. Expiry permits reacquisition of the same track ID.

Rejection during acquisition/follow restores the saved patrol or ROI context.
After pursuit has started, the existing safe checkpoint-return route runs first.
The ignore list survives ordinary mission changes until expiry; operator grace
history is cleared on explicit mission replacement or return-home.

## Configuration

Environment variables are read when the simulator controller starts. All use the
`DRONTECT_TARGET_` prefix; the suffixes and defaults are below.

| Suffix | Default | Meaning |
| --- | ---: | --- |
| `OBSERVATION_S` | 1.7 | Initial observation duration; zero restores immediate confirmation |
| `OBSERVATION_MIN_SAMPLES` | 3 | Minimum fresh observations for automatic rejection |
| `STATIONARY_CENTER_RATIO` | 0.02 | Maximum center range on each normalized image axis |
| `STATIONARY_OBSERVER_M` | 0.25 | Maximum aircraft displacement from the initial pose |
| `STATIONARY_ANGLE_DEG` | 2 | Maximum accumulated yaw or camera-pitch travel |
| `LOW_CONFIDENCE` | 0.60 | Mean-confidence threshold |
| `CONFIDENCE_RANGE` | 0.20 | Instability range, also requiring minimum confidence below LOW_CONFIDENCE |
| `IGNORE_RADIUS_M` | 5 | Horizontal distance around projected ground location |
| `IGNORE_TIMEOUT_S` | 6 | Time from rejection, not from last visibility |
| `REJECT_GRACE_S` | 1.5 | Backend grace after disappearance; tablet selection uses 1.5 s |
| `PROJECTION_MIN_ANGLE_DEG` | 5 | Minimum downward ray angle |
| `PROJECTION_MAX_RANGE_M` | 250 | Maximum horizontal ground projection distance |

The existing acquisition disappearance timeout and follow/pursuit settings remain
independent. Confidence observations can include weak matches below the minimum
flight-guidance confidence, but never detections discarded by YOLO itself.
