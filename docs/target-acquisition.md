# Target acquisition and false positives

Acquisition holds for 1.7 simulation seconds before normal tracker confirmation
can enter follow. Only fresh `lastSeenTimestamp` values count as observations;
cached predictions cannot complete the check. Takeoff continues its existing
vertical climb, and normal pursuit still requires its existing stability and
altitude checks.

Automatic rejection requires at least three fresh observations spanning the
window, a nearly unchanged image center, negligible observer/camera motion,
and low or unstable confidence. A high-confidence hovering target is accepted.
If motion or sparse observations make the test inconclusive, normal tracker
confirmation decides after the window. These thresholds are heuristics, not a
measurement of target world velocity.

The tablet's Approve and Reject buttons send `approve_target` and `reject_target`
with `params: {"trackId": "..."}`. Approve accepts the current visible acquisition
candidate immediately. Reject is available while a candidate box is visible and
for 1.5 seconds after disappearance. Rejection of a previous candidate during
that grace period cannot abort a different current candidate. A command ACK
confirms delivery; flight telemetry remains authoritative. Safety return blocks
both commands. Engage retains its existing mock behavior.

Rejection remembers the track ID and, where possible, an approximate ground
location for six simulation seconds. The location is a box-center camera-ray
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
