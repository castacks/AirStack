# mighty_bridge

Adapter between the MIGHTY planner (`asm_mighty`) and AirStack's local-planner
seam, plus the opt-in **native MIGHTY→PX4 setpoint path**.

Two nodes ship here:

| Executable | Always on? | What it does |
|---|---|---|
| `mighty_bridge_node` (`bridge_node.py`) | yes | odometry→`State`, `Trajectory`→`trajectory_override`, the `NavigateTask` action server, the `global_plan` follower |
| `mighty_native_setpoint_node` (`native_setpoint_node.py`) | **no — `MIGHTY_NATIVE_SETPOINTS`** | `Goal` → `mavros_msgs/PositionTarget` straight to MAVROS |

---

## Two command paths

There are two ways MIGHTY's plan can reach PX4. **Exactly one of them commands
the vehicle at any instant** — see "The gate" below for how that is enforced.

### 1. Legacy (default, always available)

```
mighty_node ── Trajectory (30-50 Hz) ──▶ mighty_bridge
                                            │  stride N, throttled to 1 Hz
                                            ▼
                            trajectory_controller  (trajectory_override)
                                            │  tracking_point
                                            ▼
                                     pid_controller  (position/velocity cascade)
                                            │  RollPitchYawrateThrust
                                            ▼
                                     mavros_interface
                                            │  mavros/setpoint_raw/attitude
                                            ▼
                                           PX4
```

This is the path takeoff, landing and `fixed_trajectory` use, and it keeps
working unchanged whatever the native path is doing. It is also lossy for
MIGHTY specifically: the bridge decimates the committed trajectory by
`segment_stride` and republishes at `override_period_s`, so the controller
extrapolates from a stale, position-only anchor between overrides, and the PID
cascade is (by default) pure feedback.

### 2. Native (opt-in, `MIGHTY_NATIVE_SETPOINTS=1`)

```
mighty_node ── Goal (100 Hz, = 1/par_.dc) ──▶ mighty_native_setpoint
                                                  │  mavros_msgs/PositionTarget
                                                  │  at stream_rate_hz (50 Hz)
                                                  ▼
                                    mavros/setpoint_raw/local ──▶ PX4
```

`mighty_node::publishGoal` has always published `dynus_interfaces/msg/Goal` at
`1/par_.dc` = 100 Hz on `goal` — position, velocity, acceleration, jerk, yaw,
dyaw — and until now nothing consumed it. This path takes it as-is. It mirrors
the bridge MIT-ACL actually fly (`jrached/ros2_px4_stack`'s
`dynus_offboard_node.py`); we send `PositionTarget` on `setpoint_raw/local`
rather than their `MultiDOFJointTrajectory` on `setpoint_trajectory` because it
is the same information through a simpler MAVROS plugin.

**Frames — the mapping is an identity, and that is a claim with a proof:**

* `mighty_node` stamps every Goal with `par_.map_frame_id`, which is `map` in
  every AirStack config.
* AirStack's `map` **is** the MAVROS local ENU frame: `interface.launch.py`
  runs `odometry_conversion` in `OVERWRITE` mode (`odometry_output_type: 2`) on
  `interface/mavros/local_position/odom`, and OVERWRITE only relabels
  `header.frame_id`/`child_frame_id` — it applies no transform.
* MAVROS's `setpoint_raw` plugin does the ENU→NED conversion itself for
  `coordinate_frame = FRAME_LOCAL_NED`, so the message must carry ENU values.

So the Goal's numbers go through verbatim. Any rotation in the mapping would be
a double conversion. (Contrast `mavros_interface.cpp::velocity_callback`, which
uses `FRAME_BODY_NED` — that command really is body-relative. Ours is not.)

**`type_mask` is `IGNORE_YAW_RATE` (2048) and nothing else**: position,
velocity, acceleration and yaw are all active — PX4 uses v and a as feedforward
on the position setpoint, which is the entire point — and `FORCE` stays clear so
`acceleration_or_force` is read as an acceleration. `Goal.dyaw` is dropped by
default (`send_yaw_rate:=true` to send it).

**Yaw** defaults to the direction of travel, not `Goal.yaw`. MIGHTY's `Goal.yaw`
is the DYNUS spline yaw (Sec. III of arXiv 2103.06372), not the body Euler yaw,
and it was measured to be 0 for an entire flight in this tree — the drone flew
sideways. `bridge_node` already overrides it for the legacy path; both paths now
share one `heading_from_velocity`. Set `yaw_source:=goal` to pass it through.

---

## The gate

Two command streams to PX4 at once is a fault: PX4 acts on whichever setpoint
arrived last and its `offboard_control_mode` flags follow the setpoint *type*,
so interleaving attitude targets and position targets makes the flight task
switch at the stream rate. The native path may therefore stream **only** while
MIGHTY owns the vehicle, and the pid must be silent for exactly that window.

**Engagement signal.** `mighty_bridge` already knows when the follower or a
`NavigateTask` owns the vehicle (`_route_active`). It now broadcasts that on
`native_stream_active` (`std_msgs/Bool`, transient-local, 1 Hz heartbeat) —
gated on the same env var, so with the feature off the topic does not exist.
The heartbeat is half the contract: the streamer treats *no engagement message
for `engage_timeout_s`* as disengaged, which is what makes a dead bridge give
the vehicle back rather than freeze the mute.

**The muzzle.** `pid_controller` gained a `command_muted` parameter (default
`false`), set by the streamer over `pid_controller/set_parameters`. That node is
the leaf of the legacy chain — the only publisher of
`interface/cmd_roll_pitch_yawrate_thrust`, publishing from one place — so one
`if` covers the whole path.

Two alternatives were rejected:

* a **`trajectory_controller` mode** is not a muzzle. `tracking_point_pub->
  publish()` is unconditional in its timer for `PAUSE` / `ROBOT_POSE` / `TRACK`
  alike, so the pid keeps emitting attitude in all of them.
* **gating inside `mavros_interface`/`robot_interface`** would put a new branch
  in the safety boundary that arming, takeoff and land all route through, and in
  a class shared with `px4_interface`. Same effect, bigger blast radius.

**The mute is a deadman, not a latch.** The streamer re-asserts it every
`mute_refresh_s` (0.2 s); `pid_controller` un-mutes itself if it goes
un-refreshed for `command_mute_timeout` (0.5 s). Without that, this node
crashing mid-flight would leave the cascade muted with nothing else commanding
and PX4 would trip its 500 ms OFFBOARD failsafe.

### Ordering

**Engage — mute first, then stream.** A few tens of ms with no new setpoint is
nothing to PX4 (it holds the last one), whereas overlapping the two streams
flips its control mode at the stream rate. Streaming begins only on a
*confirmed* mute, and the mute is only requested once a **fresh Goal** is in
hand — muting into silence is how you lose a vehicle with no error message
anywhere. If the pid service is missing or refuses, nothing is streamed and the
legacy path keeps flying.

### Disengage grace — why a disengage does not mean a handback

Measured on the first raven-search flight with the native path armed: the
handoff above is correct *per goal*, but raven's search issues a **new
NavigateTask every few seconds**. Each completion disengages `mighty_bridge`
and each new task re-engages it, so a disengage-means-handback rule produced a
continuous `engage → stream → handback → pid → engage` cycle — back-to-back
`MUTED`/`UNMUTED` lines in the pid log, and flight as jerky as the legacy path,
because the vehicle spent most of its time being handed between two controllers
instead of being flown by either. The original demo looked clean only because
its three goals were long-lived.

So a disengage is **provisional**. The streamer enters `GRACE`: it keeps the
vehicle, holds at the last commanded setpoint, keeps the pid muted and keeps
feeding the deadman, for `disengage_grace_s` (default 4.0 s, env
`MIGHTY_NATIVE_GRACE_S`). If engagement returns inside that window — the normal
case under goal churn — it resumes streaming fresh Goals **on the same tick**,
with zero service calls, no mode change and no un-mute. Only if the quiet lasts
the whole window does the ordered handback below run.

The hold during grace freezes on **the last setpoint actually sent**, not the
newest `Goal`: `mighty_node` keeps publishing Goals whether or not a task is
engaged, so holding "at the latest goal" would silently keep flying a route the
task layer has already finished.

Two disengage reasons deliberately **skip** the grace, because waiting is a
hazard rather than caution: the master switch going off (an operator asking for
the legacy path back should get it now), and MIGHTY's goals going stale past
`hold_max_s` (the planner is gone, and land / RTL / `fixed_trajectory` are inert
behind our mute until we release it). A refused deadman refresh also hands back
immediately.

Cost: a genuinely dead bridge now takes `engage_timeout_s + disengage_grace_s`
(≈ 7 s) to give control back, not 3 s. Keep that sum under any mission-level
watchdog. `disengage_grace_s: 0` restores hand-back-immediately.

**Nothing on the `mighty_bridge` side needed to change for this**, which is
worth recording because it is not obvious — we now stream straight across
`_finish`'s `set_trajectory_mode(TRACK)` call. Audited: the bridge's only mode
calls are `TRACK` on engage (`_follow_route`, `_execute_navigate`) and `TRACK`
on task finish (`_finish`), and all three land while the pid is muted, so they
move the trajectory controller's own timeline and nothing reaches PX4 from any
of them. Its other `_route_active` reads (the trajectory start guard, the
carried yaw) feed only the `trajectory_override` path, which is equally inert
behind the mute. The grace also *removes* the `ROBOT_POSE`-vs-`TRACK` race that
`handback_pin_s` guards: by the time a grace expires, that `TRACK` is seconds
old.

**Handback — pin, then un-mute, then stop.**

1. switch our own stream to a **hold** at the vehicle's position (latched once,
   not re-read: chasing live odometry is a drift, not a hold);
2. ask the trajectory controller for `ROBOT_POSE`, which pins its tracking point
   to the vehicle with zero velocity — so the pid wakes up onto "stay here", not
   a stale point on a MIGHTY timeline it never flew. **Wait for the answer**:
   `mighty_bridge` asks the *same service* for `TRACK` at the instant it
   disengages, and `TRACK` with a cleared trajectory freezes the tracking point
   at the last MIGHTY waypoint. Bounded by `handback_pin_s`;
3. un-mute the pid;
4. keep streaming the hold until the pid has been **seen** commanding again
   (`handback_confirm_msgs` on its output topic) or `handback_timeout_s`
   elapses, then stop.

Step 4 is a deliberate overlap — the opposite of the gap accepted on engage —
because the failure it guards is worse and silent: `pid_controller` returns
early without publishing whenever a TF lookup fails, so "un-muted" does not
imply "commanding". The overlap is safe precisely because of steps 1-2: both
paths are commanding the same hold, so it does not matter which PX4 acted on
last.

**Also hands back** when MIGHTY's Goals go stale for `hold_max_s` — land, RTL
and `fixed_trajectory` all go through the muted pid, and leaving them inert
behind a dead planner is not acceptable. (For a *short* Goal gap the streamer
holds the last commanded point instead, which keeps OFFBOARD alive without
replaying a stale velocity.)

---

## Enabling it

```bash
# .env  (or the host environment; compose passes these through bare-name)
MIGHTY_NATIVE_SETPOINTS=1
MIGHTY_NATIVE_GRACE_S=4.0   # optional; this is the default
```

The first variable arms the streamer's launch, its `enabled` parameter and the
bridge's engagement broadcast. Unset/`0`/`false`/empty = today's behaviour, with
no new node, no new topic and `command_muted` never touched. The second tunes
the disengage grace window (above); unset or unparseable falls back to 4.0.

### Parameters (`mighty_native_setpoint`)

| Parameter | Default | Meaning |
|---|---|---|
| `enabled` | `$MIGHTY_NATIVE_SETPOINTS` | master switch |
| `stream_rate_hz` | 50.0 | PositionTarget rate (PX4 needs ≥ 2 Hz to hold OFFBOARD) |
| `world_frame` | `map` | frame a Goal must be stamped in; anything else is refused, not transformed |
| `yaw_source` | `velocity` | or `goal` to pass `Goal.yaw` through |
| `yaw_min_speed` | 0.3 | below this horizontal speed the heading is held |
| `send_yaw_rate` | false | also send `Goal.dyaw` (clears `IGNORE_YAW_RATE`) |
| `engage_timeout_s` | 3.0 | engagement heartbeat older than this ⇒ disengaged |
| `disengage_grace_s` | 4.0 (`$MIGHTY_NATIVE_GRACE_S`) | how long a disengage stays provisional before the handback; 0 = hand back immediately |
| `goal_stale_s` | 0.5 | Goal older than this ⇒ hold instead of fly |
| `hold_max_s` | 5.0 | holding on stale Goals for this long ⇒ hand back |
| `mute_refresh_s` | 0.2 | deadman cadence; must stay < `command_mute_timeout` |
| `mute_retry_s` | 1.0 | retry cadence for a refused/unavailable mute |
| `handback_pin_s` | 0.3 | max wait for the `ROBOT_POSE` acknowledgement |
| `handback_confirm_msgs` | 3 | pid commands that end the handback overlap (0 disables the check) |
| `handback_timeout_s` | 1.0 | upper bound on that overlap |
| `request_robot_pose_on_handback` | true | pin the tracking point before un-muting |

### Parameters added to `pid_controller`

| Parameter | Default | Meaning |
|---|---|---|
| `command_muted` | false | suppress command publishes (set by the streamer) |
| `command_mute_timeout` | 0.5 | deadman; un-mute if not re-asserted within this. `<= 0` disables the watchdog — bench only |

---

## Tests

`test/` runs on a bare host, no ROS:

```bash
pytest robot/ros_ws/src/modules/asm_mighty/mighty_bridge/test
```

* `test_native_setpoint.py` — the Goal→PositionTarget mapping, the exact
  `type_mask`/frame constants, the yaw convention, frame refusal, the
  mirrored-constants guard, env-flag parsing.
* `test_handoff_state_machine.py` — the engagement gate, both orderings, and
  the grace window (goal churn costs zero service calls and never breaks the
  stream; the deadman keeps being fed through it; expiry runs the ordered
  handback unchanged; the two reasons that skip it).
* `test_native_setpoint_node.py` — the node driven end to end against stubbed
  ROS modules (`ros_stubs.py`); skips where a real ROS is installed.

What these **cannot** cover, and what a live bring-up still has to confirm:
that PX4 stays in OFFBOARD across the mute (no failsafe on the engage gap),
that the ENU→NED identity is right on the wire, that the handback does not
produce a visible twitch, and — for the grace window — whether *holding*
through a short inter-task gap still costs a visible decelerate/accelerate
cycle. If it does, the next knob to try is following Goals through the grace
rather than holding, which is a one-line change in `_publish_hold` but a real
behavioural risk (it would keep flying a route the task layer has finished).
