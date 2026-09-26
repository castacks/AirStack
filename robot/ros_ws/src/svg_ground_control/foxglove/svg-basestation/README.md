# SVG Basestation

Ground-station panel for the SVG counter-UAS demonstration — the SVG analogue of
the DTC *Robot Control Panel* that anchors the `foxglove_ws` basestation layout.
One panel owns agent selection, the swarm-wide safety command, mission
confirmation, the runtime gains (CBF, teleop speed cap, go-to-goal law),
per-drone state and position, and the operator's health picture.

Three ideas drive the whole panel:

| Idea | What it means |
| --- | --- |
| **Mode is wiring** | An agent is `sim` or `real`, exactly as in `swarm_commander`'s `drone_modes`. `sim` talks to the MAVROS interface (`/{name}/interface/...`); `real` talks to `px4_interface` over uXRCE-DDS (`/{name}/fmu/...`) and additionally carries mocap, EKF and timesync telemetry. The **Wiring** card spells out the resolved topics for the selected agent. |
| **Topics decide what you see** | Every section declares the topics it needs. A section with no publisher is not rendered, so a sim-only run shows no empty mocap/EKF columns and a run with no cellular reporter shows no empty cellular table. The banner lists the tasks the panel inferred. |
| **No fabricated numbers** | Anything with no source reads `--`. Every derived value is labelled with where it came from. |

There is **no Wi-Fi mesh**. The transport model is the local Wi-Fi / LAN path used
in the lab, plus a 4G/5G path carrying the same DDS traffic inside a Tailscale VPN
for field work.

## Install

The panel lives with the rest of SVG ground control in
`robot/ros_ws/src/svg_ground_control/foxglove/`. Its `install.py` (one directory
up) copies any directory there that has a `package.json` into
`~/.foxglove-studio/extensions`; the robot-desktop container runs it at start-up,
and for a Studio on the host run it yourself, then (re)start Studio —
extensions are loaded only at start-up:

```
python3 robot/ros_ws/src/svg_ground_control/foxglove/install.py   # installs airlab-cmu.svg-basestation-1.0.0
```

(The general AirStack panels — Robot Tasks, Waypoint / Polygon editors — stay
in `gcs/foxglove_extensions/` with their own `install.py`.)

`svg_basestation.json` (one directory up) is a ready-made layout with **three
instances** of this panel plus a 3D view of `/svg/viz/markers`:

```
┌──────────────────────────┬──────────────────────────┐
│ SVG Basestation          │ 3D  (/svg/viz/markers)    │
│  View = main             │                          │
│  safety · command · CBF  ├─────────────┬────────────┤
│  goal · agents · state   │ SVG Battery │ SVG Teleop │
│  link safety · cellular  │  & Power    │  (sticks)  │
│                          │ View = power│View = teleop│
└──────────────────────────┴─────────────┴────────────┘
```

The **View** setting (gear icon → Swarm) picks what an instance shows: `main`
(everything except Battery & Power and Teleop), `power` (Battery & Power only,
with the power chip and clock in its banner), `teleop` (the Teleop · Sticks
card only — it reads "Teleop off" while `safe_teleop` is not publishing), or
`full` (the old single-panel form).
The 3D view's built-in grid layer is off: it is a fixed 8 m square on the
origin and never matches the fence. The commander draws a grid on the fence
floor instead (`fence_grid_cell_m`), clipped to the fence and aligned to world
metres, inside `/svg/viz/markers`.
There are no separate battery plots — SoC, pack voltage, sag and the RTB budget
all live in the power instance; plots against `…/fmu/out/battery_status` only
exist on real hardware and read blank in simulation. Load the layout with
**Layout → Import from file** (re-import after updating the panel: the older
single-panel layout stacks everything on the left and gets cut off).

## Safety stop

The red bar under the banner is the one control that must work without reading
anything: it lands every commanded drone (`~/land`, which descends and disarms on
touchdown). It takes **two clicks** — the first arms it for 4 s, the second fires
— so it is fast under stress but an accidental brush cannot land the swarm. Next
to it, **Hold All** (`~/hold`) is the softer stop: every drone freezes at its
current position and the scenario stops.

Takeoff / Start / Reset Fence stay in the ordinary command strip below.

## Swarm Command: did the command actually happen?

A service reply is not proof. On a bad link the reply can be lost while the
command ran, or arrive while the commander rejected it for a reason the operator
did not see. So the panel reads `swarm_commander`'s own **status snapshot**
(`/svg/commander_status`, `std_msgs/String` JSON at 5 Hz, published by
`SwarmCommander.build_status`) and reports from that:

| Element | What it shows |
| --- | --- |
| **Mission chip** | `ON GROUND`, `TAKING OFF`, `READY TO START`, `NOT READY` (some commanded drones still on the ground — Start is rejected), `RUNNING` (with the scenario name and how long since Start), `HOLDING` (scenario stopped after running), `LANDING`, `FENCE BREACH`, or `NO COMMANDER` when nothing has arrived on the status topic for 2 s |
| **Last-command chip** | The commander's record of the newest lifecycle service it handled: `✓ start 12:01:33` or `✗ start …` with the rejection reason on hover. This comes from the commander, so it is present even when the reply never reached the panel |
| **Command log** | The last four commands sent from this panel, newest first: `sent, awaiting reply` → `reply: accepted / REJECTED / TIMEOUT / FAILED` → `✓ confirmed by commander` or `✗ NOT CONFIRMED`. Confirmation means the snapshot's command counter advanced with this command's name *and* the expected effect is visible (Start → `mission_active`, Hold → not active, Takeoff → drones arming/climbing, Land → drones landing, Reset Fence → latch clear). Without a reply, the snapshot alone can still confirm. A reply that never comes is reported after 6 s instead of hanging on "Calling…" |

Per drone, the **Agent State** table (below) adds the two things that prove
Start reached the *drone*: its flight state (`ACTIVE` etc.) and the rate of
velocity commands arriving on its command topic (`Cmd stream`, ~20 Hz while the
commander drives it, `silent` in red if the commander thinks it is airborne but
nothing is being published to it).

## Runtime gains (CBF alpha, safety radius, max speed · teleop max speed · goal accel, settle)

One slider row edits `swarm_commander`'s runtime parameters. The **dropdown**
on the left picks which gain the row is editing; slider and number box are one
draft value (kept per gain, so switching does not lose a half-typed number);
**Apply** sends that gain; the fixed-width **live** readout shows what the
commander is running with plus a mark: `✓` confirmed, `…` waiting, `✗` rejected
or not taken. The readout never grows, so the slider keeps its length — the
reason for a rejection goes to the status line under the row.

| Row | Parameter | Meaning |
| --- | --- | --- |
| `CBF α` | `cbf_alpha` | Class-K gain in the barrier constraint `ḣ + α h ≥ 0`. **Lower is gentler**: the filter starts yielding early and corrects softly. **Higher is more aggressive**: drones approach closer before a harder correction |
| `CBF r` | `cbf_safety_radius_m` | Each drone's safety bubble; every pair of centres is kept more than `2r` apart. Larger = wider berth. Goals or squeeze posts closer than `2r` become infeasible and trigger the emergency push-apart |
| `CBF vmax` | `cbf_max_speed_mps` | Cap on every velocity command the filter emits, exempt drones included. Higher lets drones dodge (and fly) faster |
| `Teleop vmax` | `teleop_max_speed_mps` **and** `safe_teleop`'s `max_speed_mps` | The hand-flown drone's speed at full stick. The commander caps the stick velocity at `teleop_max_speed_mps`; `safe_teleop` scales the stick to `max_speed_mps`. The lower of the two silently wins, so **Apply sets both** — the commander first, then `safe_teleop` — and the readout shows both: `live 3.00 m/s ✓ · pad 3.00 ✓`. Still capped by `cbf_max_speed_mps` unless the drone is `cbf_exempt` |
| `Goal accel` | `goal_accel_mps2` | Acceleration and braking of the go-to-goal reference profile. Braking distance is `v²/(2a) + v·settle`: higher brakes later and harder (PX4 auto uses 3, the airframe managed 5.5); too high for the airframe overshoots |
| `Goal settle` | `goal_settle_s` | Exponential tail into the goal (time constant). 0.3 is PX4-like; larger = softer stop, slower arrival; smaller = sharper arrival. `0` is allowed and removes the tail |

- **live** is what the commander is running with right now — from the status
  snapshot when it is fresh (`cbf` for the filter gains, `tuning` for the
  rest), else from a `get_parameters` read (↻ re-reads all six, and
  `safe_teleop`'s `max_speed_mps`). Under the row: which drones the CBF is
  correcting this tick, and a red `EMERGENCY push-apart` if the QP went
  infeasible.
- **pad** (Teleop vmax only) is `safe_teleop`'s `max_speed_mps`, read from
  `<teleop ns>/get_parameters` (the `Teleop node namespace` setting,
  `/safe_teleop`). `✓` when it equals the commander's `teleop_max_speed_mps`,
  `✗` in amber with a note when it does not (press Apply to set both), `--`
  when `safe_teleop` is not running, which is normal without a hand-flown
  drone. `safe_teleop` also follows the commander's value on its own from the
  status snapshot, and pushes a `ros2 param set /safe_teleop max_speed_mps`
  back to the commander, so the two agree whichever side was changed.
- Move a slider or type a value, then that row's **Apply**. The panel calls
  `<commander ns>/set_parameters` (`rcl_interfaces/srv/SetParameters`,
  double) for that one parameter. The commander validates (finite, `> 0`;
  `goal_settle_s` may be `0`) and applies it on its next control tick; a
  rejection reason is shown in the status line. For Teleop vmax the same
  number then goes to `<teleop ns>/set_parameters` as `max_speed_mps` — only
  after the commander accepted it, so a rejection never leaves the two apart. Until the snapshot reports the new value the readout shows
  `(asked 0.80…)`; if the commander keeps reporting the old value after the
  set, it turns amber.
- The same parameters can be set from a shell and the panel follows:
  `ros2 param set /swarm_commander cbf_safety_radius_m 0.8`.
- The scenario keeps the radius it was launched with for its own spacing
  checks (holder posts, random goals); only the filter, the speed cap and the
  keep-out spheres in `/svg/viz/markers` follow a runtime change.

The sliders' upper ends are the `CBF alpha slider max` (10), `CBF radius
slider max` (2 m), `CBF max-speed slider max` (3 m/s), `Teleop max-speed
slider max` (5 m/s), `Goal accel slider max` (15 m/s²) and `Goal settle
slider max` (2 s) settings; the number boxes accept any positive value (any
value `>= 0` for the settle time).

## Formation

The **Formation** row is a dropdown of the profiles named in the `Formation
profiles` setting (mirror the commander's `formation_profiles` parameter) and a
**Send** button, which publishes the selected name on `/svg/formation_command`
(`std_msgs/String`). The row only appears while that topic has a subscriber.
The commander's reserved `next` verb and ad-hoc profile names are not exposed
here — send them from a shell if needed:
`ros2 topic pub --once /svg/formation_command std_msgs/msg/String "{data: next}"`.

## Agent State

One row per agent, positions in **world ENU metres** to 2 decimals:

| Column | Source |
| --- | --- |
| **State** | `IDLE` / `ARMING` / `ASCEND` / `ACTIVE` / `LANDING` from the commander. Hover shows the role, CBF exemption and the hold target |
| **x y z** | The commander's own position for the drone (odometry + `drone_position_offsets`, i.e. exactly what the CBF filters on and the frame goals are sent in), tagged `cmdr`. With no fresh snapshot it falls back to this panel's odometry subscription + the last offsets adopted from the commander (or the *Position offsets* setting before any snapshot), tagged `odom` |
| **Speed** | Ground-truth speed from the same source |
| **Cmd stream** | Rate of velocity commands on the drone's sim or real command topic — green while the commander is publishing to it |
| **CBF** | `correcting` (amber) while the CBF is altering this drone's command, `exempt`, or `clear` |
| **Interface** | Result of the last `robot_command` the commander sent this drone's interface (`request offboard`, `arm`, `disarm`): ✓ accepted, … pending, ✗ rejected / errored / skipped because the service was not ready |
| **Odom** | Whether the commander is receiving fresh odometry for this drone (stale → it commands zero velocity) |

## Teleop · Sticks

The pad as `safe_teleop` sees it, the Foxglove form of `ros2 run
svg_ground_control teleop_monitor`. It lives in the `teleop` instance (right of
Battery & Power in the shipped layout; also in a `full` instance, never in
`main`) and exists **only while `safe_teleop` is publishing** on
`/svg/{name}/teleop_command` (it publishes at 20 Hz whenever it runs, zeros
included; 2 s of silence hides it again and the instance reads "Teleop off"),
so a run with no hand-flown drone never shows it. `Sections = Show all` forces
it on.

| Element | What it shows |
| --- | --- |
| **Joy chip** | `/joy 20 Hz` when `sensor_msgs/Joy` is arriving, red `NO /joy` / `STALE` otherwise (joy_node down or pad unplugged — `safe_teleop` publishes zero meanwhile) |
| **safe_teleop chip** | Which drone(s) the stick velocity is streaming for, and at what rate |
| **Sticks chip** | Whether the sticks reach the drone: `STICKS LIVE → drone_3` (green) when the commander lists it in `teleop_drones`, it is `ACTIVE` and Start has been called; amber `STICKS PARKED` with the reason (`press Start`, `not in teleop_drones`, `is ASCEND`); grey `NO COMMANDER` |
| **Sticks table** | Four fixed-width rows — `fwd / back`, `left / right` (right stick), `up / down`, `yaw` (left stick): **Raw** is the `/joy` axis value as the driver reports it (−1 … +1); **Mapped** is what `safe_teleop` makes of it — after its deadzone (a raw value inside `deadzone` reads 0, and the rest is rescaled so full deflection is still 1.0) and its sign flip — i.e. the fraction of full stick the drone will fly, times `max_speed_mps`; then a centred bar. Nothing that changes length is in the row: hover it for the `/joy` axis index and why it reads 0 (`inside the deadzone`, `LOCKED`, `axis missing` — the pad has fewer axes than the map, wrong `teleop_controller`); the Mapped cell is greyed inside the deadzone, amber when locked, red when the axis is missing |
| **Lock line** | The lock button's state and whether the left stick is locked (mirrors `safe_teleop`'s edge-triggered latch from the presses seen since the panel opened) and which controller profile the map came from |
| **Published** | `vx vy vz yaw` from the last `teleop_command`, each as a bar against `max_speed_mps` / `max_climb_speed_mps` / `yaw_rate_rad_s` (hover the title for the topic and the scales) |

The axis map, deadzone and speed scaling are read from `safe_teleop`'s own
parameters (`<teleop ns>/get_parameters`: `forward_axis`, `left_axis`,
`climb_axis`, `yaw_axis`, `lock_button`, the four signs, `deadzone`,
`max_speed_mps`, `max_climb_speed_mps`, `yaw_rate_rad_s`, `teleop_controller`),
so a DragonRise pad is shown on its own layout. Until that read answers the
`xbox_usb` defaults are assumed and the lock line says so. The raw joy topic is
the `Joystick (raw)` setting (`/joy`).

## Mode and wiring

`Modes` in the settings takes a comma-separated `sim|real` list in `drone_names`
order, mirroring `swarm_commander`'s `drone_modes`. Leave it blank and each agent
is **detected from the topics on the wire**: anything publishing under
`/{name}/fmu/` is real, anything under `/{name}/interface/` is sim. The Wiring
card says which of the two happened.

| Purpose | `sim` | `real` |
| --- | --- | --- |
| State | `/{name}/odometry_conversion/odometry` | same |
| Velocity command | `/{name}/interface/velocity_command` | `/{name}/fmu/velocity_command` |
| Robot command | `/{name}/interface/robot_command` | `/{name}/fmu/robot_command` |
| Battery | `/{name}/interface/mavros/battery` (`sensor_msgs/BatteryState`) | `/{name}/fmu/out/battery_status` (`px4_msgs/BatteryStatus`) |
| Mocap | — | `/{name}/pose` (`geometry_msgs/PoseStamped`) |
| EKF | — | `/{name}/fmu/out/estimator_status_flags`, `/{name}/fmu/out/vehicle_local_position` |
| Ping | — | `/{name}/fmu/out/timesync_status` (`px4_msgs/TimesyncStatus`) |

Both battery shapes are subscribed regardless of the resolved mode, so a wrong
guess never blanks the power picture.

**Position offsets** is a *fallback* only: flat `x,y,z` per agent, added to
odometry while no commander snapshot has arrived. As soon as `swarm_commander`
publishes its status, each agent adopts the commander's own
`drone_position_offsets` from it, so the Agent State positions, the distance-to-
pad calculation and the goals built with **Use Current** are all in the frame
the commander plans in. (Before this, a panel offset that differed from the
config — e.g. the sim spawn offsets against `goal_tracking.yaml`'s zeros — put
"Use Current" goals 2 m off in x.) The Goal card's note says which frame it is
currently using.

## Link Safety

| Column | Where the number comes from |
| --- | --- |
| **Path** | Which transport is live: Wi-Fi / LAN, or the 4G/5G VPN |
| **Rate** | Median inter-arrival of the telemetry on that path |
| **Ping** | Best available source, labelled in the cell: an explicit link report, then PX4's uXRCE-DDS `timesync_status.round_trip_time` (a genuinely measured RTT), then the Tailscale reporter when the VPN is the live path, then `2 × mean(rx − header.stamp)` |
| **Drop** | Best source first, tagged in the cell. `dds`: **measured** — the commander's DDS reader for this drone's odometry counts samples lost by RTPS sequence number (`message_lost` event; with a RELIABLE pairing a sample counts once the writer's history has aged it out, i.e. once it is truly unrecoverable). The status snapshot carries cumulative `odom_rx_total` / `odom_lost_total`; the panel differences them over the 10 s window: `lost / (lost + received)`. `report`: an explicit `comms/link_status` value. `est`: the arrival-timing fallback before the commander is up — nominal period `P` is the median inter-arrival gap and a gap counts as `floor(gap/P)−1` missed. `floor`, not `round`: publisher jitter (PX4 SITL lockstep, MAVLink pacing) reaches 1.5–1.9·P and must not be scored as loss. It remains an estimate: arrival timing cannot tell a late sample from a lost one, and anything that batches between publisher and Studio (the Foxglove bridge does) inflates it. It is therefore shown muted, never grades the link health, and is not used at all once the commander's counters are flowing (the cell reads `--` until the window fills). Gaps past the loss timeout are outages, not loss |
| **Max gap** | The longest single gap in the window — the blackout measure. A loss percentage cannot distinguish an evenly-spread 1 % (harmless at 25 Hz) from one 400 ms hole; only this can. Graded against half of `swarm_commander`'s `state_timeout_s`, the point at which the commander holds zero velocity |
| **Mocap age** | `rx − header.stamp` of `/{name}/pose`: the delay of the state estimate PX4 is fusing, annotated with the mocap rate. Goes to `LOST` with a silence duration once nothing arrives inside the mocap timeout |
| **EKF** | `EV FUSED` when EKF2 is fusing external vision (`cs_ev_pos/_yaw/_hgt`), `NO VISION` when a real agent is not, `DEAD RECK` on `cs_inertial_dead_reckoning` / `cs_fake_pos`, `NO POSITION` when `xy_valid`/`z_valid` clear. Hover for the flag detail |
| **Clock drift** | Least-squares slope of the per-second minima of `rx − header.stamp` over 120 s, or PX4's own `estimated_offset`. A ramping floor means the clocks are separating — read the derived Ping against it |

The loss timeout is `state_timeout_s` (0.5 s), so `LOST` appears when the
commander stops commanding the drone rather than half a second later.

**Link health**: `LOST` when neither path has been heard from inside the loss
timeout; `DEGRADED` when the measured/reported drop rate or the ping is out of spec (an estimated drop rate never degrades the link) (ping is graded against
the LAN target on the LAN and the VPN target on the VPN); `ON VPN` when the LAN is
silent but the cellular path is live and in spec; otherwise `HEALTHY`. Every
change is timestamped in the transition log.

An optional `/{name}/comms/link_status` report (`std_msgs/String`, JSON) overrides
any of `drop_rate`, `rtt_ms`, `clock_offset_ms`, `clock_drift_ms`, `active_tier`.
A report older than 10 s stops overriding, so a dead reporter cannot pin the panel
to its last-known-good numbers.

## Cellular · Tailscale VPN

Shown when a cellular reporter or a VPN-path topic exists. Per agent: network
tier, VPN ping, Tailscale path, signal, interface and report age.

Have each agent (or a GCS-side prober) publish JSON on `/{name}/comms/cellular`
(`std_msgs/String`) with any of:

```json
{
  "tier": "5G",
  "rtt_ms": 78.4,
  "state": "direct",
  "rsrp_dbm": -96,
  "interface": "wwan0"
}
```

`state: "relay"` means Tailscale could not hole-punch and is bouncing through a
DERP server — the extra hop is the latency you are looking at, and the column
flags it amber. With no reporter running, VPN ping falls back to the arrival
statistics of whatever DDS traffic is on `/{name}/cellular/odometry`.

## Battery & Power

- **SoC / voltage / draw** come straight from the battery message; the SoC bar
  carries ticks at the two RTB thresholds.
- **Sag** is measured against the highest terminal voltage seen this session
  (the least-loaded sample available, so the best open-circuit proxy). Both the
  instantaneous and the session-peak sag are shown, per cell when the pack
  reports a cell count.
- **Mission time** prefers the autopilot's own `time_remaining_s`, falling back
  to SoC divided by the measured burn rate (the SoC slope over 60 s).
- **Return budget** is the distance-to-pad energy: cruise home at the configured
  speed, descend at the land speed, priced at the measured burn rate, plus a
  reserve. `RTB NOW` fires when SoC falls to that budget — ahead of the fixed
  percentage gates (>30% nominal, 20–30% gated, <20% failsafe), which still apply
  independently.

## Section visibility

`Sections` is `Auto` by default: a section is hidden when nothing publishes what
it needs. Set it to `Show all` to force everything on. Before any topic list has
arrived — a fresh connection, or a data source that does not advertise topics —
everything is shown, because an empty panel is a worse failure than a full one.

The banner's **Tasks** chip lists what the panel inferred, so it is always clear
why a section is or is not there:

| Task | Detected from |
| --- | --- |
| goal-tracking | `/svg/{name}/goal_command` |
| formation | `/svg/formation_command` |
| teleop | `/svg/{name}/teleop_command` (the Teleop · Sticks card additionally needs it to be *streaming*) |
| mocap/hardware | `/{name}/pose`, `/{name}/fmu/out/estimator_status_flags` |
| cellular | `/{name}/comms/cellular`, `/{name}/cellular/odometry` |

The Swarm Command, runtime gains and Agent State sections are controls and always
shown; their contents say `NO COMMANDER` / `--` until the commander is up.

## Development

The panel is plain JavaScript in `dist/extension.js` (no build step). A smoke
test drives it under a minimal DOM stub and a fake Foxglove panel context —
feeding a commander snapshot, odometry and velocity commands, clicking Start,
Hold and Apply (including the two-node teleop cap) — and checks the rendered
text:

```
docker run --rm -v "$PWD/robot/ros_ws/src/svg_ground_control/foxglove/svg-basestation:/p" -w /p node:20-alpine \
  sh -c "node --check dist/extension.js && node test/smoke.js"
```
