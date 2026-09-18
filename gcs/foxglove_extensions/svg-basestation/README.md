# SVG Basestation

Ground-station panel for the SVG counter-UAS demonstration — the SVG analogue of
the DTC *Robot Control Panel* that anchors the `foxglove_ws` basestation layout.
One panel owns agent selection, the swarm-wide safety command, and the operator's
health picture.

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

`install.py` picks it up with the other extensions — it copies any directory
here that has a `package.json` into `~/.foxglove-studio/extensions`:

```
python3 gcs/foxglove_extensions/install.py     # installs airlab-cmu.svg-basestation-1.0.0
```

`svg_basestation.json` (one directory up) is a ready-made layout: the panel
alongside a 3D view of `/svg/viz/markers`. There are no separate battery plots —
SoC, pack voltage, sag and the RTB budget all live in the panel's own Battery &
Power section, and the plots duplicated it against a topic (`…/fmu/out/battery_status`)
that only exists on real hardware, so they read blank in simulation. Load it
with **Layout → Import from file**.

## Safety stop

The red bar under the banner is the one control that must work without reading
anything: it lands every commanded drone (`~/land`, which descends and disarms on
touchdown). It takes **two clicks** — the first arms it for 4 s, the second fires
— so it is fast under stress but an accidental brush cannot land the swarm. Next
to it, **Hold All** (`~/hold`) is the softer stop: every drone freezes at its
current position and the scenario stops.

Takeoff / Start / Reset Fence stay in the ordinary command strip below.

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

**Position offsets** takes the same flat `x,y,z` per agent as
`drone_position_offsets`, so real and simulated agents share one world frame for
the distance-to-pad calculation.

## Link Safety

| Column | Where the number comes from |
| --- | --- |
| **Path** | Which transport is live: Wi-Fi / LAN, or the 4G/5G VPN |
| **Rate** | Median inter-arrival of the telemetry on that path |
| **Ping** | Best available source, labelled in the cell: an explicit link report, then PX4's uXRCE-DDS `timesync_status.round_trip_time` (a genuinely measured RTT), then the Tailscale reporter when the VPN is the live path, then `2 × mean(rx − header.stamp)` |
| **Drop** | Nominal publish period `P` is the median inter-arrival gap; a gap counts as `floor(gap/P)−1` missed, over a 30 s window. `floor`, not `round`: a real loss lands at ~2·P, while publisher jitter reaches 1.5–1.9·P and must not be scored as loss. Gaps past the loss timeout are outages, not loss |
| **Max gap** | The longest single gap in the window — the blackout measure. A loss percentage cannot distinguish an evenly-spread 1 % (harmless at 25 Hz) from one 400 ms hole; only this can. Graded against half of `swarm_commander`'s `state_timeout_s`, the point at which the commander holds zero velocity |
| **Mocap age** | `rx − header.stamp` of `/{name}/pose`: the delay of the state estimate PX4 is fusing, annotated with the mocap rate. Goes to `LOST` with a silence duration once nothing arrives inside the mocap timeout |
| **EKF** | `EV FUSED` when EKF2 is fusing external vision (`cs_ev_pos/_yaw/_hgt`), `NO VISION` when a real agent is not, `DEAD RECK` on `cs_inertial_dead_reckoning` / `cs_fake_pos`, `NO POSITION` when `xy_valid`/`z_valid` clear. Hover for the flag detail |
| **Clock drift** | Least-squares slope of the per-second minima of `rx − header.stamp` over 120 s, or PX4's own `estimated_offset`. A ramping floor means the clocks are separating — read the derived Ping against it |

The loss timeout is `state_timeout_s` (0.5 s), so `LOST` appears when the
commander stops commanding the drone rather than half a second later.

**Link health**: `LOST` when neither path has been heard from inside the loss
timeout; `DEGRADED` when drop rate or ping is out of spec (ping is graded against
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
| teleop | `/svg/{name}/teleop_command` |
| mocap/hardware | `/{name}/pose`, `/{name}/fmu/out/estimator_status_flags` |
| cellular | `/{name}/comms/cellular`, `/{name}/cellular/odometry` |
