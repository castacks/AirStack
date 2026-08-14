# PX4 External-Vision (OptiTrack) Setup

Runbook for flying a PX4 vehicle (Cube Orange) on **OptiTrack mocap as the sole
position source** — no GNSS, no magnetometer — with an onboard companion
computer (Jetson) running the AirStack robot stack.

It covers three things that must all be right:

1. **EKF2 parameters** — tell PX4 to fuse external vision instead of GPS/baro/mag.
2. **Companion MAVLink link** — how the Jetson talks to the Cube (see the PX4 docs).
3. **Vision pose pipeline** — how a mocap pose becomes a `VISION_POSITION_ESTIMATE`,
   and how PX4 gets a global position without GNSS.

> Scope: PX4 ≥ 1.14 (the `EKF2_EV_CTRL` / `EKF2_GPS_CTRL` era). For older
> firmware use `EKF2_AID_MASK: 24` and `EKF2_HGT_MODE: 3` instead of the bitmask
> params below.

---

## 1. EKF2 parameters (external vision)

These are enforced automatically at startup by the `px4_param_setter` node (see
below), sourced from
[`robot/ros_ws/src/perception/natnet_ros2/config/px4_params.yaml`](../../robot/ros_ws/src/perception/natnet_ros2/config/px4_params.yaml).
You can also set them by hand in QGroundControl — PX4 persists parameters, so
either way it's a one-time thing per airframe.

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_EV_CTRL` | `11` | Fuse vision **horizontal pos (1) + vertical pos (2) + yaw (8)**. Add bit **4** (velocity) only if a vision *speed* source is also streamed. |
| `EKF2_HGT_REF` | `3` | Vision is the primary height reference (not baro / GPS). |
| `EKF2_GPS_CTRL` | `0` | No GPS fusion. |
| `EKF2_MAG_TYPE` | `5` | Magnetometer disabled — yaw comes from vision. |
| `EKF2_BARO_CTRL` | `0` | No baro fusion; height is pure vision. Set to `1` to keep baro as a backup height source. |
| `EKF2_EV_DELAY` | `7.0` | Measured OptiTrack→EKF2 latency (ms): ~0.7 ms LAN + ~5 ms Cube hop. `natnet_ros2_node` logs the measured figure. **Do not raise this to chase apparent lag — see below.** |
| `EKF2_EV_NOISE_MD` | `1` | Use the `EKF2_EV*_NOISE` floors below instead of the message covariance (which is `1e-6` — too optimistic to fuse safely). |
| `EKF2_EVP_NOISE` | `0.05` | Vision **position** noise floor (m). Not marker precision — it also sets the innovation gate, `EKF2_EVP_GATE` (default 5) sigma wide, so this is a 25 cm gate. |
| `EKF2_EVA_NOISE` | `0.05` | Vision **angle** noise floor (rad). |
| `COM_ARM_WO_GPS` | `1` | Allow arming without GPS. |

**Type matters.** Integers are written bare (`11`); floats need a decimal point
(`7.0`) so the MAVLink param type matches the FCU's declaration. Getting this
wrong makes the set silently reject.

### Two tuning results worth not rediscovering

Both came out of back-to-back flight bags with everything else held constant.

**Raising `EKF2_EV_DELAY` makes tracking worse, not better.** 50.0 was trialled against
7.0: median |odom − mocap| while moving went 0.037 → 0.060 m, and X-axis RMS 0.023 →
0.049 m. The tell is the *negative* best-fit time shift (−20 ms → −60 ms, pinned at the
sweep edge): over-declaring the delay makes EKF2 attribute the measurement to a state
that is too old, so the estimate runs **ahead** of truth during motion rather than
behind. Raising this value can never compensate for apparent lag in RViz — it does the
opposite.

**Large drift-and-snap excursions are a Motive problem, not a gate problem.** They were
first blamed on `EKF2_EVP_NOISE` being too tight. That was wrong. The cause was a 90°
body-yaw offset in the Motive rigid-body definition: EKF2 fuses vision yaw and snaps its
heading to it, so its nav frame was 90° off and IMU-predicted motion fought the (correct)
vision position. Fixing the rigid body in Motive cut moving error 0.25 → 0.04 m and the
odom/mocap path-length ratio 2.33× → 1.12×.

If you see drift-and-snap, **check the Motive rigid-body definition first**. Do not add
yaw compensation in code — `natnet_ros2` and `vision_pose_converter` are deliberate
identity pass-throughs, and a code-side correction would double-compensate once Motive is
fixed.

The wider `EKF2_EVP_NOISE` (0.05) is still the right value on its own merits: the previous
0.01 gave only a 5 cm gate, tight enough to reject legitimate updates and refuse to arm.

**Reboot after any change.** Fusion-source (`EKF2_*`) params are safest applied
from a clean estimator start — reboot the flight controller before flying. The
param setter prints a warning whenever it actually changes something.

---

## 2. The param checker (`px4_param_setter`)

Set the table above **once in QGroundControl**. To catch a mis-configured FCU
before flight, the stack runs a one-shot node at startup that **checks** the live
params against the desired set. **By default it only checks and flags — it does not
write to the FCU.**

- **Node:** [`px4_param_setter_node.py`](../../robot/ros_ws/src/perception/natnet_ros2/src/px4_param_setter_node.py)
- **Config:** [`config/px4_params.yaml`](../../robot/ros_ws/src/perception/natnet_ros2/config/px4_params.yaml)
  (everything under `params.` is a desired FCU parameter)
- **Launch:** [`launch/px4_param_setter.launch.xml`](../../robot/ros_ws/src/perception/natnet_ros2/launch/px4_param_setter.launch.xml),
  included from `natnet_ros2.launch.py` when the robot's `vision_pose` block is enabled.

Two safety flags in `px4_params.yaml`:

| Flag | Default | Behaviour |
|------|---------|-----------|
| `auto_set` | `false` | `false`: read + compare only, never write. `true`: also push mismatched params via `param/set` and verify (the legacy enforce path). |
| `on_mismatch` | `warn` | With `auto_set: false`, on a wrong param — `warn`: log the diffs, keep the stack up. `halt`: log fatal + exit non-zero so a `required` launch node tears the stack down before flight. |

Per parameter it waits for an FCU connection + `settle_sec` (default 10 s), reads
the current value, and compares (float32 tolerance). A clean run logs
`10 already correct, 0 mismatched`. A mismatch under the default (`auto_set: false`,
`on_mismatch: warn`) logs, e.g., `EKF2_HGT_REF: FCU has 1, expected 3 (not set —
auto_set=false). Fix in QGroundControl.`

Disable it entirely with `enabled: false`.

> **The checker does NOT configure the companion link** (`MAV_*` / `SER_*`
> params in section 3) — those are set once in QGC.

---

## 3. Companion MAVLink link (Jetson ↔ Cube)

`mavros` reaches the FCU over the serial link named by `FCU_URL` in the deployment env.
Configuring that link is standard PX4 setup, not AirStack-specific — see the PX4 docs:

- [Companion computer setup](https://docs.px4.io/main/en/companion_computer/)
- [MAVLink peripherals (`MAV_n_CONFIG`, `MAV_n_MODE`)](https://docs.px4.io/main/en/peripherals/mavlink_peripherals.html)
- [Serial port configuration](https://docs.px4.io/main/en/peripherals/serial_configuration.html)

Use the **TELEM2 UART** for the companion link rather than USB. On Cube Orange the USB
CDC-ACM path intermittently stalls outbound transfers for 10–30 s at a time — visible as
`DROPPED Message-Id 102 … TX queue overflow` — which starves EKF2 of vision updates and
makes it dead-reckon between bursts. It is not a bandwidth problem and rate-limiting the
vision stream does not help.

> In compose list-syntax `environment:`, values are literal — write `FCU_URL=/dev/ttyTHS1:115200`
> bare. Quoting it passes the quotes through and breaks MAVROS URL parsing.


## 4. Vision pose pipeline (mocap → PX4)

```
Motive (OptiTrack, 100 Hz)
  → natnet_ros2_node        publishes the rigid body as a ROS pose (ENU)
  → vision_pose_converter   rate-limit + quaternion canonicalize (passthrough)
  → mavros vision_pose      converts ENU→NED, sends VISION_POSITION_ESTIMATE (msg 102)
  → PX4 EKF2                fuses per the params in section 1
```

**Frame convention — the thing to get right.** MAVROS's `vision_pose` plugin
expects **ROS ENU** and converts to PX4 NED internally. The
[`vision_pose_converter_node.py`](../../robot/ros_ws/src/perception/natnet_ros2/src/vision_pose_converter_node.py)
does **no coordinate transform** — it only rewrites `frame_id`, optionally
canonicalizes the quaternion sign (`qw ≥ 0`), and rate-limits. **So
`natnet_ros2_node` must already publish ENU.** If position/yaw come out rotated
or axis-swapped, fix it there, not in the converter.

**Rate limiting.** `max_rate_hz` (default 50 in
[`vision_pose_converter.yaml`](../../robot/ros_ws/src/perception/natnet_ros2/config/vision_pose_converter.yaml))
caps the stream to MAVROS. EKF2 only needs 30–50 Hz. Note this is about not
saturating a healthy serial link — it does **not** fix the USB CDC stall in
section 3.

### Injecting a global position — `mavros_gp_origin`

Vision gives PX4 a valid *local* position, but with GNSS disabled it has no *global*
one, and modes that require a global position (e.g. `AUTO.LOITER`) refuse to arm.

[`mavros_gp_origin_node.py`](../../robot/ros_ws/src/perception/natnet_ros2/src/mavros_gp_origin_node.py)
publishes a **synthetic GPS origin** once at startup, which lets PX4 derive a global
position from the fused vision estimate. It waits for MAVROS to connect, listens for an
existing origin, and only publishes if none is present — so a GNSS-equipped vehicle is
left untouched. Location and behaviour come from
[`mavros_gp_origin.yaml`](../../robot/ros_ws/src/perception/natnet_ros2/config/mavros_gp_origin.yaml);
the defaults match the AirStack shared world datum so sim, the GCS, and the robot agree
on where world origin sits on Earth.

!!! note "Real deployments: the origin altitude needs a geoid correction"

    `geographic_msgs/GeoPoint.altitude` is a height above the **WGS-84 ellipsoid**, and
    MAVROS converts it to AMSL with the **egm96-5 geoid** before handing it to PX4. Send
    the shared datum's literal `90.0` and PX4 anchors its vertical frame at
    `AMSL = 90 − N ≈ 36 m`, while OptiTrack says the floor is `z = 0` — the drone reads
    ~36 m of altitude sitting on the floor. The gap is exactly the geoid undulation `N`.

    With `use_geoid_altitude: true` the node publishes `N + desired_floor_amsl` instead,
    computing `N` at runtime via `GeoidEval` with the same egm96-5 model MAVROS uses, so
    the conversion cancels out.

    `desired_floor_amsl` chooses what AMSL the mocap floor reports; `local_position.z`
    equals the OptiTrack height either way. We use **36.0**, the shared datum in AMSL, so
    the robot's global altitude agrees with sim and the GCS.

    **Not needed in sim.** The geoid path is skipped when `use_sim_time: true`: sim's
    synthetic GPS is self-consistent with the spawn and uses the literal datum altitude
    on both ends, so there is no ellipsoidal-vs-AMSL mismatch to correct.


## 5. Verify it's actually fusing

**Live, in the QGC MAVLink _Console_** (not the Inspector — it can't see
companion→FCU messages):

```
listener vehicle_visual_odometry     # should be steady ~50 Hz, not gappy
listener estimator_status
```

On the ROS side: `/{ROBOT_NAME}/interface/mavros/local_position/pose` should
publish and track the mocap. Hand-lift test: raise the vehicle, Z should go up
(Motive Z-up correct); translate it and check the sign/axis match.

**Definitive, from the SD-card ulog** ([Flight Review](https://logs.px4.io) or
PlotJuggler):

- `estimator_innovations` → **`ev_hpos` / `ev_vpos` / `ev_yaw`** and their
  **test ratios**. Ratio > 1 ⇒ EKF2 is *rejecting* the measurement
  (frame / timing / covariance). Near-zero with occasional gaps ⇒ fusing fine
  but starved by dropped messages (section 3).
- `estimator_status_flags` → **`cs_ev_pos` / `cs_ev_yaw`** — confirms EV fusion
  is actually active. If unset, EKF2 isn't fusing vision regardless of params.
- `vehicle_visual_odometry` rate in the log quantifies how many `102`s actually
  arrived.

---

## Troubleshooting quick reference

| Symptom | Likely cause | Where to look |
|---|---|---|
| `DROPPED Message-Id 102 … TX queue overflow` | Cube USB CDC OUT stall | Section 3 → move to TELEM2 |
| mavros local pos drifts away from mocap over time | Dropped `102`s starving EKF2 | Fix link first, then recheck |
| Constant rotation between mocap and EKF2 pose | Yaw/frame misalignment | `natnet_ros2_node` frame (must be ENU); `EKF2_EV_CTRL` yaw bit |
| Axes swapped / uncorrelated | Wrong frame convention | `natnet_ros2_node`, not the converter |
| Param set "rejected or readback mismatch" | Wrong literal type (int vs float) | Section 1 — floats need a decimal point |
| Won't arm | GPS still required | `COM_ARM_WO_GPS: 1`, reboot |
| EV innovation test ratio > 1 | EKF2 rejecting vision | Retune `EKF2_EV_DELAY`, `EKF2_EVP_NOISE` / `EKF2_EVA_NOISE` |
