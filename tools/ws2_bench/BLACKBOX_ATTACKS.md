# Black-box Adversarial Attacks — Design Doc (for review)

Status: **draft for review, no code written yet.** Scope: the black-box (B-series)
attacks from `airstack_attack_sim_implementation.md`, realized by reusing the WS2
bench rather than building the standalone `airstack_adversary` ROS package. Gray-box
(R-series) and white-box (W-series) are deliberately deferred.

This is a sim-only defensive red-teaming harness: every "physical" attack is a
software perturbation of simulated sensor data or the USD scene, used to test
detectors and hardening. No RF, no real airframes, no hardware code.

## Implementation status

- **B1 GPS, B4 IMU, B5 magnetometer (faithful): implemented.** B9 (re-ID) dropped — the
  algorithm is obstacle avoidance now. B2 (LiDAR) deferred (no consumer: Office spawns no
  LiDAR, targets are monocular). **B6 baro deferred** — it is the least remotely realizable
  (no standoff pressure-spoof; only tamper/environmental), revisit if an altitude-attack
  scenario is needed.
- **Physical realism drives the models and bounds**, and each attack records the real-world
  mechanism it emulates. B4 IMU is acoustic-resonance-centric (Son et al. USENIX 2015;
  Trippel et al. WALNUT 2017): a resonant oscillation is primary, a bounded rectified bias
  secondary, magnitudes sub-saturation (gyro ≤5 rad/s, accel ≤40 m/s²). B5 magnetometer is an
  additive external-field bias (magnet/coil), bounded near Earth-field scale (≤5 gauss). B1 GPS
  is *high* plausibility (remote standoff RF, global in the scene). `attack_metadata()` emits
  `physical_mechanism` + `standoff_plausibility` into the result JSON so no run overclaims.
- **Tampering is out of scope** — no pre-flight/contact attacks. Any attack that needs close
  proximity is a *localized in-flight disruptor at a fixed world position*: B4/B5 carry a
  required `location` (ENU world) + `range_m`, and the installer scales the effect by a
  distance gate (full at the device, linear taper to zero at `range_m`, zero beyond; a bounded
  proxy for ~1/r²–1/r³). So a proximity attack bites only while the drone passes the device,
  never everywhere. An unknown drone position fails safe to no effect, and `summary().active`
  / `perturbed_samples` count only in-range samples, so results show whether the drone actually
  entered the zone. Only remote RF (B1 GPS) is global. The drone position comes from the
  Pegasus `State` the wrapped `update()` already receives (ENU world, same frame as ground
  truth) — still no AirStack/Pegasus source edited.
- Injection point confirmed: a real spoofer makes the sensor *report* a false value,
  which PX4's estimator then fuses. The faithful tap is the output of a Pegasus sensor
  model's `update()`, between the sensor and the MAVLink backend
  (`vehicles/vehicle.py:update_sensors`), below the estimator. We wrap the sensor
  instance's bound `update` from `launch_office.py` at spawn — **no AirStack or Pegasus
  source edited**; the attack lives entirely in the sim environment.
- New/changed (uncommitted): `sensor_attacks.py` (validator + pure perturbation +
  installer), `test_sensor_attacks.py` (12 tests, pass), `scenarios/gps_spoof_kim.yaml`,
  `attack:` wiring in `episode.py` + `launch_office.py`. The spoof is reached two ways:
  (1) a `gps_spoof` feedback profile that ramps the drift with the search level
  (`feedback.py`), and (2) **`gps_spoof_m`, an optional scalar in the corridor action
  space** (`corridor_space.py`), so random/search/llm *choose* the east fix drift
  (0..8 m, absent/0 = off) per flight like any other attack knob. `action_to_episode`
  derives the faithful B1 `attack:` block from it on the perturbed flight; the clean
  twin strips it. It is a scalar knob only — deliberately kept out of
  `experiment_protocol.FACTORS`, since GPS has no camera-frame exposure oracle for the
  isolate/threshold probes. The earlier campaign-level `--gps-spoof-offset` /
  "GPS spoof drift" dashboard control is removed now that the policy picks the value.
  A **Manual method** is implemented: a `manual` corridor policy (`CorridorPolicy`,
  `--manual-action` / dashboard `manual_action`) flies an operator-specified action
  verbatim (explore once, then repeat), deriving the same B1 attack from `gps_spoof_m`;
  the dashboard "Action configuration" panel edits every action field when Manual is
  selected and shows the live policy-chosen action read-only otherwise. A live Attack
  chip still shows armed/active + drift. (`feedback.py`, `corridor_space.py`,
  `agent_campaign.py`, `dashboard.py`, `presentation.html`,
  `test_scheduled_corridor.py`.)
- **Effect confirmed in sim (2026-11-02).** The stack *does* fly on the PX4/GPS-fused
  estimate: with an east drift, the controller believes it is too far east and flies
  **west**, so the true trajectory goes the wrong way (clean flight reached x≈+4 m
  toward the goal; the spoofed flight was driven to x≈−8 m and never progressed). B1
  has a real consumer; the open "is there a consumer" risk is resolved for GPS.
- Effect for the record is read from the bag (`/ws2/ground_truth/pose` vs
  `/robot_1/odometry_conversion/odometry`); the first checks used `--no-bag` and read
  the ground-truth trajectory + outcome, which already showed the divergence.

---

## 1. The reuse decision

The bench already implements the brief's **core MITM pattern** — twice — so we extend
those two channels instead of adding interposer nodes and a parallel results stack.

| Brief component | Already exists as | Reuse for |
|---|---|---|
| USD scene hooks (B2/B3/B9) | `scene_command.json` → `launch_office.apply_condition` | geometry / camera / decoy edits |
| sensor perturbation (faithful) | Pegasus sensor models `update(state,dt)` → PX4 MAVLink | B1/B4/B5/B6 below the estimator |
| perturbation library | `mononav_bridge/disturbances.py` (seeded RNG, schedules, bias, delay buffer) | all perturbation math |
| config-per-experiment | `conditions.py` `validate()` + `sensor_parameters()` | new attack knobs, validated/replayable |
| triggers (time/window) | `disturbances.parse_schedule` / `schedule_active` (sim-clock, half-open windows) | attack onset/ramp timing |
| baseline-vs-attack + metrics | `episode.py` clean/perturbed pairing, result JSON, `MetricsRecorder` | effect + detection + defense fields |
| run matrix / reporting | campaign / dashboard / `parse_metrics` | aggregate across the B matrix |

**Consequence (confirmed with the user):** no new colcon package. New code lives in
`tools/ws2_bench/` and in one new node installed into the robot container at spawn
time, mirroring how `mononav_bridge` and `ros_control.py` already work. AirStack
source stays unmodified; Pegasus is wrapped, not edited (see §4).

---

## 2. Where each sensor actually enters the stack (confirmed)

```
Pegasus sensor models ──MAVLink──> PX4 SITL/EKF ──> interface/MAVROS ──> /ROBOT/odometry
  gps.py  imu.py  magnetometer.py  barometer.py                              │
                                                                             ▼
ZED stereo camera (graphical) ──ROS──> sensors/front_stereo ─> mononav_bridge ─> planner
  lidar.py (graphical, ENABLE_LIDAR)                           (vision interposer, today)
```

Two distinct tap points, which is exactly the brief's faithful-vs-shortcut split:

- **Scalar sensors (GPS/IMU/mag/baro)** reach the stack through **PX4's estimator**,
  not a ROS topic. The honest place to attack them is the Pegasus sensor model
  output, *before* the EKF — so the attack actually exercises estimator consistency
  gates. These models already carry `_gps_bias`, random-walk, and noise-density
  params (`gps.py:53-69`), so the perturbation hook is a natural extension.
- **Camera / LiDAR** reach perception through **ROS topics**. Camera attacks already
  run through the USD scene (patch quad, dome/fill light) and the vision bridge.

This is why B1/B4/B5/B6 do **not** belong on the vision bridge: that bridge sits
above the estimator and can only perturb the image, not the pose solution.

---

## 3. Attack catalog → concrete mechanism (B-series only)

Fidelity column: **F** = faithful (below the defense under test), **S** = shortcut
(topic pub, modeled one layer high — fast iteration, flagged in results).

| ID | Attack | Mechanism (reuse) | Path | Defense exercised |
|---|---|---|---|---|
| **B3** | Camera dazzle / patch | **Already built**: dome+fill intensity, `learned_patch.png` quad, `rgb_noise`, schedules | F (USD) | perception robustness / depth-net |
| **B1** | GPS spoof | bias-walk on `gps.py` output before MAVLink | F | EKF innovation gate, GPS/vision consistency |
| **B4** | IMU bias/resonance | injected bias / resonant term on `imu.py` output | F | attitude estimator, vibration monitor |
| **B5** | Magnetometer spoof | heading bias on `magnetometer.py` output | F | heading/yaw consistency |
| **B6** | Baro / altitude spoof | offset/drift on `barometer.py` output | F | altitude fusion, rangefinder cross-check |
| **B2** | LiDAR spoof | USD: absorptive/fake geometry (scene cmd); `ENABLE_LIDAR` on | F (USD) | obstacle-map integrity |
| **B2'/B1'** | fast-iteration variants | topic pub+remap of pointcloud / odom | S | — (triage only) |
| **B9** | Re-ID decoy | USD: spawn decoy prim matching target appearance | F (USD) | re-ID / tracker — **needs a target-tracking behavior; likely out of scope, see §7** |

B7 (WiFi jam/deauth) is inter-robot and was deprioritized with R-series. B8 (MAVLink
UDP inject) is a gray-box faithful path, deferred.

---

## 4. The one new piece: a sensor perturbation hook

Everything scalar-sensor reuses a single thin wrapper, kept in ws2_bench and installed
at spawn like the other bench scripts. The Pegasus model stays unedited; we wrap its
`update()`:

```python
# perturb(reading, sim_time, params, rng) -> reading   # pure, seeded, testable
# params come from the episode config, same channel as rgb_noise today
clean = sensor.update(state, dt)
return perturb(clean, world.current_time, attack_params, rng) if schedule_active(...) else clean
```

- `perturb` lives in an extended disturbance library next to `disturbances.py`, reusing
  its seeded `_rng`, `parse_schedule`, and `schedule_active` verbatim so attacks are
  replayable and windowed on the sim clock (the brief's time trigger, already built).
- Onset/ramp = the existing schedule dict `{start_s, duration_s, period_s}`.
- The wrapper is wired in `launch_office.py` where sensors are constructed; the
  Pegasus tree is touched only by composition, not edits.

This keeps the brief's `perturb(msg,t,params)->msg` contract and determinism rule
(seed recorded in the result) without an interposer node or a remap.

---

## 5. Config schema (extends `conditions.py`, stays validated/replayable)

New optional block, rejected unless well-formed, so every run is reproducible and
historical results can't be silently reinterpreted (matching the existing legacy-field
guard):

```yaml
attack:
  id: B1_gps_spoof            # one of the B-series ids
  path: faithful              # faithful | shortcut  (recorded in metrics as fidelity)
  params: {axis: [1,0,0], rate_mps: 0.3, bounded: true}   # per-attack, schema-checked
  schedule: {start_s: 20, duration_s: 0, period_s: 0}     # reuses validate_schedule
  seed: 42
```

`validate()` gains an `attack` branch that dispatches to a per-id param validator
(same pattern as the existing `corridor`/`placement`/`patch_mount` branches). No id,
no attack — clean baseline. `sensor_parameters()` / the scene command forward it to
the right tap.

---

## 6. Metrics (reuse `MetricsRecorder`, add the brief's §6 fields)

Each run already records effect-style outcomes (route completion, collision, pose).
Add per the brief:

- **attack effect** — true-vs-estimated pose divergence, goal error, crash flag,
  mission-time delta (we have ground-truth telemetry over UDP + oracle).
- **detection** — did any monitor fire, and time-to-detect (populated in Phase 5).
- **fidelity** — `faithful` | `shortcut`, straight from `attack.path`.
- **defense_exercised** — the control this attack stresses (table §3).
- **seed** — already required for replay.

Emitted into the existing per-run result JSON so the campaign/dashboard aggregates the
B matrix with no new reporting code.

---

## 7. Open questions for review

1. **Faithful sensor tap vs MAVLink boundary.** B1/B4/B5/B6 faithful path assumes we
   can wrap the Pegasus sensor `update()` before its MAVLink backend. I've confirmed
   the models and their bias/noise params exist; I have **not** yet confirmed the exact
   call site in the spawn path (`spawn_px4_multirotor_node`) where the wrapper installs
   cleanly. First implementation task, per the brief's confirm-before-coding rule.
2. **B9 re-ID decoy** needs a target-tracking / re-ID behavior to attack. I don't see
   one in the autonomy stack (perception is depth/obstacle, not person re-ID). I
   recommend **dropping B9** unless you know of a tracker to point it at.
3. **B2 LiDAR**: the Office scene spawns no LiDAR by default (`ENABLE_LIDAR=false`) and
   the tested targets (MonoNav/PULP/Kim) are monocular-depth, not LiDAR consumers. B2
   may have **no consumer to affect** in the current target set — confirm whether any
   planner ingests the pointcloud before investing here.
4. **Proposed Phase-2/3 order** (smallest honest result first):
   **B1 GPS faithful** → **B4 IMU** → **B6 baro** → **B5 mag**, then B2 only if (3)
   resolves. B3 is done and becomes the worked reference. Shortcut paths only for
   triage, always flagged `fidelity: shortcut`.
5. **Scope check**: B3 is already live — do you want it re-expressed under the new
   `attack:` schema for a uniform matrix, or left as the existing patch/light condition
   and referenced?

---

## 8. First implementation step (after this review)

Confirm §7.1 (the sensor wrapper call site), then land **B1 GPS spoof, faithful path**
end to end: `attack:` schema + validator, the `perturb` hook, episode wiring, one
`experiments/B1_gps_spoof.yaml`, a clean-vs-attack run, and the §6 metrics — as the
template every other B attack is cloned from.
