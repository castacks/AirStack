# AirStack × IsaacSim — Adversarial Attack Simulation: Implementation Brief

> **Purpose.** Implement the attacks from the drone-autonomy-stack threat model as
> reproducible, sim-only experiments in IsaacSim + AirStack, for defensive
> red-teaming (testing detectors and hardening). Each attack should be toggleable,
> parameterized, and paired with a metric that says whether it succeeded and which
> defense it exercises.
>
> **Scope / ethics.** Simulation only. No RF, no real airframes, no code targeting
> hardware. Every "physical" attack is realized as a software perturbation of
> simulated sensor data or the USD scene. This is an evaluation harness for
> consistency gates, estimator monitors, and comms auth — not an operational tool.

---

## 0. Status of this document

This is a starting brief written from a threat model + architecture discussion.
**Topic/node names below are anchored to the system diagram, not verified against
the current AirStack source.** Before implementing, confirm the items in
[§8 Confirm-before-coding](#8-confirm-before-coding). Treat anything in `code font`
that names an AirStack topic/node as a hypothesis to validate.

---

## 1. System assumptions

AirStack (CMU AirLab) autonomy stack, ROS 2, Docker-based, driven in sim by
IsaacSim. Pipeline mirrors the threat-model diagram:

```
Sensors → Perception → World Model (SLAM) → Planning (global/local) → Controls → Robot Interface → PX4/ArduPilot (SITL)
                                           ↕ Communication Manager ↔ other robots
```

Likely integration points:
- **IsaacSim → ROS**: sensor bridge publishes camera / LiDAR / IMU / GPS(odom) onto ROS 2 topics.
- **Robot Interface → flight controller**: MAVROS ↔ PX4 SITL over MAVLink (UDP).
- **Graph security**: default DDS, **no SROS2** → any node on the domain can pub/sub freely.

---

## 2. Access primitives (the levers the harness pulls)

| Primitive | Read | Edit | Used by |
|---|---|---|---|
| ROS domain membership | — | join `ROS_DOMAIN_ID` | all R-series, publish-path sensor attacks |
| ROS topic sub / pub | subscribe | publish (+ remap to override) | R1–R9, B-series (pub path) |
| IsaacSim USD stage | inspect prims | add/modify prims, lights, materials | B2, B3, B9 |
| IsaacSim sensor bridge | read config | edit plugin output | B1, B4, B5, B6 (faithful path) |
| Node params / YAML / PX4 params | read | edit | W6, tuning |
| Node source / model weights | read | edit | W1, W2, W5 |
| Launch + remap | — | edit | MITM insertion, B7 |
| MAVLink bridge (UDP) | sniff | inject | R9 (faithful path), B8 |
| Build / container image | — | edit | W7 |

**Design consequence:** most attacks are a *add-on node + remap* (no AirStack source
edit). Only W-series and the faithful sensor/scene paths need deeper access.

---

## 3. Harness architecture

Build a standalone package, **`airstack_adversary`** (own ROS 2 package / repo, kept
*outside* the AirStack source tree so the stack stays unmodified for most attacks).

### 3.1 Core pattern — the MITM interposer
A generic node that: subscribes to a *source* topic, applies a perturbation
function, republishes on the *target* topic name. Original producer is remapped to
a shadow name via launch so the interposer owns the real name.

```
<producer> --(remap)--> /topic__truth    [interposer subscribes]
                                          [interposer applies perturbation]
                        /topic           [interposer publishes → consumers]
```

### 3.2 Config-driven
One YAML per experiment selects attack id, target, params, trigger (time/geo/state),
and ramp profile. No code change to switch attacks. Example:

```yaml
attack: R2_state_spoof
target_topic: /world_model/odometry
source_topic: /world_model/odometry__truth
trigger: {type: time, t_start: 20.0}
perturbation: {type: bias_walk, rate_mps: 0.3, axis: [1,0,0], innovation_bounded: true}
```

### 3.3 Components
- `interposer_node` — generic sub→perturb→pub (sensor + ROS topics).
- `perturbations/` — library: bias_walk, replay_buffer, flood, phantom_injector, dropper, blob_fuzzer.
- `usd_hooks/` — IsaacSim scripts: spawn/modify prims, lights, materials (for B2/B3/B9).
- `sensor_bridge_patches/` — edited Isaac sensor outputs (for B1/B4/B5 faithful path).
- `triggers/` — time / geofence / vehicle-state / message-count gates.
- `metrics/` — per-attack success + which-defense recorders (see §6).
- `launch/` — per-attack launch with the right remaps.
- `experiments/*.yaml` — the configs.

---

## 4. Attack catalog → sim implementation

Tiers: **B** black-box (physical, realized in software), **R** gray-box (ROS graph),
**W** white-box (node internals). "Access" is the *minimal* lever.

### 4.1 Gray box — R-series (cheapest: domain + topic op)

| ID | Attack | Target hook | Access | Perturbation |
|---|---|---|---|---|
| R1 | Setpoint inject | `controller/tracking_point` | pub (outpace real) | emit attacker setpoints at > real rate |
| R2 | State / map spoof | `world_model/odometry`, `/map` | pub (+ remap) | bias_walk on pose; costmap edits |
| R3 | Perception inject | `perception/{topic}` | pub (+ remap) | phantom_injector / suppressor |
| R4 | Peer spoof / Sybil / replay | inter-robot topics | pub; sub to capture | spoof peer id; replay_buffer |
| R5 | Bad deserialize | `*/serialized` | pub | blob_fuzzer (malformed/oversized) |
| R6 | Task poison | task-allocator bid topic | pub | forged bids |
| R7 | Mode confusion | behavior-tree cmd/mode topic | pub | ill-timed mode transitions |
| R8 | Topic / DDS flood | any topic + discovery | pub | flood; DDS discovery spam |
| R9 | MAVLink inject | MAVROS setpoint topics (ROS) **or** MAVLink UDP (faithful) | pub **or** bridge inject | guided setpoints / mode / arm |

### 4.2 Black box — B-series (pick path by fidelity need)

| ID | Attack | Faithful path (access) | Shortcut path (access) | Perturbation |
|---|---|---|---|---|
| B1 | GPS spoof | sensor bridge GPS/odom (edit) | pub GPS/odom topic | bias_walk the fix slowly |
| B2 | LiDAR spoof | USD scene (edit) | pub pointcloud (+ remap) | fake/absorptive geometry; inject/hide returns |
| B3 | Camera dazzle / patch | USD light/decal prim (edit) | pub image (+ remap) | saturation; adversarial patch; false features |
| B4 | IMU resonance | sensor bridge IMU (edit) | pub IMU topic | inject resonant noise/bias → attitude error |
| B5 | Magnetometer spoof | sensor bridge mag (edit) | pub mag topic | heading bias |
| B6 | Optical-flow / alt | flow/rangefinder topic | pub topic | false velocity / altitude |
| B7 | WiFi jam / deauth | launch relay (edit) | — | dropper / latency on inter-robot msgs |
| B9 | Re-ID decoy | USD scene (edit) | — | spawn decoy matching target appearance |

> For B1/B4/B5 prefer the **bridge path** — the attack must live below perception to
> honestly test consistency gates. For B2/B3/B9 prefer the **USD path** — exercises
> real raycasting/rendering. Shortcut (pub) paths are for fast iteration only; note in
> results that injection is modeled one layer high.

### 4.3 White box — W-series (needs node internals)

| ID | Attack | Access | Approach |
|---|---|---|---|
| W1 | Innovation-bounded EKF drift | read estimator source/params + edit a measurement source (or edit estimator) | step bias ≤ per-cycle innovation gate; accumulate |
| W2 | Model backdoor | edit model weights / training data | planted trigger pattern → flip detection |
| W5 | Timing / logic bomb | edit node source or launch | scheduled CPU starvation / conditional fault |
| W6 | RTL / geofence hijack | edit PX4 params / failsafe config | move home point / disable geofence |
| W7 | Supply chain | edit build / container image | backdoored dependency across fleet image |

---

## 5. Implementation roadmap (phased)

**Phase 0 — scaffolding**
- Create `airstack_adversary` package, launch skeleton, YAML loader, metrics recorder.
- Build `interposer_node` + `bias_walk` + a time trigger.
- Bring up AirStack + IsaacSim with one quad; record a clean baseline run.

**Phase 1 — gray box (highest impact / lowest access)**
- R1 setpoint inject, R2 state spoof, R8 flood. These prove the harness and the
  "unsecured graph" point. R5 blob_fuzzer next (also validates the `Type: tbd` risk).
- Add replay_buffer + peer spoof → R4; R3/R6/R7 reuse the interposer.

**Phase 2 — black box, shortcut path**
- B1/B4 via topic pub (fast). Then B6, B2/B3 pub-path. Confirm estimator/perception
  reactions before investing in faithful paths.

**Phase 3 — black box, faithful path**
- B1/B4/B5 via sensor-bridge edits; B2/B3/B9 via USD hooks. These are the ones that
  actually test sensor-fusion consistency gating.

**Phase 4 — white box**
- W6 (params, easy) → W1 (needs estimator internals) → W5 → W2 → W7.

**Phase 5 — defenses + eval loop**
- Turn on candidate mitigations (auth, consistency gates, cumulative-innovation
  monitor, safe schema) and re-run every attack; record detect/miss + effect.

---

## 6. Metrics (per attack)

Record for each run:
- **Attack effect** — divergence of true vs estimated pose; distance from
  intended vs achieved goal; crash flag; mission-time delta; loop-rate / CPU.
- **Detection** — did any monitor fire (innovation gate, consistency check, auth
  reject, rate limiter)? time-to-detect.
- **Fidelity note** — faithful vs shortcut injection layer.
- **Defense exercised** — which control this attack is meant to stress (ties back to
  the threat model's mitigation column).

Emit as a rosbag + a per-run JSON summary so results aggregate across the matrix.

---

## 7. Suggested layout

```
airstack_adversary/
├── package.xml / setup.py
├── airstack_adversary/
│   ├── interposer_node.py
│   ├── perturbations/{bias_walk,replay_buffer,flood,phantom_injector,dropper,blob_fuzzer}.py
│   ├── triggers/{time,geofence,vehicle_state,msg_count}.py
│   └── metrics/{recorder.py,summarize.py}
├── usd_hooks/{lidar_spoof.py,camera_dazzle.py,reid_decoy.py}
├── sensor_bridge_patches/{gps_bias.md,imu_resonance.md,mag_bias.md}
├── launch/<attack_id>.launch.py
├── experiments/<attack_id>.yaml
└── results/<run_id>/{bag,summary.json}
```

---

## 8. Confirm-before-coding

Resolve against the actual AirStack + sim setup before Phase 1:

1. **Topic names & types** for: controller setpoint/tracking point, local-planner
   setpoints, world-model odometry & map, perception outputs, inter-robot /
   serialized topics, task-allocator bids, behavior-tree mode. (Diagram names are
   placeholders.)
2. **Remap mechanism** — does AirStack's launch allow clean remap of a producer to a
   shadow name, or is a node/namespace override needed?
3. **Sensor bridge** — which Isaac sensors publish where; where to tap for the
   faithful B1/B4/B5 path (Isaac sensor API vs a ROS republisher).
4. **Flight-controller link** — MAVROS topic names for setpoints/mode/arm, and
   whether a direct MAVLink-UDP inject point exists for the faithful R9/B8 path.
5. **Estimator** — which package/algorithm (for W1); are gains/gates in params or code?
6. **Perception models** — artifact format & load path (for W2).
7. **DDS/SROS2** — confirm the graph is unsecured by default (so R-series need only
   domain membership), and whether an SROS2 profile exists to test against.
8. **PX4 params / failsafe config** path (for W6).
9. **Run topology** — single vs multi-robot launch, `ROS_DOMAIN_ID`, container
   network (for R4/R8/B7 inter-robot work).

---

## 9. First task for Claude Code

Start at Phase 0 + the first Phase 1 attack:
1. Scaffold `airstack_adversary` (package, YAML loader, metrics recorder).
2. Implement `interposer_node` + `bias_walk` + time trigger.
3. Implement **R1 setpoint injection** and **R2 state spoof** against the real topic
   names once confirmed in §8.
4. Add a baseline-vs-attack run script emitting `summary.json` per §6.
Keep the AirStack tree unmodified; everything lives in `airstack_adversary` and
launch-time remaps.
