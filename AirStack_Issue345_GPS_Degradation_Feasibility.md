# Realistic GPS Sensor Degradation in AirStack (Issue #345) — Feasibility Analysis

**Author:** Apurva Singh, CMU/AirLab intern
**Repository:** castacks/AirStack
**Issue:** #345 (opened by @andrewjong on Apr 23, 2026)
**Date:** May 13, 2026

---

## 1. Problem statement (from Issue #345)

AirStack currently does not produce realistic state estimation for GPS-degraded environments. The simulated GPS comes from Pegasus Simulator's default sensor suite (referenced through `px4_mavlink_backend.py`) and the GPS sensor model "doesn't degrade based on environment occlusions." The issue author asks for shadow interference, multipath interference, and jamming, with the immediate deliverable being:

> "an environment-aware degraded GPS path for Isaac Sim / Pegasus, plus a clean way to test open-sky, degraded, denied, and recovery transitions against MACVIO."

The issue is needed for the DTC, DSTA, and Shimizu projects — "Anything outdoors with GPS degradation."

## 2. How GPS actually flows through AirStack today

I traced this from the Pegasus Simulator API docs and the AirStack docs site. The pipeline is:

1. **Isaac Sim (NVIDIA Omniverse)** runs the physics at 100 Hz in AirStack's configuration (lockstep with PX4's `IMU_INTEG_RATE`, set by `PX4_PHYSICS_HZ` in the top-level `.env`).
2. **Pegasus Simulator** (Python extension running inside Isaac Sim) instantiates a multirotor with a default sensor suite: barometer, IMU, magnetometer, GPS. The Pegasus node lives in the AirStack fork at `castacks/PegasusSimulator-AirStack-Integration` and is wrapped in an OmniGraph action graph so scenes can be saved as USD.
3. The **GPS sensor** is `pegasus.simulator.logic.sensors.gps.GPS`, a subclass of `Sensor`. Its `update(state, dt)` is called by the vehicle at the sensor's `update_rate` (default 1 Hz). It receives the vehicle's ground-truth `State` (lat/lon/alt origin saved at init via `initialize(...)`, plus current position/velocity from the physics step) and returns a dictionary with noisy `latitude`, `longitude`, `altitude`, velocity, plus `fix_type`, `eph`, `epv`, and `sattelites_visible` (sic — typo preserved from upstream). The default config is:

   ```
   fix_type: 3, eph: 1.0, epv: 1.0, sattelites_visible: 10,
   gps_xy_random_walk: 2.0 (m/s)/√Hz, gps_z_random_walk: 4.0 (m/s)/√Hz,
   gps_xy_noise_density: 2.0e-4 m/√Hz, gps_z_noise_density: 4.0e-4 m/√Hz,
   gps_vxy_noise_density: 0.2 (m/s)/√Hz, gps_vz_noise_density: 0.4 (m/s)/√Hz,
   gps_correlation_time: 60 s, update_rate: 1.0 Hz
   ```

   The model is a Gaussian + Ornstein–Uhlenbeck bias process — exactly the PX4 SITL-Gazebo model by Wagoner & Marques. There is no environment awareness in the `update` method; it only sees the vehicle's State.

4. **PX4MavlinkBackend** is the vehicle's Backend. The flow is `Vehicle.update_sensor("GPS", data)` → `PX4MavlinkBackend.update_gps_data(data)` → buffered until `send_gps_msgs(time_usec)` is called from `update(dt)` at the physics step → MAVLink `HIL_GPS` over TCP (default port 4560).
5. **PX4 SITL (EKF2)** consumes `HIL_GPS`, fuses with simulated IMU/mag/baro, runs the navigation EKF at 100 Hz.
6. **MAVROS** bridges PX4 → ROS 2: publishes `/mavros/global_position/global` (`sensor_msgs/NavSatFix`), `/mavros/global_position/raw/fix`, `/mavros/local_position/pose`, etc.
7. **AirStack robot stack** (`robot/ros_ws/src/autonomy/perception/state_estimation`) consumes those topics for downstream planning (DROAN local planner, exploration, etc.). MACVIO/MAC-SLAM is the VIO replacement the issue author wants to validate transitions against.

The clean architectural truth: **the GPS sensor model is the only place in the chain that has access to both the ground-truth pose of the drone and the USD scene graph (via Isaac Sim's physx ray-cast API).** Anywhere downstream — MAVLink, PX4, MAVROS — has already lost the environmental context needed to compute occlusion. That makes `GPS.update()` the right hook.

## 3. The proposed papers — what each gives you, what each does not

### Cuenca et al. (2023) — DOI 10.2514/6.2023-2648

**Method.** Models GPS positioning error as a function of (i) the number of visible satellites after applying an obstruction mask derived from a 3D building model, (ii) the geometry of the visible satellites (DOP / HDOP / VDOP), and (iii) standard GPS measurement noise. Outputs a position covariance per epoch over a city volume. Used for UAS corridor risk assessment.

**Fit to AirStack.** Strong. This is exactly the model you want for the *first tier* of degradation: cheap, continuous, environment-aware. Implementation reduces to:

1. For each visible-sky satellite (synthetic almanac, e.g. 31 active GPS PRNs), compute an azimuth/elevation pair at simulated time.
2. From the drone position, cast a ray toward each satellite. If blocked by USD geometry → satellite is non-visible.
3. Compute HDOP/VDOP from the remaining visible-satellite geometry.
4. Map HDOP → `eph`, VDOP → `epv`, count → `sattelites_visible`, and apply a downgraded `fix_type` if `n_visible < 4`.
5. Inflate the noise covariance accordingly. Optionally add a bias proportional to the asymmetry of the visible constellation (this captures the systematic east-skewed error you see when a tall building blocks the west).

**What it doesn't give you.** Multipath bias. Cuenca treats blocked satellites as removed; it does not add the characteristic NLOS pseudorange bias that fakes a position offset toward the reflector. For the issue's "shadow interference" and "multicast (multipath) interference" wording, Cuenca covers shadow but not multipath.

### Pant et al. (2022) — arXiv 2212.04018

**Method.** A Gazebo plugin that for each satellite casts a ray from drone to satellite, classifies the path as LOS / NLOS / multipath, and adds the corresponding pseudorange error before pseudo-running a least-squares position solution. Validated against the UrbanNav Hong Kong dataset.

**Fit to AirStack.** Approach is right; the **code is not portable.** AirStack uses Isaac Sim, not Gazebo. The plugin depends on Gazebo's `RayShape` / ignition::physics scene query API. The math, the satellite visibility loop, the LOS/NLOS/MP classification, and the pseudorange-bias model are all engine-agnostic and can be re-implemented in ~300–500 lines of Python against Isaac Sim's `omni.physx.scene_query` (`raycast_closest`) or the newer `RaycastAny` API.

**Cost to port.** The risk you should call out is real-time cost. Pant claims real-time at 1 Hz update for ~10 satellites in a small Hong Kong block. AirStack runs at 100 Hz physics, but GPS is 1 Hz, so the budget per GPS update is ~10 ms. Issue #342 ("Pegasus using whole CPU core") already flags simulator CPU pressure, so the Pant-tier path should be opt-in via config, not the default.

**What it adds over Cuenca.** Per-satellite LOS classification gives you a more realistic shape of the position error cloud — when one strong reflector dominates, you get a *biased*, not just inflated, position. That matters for testing how MACVIO behaves under correlated GPS bias (the failure mode that is not just a "bigger noise circle").

### Lee et al. (2020) — DOI 10.1109/ICRA40945.2020.9197029 (OpenVINS group)

**Method.** Estimator-side. Online initialization and calibration of the GPS-IMU extrinsics + time offset under intermittent GPS, with a proof of four unobservable directions in the VIO frame and a state transformation to the GPS frame for full observability.

**Fit to AirStack.** This is the consumer-side reference, not a sensor model. It is exactly the right citation for justifying the **recovery scenario** — what should MACVIO do when GPS comes back after a denial? Lee tells you (a) how the initialization should be designed, (b) what the unobservability gotchas are, and (c) what evaluation metrics matter (RMSE before and after re-acquisition; calibration convergence). If MACVIO does not already implement Lee-style intermittent fusion, that is a follow-on issue.

### Simulating GNSS multipath in urban environments using 3D ray tracing for automotive applications (2026, ScienceDirect, S1574119226000799)

**Method.** Builds on CARLA + octree ray-tracing to model LOS/NLOS pseudorange errors for ADAS. Releases a public dataset.

**Fit to AirStack.** Useful as the "still computationally tractable" argument. CARLA-class urban environments are an order of magnitude denser than what AirStack typically loads, and they get real-time multipath with an octree-accelerated ray-tracer. That bolsters the Pant-tier feasibility argument. The paper is also a source of validation data (city + GNSS + IMU + ground truth) you could cross-check against, even though it's automotive not aerial.

## 4. Adjacent literature worth citing (or pulling in if a reviewer pushes)

These came up while I was checking the references. None of them replace what you already have; they fill gaps.

- **Groves (2013), *Principles of GNSS, INS, and Multisensor Integrated Navigation*** — textbook standard for DOP / pseudorange error models. The Cuenca paper is essentially this model parameterized for an urban UAS.
- **Hsu et al., 3D Mapping-Aided (3DMA) GNSS** (e.g. Ng & Hsu 2021, "3D Mapping Database Aided GNSS RTK") — cheaper than full ray-tracing; uses LOD2 building footprints + heights to compute satellite visibility via skymask matching. A good middle tier between Cuenca and Pant.
- **Wen & Hsu (2022), "3D LiDAR Aided GNSS NLOS Mitigation"** (arXiv 2212.05477) — useful if you want to also publish the GPS state as fused with a perception map at runtime.
- **Kerns et al. (2014), "Unmanned Aircraft Capture and Control via GPS Spoofing"; Humphreys (2008), "Assessing the spoofing threat"** — the canonical adversarial threat models. If "jamming" in the issue is to be modeled honestly, you'll want at minimum: barrage jamming (noise floor inflation), narrowband CW jamming (specific PRN drop), and meaconing/spoofing (coherent position drift). Cuenca + Pant cover none of these.
- **EUSPA Open Service Performance Specification** — defines nominal accuracy benchmarks (e.g. 95% horizontal accuracy ≤ 3 m). Useful for setting baseline pass/fail thresholds in your tests.
- **Spirent GSS9000 urban-canyon test scenarios** — industry-standard preset names ("Open Sky", "Urban Canyon", "Heavy Multipath", "Foliage", "Tunnel"). Reusing this naming makes your preset modes recognisable to anyone coming from GNSS testing.

## 5. Recommended approach — two-tier, drop-in, opt-in raycasting

I'd structure the implementation as one new sensor class with two cost tiers, plus a small visualization topic. This matches both the issue's deliverable and the constraints I see in #342 (CPU pressure) and #343 (CI/CD-friendliness).

### 5.1 Tier 1 — Cuenca-style DOP model (default; cheap)

- New class `DegradedGPS(GPS)` in `pegasus/simulator/logic/sensors/degraded_gps.py`, subclassing the existing GPS so you inherit the WGS84 projection and the Wagoner-style noise process.
- Override `update(state, dt)`:
  1. Generate satellite az/el set once at sim start from a synthetic almanac (or load a real RINEX almanac stamped at the simulated UTC start time).
  2. At each GPS tick, cast one ray per satellite from the drone position outward. Use Isaac Sim's `omni.physx.scene_query.raycast_closest` against the static scene collider mesh that was already created by `add_colliders(stage_prim)` in `simulation/isaac-sim/utils/scene_prep.py`.
  3. Build the visible-satellite list. Compute HDOP/VDOP via the standard 4×4 geometry matrix.
  4. Set `sattelites_visible = n_visible`, `eph = sigma_UERE * HDOP`, `epv = sigma_UERE * VDOP`, `fix_type = 3 if n_visible ≥ 4 else 1 if n_visible ≥ 1 else 0`.
  5. Sample position noise from `N(0, eph)` horizontally and `N(0, epv)` vertically, applied on top of the Ornstein–Uhlenbeck bias already in the base class.
- Default `sigma_UERE` = 5 m (standard GPS L1 C/A figure).
- Per-tick cost at 1 Hz with 12–31 raycasts is on the order of microseconds in Isaac Sim — well within budget.

### 5.2 Tier 2 — Pant-style multipath (opt-in; high-fidelity)

- Enabled by config flag `multipath: true`.
- For every satellite that the visibility ray hit a building, instead of just dropping it, run an additional reflected-ray search (Pant's "first-order reflection" approximation): cast a ray from the drone toward a small set of candidate reflector points sampled on nearby walls; if any of them have an unobstructed reflected path to the satellite, classify the signal as NLOS-multipath and add a positive pseudorange bias `b_mp ~ Uniform(5, 50) m`.
- For LOS-but-attenuated cases (foliage, low-elevation skim through buildings) apply a `C/N0`-derived eph weighting.
- Project the per-satellite pseudorange errors back to a position bias by running a one-shot weighted least squares over the visible-satellite geometry. The output bias replaces the symmetric noise, producing the *correlated* error structure Pant's plugin produces.
- This is the path where the 2026 ScienceDirect paper's octree-accelerated approach becomes relevant if performance hurts.

### 5.3 Jamming mode (orthogonal)

- Config flag `jamming: { type: barrage|cw|meaconing, intensity: 0..1 }`.
- Barrage = inflate noise covariance and drop random satellites at rate proportional to intensity.
- CW = drop a specific PRN.
- Meaconing = inject a slowly drifting position bias and keep `fix_type == 3` so the EKF accepts it (this is the dangerous spoofing case).
- These compose with Tier 1/2 — jamming intensity is independent of geometry.

### 5.4 Diagnostics and visualization

- Add a debug `/sim/gps/diagnostics` topic with `n_visible`, `eph`, `epv`, `fix_type`, the LOS/NLOS classification per satellite, and per-satellite pseudorange error. This makes MACVIO testing much easier to triage and gives the GCS a hook to draw a polar plot of the sky mask. It also gives a clean way to write unit-test assertions.

### 5.5 Where to hook it in the launch path

- Modify the example launch scripts under `simulation/isaac-sim/launch_scripts/` (`example_one_px4_pegasus_launch_script.py`, `example_multi_px4_pegasus_launch_script.py`) to allow `DegradedGPS` to be selected via a single config-dict entry.
- The OmniGraph node wrapper in `castacks/PegasusSimulator-AirStack-Integration` will need a corresponding parameter so the saved `.pegasus.robot.usd` files can opt in.

## 6. Limitations to flag in the PR description

These are the failure modes a reviewer will (and should) push back on. Acknowledging them upfront wins the review.

1. **No real satellite ephemeris.** The "constellation" is a synthetic az/el set. Validates well-enough for VIO testing; will not be acceptable if a Shimizu/DSTA reviewer wants Klobuchar-class ionospheric realism. Mitigation: optional RINEX almanac load.
2. **Pant is Gazebo; we are not.** The plugin can't be reused — we re-implement against `omni.physx`. The math is identical; the bug surface is in the engine binding. Plan an early Pant-versus-ours regression on a shared Hong Kong scene if you want to claim numerical agreement.
3. **PX4 EKF2 masks the degradation.** PX4's innovation gating may reject overtly bad GPS, hiding the degradation from the consumer. Mitigations: (a) inflate `eph`/`epv` honestly so EKF down-weights instead of rejects; (b) provide a separate degraded-GPS ROS 2 topic that MACVIO can subscribe to directly, bypassing the PX4 EKF path. The issue author hints at this with "we're not doing super realistic state estimation, it's coming from mavros sim."
4. **Ray budget vs. multi-agent.** Tier 1 is cheap; Tier 2 with M reflector samples × N satellites × K drones at 1 Hz is O(MNK) raycasts/sec. Issue #342's CPU bottleneck is already in the same neighbourhood. Add a spatial cache and amortize satellite visibility across multiple ticks if needed.
5. **No urban-canyon asset library.** AirStack ships with Isaac Sim environments but I don't see a dense urban canyon in the docs. You will need at least one before scenarios B–D are testable. Options: NVIDIA Omniverse's city assets, a CARLA Town01 USD conversion, or a small hand-built Pittsburgh-block USD. Worth scoping as a sibling issue.
6. **Magnetometer / barometer ignored.** Real urban canyons also corrupt the magnetometer (metal structures) and barometer (HVAC pressure). Out of scope for #345 but flag it — Cuenca explicitly says the position-error model assumes the other sensors are unaffected, which is a simplification.
7. **MACVIO interface not yet verified.** I have not located the MACVIO GPS-fusion subscriber in this repo (no GitHub matches surfaced). Before writing the recovery test, verify which message type MACVIO expects (almost certainly `sensor_msgs/NavSatFix` and `geometry_msgs/PoseWithCovarianceStamped`), the frame, and how it tags initialization. Lee 2020's online init/calib design is the reference if MACVIO needs it.

## 7. Test plan

### 7.1 Unit tests (run inside the Pegasus extension, no Isaac Sim runtime needed)

- **Open sky.** `DegradedGPS` with no scene geometry must reduce to baseline `GPS` statistics. Run 60 seconds, compare `eph`, `epv`, `n_visible` distributions to upstream. Tolerance: KS test p > 0.05.
- **Full occlusion.** Put the drone inside a sealed box. Expect `fix_type → 0`, `n_visible → 0`, `eph, epv → inf` (or a sentinel large value).
- **50 % sky mask.** Half-dome occlusion. Expect HDOP/VDOP to roughly double versus open sky, `n_visible` to halve.
- **Hysteresis.** Smoothly ramp the obstruction. `eph` should rise smoothly; `fix_type` may step but should not chatter at the 3/2 boundary. Add a 1-second lowpass if it does.
- **Performance.** Tier 1 GPS `update()` must finish in < 200 µs on a single core with 31 satellites against a 10k-tri collider. Tier 2 with `multipath: true` and 8 reflectors must finish in < 5 ms. Measure with `pytest-benchmark` against a fixed seed.

### 7.2 Integration tests (Isaac Sim + PX4 SITL + MACVIO)

Run each scenario as a ROS 2 launch + rosbag2 recording, assert against the bag. Tie into Issue #343 (AirLab Cloud CI/CD).

- **Scenario A — Open sky baseline.** Straight-line waypoint mission in an empty environment. Expect MACVIO and PX4 EKF position estimates to agree within 0.5 m RMS over 60 s. Failure mode: integration broken or new sensor regressed baseline behaviour.
- **Scenario B — Urban canyon.** Same mission between two ~30 m building rows. `eph` should rise from 1 m to > 5 m; PX4 EKF2 should down-weight (not reject) GPS via its innovation chi-squared; MACVIO should hold position within 2 m RMS during the canyon segment.
- **Scenario C — GPS denied (tunnel / overpass).** Complete sky occlusion for 20 s. Verify `fix_type < 3`, PX4 falls back to vision/inertial, MACVIO does not diverge (drift < 1 m/s).
- **Scenario D — Recovery.** Re-emerge to open sky after 20 s denial. Measure time-to-re-acquire (`fix_type == 3` stable for 3 s), and time-to-re-align between MACVIO and the GPS frame. This is the Lee 2020 evaluation scenario.
- **Scenario E — Jamming.** Static drone, raise `jamming.intensity` from 0 to 1 over 30 s with no geometry change. Verify EKF starts rejecting GPS at the right innovation threshold, and that MACVIO is robust.
- **Scenario F — Meaconing/spoofing (stretch).** Constant 0.3 m/s lateral drift bias injected with `fix_type` held at 3. Verifies that MACVIO does *not* silently follow the spoofed GPS when the visual map disagrees. This is the most adversarial test and the most useful for DTC/DSTA.

### 7.3 CI integration

- Headless Isaac Sim in a Docker container (`airstack up isaac-sim` is already supported per the docs).
- ROS 2 bag of the waypoint mission per scenario.
- Pass/fail thresholds in YAML committed alongside the scenarios.
- Per-scenario plots (eph timeline, position error timeline, sky mask snapshot) saved as CI artifacts. Foxglove visualization with `/sim/gps/diagnostics` can be linked from the report.
- Run on PRs that touch `pegasus/simulator/logic/sensors/`, the MAVLink backend, or the state estimator.

## 8. Open questions for review

1. Should the degraded GPS path bypass PX4 entirely and publish directly to MACVIO, or stay routed through PX4's `HIL_GPS`? The issue author leans toward the former. Confirm before implementation.
2. Which urban-canyon USD asset should be canonical? Need at least one to make scenarios B–D real.
3. Is a synthetic constellation acceptable for the first PR, with RINEX-almanac support as a follow-on?
4. Does MACVIO already implement Lee-style intermittent GPS initialization, or do we need a separate issue tracking that?
5. Tier 2 (Pant-style multipath) is the long pole; do we want it in the same PR or split it out behind the `multipath` flag for a fast-follow?

## 9. References

- castacks, AirStack repository, https://github.com/castacks/AirStack
- castacks, AirStack Issue #345 "Realistic GPS Sensor Degradation", https://github.com/castacks/AirStack/issues/345
- castacks, PegasusSimulator-AirStack-Integration, https://github.com/castacks/PegasusSimulator-AirStack-Integration
- AirStack docs, "Aerial Robot Simulation via the Pegasus Extension", https://docs.theairlab.org/main/docs/simulation/isaac_sim/pegasus_scene_setup/
- Pegasus Simulator docs, GPS sensor API, https://pegasussimulator.github.io/PegasusSimulator/source/api/sensors.gps.html
- Pegasus Simulator docs, PX4 Mavlink Backend, https://pegasussimulator.github.io/PegasusSimulator/source/api/backends.px4_mavlink_backend.html
- Cuenca et al. (2023), "Modeling of GPS Degradation Conditions for Risk Assessment of UAS Operations in Urban Environments", DOI 10.2514/6.2023-2648
- Pant et al. (2022), "An Open-Source Gazebo Plugin for GNSS Multipath Signal Emulation in Virtual Urban Canyons", arXiv 2212.04018, https://arxiv.org/abs/2212.04018
- Lee, Eckenhoff, Geneva, Huang (2020), "Intermittent GPS-aided VIO: Online Initialization and Calibration", ICRA, DOI 10.1109/ICRA40945.2020.9197029
- "Simulating GNSS multipath in urban environments using 3D ray tracing for automotive applications" (2026), ScienceDirect S1574119226000799
- Ng & Hsu (2021), 3D Mapping Database Aided GNSS RTK; Wen & Hsu (2022), 3D LiDAR Aided GNSS NLOS Mitigation, arXiv 2212.05477
- Groves (2013), *Principles of GNSS, INS, and Multisensor Integrated Navigation*, 2nd ed., Artech House
- Kerns et al. (2014), "Unmanned Aircraft Capture and Control via GPS Spoofing"; Humphreys (2008), "Assessing the spoofing threat"
- Related AirStack issues: #342 (Pegasus CPU bottleneck), #343 (CI/CD integration), #334 (PX4 EKF convergence delay)
