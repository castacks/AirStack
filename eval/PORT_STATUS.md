# AirStack-native SAFE port status

Target: fly each policy through AirStack's own PX4/ROS interfaces in the same
named SAFE aerial worlds, with SAFE's walking humans and scenario protocol.
The existing SAFE aerial run remains the reference. Changing to AirStack's
control path is a new experiment, even when the names and seeds match.

## What actually runs today

- `droan`: AirStack full-stack planner profile (source is AirStack, not a SAFE
  aerial policy). Its native task interface is `NavigateTask`.
- `random`, `aggressive`, `conservative`: older AirStack-specific baselines.
  Their implementations and speeds differ from SAFE's namesakes. Do not pool
  these results with the SAFE aerial runs.
- `saferandom`, `safeaggressive`, `safeconservative`, `safeadaptiveapf`: import SAFE's source
  classes from `integrations.aerial_nav.agents.baselines`, call their policy
  logic on AirStack observations, and convert their m/s velocity to the
  normalized AirStack trajectory-override input. Their observations and
  actions cross SAFE's X/Y-swapped benchmark frame at this boundary. CPU
  delegation and frame tests pass;
  no Isaac/PX4 flight has been validated. A SAFE source checkout must be on
  `PYTHONPATH`: the current `safe-benchmark` wheel packages `safe_core` and
  `safe_isaac`, not `integrations.aerial_nav`.
- `deploy_diffaero`: generic SVG commander for simulated or real PX4. It is
  separate from `eval/run.py` and has CPU interface tests only. Its checkpoint
  path in the example YAML is a placeholder until the identical SAFE
  checkpoint is installed in the robot container.

## Main-study policy ports

Every one of the 16 `main_moving.yaml` policies now has an AirStack `safe*`
wrapper. Source velocity baselines use trajectory override; learned policies
use AirStack's attitude/thrust ROS/PX4 path; eight planner policies start their
SAFE ROS sidecars and use AirStack's full setpoint path. VitFly starts its own
SAFE ROS sidecar and issues velocity commands. The AirStack bridge carries
ZED depth/RGB, IMU, and real Ouster lidar observations; learned camera settings
come from each source policy. SAFE source is mounted or imported read-only.

This is **code integration, not a completed benchmark run**. No source-backed
Isaac scene, sidecar, learned model, or PX4 policy flight has been run during
this port. The AirStack robot workspace/container must be rebuilt for the PID
control gate, then one episode per action/sensor family must pass flight and
telemetry checks. Policy-specific camera orientation, stereo baseline, frame
conversions, command timing, and physical command limits are still unverified.
Do not pool these unflown adapters with the SAFE aerial run.

## Source-backed worlds: current state

AirStack now reads domain names and authored properties from SAFE's Isaac
registry. The Isaac and robot containers mount that SAFE checkout read-only.
AirStack's Isaac launcher calls SAFE's `world_builder` to load the USD stage,
measure the floor/ground map and static meshes, and calls
`CctHumanController` when the experiment sets `humans: true`. Pegasus still
owns the AirStack PX4 drone and ROS sensors. The measured stage bounds and
mesh AABBs return to the host. Seeded starts, goals, static obstacle shapes,
wall gaps, pedestrian positions and perturbations now come from SAFE's own
scenario sampler inside Isaac. SAFE's tree and shape builders are called for
procedural static obstacles.

This path has only passed Python compilation and Docker Compose configuration.
It has **not flown in Isaac**. The remaining gaps are AirStack's approximate
contact monitor and safety adapter, and its different PX4/ROS drone dynamics.
Matching the source world and scenario code does not yet prove the generated
scene or contact labels agree. The next check is one domain/seed against a
SAFE scene manifest, followed by an AirStack flight and contact audit.

## Sim and real trial flow

The SVG deployment commander accepts `drone_mode: sim|real` and now can write
raw JSONL trials with `trial_log_dir`: policy name, goal, start, checkpoint
path, odometry, policy commands, and end reason. The example DiffAero sim
config writes under the robot workspace bind mount. This is deliberately not
a SAFE score yet. Sim contact and object geometry must come from Isaac;
physical contact and clearance must come from independent mocap/object
tracking (or another measured source). A real trial needs an explicit
scenario ID and policy/checkpoint identity so its data can be compared to the
intended simulated condition. The SAFE adapter can then score both using the
same outcome and safety definitions; no new SAFE core metric is required.

## Validation required before claims

1. Once GPU use is available, boot one source-backed domain without
   disturbing the SAFE benchmark workload.
2. Smoke one policy in one SAFE domain; compare stage USD, transform, static
   meshes, human actor count/paths, seeded start/goal, and collision/clearance
   against SAFE's source manifest.
3. Check ROS command frames and rates for each policy family, then fly one
   episode and inspect telemetry, safety events, and SAFE report output.
4. Only after those gates, expand to every policy and domain and run the
   sim/real matched-trial analysis.
