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

## Remaining policy ports

| SAFE source policy | AirStack control path needed | State |
| --- | --- | --- |
| Adaptive | trajectory override plus its perception inputs | Not ported |
| AdaptiveApf | source-imported velocity policy | CPU-tested adapter; no Isaac flight |
| AgileAutonomy, AgileRGB variants, DepthNav, DiffAero, DiffPhysDrone, Vitfly | policy-specific RGB/depth/IMU/odometry observation adapter and attitude/thrust command via PX4 | Not ported to `eval`; DiffAero has an unflown SVG deployment adapter |
| Dynus, EgoPlanner, FastPlanner, Faster, Mighty, Panther, Raptor, Super | their ROS planner process, sensor/map bridge, goal and setpoint handoff through AirStack | Not ported |
| Crash, CrashHVC, CrashLVC, SR, SHP | diagnostic baselines and probe-specific contracts | Not ported |

The ROS planners still need their processes. AirStack may let some run in its
robot container instead of a separate sidecar container, but it does not
remove their ROS node, dependencies, mapping, or timing contract. A planner
must replace AirStack's local planner rather than publish competing commands.

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
