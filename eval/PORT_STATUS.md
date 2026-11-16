# AirStack-native SAFE port status

Target: fly each policy through AirStack's own PX4/ROS interfaces in the same
named SAFE aerial worlds, with SAFE's walking humans and scenario protocol.
The existing SAFE aerial run remains the reference. Changing to AirStack's
control path is a new experiment, even when the names and seeds match.

Last updated 2026-11-16 against SAFE at 82af3f0, where the navigation task is the
top-level `safe_navigation` package. Isaac flights have run on this branch:

| Run | Episodes | Result |
|---|---|---|
| `droan_empty_1015` | 1 | reached the goal |
| `airstack_all_policies_primitive_sparse_1015` | 1 per policy, 17 policies | 8 reached the goal, 8 catastrophic failures, agileautonomy wrote no record; `random`, `adaptiveapf` and `panther` were not in the run |
| `droan_primitive_photoreal_1031` | 1 per domain, 8 domains | none reached the goal |
| `droan_isaac_moving_primitive_seed100_1113` | 99 | 84 timeouts, 15 catastrophic failures |

DROAN's timeouts follow from its speed. AirStack runs DROAN as `droan_gl` (the CPU
`droan_local_planner` and its `droan.yaml` are commented out of `local.launch.xml`), whose
candidate trajectories head out at a fixed 1 m/s written in `droan_gl/src/gl_interface.cpp`.
DROAN flies at a median 0.91 m/s, and SAFE's paper grid gives 60 s for a 60 m goal, a budget
SAFE's Iris meets at its 5 m/s cruise.

## What actually runs today

- `droan`: AirStack full-stack planner profile (source is AirStack, not a SAFE
  aerial policy). Its native task interface is `NavigateTask`.
- `airstackrandom`, `airstackaggressive`, `airstackconservative`: older
  AirStack-specific baselines. Their implementations and speeds differ from
  SAFE's namesakes. Do not pool these results with the SAFE aerial runs.
- `random`, `aggressive`, `conservative`, `adaptive`, `adaptiveapf`: import
  SAFE's source classes from `safe_navigation.aerial.policies.baselines`, call
  their policy logic on AirStack observations, and fly their m/s velocity on
  AirStack's trajectory override with no speed ceiling. Their observations and
  actions cross SAFE's X/Y-swapped benchmark frame at this boundary.
- The nine planners (`fastplanner`, `egoplanner`, `raptor`, `faster`, `super`,
  `mighty`, `dynus`, `panther`, `sando`) subclass SAFE's wrappers from
  `safe_navigation.aerial.policies.planners`, start SAFE's sidecars and send
  their setpoints to PX4. The learned policies (`diffphysdrone`, `depthnav`,
  `diffaero`, `agileautonomy`, `vitfly`) load SAFE's classes from
  `safe_navigation.aerial.policies.learned` and send attitude and thrust, or
  velocity for `vitfly`.
- The benchmark checkout must be on `PYTHONPATH`.
- `deploy_diffaero`: generic SVG commander for simulated or real PX4. It is
  separate from `eval/run.py` and has CPU interface tests only
  (`svg_ground_control/test/test_deploy_interfaces.py`).

## Matches current SAFE (2026-11-16)

The branch was written against SAFE as of 2026-10-03. These breaks were fixed:

| Area | Current SAFE contract | Where it is handled |
|---|---|---|
| Package moves | `safe_navigation.common.config` (domains, scenarios, bands, stage exports), `safe_navigation.aerial.policies` (baselines, learned, planners), `safe_sim.sim_config`, `safe_core.run` (jobs, container reaper), `safe_core.records` (episodes, experiment yaml) | every import in `eval/` and the Isaac launcher |
| Floating domain | the movers are walking people; the drifting cubes are gone | the launcher builds people on floating as on every other domain |
| Integration lookup | SAFE core finds an integration's package (provenance code hashes, scoring-code version, policy directory) through `thesafebenchmark.integrations` entry points | `eval/run.py` `register_integration_paths()` patches the lookup to map `airstack` to this `eval` package; `get_experiments_dir()` points at `eval/experiments` |
| Robot | `robot:` is a required, registered yaml key | `robot: iris` in the yaml; `AirstackRunner.required_yaml_keys`; any other robot is refused |
| Rollout info | `info["failure"]` must be present and sticky | `AirstackStackEnv.step` |
| Env params | core reads `RE_max` (npz, provenance, video risk bar) | `get_env_params()` takes it from `configs/certification.yaml` |
| Episode record | `scenario_difficulty`, `n_collisions`; npz in SAFE's navigation schema with `RECORD_VERSION` | `Adapter.extract_episode_context`, `Adapter.trajectory_arrays` |
| Scenario seeds | derived from the domain name (`domain-name-sha256-v1`) by the framework | the adapter passes the framework seed through unchanged |
| Density bands | `env._scenario_index` selects a photoreal stage's band | `Adapter.apply_scenario` sets it; it travels to SAFE's sampler inside Isaac, which returns `scenario_difficulty` |
| Stage exports | photoreal stages need `attach_stage_export` after load, plus the files under `~/safe_benchmark_assets` | `safe_source_scene.load_domain`; the Isaac container mounts the assets read-only at their host path and sets `SAFE_BAKED_STAGES`, `SAFE_STAGE_EXPORTS`, `SAFE_SKIES`, `SAFE_URBANVERSE` |
| Floor | the sampler moves `origin_z` (which carries `floor_z`) to each scenario's floor | the launcher returns `ground_z` per scenario; the host teleports onto it |
| Lighting | SAFE lights USD stages too, with the domain's sky HDR when it has one | `safe_source_scene.load_domain` |
| Optional hooks | `resolved_domain`, `perturbations_vary_episodes` | `eval/adapter.py` |
| Planner sidecar registry | `fault()` is gone; records are resolved and probed | `run_agent_hook` polls `resolve()` and `is_sidecar()` |
| Paper roster | `paper/isaac_moving_primitive.yaml` flies the nine planners and `depthnav`, `diffaero`, `vitfly`, `agileautonomy` | every one has a wrapper (`eval/test/test_main_policy_coverage.py`); `panther` has not flown here |
| Episode video | SAFE's replay renderer draws episode videos from the npz after a run | not yet; AirStack records no video until the hand-off below is built |

Container safety on a shared machine:

- SAFE's planner hooks `docker rm -f` and `pkill` by container name and port.
  `run_agent_hook` now gives every hook an `airstack_safe_<planner>` container
  name, a port 1000 above SAFE's base port (first free of 16), its own sidecar
  record directory (`eval/runs/.sidecars`) and SAFE's owner labels. A live SAFE
  benchmark's sidecars are never named.
- Container lookups are limited to AirStack's compose project label, and
  `airstack down` is called only for the two services the eval starts
  (`robot-desktop`, `isaac-sim`), not the whole project. The `safe-session`
  container is never touched.

Scoring geometry: clearance is measured from the drone's oriented collider box
with each rotor counted as its swept disk (0.35 m half width), grown with PhysX
`overlap_box` to 2 cm resolution against every stage collider and people's
capsules. Arena walls are exact. The shell width (0.3 m) is the rulebook's.

Open items:

- Episode video goes through SAFE's replay renderer only. `AirstackRunner` does not yet
  hand a finished run to it, SAFE's renderer finds policy classes only in SAFE's own
  aerial policies folder, and the `droan` class declares `full_state`, which draws no
  sensor panel, where its real input is the ZED stereo disparity.

- `safe report RUN` and SAFE's other standalone tools find integrations through
  `thesafebenchmark.integrations` entry points, and nothing registers `airstack`
  there, because `eval/` is a folder on `PYTHONPATH` rather than an installed
  package. The runner writes its own report at the end of a run. A pyproject that
  installs this package and registers `airstack` would let SAFE's tools read
  these runs and replace the patch in `register_integration_paths()`.
- When Pegasus did not autolaunch PX4, the launcher skips the PX4 relaunch after
  a teleport with a warning and the next arm can fail. It should raise instead.
- The launcher's branches for a domain without a SAFE config (AirStack's own
  obstacle pool columns) are unreachable since the eval reads only SAFE's domains.

## CPU tests

`PYTHONPATH=/home/kayla/workspace/benchmark:/home/kayla/AirStack python3 -m pytest -q eval/test`
runs them; they were last run on 2026-10-14. `eval/test/test_cpu_smoke.py` checks:

- every `eval` module imports, and the rclpy container bridge and the Isaac
  launcher compile;
- `droan_empty.yaml` passes SAFE's loader, required keys and run validation;
- the adapter, the `droan` policy and `AirstackStackEnv` construct;
- `eval/run.py --config droan_empty.yaml` runs as SAFE's job pool runs its
  worker child, with a stand-in for the stack (SAFE's real scenario sampler for
  `empty`, no Docker): SAFE's rollout, simple-mode scoring, `trajectories/ep1.npz`
  with its record, `meta/episodes.csv` and `report.html`.

`docker compose config isaac-sim` resolves the new mounts and variables.

## Validation required before claims

1. Fly the `droan_empty` smoke flight below and check the video, the npz and
   the report. Flown 2026-10-15; DROAN reached the goal.
2. Smoke one policy in one SAFE domain; compare stage USD, transform, static
   meshes, human actor count/paths, seeded start/goal, and collision/clearance
   against SAFE's source manifest.
3. Check ROS command frames and rates, sensor frames, camera geometry, sidecar
   startup and controller ownership for each policy family, then fly one
   episode and inspect telemetry, safety events, and SAFE report output.
4. Only after those gates, expand to every policy and domain and run the
   sim/real matched-trial analysis.

## Smoke flight: droan_empty on this machine

Run these from a shell on this machine when the GPU has room for a second
Isaac instance. The images
`v0.18.0_robot-x86-64_dev` and `v0.18.0_isaac-sim` named by `.env` are
already pulled. DROAN never gates the PID output, so it does not need the
robot image rebuilt; the robot workspace builds itself (`bws`) when the
container starts.

```bash
# 1. Check the box: GPU headroom, and what is already running.
nvidia-smi --query-gpu=memory.used,memory.total --format=csv
docker ps --format '{{.Names}}\t{{.Status}}'

# 2. Only if `docker images | grep v0.18.0_` shows nothing (needs the AirLab registry login).
cd /home/kayla/AirStack && airstack image-pull

# 3. The flight. The runner itself calls `airstack up robot-desktop isaac-sim`
#    with the SAFE eval launcher, waits for PX4, EKF and the ScenarioManager,
#    then flies one episode.
cd /home/kayla/AirStack
export PYTHONPATH=/home/kayla/workspace/benchmark:/home/kayla/AirStack
python3 eval/run.py --config eval/experiments/droan_empty.yaml -y --headless
#    (`airstack safe --config eval/experiments/droan_empty.yaml -y --headless`
#    is the same entry point.)

# 4. keep_up: true leaves the stack running after the run, for `airstack connect`
#    and logs. The next run restarts it anyway. Stop it with:
COMPOSE_PROFILES=desktop,isaac-sim airstack down robot-desktop isaac-sim
```

Where the results land (the date is the run's `MMDD`; a second run the same day
gets a letter suffix):

| Output | Path |
|---|---|
| Run folder | `/home/kayla/AirStack/runs/airstack/droan_empty_<MMDD>/` |
| Report | `.../droan_empty_<MMDD>/report.html` |
| Episode table | `.../droan_empty_<MMDD>/meta/episodes.csv` |
| Trajectory and record | `.../droan_empty_<MMDD>/raw/empty/scenario_1/perturbation_1/trajectories/ep1.npz` |

Expected time: 3 to 10 minutes for Isaac and PX4 to boot (the readiness wait
allows 10), up to 3 for EKF convergence, about 1 for land, teleport, takeoff and
run-up, then the flight itself (at most 90 s of sim time, longer in wall time if
Isaac runs below real time), plus about a minute for the report. Plan on 10 to
20 minutes.
