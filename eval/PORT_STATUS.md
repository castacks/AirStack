# AirStack-native SAFE port status

Target: fly each policy through AirStack's own PX4/ROS interfaces in the same
named SAFE aerial worlds, with SAFE's walking humans and scenario protocol.
The existing SAFE aerial run remains the reference. Changing to AirStack's
control path is a new experiment, even when the names and seeds match.

Last updated 2026-10-14 against SAFE `main` at 3951bd3. No Isaac flight has run
on this branch since that update, or before it. Everything below marked
"verified" was verified on CPU only.

## What actually runs today

- `droan`: AirStack full-stack planner profile (source is AirStack, not a SAFE
  aerial policy). Its native task interface is `NavigateTask`.
- `random`, `aggressive`, `conservative`: older AirStack-specific baselines.
  Their implementations and speeds differ from SAFE's namesakes. Do not pool
  these results with the SAFE aerial runs.
- `saferandom`, `safeaggressive`, `safeconservative`, `safeadaptive`,
  `safeadaptiveapf`: import SAFE's source classes from
  `integrations.navigation.aerial.policies.baselines`, call their policy logic
  on AirStack observations, and convert their m/s velocity to the normalized
  AirStack trajectory-override input. Their observations and actions cross
  SAFE's X/Y-swapped benchmark frame at this boundary. A SAFE source checkout
  must be on `PYTHONPATH`; the `safe-benchmark` wheel packages `safe_core` and
  `safe_isaac`, not `integrations/`.
- `deploy_diffaero`: generic SVG commander for simulated or real PX4. It is
  separate from `eval/run.py` and has CPU interface tests only.

## Matches current SAFE (2026-10-14)

The branch was written against SAFE as of 2026-10-03. These breaks were fixed:

| Area | Current SAFE contract | Where it is handled |
|---|---|---|
| Package moves | `integrations.navigation.common.config` (domains, scenarios, bands, stage exports), `integrations.navigation.aerial.policies` (baselines, learned, planners), `integrations.navigation.aerial.envs.setpoint` | every import in `eval/` and the Isaac launcher |
| Integration lookup | SAFE core resolves an integration name to a package under its own `integrations/` tree (provenance code hashes, scoring-code version, policy directory) | `eval/run.py` `register_integration_paths()` maps `airstack` to this `eval` package; `get_experiments_dir()` points at `eval/experiments` |
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
| Main study roster | `paper/main_moving.yaml` now flies `sando` and leaves `panther` out | `safesando` wrapper added (unflown) |
| Episode video | `debug:` profiles with capture call `env.get_debug_frames()` | the launcher's `capture_frames` command renders an overview and a drone-tracking camera; the env decodes them for SAFE's recorder |

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

Known gaps, not fixed:

- SAFE's oracles (`safeaggressive`, `safeconservative`, `safeadaptive`) now read
  `static_cylinder_radii`, `dynamic_obstacle_targets` and `dynamic_obstacle_radii`
  from `full_state`. AirStack's observation does not carry them yet, so those
  wrappers raise a `KeyError` at their first step. DROAN is unaffected.
- AirStack scores with its own centre-distance contact model (drone 0.31 m plus
  obstacle 0.30 m, matching the ScenarioManager's latched contact). The
  certification rulebook's point-obstacle `r_failure_m` is 0.31 m. The shell
  width (0.3 m) agrees, so `RE_max` agrees, but the radii are not the rulebook's.
- SAFE tools that resolve a run's integration on their own (`safe report RUN`,
  `--compare`, `--stop-at-collision`) do not import `eval/run.py`, so they do not
  know `airstack`. The runner writes its own report at the end of a run. A
  SAFE-side registration hook for out-of-tree integrations would fix this.
- AirStack's Isaac free-runs. SAFE does not step it, so episodes are observed,
  not stepped, and repeats vary with timing (see SAFE's
  `docs/reference/safe_in_projects.md`).

## Verified on CPU (2026-10-14)

`PYTHONPATH=/home/kayla/workspace/benchmark:/home/kayla/AirStack python3 -m pytest -q eval/test`
passes 17 tests. `eval/test/test_cpu_smoke.py` checks:

- every `eval` module imports, and the rclpy container bridge and the Isaac
  launcher compile;
- `droan_empty.yaml` passes SAFE's loader, required keys and run validation;
  `debug: default` turns live video capture on;
- the adapter, the `droan` policy and `AirstackStackEnv` construct;
- `eval/run.py --config droan_empty.yaml` runs as SAFE's job pool runs its
  worker child, with a stand-in for the stack (SAFE's real scenario sampler for
  `empty`, no Docker, render products that stay empty for the first renders):
  SAFE's rollout, simple-mode scoring, `trajectories/ep1.npz` with its record,
  `meta/episodes.csv`, `report.html` and
  `ep1_task_completed/episode_composite.mp4`, with the per-step risk bar fed.

`docker compose config isaac-sim` resolves the new mounts and variables.

Not verified: anything inside Isaac or the robot container. In particular
`FrameCapture` (replicator cameras), `attach_stage_export` on a photoreal stage,
SAFE's world builder inside AirStack's Isaac image, and DROAN reaching the goal
in SAFE's 30 m `empty` room (its start and goal can sit 5 m from a wall).

## Validation required before claims

1. Fly the `droan_empty` smoke flight below and check the video, the npz and
   the report.
2. Smoke one policy in one SAFE domain; compare stage USD, transform, static
   meshes, human actor count/paths, seeded start/goal, and collision/clearance
   against SAFE's source manifest.
3. Check ROS command frames and rates for each policy family, then fly one
   episode and inspect telemetry, safety events, and SAFE report output.
4. Only after those gates, expand to every policy and domain and run the
   sim/real matched-trial analysis.

## First smoke flight: droan_empty on this machine

Run these from a shell on this machine when the GPU has room for a second
Isaac instance, or after the SAFE paper runs finish. The images
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

If an Isaac capture camera never returns a frame, the flight stops with an error
naming the camera rather than flying without video.

Where the results land (the date is the run's `MMDD`; a second run the same day
gets a letter suffix):

| Output | Path |
|---|---|
| Run folder | `/home/kayla/AirStack/runs/airstack/droan_empty_<MMDD>/` |
| Report | `.../droan_empty_<MMDD>/report.html` |
| Episode table | `.../droan_empty_<MMDD>/meta/episodes.csv` |
| Trajectory and record | `.../droan_empty_<MMDD>/raw/empty/scenario_1/perturbation_1/trajectories/ep1.npz` |
| Episode video | `.../droan_empty_<MMDD>/raw/empty/scenario_1/perturbation_1/ep1_<outcome>/episode_composite.mp4` |

The video is the overview and the drone-tracking view side by side, at 2 frames
per second (one per 0.5 s `env_dt` step), covering the episode from the goal
handoff to the end.

Expected time: 3 to 10 minutes for Isaac and PX4 to boot (the readiness wait
allows 10), up to 3 for EKF convergence, about 1 for land, teleport, takeoff and
run-up, then the flight itself (at most 90 s of sim time, longer in wall time if
Isaac runs below real time), plus about a minute for the video and report. Plan
on 10 to 20 minutes.
