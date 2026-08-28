# Design Spec: Pre-RFC Workflow Cleanup (sim launch DX)

> Notebook entry: `notebook/001-pre-rfc-workflow-cleanup/` · Date started: 2026-08-20 · Last updated: 2026-08-20 (impl day) · Branch: `feature/pre-rfc-workflow-cleanup`
>
> **Status: `DONE`** — all sections implemented, committed (87a22f4b..90b9955c), and validated; see results/results_summary.md

## 1. Problem Context

A design session (2026-08-19/20) mapped the simulation launch + customization workflow and found heavy friction. RFCs #379 (Modular AirStack) and #380 (Heterogeneous AirStack) define the long-term overhaul (stacks, fleet files, vehicle configs, generic fleet spawner). **This campaign is explicitly scoped to everything pre-RFC or tangential to the RFCs** — cleanup that makes the workflow better today and makes the RFC work smaller, without implementing any RFC-specified design (no fleet schema, no vehicle configs, no stacks, no module system).

Evidence from the session's audit (agent exploration of the repo at `develop` @ `9e2e0e39`):

- **Isaac launch scripts are ~80–90% copy-pasted boilerplate.** Six scripts in `simulation/isaac-sim/launch_scripts/`; identical blocks (extension enabling, `wait_for_stage`, scale/collider/dome-light, dead `SAVE_SCENE_TO` export, run loop, main guard) appear 4–5×. Drift already happened: `example_multi_drone_scene_import.py` uses ZED offset `[0.21, 0, 0.05]` vs `[0.2, 0, -0.05]` everywhere else; livestream/WebRTC exists only in the two `_one_` scripts (so the `isaac-sim-livestream` compose service silently breaks with multi scripts); `ISAAC_SIM_HEADLESS` is honored only by the multi scripts; `barebones_pegasus_launch.py` (the documented template) crashes with `NameError` (`os` never imported).
- **`airstack up` requires knowing coordinated env-var sets.** Choosing a sim = editing `COMPOSE_PROFILES` + `URDF_FILE` (+ remembering `overrides/ms-airsim.env`); `NUM_ROBOTS>1` silently requires `ISAAC_SIM_SCRIPT_NAME=example_multi_...` (worst footgun: 3 containers, 1 drone, no warning). The test harness already automates the correct derivation (`tests/harness/sim.py`).
- **Guards validate the wrong thing.** `cmd_up` (airstack.sh:867-907) `sed`s `.env` only, so `--env-file overrides/ms-airsim.env` bypasses the one-simulator guard and URDF pairing warning.
- **`airstack down` can miss running containers**: compose only operates on active profiles, so `up` with an override env-file followed by plain `down` leaves the other profile's containers running.
- **No readiness signal.** `up` prints success when `docker compose up -d` returns — before colcon builds, sim load, PX4 boot. Real budgets (from the system tests): `/clock` up to 600 s, sentinel nodes 300 s, PX4 arm-ready 300 s. Users guess.
- **`airstack logs` is near-useless by construction**: every service runs its real workload inside tmux; docker stdout has nothing. Failures (colcon errors, Pegasus import errors, missing binary) land in invisible tmux panes.
- **Docs drift**: ~12 confirmed wrong statements (nonexistent `ISAAC_SIM_SCENE` var in 4+ places, wrong defaults in isaac docker.md, `airstack stop`/`build` in AGENTS.md aren't registered commands, getting_started describes RViz/auto-play that don't match `.env` defaults, wrong MAVROS ports in ms-airsim docs, etc.).

RFC-forward-compatibility constraints adopted for this campaign (from #380 §3–4):
- CLI flags **select among declared configs and set leaf values, never define new structure**.
- Every flagged run dumps its resolved leaf values (precursor of `effective_config.yaml`).
- No new config file formats/schemas that the RFCs would have to migrate (no scenario.yaml, no fleet.yaml).
- Launch script external interface unchanged (`ISAAC_SIM_SCRIPT_NAME` still selects a script; env vars keep their names) — the dedup is internal so the future fleet spawner is a small diff, not a rewrite.

User directives for the campaign: branch off develop (done: `feature/pre-rfc-workflow-cleanup`); run airstack tests as baseline BEFORE changes; validate with automated tests + system tests; commit in sensible stages; update notebook as I go; proceed autonomously unless something surprising needs attention.

## 2. Proposed Implementation

### 2.1 Isaac launch-script dedup — `DONE` (commit 87a22f4b)

New `simulation/isaac-sim/launch_scripts/pegasus_app.py`: a `PegasusApp` base class owning all shared boilerplate — SimulationApp construction (incl. livestream + headless handling, previously inconsistent), extension enabling, `wait_for_stage`, world/env loading, stage prep (scale/collider/dome light), optional contained-USD export (replacing the 4 dead `SAVE_SCENE_TO` copies), drone spawn loop from a `DRONE_CONFIGS`-style list, ZED/lidar attachment with canonical default offsets, PX4 param plumbing, timeline start (`PLAY_SIM_ON_START`), and the run loop. The six scripts become thin subclasses/config declarations (~20–40 lines each) that preserve today's observable behavior:

- `example_one_px4_pegasus_launch_script.py` — 1 drone; **gains** `ISAAC_SIM_HEADLESS` support (base class), keeps livestream.
- `example_multi_px4_pegasus_launch_script.py` — `NUM_ROBOTS` row spawn; **gains** livestream support (base class), keeps `ENABLE_LIDAR` gate.
- both `_natnet_` variants — natnet server/body authoring becomes a base-class hook; the single-drone pose duplication (two places that can disagree) collapses to one.
- `example_multi_drone_scene_import.py` — keeps its explicit `DRONE_CONFIGS` incl. its divergent ZED offset `[0.21,0,0.05]` **passed explicitly** (dedup must not silently change behavior; drift is now visible in the diff-able subclass, noted for the RFC campaign).
- `barebones_pegasus_launch.py` — fixed (`os` import bug disappears by construction).

Non-goals: no generic fleet spawner, no config file, no changes to which env vars exist.

### 2.2 CLI intent flags on `airstack up` — `DONE` (commit af9bb191; 14 contract tests green)

`airstack up [--sim isaac|isaacsim|airsim|msairsim] [--robots N] [--headless] [--play|--no-play] [--no-autolaunch] [--dry-run]`. Parsing pulls intent flags before `classify_compose_args`; derivation **exports** env vars (shell env beats `--env-file` in compose interpolation — same mechanism the test harness uses):

| Flag | Exports |
|---|---|
| `--sim isaac` | `COMPOSE_PROFILES="desktop,isaac-sim"`, `URDF_FILE=robot_descriptions/iris/urdf/iris_with_sensors.pegasus.robot.urdf` |
| `--sim airsim` | `COMPOSE_PROFILES="desktop,ms-airsim"`, `URDF_FILE=robot_descriptions/iris/urdf/iris_stereo.ms-airsim.urdf` |
| `--robots N` | `NUM_ROBOTS=N`; if isaac: `ISAAC_SIM_SCRIPT_NAME=example_{one,multi}_px4_pegasus_launch_script.py` by N (unless user explicitly exported a script or set a `_natnet_` variant in .env — then warn instead of override... resolution: flag-derived export only when the currently-resolved script is one of the two example defaults; otherwise warn and leave) |
| `--headless` | `ISAAC_SIM_HEADLESS=true` (isaac) — honored everywhere after 2.1 |
| `--play`/`--no-play` | `PLAY_SIM_ON_START=true/false` |
| `--no-autolaunch` | `AUTOLAUNCH=false` |

Every `up` prints an effective-config banner (sim, robots, script, urdf, autolaunch, play_on_start, profiles) whether or not flags were used. Flagged values + resolved leaf set dumped to `.airstack/runs/<ts>/effective_config.env` (gitignored). `--dry-run` prints banner + dump and exits 0 without compose — this is also the automated-test seam.

### 2.3 Preflight validation — `DONE` (commit af9bb191)

> **Finding (2026-08-20):** the planned `down` cross-profile fix is unnecessary — `cmd_down` already runs `--profile "*"` when no services are given (airstack.sh:1055). Dropped from scope.
>
> **Finding (2026-08-20):** baseline first attempt failed fast: merged PR #384 bumped `.env` VERSION to `0.19.0-alpha.17` but only `alpha.16` images were local — live proof of the missing-images footgun the preflight check addresses. Fixed via `airstack image-pull` (registry had the images). The harness's exact-image-names error message is the template for the CLI check.

In `cmd_up`, after flag derivation, validate **resolved** values (env > .env, matching compose interpolation) instead of sed-ing `.env`:

- one-simulator guard now sees `--env-file` contributions (read profile from resolved env; if `--env-file` args present, also merge their contents for the check).
- `NUM_ROBOTS>1` + single-drone Isaac script → hard error naming the fix (`--robots N` auto-derives; manual env mismatch errors).
- missing images for active profiles → warn with exact image names + `airstack image-pull` hint before compose's implicit build starts (reuses the image list from `docker compose config`).
- isaac profile + missing `simulation/isaac-sim/docker/omni_pass.env` → error naming `airstack setup`.
- isaac profile + empty PegasusSimulator submodule → error naming `git submodule update --init --recursive`.
- Docker < 29 → warn that ROBOT_NAME resolution degrades to `unknown_robot` (surfaced on host, not in tmux).
- `NUM_ROBOTS` exceeding robot-name-map entries → warn.

`cmd_down`: run with `COMPOSE_PROFILES="*"` so `down` stops all project containers regardless of which profile launched them.

### 2.4 Readiness: `airstack ready` + `airstack up --wait` — `DONE` (af9bb191; success path live-validated 2026-08-20: flight-ready in 92s, staged gates; failure path + --json verified)

New CLI command `ready` (module `.airstack/modules/`): staged, polled gates mirroring the system-test harness's (source of truth for budgets):

1. containers running (compose ps) — 120 s
2. sim `/clock` publishing (docker exec in robot container, domain per robot) — 600 s
3. per-robot sentinel nodes (`mavros`, `robot_state_publisher`, `trajectory_control_node`) — 300 s
4. PX4 ready: `mavros/state` connected **and** `local_position/odom` streaming — 300 s

Output: live per-gate/per-robot status lines; exit 0 all-ready / 1 timeout naming the stuck gate + the tmux window to inspect. `airstack up --wait` chains into it. `--json` for scripts. Robot name/domain discovery reuses `airstack status`'s existing resolution.

### 2.5 tmux output → `docker logs` — `DONE` (090356bf; live-validated: colcon/MAVROS/Kit output visible in docker logs, 535/576 lines vs ~0 before)

Every autolaunch tmux session gets `tmux pipe-pane -o 'cat >> /proc/1/fd/1'` on its panes right after creation, so pane output reaches container stdout → `docker logs` / `airstack logs` become truthful. Implemented where sessions are created (compose `command:` lines / entrypoints) via one shared helper to avoid 5 copies of the incantation. Colcon build progress, Pegasus import errors, scene-download progress become visible from the host.

### 2.6 Docs & config drift fixes — `DONE` (commits a5ff5723 + b1a3e4a2: drift fixes, PegasusApp authoring docs, CLI flags/ready reference, write-isaac-sim-scene skill re-taught + 5 skills updated)

Fix the audited wrong statements (docs match code; no behavior changes): getting_started (paused-by-default sim + play button, Foxglove not RViz, correct scene name); simulation/index + isaac docker.md (`ISAAC_SIM_SCENE` → `ISAAC_SIM_GUI` USD path semantics, correct defaults table, multi-robot needs multi script — now auto via `--robots`); isaac docker.md log-viewing section (tmux/`airstack connect`, now also true `airstack logs` after 2.5); ms-airsim index (MAVROS port formula, FOV default, vehicle naming); AGENTS.md (`airstack down`/`image-build` names, new flags); docker_usage.md (`robot-test` service name); gcs user_interface.md (profile/service pairing); `.env` header comment (correct invocation). Plus `.env` comment for `PLAY_SIM_ON_START` documenting the paused-by-default choice.

### Affected packages

| Package / area | Change |
|---------|--------|
| `simulation/isaac-sim/launch_scripts/` | new `pegasus_app.py` base; 6 scripts become subclasses |
| `airstack.sh` + `.airstack/modules/` | intent flags, banner, dry-run, preflight, down fix, `ready` command |
| `robot/docker/docker-compose.yaml`, `simulation/*/docker/*`, gcs compose | tmux pipe-pane teeing |
| `docs/**`, `AGENTS.md`, `.env` comments | drift fixes |
| `tests/` | none required (harness unchanged); baseline + comparison runs use it |

### Interfaces

| Interface | Type | Direction | Purpose |
|-------------------------|------|-----------|---------|
| `airstack up --sim/--robots/--headless/--play/--no-play/--no-autolaunch/--dry-run` | CLI flags | in | intent-based launch (leaf-value overrides only) |
| `airstack ready [--wait-timeout S] [--json]` | CLI command | out | staged readiness gates |
| `.airstack/runs/<ts>/effective_config.env` | file (gitignored) | out | resolved leaf values per run |
| env vars | unchanged names | — | `ISAAC_SIM_HEADLESS` + livestream now honored uniformly (2.1) |

## 3. Test Plan

### (a) Baseline (before any change)

- **What is run:** `./airstack.sh test -m unit -v`; `./airstack.sh test -m "liveliness or sensors" --sim isaacsim --num-robots 1 --stress-iterations 1 -v`; `./airstack.sh test -m takeoff_hover_land --sim isaacsim --takeoff-velocities 1.0 -v` — all on clean `develop` (`9e2e0e39`) before edits.
- **What is measured:** pass/fail set; `metrics.json` (topic Hz, RTF, bring-up durations, flight phases).
- **Pass criteria:** recorded as-is (baseline); artifacts copied to `results/a-baseline/`.

### (b) Launch-script dedup parity

- **What is run:** static: `python -m py_compile` on all launch scripts + a diff review that each subclass passes its historical scenario values; dynamic: re-run the (a) Isaac system marks (`liveliness or sensors`, `takeoff_hover_land`) on the branch.
- **What is measured:** same metrics as (a); `tests/parse_metrics.py` comparison vs baseline campaign.
- **Pass criteria:** all previously-passing tests pass; no genuine metric regression per parse_metrics; multi-script gains (headless honored) verified by grep/静 review.

### (c) CLI intent flags + preflight

- **What is run:** scripted `--dry-run` matrix (new `tests/test_airstack_cli_launch.py` unit-mark test shelling `./airstack.sh up --dry-run ...` and asserting exported derivations, banner content, guard errors: airsim profile+urdf pairing, robots>1 script switch, natnet-script no-override warn, one-sim guard with `--env-file`); plus one real `./airstack.sh up --sim isaac --robots 1 --no-autolaunch` bring-up + `down` (containers gone incl. cross-profile check).
- **What is measured:** exact env derivations, exit codes, error messages; container up/down state.
- **Pass criteria:** matrix green in `airstack test -m unit`; real bring-up starts isaac profile without .env edits; `down` removes all containers.

### (d) Readiness gates

- **What is run:** `./airstack.sh up --sim isaac --wait` on the live stack (piggybacks on (b) bring-up if practical); `airstack ready --json` against an idle (`--no-autolaunch`) stack.
- **What is measured:** gate progression timestamps vs harness budgets; exit codes; JSON shape.
- **Pass criteria:** all gates reach ready ≤ harness budgets on the live stack; idle stack times out gate 2+ with actionable message and exit 1.

### (e) Log visibility

- **What is run:** during (d) live stack: `docker logs airstack-robot-desktop-1` and `airstack logs`.
- **What is measured:** presence of colcon/ros2-launch output in docker logs (pre-change: empty).
- **Pass criteria:** ros2 launch banner + node startup lines visible via `docker logs`; excerpt saved.

### (f) Docs correctness

- **What is run:** `mkdocs build` (via docs container or local) + grep assertions (no `ISAAC_SIM_SCENE` outside changelog contexts; no `airstack stop`/`airstack build` in AGENTS.md; port formula corrected).
- **What is measured:** build success; grep hit counts.
- **Pass criteria:** build clean; greps return zero stale hits.
