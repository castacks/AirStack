# Release Notes

Feature docs deliberately describe only the system as it exists — never how
it got that way — so that every page reads standalone. This page is the one
place where change context lives: what changed, from what, and why.

The published site shows only the notes for the docs version you are
viewing. To read another version's notes, switch versions with the selector
in the header. Versioned release-notes pages exist from **0.19.0** onward;
notes for older releases (0.18.0 and earlier) live on the
[GitHub releases page](https://github.com/castacks/AirStack/releases).

<!-- Contributors: this source file keeps one `##` section per version
(0.19.0 and newer — older notes live only on GitHub Releases), newest
first. A build hook (docs/hooks/release_notes_current_version.py,
registered in mkdocs.yml) trims the rendered page to the section matching
the repo-root .env VERSION, so each mike-deployed docs version carries only
its own notes. -->

## 0.21.0 (Unreleased)

- **Fixed: post-release main→develop sync no longer skips.** The
  `sync-develop-from-main` workflow skipped whenever the merge result was
  content-identical to `develop` — exactly the situation right after a
  release — so it never pushed the ancestry-advancing merge commit or
  rolled `develop`'s VERSION forward. It now skips only when `main` is
  already an ancestor of `develop`.

## 0.20.0 — 2026-08-29

- **Pre-release versioning terminology: `-alpha.N` → `-dev.N`.** The
  development line on `develop` now uses `X.Y.Z-dev.N` pre-release versions
  (first: `0.21.0-dev.0`) instead of `X.Y.Z-alpha.N`. The
  `check-version-increment` PR gate accepts exactly `dev`, `beta`, `rc`
  (ordering `dev < beta < rc < release`); the main→develop sync workflow
  rolls develop forward to the next minor's `-dev.0` after a release. Docs,
  skills, and the PR template were updated to match. Existing `-alpha.N`
  versions in git history and in declared module-compat ranges remain valid
  historical references.

- **Single, revamped repo README.** The repo now has one `README.md` (at the
  root — `docs/README.md` was removed), rewritten around the redesigned docs
  home page: quickstart, one-command bring-up, sim-to-real parity, CI flight
  campaigns, and agent-driven workflows.

- **Module-catalog sync automation + drift alarm.** Registering a module is
  two merges (registry PR to
  [airstack-modules-index](https://github.com/castacks/airstack-modules-index)
  + trunk fixture/catalog sync), and the docs deploy regenerates the
  published catalog from the **live** registry — so a missed half used to
  drop the module from the site silently. Now the develop docs deploy
  raises a `docs-catalog-drift` issue whenever the committed catalog and
  the live registry disagree, and the new `sync-modules-index` workflow
  (daily + manual dispatch) opens the trunk sync PR automatically
  (`.github/workflows/scripts/registry_sync.py`).

- **Trustworthy system-test outcomes.** A red system-tests run now always
  means the code under test got worse, never that CI infrastructure hiccuped:
  `run_meta.json` (schema v2) classifies every failure as
  `assertion` / `infrastructure` / `collection` / `ci_integrity`, and CI fails
  only on those — comparable numeric metric deltas (Hz, CPU, error metrics)
  are **advisory** in the report and no longer fail the PR. Metric
  comparisons only happen between fingerprint-identical campaigns (same
  tests **and** same behavior-changing CLI config: sim, robot count,
  trajectories, velocities, tolerances), with the baseline selected from
  recent base-branch artifacts by fingerprint instead of "newest artifact of
  any shape". Bring-up/readiness failures fail fast when the simulator
  process dies (instead of burning the full `/clock` timeout on an ephemeral
  GPU pod) and capture a bounded, secret-free `diagnostics/` bundle before
  the pod is destroyed. Maintainers can dispatch focused flight campaigns
  (`trajectory_types`, `takeoff_velocities`) from `workflow_dispatch`. New
  `airstack up --config-only`: dry-run restricted to logical launch-config
  contracts (no Docker/credentials/image/submodule prerequisites).

- **CI un-redded: Metrics Report and Unit Tests fixed.** Every PR had been
  failing since ~2026-08-20 for reasons unrelated to the code under test.
  The system-tests **Metrics Report** job installed only `tabulate`, so
  `tests/parse_metrics.py` (which imports the `tests/harness` package)
  crashed with `ModuleNotFoundError: No module named 'yaml'` — and the crash
  was misreported as "Metric regression detected"; the job now installs
  `tests/requirements.txt`, and the regression verdict additionally requires
  a written `report.md` so a parser crash can never masquerade as a metric
  regression. The **Unit Tests** workflow ran the `tests/meta/` contract
  suite on a checkout with no submodules and no `omni_pass.env`, so every
  `airstack up --dry-run --sim isaac` contract hard-failed preflight and the
  docs-catalog contract missed the submodule-resident vdb_mapping_ros2
  README; the workflow now checks out submodules recursively and provisions
  the same guest `omni_pass.env` stub that `module-system-tests.yml` uses
  (preflight itself stays strict).

- **New reference stack `full_mighty` + registered `mighty` module.** The
  MIGHTY Hermite-spline local planner (MIT ACL, RA-L 2026) with its
  acl-mapping voxel world model and a NavigateTask/trajectory_controller
  bridge, packaged as the external
  [asm_mighty](https://github.com/castacks/asm_mighty) module (pinned at
  v0.1.1 in the stack's `modules.repos`; repo private until the AirStack
  agent study concludes). [`full_mighty`](../../stacks/full_mighty/README.md)
  is `full_default` with only the local-planner include swapped — the
  module-swap demonstration for the modular architecture. Registered in the
  [module catalog](../modules/index.md); validated on Isaac Sim (44/44
  vendored gtests, empty-world route flight, 7/7 pillar-field traversals,
  5/5 judged obstacle-route flights at 1.59–1.65 m min clearance vs a 1.0 m
  gate).

- Fixed: the docs search dropdown rendered behind the nav-tabs bar and the
  version-selector text (custom z-indexes inside the header's stacking
  context); the search subtree is now lifted above both.

**Documentation overhaul (Diátaxis restructuring).** The docs site was
audited against the [Diátaxis](https://diataxis.fr) framework and
reorganized; page URLs are preserved (moves are covered by redirects):

- The nav is now organized by document kind — **Tutorials / How-to Guides /
  Reference / Concepts** tabs — replacing the difficulty-tier
  ("Beginner/Intermediate/Advanced Tutorials") buckets, which contained no
  tutorials. The doc-authoring standards (Documentation Guide, mkdocs
  skills) now prescribe the quadrant taxonomy and include a decision tree.
- 18 verified doc/code mismatches fixed, including: a phantom
  `--recreate` flag and missing `--scene` in the CLI reference; phantom
  `ROBOT_LAUNCH_PACKAGE`/`ROBOT_LAUNCH_FILE` env vars (real: `LAUNCH_PACKAGE`);
  nonexistent `airstack_msgs/TrajectorySegment`/`TrajectoryOverride` types in
  doc templates (real: `TrajectoryXYZVYaw`); the MS-AirSim tmux window
  ordering (bridges launch before PX4) and MAVLink port math; the Getting
  Started Foxglove step (layout now auto-seeds; manual import retired).
- Removed superseded pages with redirects: the Ascent-era scene-setup pair,
  the pre-harness testing-frameworks page, the orphaned tutorials index and
  two stale duplicate pages (development_environment, airstack-cli index).
  The git-hooks docker-versioning READMEs are now deprecation notices — the
  hook they described conflicts with the semver `check-version-increment`
  gate (note: `airstack config git-hooks` still installs it; CLI removal is
  a follow-up).
- Pages that documented never-built or fabricated behavior were rewritten
  from the code: global planning (the unimplemented Global-Manager/
  PlanRequest protocol is gone), the robot interface page (state flows via
  `odometry_conversion`), robot configuration, HITL (now uses the
  `gcs-real` `hitl` profile and Foxglove verification), rosbags, and the
  robot-side data-offloading page (now points at the storage-tools
  workflow).
- Hybrid pages split by audience: `system_architecture.md` (explanation
  core; drifted topic tables replaced with links into the interface
  conventions spec), CI/CD (new **Using CI** how-to), GCS Foxglove (new
  **Extending the Visualizer**), Isaac Sim docker (new **Container
  Workflows**). Duplicated hot tables (topics, CLI flags, pytest marks,
  requirements) now live in one canonical home each.
- Previously off-site references added to the nav: the `vehicle.yaml` and
  `module.yaml` schemas, local calibration contract, the RViz tasks/waypoint
  panel manuals, the LiDAR point-cloud filter README, and the OSMO lab-admin
  guide. New reference pages: the complete `.env` schema, the
  `airstack_msgs` interface reference, the trajectory-library YAML format,
  and a supported-platform matrix. New/rewritten onboarding: Deploying to
  Hardware and Operating the GCS.
- The Interface Conventions Specification was bumped to **v1.0.1**: §8 now
  lists all eight `task_msgs` actions (added `tasks/coverage` and
  `tasks/chat`, both defined with no shipped executor).
- Six new how-to guides: Adding a State Estimator, Adding a Planner,
  Creating a Multi-Agent Coordination Algorithm (grouped under a new
  How-to → Autonomy section), Creating a Custom Stack Topology, Adding a
  Vehicle Type/Unit/Platform, and Getting the Most out of Your Coding
  Agent (the feature-notebook workflow). Each presents the in-tree
  package vs `airstack module create --in-tree` module-scaffolding
  choice. The Concepts tab now sits directly after Tutorials, and the
  UE→Isaac export tutorial was refreshed (new walkthrough video, export
  as Z-up in meters, note that UE Decals — paint markings, dirt,
  puddles — do not export).
- Five new beginner tutorials completing the learning path: Fly a Mission
  from the GCS, Change a Parameter (the edit→relaunch loop; config YAML is
  symlink-installed from the bind-mounted source, so no rebuild), Write
  Your First Module (`airstack module create --in-tree` scaffold, with a
  fix-it note for the scaffold's double-namespace stub), Your First Fleet
  (two-robot fleet file, per-robot Foxglove tabs), and Build and Fly Your
  Own Scene (GUI stage → scene catalog → `--scene` flight → baked
  `*.scene.usd`). New how-to: Adding a Controller (verified
  trajectory_controller → pid_controller → interface command chain);
  Adding a Planner expanded into Adding a World Model and Planner
  (local world-model/planner matched pairs vs the spec'd global map
  interchange).

This release restructures AirStack from a monolith into **modules**,
**stacks**, and **fleets**, implementing
[RFC #379 (Modular AirStack)](https://github.com/castacks/AirStack/discussions/379)
scales 1–2 of
[RFC #380 (Heterogeneous AirStack)](https://github.com/castacks/AirStack/discussions/380),
and the stack-folder anatomy of
[RFC #385 (Directory Atlas)](https://github.com/castacks/AirStack/discussions/385).
Feature docs deliberately cite none of these — the design sources live here:

- **Modules** are thin external repos with a small `module.yaml`, pulled on
  demand: `airstack module add <url> --version <pin>`. Three capabilities
  were extracted from trunk into new repos:
  [asm_macvo](https://github.com/castacks/asm_macvo) (with its torch/TensorRT
  stack moved out of the base robot image — 17.1 GB → ~6 GB, −65%),
  [asm_optitrack](https://github.com/castacks/asm_optitrack) (including the
  Isaac Sim NatNet emulator), and
  [asm_dfm2_disturbances](https://github.com/castacks/asm_dfm2_disturbances).
  The registry lives at
  [castacks/airstack-modules-index](https://github.com/castacks/airstack-modules-index).
- **Stacks** (`stacks/`) are self-contained topology folders — pinned
  `modules.repos`, plain XML entry launch files, and a CI-observed
  `wiring.md` graph baseline. Wiring that used to be spread across per-layer
  `*_bringup` packages (`local_bringup`, `onboard_all`, and the
  `local*.launch.xml` variant files) now lives in one place: the selected
  stack's entry launch file.
- Module provenance: asm_dfm2_disturbances originates from the DFM2
  ("don't fool me twice") AirStack fork, hand-built as the pilot module
  before the tooling existed (its FRICTION_LOG.md records every manual
  step; the port drops `omni.isaac.dynamic_control` in favor of the PhysX
  simulation interface). asm_optitrack was extracted from the trunk
  OptiTrack PR series (#359/#374/#375/#376) with git history preserved; its
  unit/integration/e2e tests run in the module's own CI.
- **Fleets** (`config/fleets/`) declare who exists, which vehicle, which
  stack, and which ground hosts run split-stack halves:
  `airstack up --fleet <name>`.

### Launch-path changes (breaking)

- **`AUTONOMY_ROLE` is removed.** The env-var role dispatch
  (`full`/`onboard`/`offboard`) is gone; a set `AUTONOMY_ROLE` is a preflight
  hard error (as is setting it alongside `--stack`, which previously let the
  stack silently win). Stacks are the only dispatch: `airstack up --stack
  <name>[:<entry>]`, defaulting to `full_default`. The per-role launch trees
  (`onboard_all/`, `onboard_local_offboard_global/`) are deleted, the
  wrap-then-flatten migration is complete (every reference stack composes
  flat module-launch includes; the launch-lint allowlist is down to the two
  deliberately wrapped blocks: `interface.launch.py` and the DDS-router/
  gossip helpers). Migration map:

    | Before | Now |
    |---|---|
    | no role set / `AUTONOMY_ROLE=full` | `full_default` (default; machine-proven graph-identical) |
    | `full` + `local_droan_cpu.launch.xml` variant | `--stack full_droan_cpu` (variant file deleted; wiring captured from it before deletion) |
    | `local_macvo_obstacle_avoidance.launch.xml` variant | `--stack full_macvo` — the variant was broken three ways (unprefixed args silently ignored, stale disparity topic, `launch_macvo` never enabled); the stack fixes all three |
    | `AUTONOMY_ROLE=onboard` (no split) | `--stack lite_default` (on the desktop profile the onboard role was unreachable anyway — `robot-desktop` hardcoded `full`) |
    | `onboard`/`offboard` split pair | `--stack lite_offload_global:onboard` / `:offboard` |
- The split onboard/offboard deployment is now the `lite_offload_global`
  stack (`:onboard` / `:offboard` entries) bridged per its `bridge.yaml`;
  the DDS-router config is generated by `tools/gen_dds_router.py` rather
  than hand-maintained. The generated bridge deliberately drops the legacy
  split's `set_trajectory_mode` crossing (control-mode topics may not cross
  a bridge — `airstack doctor` hard gate).
- Module launch files are remap-free and declare prefixed, described
  arguments with canonical defaults; generically named arguments (like
  `config_file`) were renamed with per-module prefixes because ROS 2 launch
  configurations are global across includes.
- Shared DDS-router configs moved out of the per-role `onboard_all/` tree up
  to `autonomy_bringup/config/`.
- **OptiTrack activation changed:** trunk's
  `overrides/isaac-optitrack-simulation.env` and the `LAUNCH_NATNET` toggle
  are gone (a set `LAUNCH_NATNET` draws a preflight warning). Mocap is
  brought up by a stack that includes `natnet_ros2` unconditionally — the
  asm_optitrack module's `test_stack/` is the reference.
- `SKIP_MACVO` / `SKIP_TENSORRT` build args removed from `Dockerfile.robot`
  along with the payload they gated; MAC-VO deps enter an image only via
  `airstack module lock --build`. `stacks/full_macvo` includes the module's
  own `macvo.launch.xml` (the in-tree `perception/macvo_ros2` copy is
  deleted; the module preserves its git history, and fixes the previously
  hardcoded `camera_info` subscription to honor its topic parameter).

### Added

- `airstack up --stack <name>[:<entry>]` stack dispatch with 5 reference
  stacks under `stacks/` (`full_default`, `full_droan_cpu`, `full_macvo`,
  `lite_default`, `lite_offload_global`), each carrying pinned
  `modules.repos` and a CI-observed `wiring.md` graph baseline
- Module CLI — `airstack module add <url> --version <tag|sha>` (branches
  refused; local paths allowed), `list|sync|remove|create --in-tree|doctor`,
  workspace overlay of module packages, and auto-generated module compose
  overrides included by `airstack up`; Docker module layers composed via
  `modules.lock` (`airstack module lock --build`)
- Fleet system: `airstack up --fleet <name>` driven by
  `config/fleets/*.yaml` (identity, vehicle from `config/vehicles/`, stack
  selection, spawns) with `hosts:` split-stack placement onto ground hosts;
  `airstack fleet list|generate` per-robot compose for heterogeneous fleets;
  `airstack sync` reconciles `airstack.yaml` (modules, external stack repos,
  fleet validation)
- `airstack doctor [--live|--snapshot] [--stack NAME]` — observe-and-report
  checks with exactly two hard gates (module dependency-conflict gate;
  bridge gate: no control-setpoint / trajectory-group topics may cross a
  split-stack bridge); `--live` diffs the RUNNING ROS graph against the
  stack's committed `wiring.md`
- `airstack stack list|new <src> <dst>|diff <a> <b>` (diff compares
  generated wiring, not launch XML)
- `tests/meta/` contract-test tier (unit mark) pinning the CLI/docs/stack
  contracts, plus the `wiring` system-test mark: an observed wiring snapshot
  of the running graph drift-checked against the stack's committed
  `stacks/<name>/wiring.md`
- Split-stack bridging: `stacks/lite_offload_global/bridge.yaml` explicitly
  lists every boundary crossing and `tools/gen_dds_router.py` generates the
  DDS-router config from it deterministically (`--check` enforces the bridge
  hard gate)
- New docs: Modules, Stacks, Fleets, Module CI guides, the generated
  Module & Stack Catalog marketplace, and the Modular AirStack Walkthrough
- Intent flags on `airstack up` — `--sim isaac|airsim|simple`, `--robots N`,
  `--headless`, `--play`/`--no-play`, `--no-autolaunch`, `--wait`,
  `--dry-run` — deriving the coordinated env-var sets as exported leaf
  values, with a resolved-config banner and a per-run
  `.airstack/runs/<ts>/effective_config.env` dump; contract-tested
- `airstack up --scene <shortname>`: simulator-agnostic scene selection via
  a new catalog (`simulation/scenes.yaml`, resolved host-side by
  `simulation/resolve_scene.py`). Isaac maps shortnames to Pegasus catalog
  keys or Nucleus USD URLs (exported as `ISAAC_SIM_SCENE` +
  `ISAAC_SIM_STAGE_SCALE`; the example launch scripts and `fleet_spawn.py`
  now resolve their scene from these instead of hardcoding `env_url`);
  MS AirSim maps to `fetch_scene.sh` keys (`MS_AIRSIM_SCENE` — the
  entrypoint's auto-fetch, previously Blocks-only, now fetches any catalog
  scene, and an interactive `airstack up` asks before a multi-GB download).
  The `fetch_scene.sh` catalog was corrected to the UE4 binaries that
  actually exist in the AirSim v1.8.1 release: `forest`, `soccerfield`, and
  `building99` were removed (their zips were never published, or are empty),
  `airsimnh` now downloads `AirSimNH.zip` (was the nonexistent
  `Neighborhood.zip`), and `africasavannah` / `msbuild2018` were added; the
  entrypoint resolves the UE launcher by glob after extraction, so zips whose
  inner `.sh` doesn't match the folder name still boot.
  Unknown scene = error + per-simulator availability table. Five stages were
  published to the guest-readable
  `omniverse://airlab-nucleus.andrew.cmu.edu:443/Public/AirStack/Stages/`
  (AbandonedFactory, AbandonedWarehouse day/night, ChemicalPlant,
  ConstructionSite, RetroNeighborhood) and cataloged with per-stage scale.
  See [Simulation Scenes](../simulation/scenes.md)
- `airstack ready` (and `airstack up --wait`): staged flight-readiness gates
  mirroring the system-test budgets — containers → sim `/clock` → per-robot
  sentinel nodes → PX4 MAVROS-connected + `local_position/odom` streaming —
  with per-gate diagnostics and `--json` for scripts
- Preflight validation in `airstack up` on resolved configuration (env >
  `--env-file` > `.env`): one-simulator guard, `NUM_ROBOTS>1` vs
  single-drone Isaac script as a named hard error, missing images listed
  with an `image-pull` hint, missing `omni_pass.env` / empty Pegasus
  submodule / Docker < 29 surfaced on the host
  (`AIRSTACK_SKIP_PREFLIGHT=1` downgrades errors to warnings)
- tmux pane output mirrored to container stdout via shared `.tmux.conf`
  hooks, so `docker logs` / `airstack logs` show colcon builds,
  `ros2 launch` output, sim loading, and crashes
- simple-sim as a first-class simulator: `airstack up --sim simple` and a
  `simple_sim` smoke-test mark (it had been broken since the ROS Jazzy
  migration — its container sourced a Humble path — and is fixed)
- Automatic `unit-tests.yml` PR gate on `ubuntu-latest`, plus
  `run_meta.json` outcome metadata so reports distinguish completed
  simulation campaigns from collection errors, empty selections, timeouts,
  and cancellations
- Feature notebook workflow (`use-feature-notebook` skill): gitignored
  `notebook/NNN-feature-slug/` entries whose `design_spec.md` and
  `results_summary.md` populate feature PR descriptions
- Battery and telemetry display in the GCS control panel (voltage and
  percentage per robot when the MAVROS battery topic is bridged)
- `TARGET_ARCH` build arg (default `x86_64`) in `Dockerfile.robot`;
  `docker-compose.yaml` passes `TARGET_ARCH: aarch64` to the `voxl` and
  `l4t` real-robot image builds
- `ros-${ROS_DISTRO}-mavros-extras` in the robot image (provides the
  vision_pose plugin used for external-pose deployments)
- `overrides/l4t-px4-realrobot.env` — site-agnostic deployment override for
  a single real PX4 robot on a Jetson (aarch64/l4t)
- `integration` test tier (`tests/integration/`, `integration` mark) with a
  shared `robot_autonomy_stack` fixture (robot container, no sim/GPU)
- `waypoint_flight` system test: takeoff → ordered waypoint route via
  `NavigateTask` → land, judged on the odometry track by the standalone
  `tests/waypoint_checker.py`; the standard acceptance check after
  integrating or swapping a planner module
- Foxglove auto-loaded layout: `render_layout.py` seeds the rendered
  `NUM_ROBOTS`-matched layout directly into the Foxglove desktop app's
  local layout store and `gcs.launch.xml` selects it via the `layoutId`
  deep link, so Foxglove opens on the right layout with no manual
  **Import from file...** step; user-saved edits are hash-detected and
  never overwritten (delete the layout in the UI to reset). Requires the
  pinned Foxglove version — see Changed

### Changed

- **The simulator now starts playing by default**: `PLAY_SIM_ON_START`
  defaults to `true` in `.env` (was `false`). Pass `airstack up --no-play`
  to come up paused and press Play yourself; `--play` remains available as
  an explicit override
- Container autolaunch (robot desktop/voxl/l4t and GCS) now runs through an
  `autolaunch` shell helper instead of a bare `bws && sws && ros2 launch`
  chain: a build failure or launch crash prints an unmissable red
  "AUTOLAUNCH FAILED" banner in the tmux pane (mirrored to `docker logs` /
  `airstack logs`) instead of silently returning to a prompt with the stack
  down
- `Dockerfile.gcs` pins the Foxglove desktop version (`FOXGLOVE_VERSION`
  build arg, currently 3.0.0) instead of installing `latest`, since the
  layout auto-load writes the app's on-disk local-layout record format
  directly (verified against 3.0.0; re-verify before bumping the pin)
- `robot-desktop` image slimmed 17.1 GB → ~6 GB (−65%) by moving MACVO's
  torch/TensorRT/weights into the `asm_macvo` module Docker layer; a further
  dependency purge removed unused apt/pip packages (−152 MB) and `droan_gl`'s
  GL dependencies are declared explicitly
- Isaac launch scripts deduplicated onto a shared `pegasus_app.PegasusApp`
  base: the scripts become scenario declarations (~40–170 lines each, net
  −438 lines) with hooks for NatNet/scene-import extras; behavior verified
  by full system-test parity. `ISAAC_SIM_HEADLESS` and
  `ISAAC_SIM_LIVESTREAM` work uniformly in every launch script (each was
  honored by only half of them before)
- Launch-workflow docs corrected against actual behavior: `ISAAC_SIM_SCENE`
  (nonexistent) replaced by `ISAAC_SIM_SCRIPT_NAME`/`ISAAC_SIM_GUI`,
  getting-started reflects the paused-by-default sim and Foxglove UI, isaac
  docker.md defaults match `.env`, ms-airsim MAVROS ports/FOV/vehicle naming
  fixed
- Unit-test documentation matches the co-located layout: C++ gtests run via
  `colcon test` under the `build_packages` mark; Python via the root harness
  (`conftest.py` applies the `unit` mark by file location)
- The CI orchestrator polls a `repos:` list (one instance covers trunk and
  every asm_* module repo; the singular `repo:` key still works) — module CI
  jobs on `airstack-ephemeral` no longer need per-repo orchestrator instances
- Ephemeral CI GPU runners spawn via NVIDIA OSMO as a drop-in replacement
  for the earlier OpenStack-Nova backend: the GitHub side (labels, JIT
  tokens, fork guard) is unchanged; only the spawn target moved. The OSMO
  service-account token plays the old application-credential role,
  `osmo workflow exec` replaces SSH-via-floating-IP debugging, and the
  runner image prebakes what cloud-init used to install at boot
- Pegasus launch scripts drive lidar through the RTX OmniLidar API
  (`add_rtx_lidar_subgraph`) in place of the Ouster graph path, with ROS
  topics reconciled (raw cloud on `…/sensors/ouster/point_cloud_raw`,
  filtered on `…/sensors/ouster/point_cloud`)
- The `airstack-osmo` SSH config block for OSMO IDE sessions is
  `StrictHostKeyChecking no` + `UserKnownHostsFile /dev/null` (replacing
  `accept-new`); users with the earlier block should replace it and run
  `ssh-keygen -R "[localhost]:2200"` once
- Default system-test `--sim` is `isaacsim`; pass `--sim msairsim` to opt in
- `-m build_packages` CI runs pull `cache_*` images instead of baking sim
  images; `docker-build.yml` retags unchanged images on VERSION bumps
  (content fingerprint) instead of always rebuilding
- Automatic OSMO validation runs the pull-only `build_packages` gate on
  every PR update; GPU simulation campaigns are selected through `/pytest`
  or `workflow_dispatch`
- `robot-l4t` compose service knobs are env-overridable (`FCU_URL`, rosbag
  path via `BAG_STORAGE_PATH`); `FCU_URL` unquoted so the literal serial
  path reaches MAVROS
- `zed-l4t` image: ZED SDK 4.2 → 5.2 with coupled ROS deps (`zed_msgs`
  5.2.1, `point_cloud_transport(_plugins)` 4.x, `backward_ros`)
- Unit tests are defined by `tests/colcon_unit_test_packages.yaml`
- Repo-wide relicense to **BSD 3-Clause Clear** (vendored packages keep
  their upstream licenses); `airstack_msgs` stabilized at 1.0.0; every
  package.xml carries a real maintainer and description (contract-tested)
- `DOCKER_IMAGE_BUILD_MODE=prebuilt` is a tag discriminator only; a real
  prebuilt-workspace image stage is future work
- The repository CHANGELOG.md is removed in favor of this page — all
  change records live here, per version
- **Docs policy: standalone snapshots.** Feature docs describe only the
  current system; all change-relative language (including RFC citations)
  coalesces here. A full docs audit applied the policy and corrected pages
  that had drifted from the code, notably: bag recording is NOT
  auto-triggered at takeoff (the recorder starts idle; toggle via
  `/{robot_name}/bag_record/set_recording_status`); `tracking_point` /
  `look_ahead` carry `airstack_msgs/msg/Odometry` (older docs said
  `geometry_msgs/PointStamped`) and trajectory topics are
  `airstack_msgs/TrajectoryXYZVYaw`; MAVROSInterface targets any
  MAVLink-compatible FC (the documented Ascent/Ardupilot specificity does
  not exist in code); the RobotInterface command topic is `cmd_pose`; the
  DDS-router allowlist table is regenerated from the real config; the
  Jetson install flow is `./airstack.sh setup` + the `robot-l4t` service.
  Two orphaned pages documenting the behavior-tree framework
  (`behavior_tree`, `behavior_executive` — packages removed in an earlier
  release, PR #332; only `behavior_tree_msgs` remains) were deleted; the
  behavior layer is `drone_safety_monitor` with mission sequencing via
  GCS-sent task goals

### Removed

An audit removed dead or superseded code wholesale. Anything here is
recoverable from git history, and hardware-specific capabilities return as
out-of-trunk modules:

- The `AUTONOMY_ROLE` launch dispatch and its per-role launch trees —
  stacks are the only launch path
- **MACVO extracted to** [castacks/asm_macvo](https://github.com/castacks/asm_macvo);
  **OptiTrack/NatNet extracted to**
  [castacks/asm_optitrack](https://github.com/castacks/asm_optitrack)
  (client, PX4 external-vision fusion, NatNet emulator + Isaac wrapper,
  e2e/integration tests, env overrides)
- `px4_interface` + vendored `px4_msgs` — a native PX4 uXRCE-DDS interface
  is tracked as a fresh design in
  [#387](https://github.com/castacks/AirStack/issues/387); MAVROS remains
  the flight interface
- `waypoint_interface`, `attitude_controller`(+`_msgs`) — dead code
- The RQT/RViz GUI set: `rviz_behavior_tree_panel` (with the `xdot_cpp`
  submodule), `rqt_behavior_tree_command`, `rqt_behavior_tree`, `rqt_gcs`,
  `rqt_airstack_control_panel` — Foxglove is the GCS surface
- Sensors-layer hardware packages `camera_param_server`,
  `gimbal_stabilizer`, `sensor_interfaces`
  (`lidar_point_cloud_filter` remains)
- WinTAK / TAK integration (`ros2tak_tools`, CLI plumbing, GCS image
  dependencies) — can return as a module
- Isaac Sim `standalone_examples` copies, stale robot-docker helpers, the
  `ensemble_planner` skeleton, the Gazebo parallel-bringup tree, and
  pre-co-location test scaffolding
- trajectory_library's vestigial rqt selector (catkin-era GUI source,
  `plugin.xml`, `setup.py`, launcher script — never installed by its
  CMakeLists), the orphaned behavior-tree docs images, and an empty
  integration-testing stub page
- `tests/goldens/wiring/` — wiring baselines live per-stack as
  `stacks/<name>/wiring.md`

### Fixed

- `airstack up` guards (one-simulator, URDF pairing) validated `.env` only
  and were bypassed by `--env-file`; they now check the resolved
  configuration
- `pytest tests/` collects the co-located unit tests before mark filtering
  (CI previously collected 97 of 252 items, so the Python unit tests ran
  nowhere); empty CI pytest arguments no longer recurse the repository;
  non-comparable artifacts are reported instead of false 0% results
- `barebones_pegasus_launch.py` crashed with `NameError: os`;
  `isaac-sim-livestream` produced a black stream with multi-drone scripts;
  `NATNET_BODY_NAME`/`NATNET_TARGET_NAME` overrides now work
- Isaac Sim image: PX4 `ubuntu.sh` no longer fails dpkg configure on the
  NVIDIA base; robot image pins `pytest<8.1` and disables `launch_testing`
  for colcon unit tests
- Robot identity: a pre-set `ROBOT_NAME` is honored; the name-map catch-all
  maps to `unknown_robot` (valid ROS namespace token) and logs a warning
  naming both fixes; inert `ROBOT_NAME`/`ROS_DOMAIN_ID` lines dropped from
  `overrides/l4t-px4-realrobot.env`
- l4t robot image: dustynv's `/ros_entrypoint.sh` replaced with a
  passthrough so stale prebuilt `fastcdr` libs no longer crash apt-built
  nodes like MAVROS; the GeographicLib `egm96-5` geoid is asserted at build
  time (MAVROS dies at startup without it)
- Bag recording: `RECORD_BAGS=true` now actually starts the recorder on a
  robot; recorder status is bridged in the correct direction so GCS
  indicators work; `ros2 bag record --exclude` updated for Jazzy's
  `--exclude-regex` (multiple excludes alternated into one regex)
- OptiTrack/NatNet: `NATNET_SERVER_IP` is forwarded to the robot container;
  the default tracked body matches the emulator; EKF2 external-vision
  parameters are passed as `PX4_PARAM_*` so PX4 actually fuses mocap;
  external-vision tuning corrected from real-flight bags (`EKF2_EV_DELAY`
  7.0, `EKF2_EVP_NOISE` 0.05); an unrecognized `connection_type` fails at
  startup instead of silently falling back; MODELDEF drone-body count is
  cross-checked against `NUM_ROBOTS` after the handshake; the emulator is
  installed as a Kit extension so launch scripts can import it

### Landing page & simulation camera/lighting (alpha.13)

- **Docs landing page redesigned** around four demonstrated pillars
  (one-command bring-up, sim-to-vehicle code parity, full-stack CI, agent
  readiness): real quickstart commands with copy buttons, the actual
  `airstack ready` checklist, a live test-mark matrix, and a fresh
  autoplay hero video (Isaac full-warehouse and office, Foxglove's live
  3-robot trajectory traces, MS AirSim neighborhood and ZhangJiajie) —
  40 s / 5.4 MB, replacing the 37 MB splash GIF. Per-simulator and Foxglove
  clips are embedded in their docs sections, and the architecture image is
  now an unedited full-system desktop capture (sim + GCS + CLI, live).
- **Isaac launch scripts grew camera/spawn/lighting env knobs**
  (`pegasus_app.py`, plumbed through the isaac-sim compose service):
  `ISAAC_SIM_FOLLOW_CAM` / `_OFFSET` — a smoothed viewport chase camera
  tracking a drone via live Pegasus vehicle state (also fixes the black
  viewport at spawn on cm-authored stages; on by default, `off` disables);
  `ISAAC_SIM_SPAWN_XY` — recenter the spawn row away from a cluttered scene
  origin; `ISAAC_SIM_LIGHT_BOOST` — multiply the scene's own lights
  (de-instances light-bearing subtrees first, so e.g. the NVIDIA office's
  121 instanceable ceiling lights become editable);
  `ISAAC_SIM_FOLLOW_CAM_LIGHT` — headlight riding the follow camera;
  `ISAAC_SIM_DOME_LIGHT` — dome-light intensity/exposure override.

## 0.19.0 — 2026-08-22

The launch-workflow and CI-infrastructure release preceding the modular
transition.

### Added

- Intent flags on `airstack up` — `--sim isaac|airsim`, `--robots N`, `--headless`, `--play`/`--no-play`, `--no-autolaunch`, `--wait`, `--dry-run` — deriving the coordinated env-var sets (compose profiles, URDF, single/multi Isaac launch script) as exported leaf values, with a resolved-config banner and a per-run `.airstack/runs/<ts>/effective_config.env` dump; contract-tested in `tests/meta/test_launch_intent_contract.py` (unit mark)
- `airstack ready` (and `airstack up --wait`): staged flight-readiness gates mirroring the system-test budgets — containers → sim `/clock` → per-robot sentinel nodes → PX4 MAVROS-connected + `local_position/odom` streaming (the armable signal) — with per-gate diagnostics and `--json` for scripts
- Preflight validation in `airstack up` on **resolved** configuration (env > `--env-file` > `.env`): one-simulator guard no longer bypassed by `--env-file`; `NUM_ROBOTS>1` with the single-drone Isaac script is a named hard error; missing images are listed with an `image-pull` hint before compose starts an implicit build; missing `omni_pass.env` / empty Pegasus submodule / Docker < 29 surfaced on the host (`AIRSTACK_SKIP_PREFLIGHT=1` downgrades errors to warnings)
- tmux pane output is mirrored to container stdout via shared `.tmux.conf` hooks, so `docker logs` / `airstack logs` now show colcon builds, `ros2 launch` output, sim loading, and crashes
- Automatic `unit-tests.yml` PR gate on `ubuntu-latest`, plus `run_meta.json` outcome metadata so reports distinguish completed simulation campaigns from collection errors, empty selections, timeouts, and cancellations
- `overrides/isaac-optitrack-simulation.env` — brings up Isaac Sim with the NatNet emulator and PX4 flying on mocap EKF2 external vision (GPS/baro/range aiding off), i.e. the configuration `tests/system/test_optitrack_e2e.py` runs, reproducible by hand
- `overrides/l4t-optitrack-realrobot.env` — deployment override for a real Jetson robot flying on OptiTrack mocap (PX4 EKF2 external vision instead of GPS): the NatNet server/body settings, plus the multi-NIC and FCU-parameter notes that path needs
- Feature notebook workflow (`use-feature-notebook` skill): every agent-implemented feature gets a local, gitignored `notebook/NNN-feature-slug/` entry with a status-tracked `design_spec.md` (written before coding) and `results/` artifacts + self-contained `results_summary.md` that populate the feature's PR description
- Battery and telemetry display in GCS RQT control panel (voltage and percentage per robot when MAVROS battery topic is bridged)
- `TARGET_ARCH` build arg (default `x86_64`) in `Dockerfile.robot` to arch-parametrize `LD_LIBRARY_PATH`; `docker-compose.yaml` passes `TARGET_ARCH: aarch64` to the `voxl` and `l4t` real-robot image builds
- `ros-${ROS_DISTRO}-mavros-extras` in the robot image (provides the vision_pose plugin used for external-pose deployments)
- `overrides/l4t-px4-realrobot.env` — site-agnostic deployment override for a single real PX4 robot on a Jetson (aarch64/l4t)
- `integration` test tier (`tests/integration/`, `integration` mark) with a shared `robot_autonomy_stack` fixture (robot container, no sim/GPU)
- `waypoint_flight` system test (`tests/system/test_waypoint_flight.py`): takeoff → ordered waypoint route via `NavigateTask` (dispatched as a dense plan) → land, judged on the odometry track by the standalone stdlib-only `tests/waypoint_checker.py` (in-order corridor arrival within `--waypoint-tolerance`, final goal within `--goal-tolerance`, per-waypoint `--waypoint-timeout`); validated end-to-end in Isaac Sim; serves as the standard acceptance check after integrating or swapping a planner module
- Real-robot PX4 external-vision fusion in `natnet_ros2` (OptiTrack mocap → EKF2): `mavros_gp_origin` (geoid-corrected synthetic GPS origin so `local_position.z` == OptiTrack z, fixing the ~36 m boot offset), `vision_pose_converter`, and a PX4 param **checker** (`px4_param_setter`, `auto_set` off by default; `on_mismatch` warn/halt) — setup guide at `docs/robot/px4_external_vision.md`
- NatNet server emulator (`optitrack.natnet.emulator`, protocol core) — pure-Python OptiTrack Motive server emulation so `natnet_ros2` can be driven without hardware; host integration tests (`tests/integration/natnet/`) wire it to the robot client
- Isaac wrapper for the NatNet emulator (USD scene → server) + natnet Pegasus launch scripts, and a dedicated OptiTrack sim e2e test (`optitrack` mark, `tests/system/test_optitrack_e2e.py`) that flies a **Circle trajectory on mocap EKF2 fusion** — GPS, baro and range aiding are disabled for the run, so the OptiTrack stream is the vehicle's only position source and cross-track error scores the whole chain

### Changed

- Isaac launch scripts deduplicated onto a shared `pegasus_app.PegasusApp` base (`simulation/isaac-sim/launch_scripts/pegasus_app.py`): the six scripts become scenario declarations (~40–170 lines each, net −438 lines) with hooks for NatNet/scene-import extras; behavior verified by full system-test parity (liveliness, sensors, takeoff/hover/land on Isaac). `ISAAC_SIM_HEADLESS` and `ISAAC_SIM_LIVESTREAM` now work uniformly in **every** launch script (previously each was honored by only half of them)
- Launch-workflow docs corrected against actual behavior: `ISAAC_SIM_SCENE` (nonexistent) replaced by `ISAAC_SIM_SCRIPT_NAME`/`ISAAC_SIM_GUI`, getting-started reflects the paused-by-default sim and Foxglove UI, isaac docker.md defaults table matches `.env`, ms-airsim MAVROS ports/FOV/vehicle naming fixed, AGENTS.md uses the real `down`/`image-build` command names
- Unit-test documentation now matches the co-located layout: the `add-unit-tests` and `run-system-tests` skills and the testing docs record which runner each language uses (C++ gtests via `colcon test` under the `build_packages` mark; Python via the root harness, plus `colcon test` for `ament_python` packages), and stop instructing authors to write `@pytest.mark.unit` by hand — `conftest.py` applies it by file location
- Ephemeral CI GPU runners spawn via NVIDIA OSMO (not OpenStack); `system-tests.yml` / `docker-build.yml` still use `airstack-ephemeral`
- Default system-test `--sim` is `isaacsim`; pass `--sim msairsim` to opt in to Microsoft AirSim
- `-m build_packages` CI runs pull `cache_*` images instead of baking sim images
- `docker-build.yml` retags unchanged images on VERSION bumps (content fingerprint) instead of always rebuilding; floating `cache_*` tags still seed PR layer cache
- Automatic OSMO validation runs the pull-only `build_packages` gate whenever a PR is opened, updated, or reopened; GPU-intensive simulation campaigns (including OptiTrack) are selected through `/pytest` or `workflow_dispatch`
- `robot-l4t` compose service knobs are now env-overridable (`AUTONOMY_ROLE`, `FCU_URL`, and the rosbag path via `BAG_STORAGE_PATH`); `FCU_URL` unquoted so the literal serial path reaches MAVROS
- `zed-l4t` image: ZED SDK 4.2 → 5.2 with the coupled ROS deps (`zed_msgs` 5.2.1, `point_cloud_transport(_plugins)` 4.x, add `backward_ros`)
- Unit tests are defined by `tests/colcon_unit_test_packages.yaml`: `conftest.py` collects each listed package's co-located `test/` dir under `--import-mode=importlib` and marks it `unit` (ament lint files are skipped and run under `colcon test`)

### Removed

- Pre-co-location unit-test scaffolding: the six per-layer stub READMEs under `tests/robot/` (which instructed authors to add tests in directories tests no longer live in) and `tests/sim/motive_emulator/README.md` (superseded by `simulation/isaac-sim/extensions/optitrack.natnet.emulator/` and `tests/integration/natnet/`)

### Fixed

- `barebones_pegasus_launch.py` (the documented template script) crashed with `NameError: os` on construction
- `isaac-sim-livestream` compose service silently produced a black stream when `ISAAC_SIM_SCRIPT_NAME` was a multi-drone script (livestream setup existed only in the single-drone scripts)
- `NATNET_BODY_NAME`/`NATNET_TARGET_NAME` env overrides documented by the single-drone NatNet script now actually work
- `airstack up` guards (one-simulator, URDF pairing) validated `.env` only and were bypassed by `--env-file overrides/...`; they now check the resolved configuration
- `pytest tests/` now collects the co-located unit tests before mark filtering. The old guard skipped injection whenever any path was on the command line, and `tests/` is a path — CI collected 97 of 252 items and the Python unit tests ran nowhere. Narrowing (`pytest tests/system/test_x.py`) still skips injection; repository-root and empty-path collection are rejected
- Empty CI pytest arguments no longer become `pytest tests/ ""` and recurse through the repository; collection/import, setup/teardown, partial, and interrupted artifacts are reported as non-comparable instead of false 0% simulation-policy results, and metric regression runs only for an identical simulation campaign fingerprint
- Isaac Sim image: PX4 `ubuntu.sh` no longer fails dpkg configure on the NVIDIA base (`ca-certificates` / `software-properties-common`); use `--no-nuttx --no-sim-tools` like ms-airsim
- Robot image: pin `pytest<8.1` and disable `launch_testing` for colcon unit tests so ROS Jazzy's outdated pytest hook does not abort `colcon test`
- Robot name resolution now honors a pre-set `ROBOT_NAME` (e.g. injected via docker compose) instead of always overriding it from the container/hostname mapping (`robot/docker/.bashrc`)
- Robot name-map catch-all fallback now maps to `unknown_robot` (valid ROS namespace token) instead of `unknown-robot` (`default_robot_name_map.yaml`)
- l4t robot image: replace dustynv's `/ros_entrypoint.sh` with a passthrough so its prebuilt source-ROS libs (older `fastcdr`) no longer shadow the apt Jazzy runtime and crash apt-built nodes like MAVROS
- `RECORD_BAGS=true` never brought the bag recorder up on a robot: `logging.launch.xml` hardcoded `record_bag=false` and `onboard_autonomy_all.launch.xml` includes it with no arguments, so the variable was forwarded into the container and read by nobody (only `gcs.launch.xml` consumed it). With no `bag_record` node running, the GCS control panel's `set_recording_status` toggle had nothing to reach despite being bridged in `domain_bridge.yaml` / `dds_router.yaml`. It now reads `RECORD_BAGS` and selects its topic set via `LOG_CONFIG`
- Falling back to `unknown_robot` / domain 0 now logs a warning naming both fixes (rename the device `robot-<n>` on the host, or supply a `ROBOT_NAME_MAP_CONFIG_FILE` matching your hostnames). The fallback itself is unchanged — it deliberately keeps an unidentified robot out of every real robot's namespace — but it used to resolve silently, so the symptoms surfaced far from the cause
- Dropped `ROBOT_NAME` / `ROS_DOMAIN_ID` from `overrides/l4t-px4-realrobot.env`: no compose service declares either, so an env file could never set them and the lines were inert
- `bag_record/bag_recording_status` was bridged GCS -> robot in `domain_bridge.yaml`, the same direction as the command it answers, so recorder status never reached the GCS and every recording indicator stayed blank
- `bag_record_node` passed `--exclude` to `ros2 bag record`, which Jazzy renamed to `--exclude-regex`. It is now an ambiguous prefix of four options, so argparse rejected the command and any section using `exclude:` (including `log.yaml`'s `airstack` section, i.e. everything but the cameras) recorded nothing — surfacing only as a usage dump in the node's stdout. Multiple `exclude:` entries are now alternated into one regex instead of repeating a single-valued flag, which had silently kept only the last
- `natnet_config.yaml`'s `$(env NATNET_SERVER_IP ...)` could never resolve: no compose service declared the variable, so the NatNet client always fell back to its hardcoded default and could reach neither the in-sim emulator nor a real Motive host. It is now forwarded in `robot-base-docker-compose.yaml`, defaulting to the in-sim emulator
- The NatNet rigid body tracked by `robot_1` defaulted to a site-specific body (id 1146) that no emulator streams; since the client filters frames by numeric id, that produced a connected client that never published. It now defaults to the emulator's body (`Drone`, id 1). Per-robot bodies are configured in each robot's profile in `natnet_config.yaml`, selected by `ROBOT_NAME`
- OptiTrack external-vision tuning corrected from real-flight bags: `EKF2_EV_DELAY` 8.0 → 7.0 and `EKF2_EVP_NOISE` 0.01 → 0.05. The old 0.01 gave a 5 cm innovation gate (`EKF2_EVP_GATE` × 5σ) that rejected valid mocap updates and blocked arming; `px4_params.yaml` now records the supporting measurements and the drift-and-snap misdiagnosis so neither is repeated
- The synthetic GPS origin now places the mocap floor at the shared world datum (`desired_floor_amsl: 36.0`, i.e. 90 m ellipsoidal in AMSL) rather than at sea level, so a mocap robot's reported global altitude agrees with sim and the GCS. `local_position.z` still equals the OptiTrack height either way
- The robot image could ship without the GeographicLib `egm96-5` geoid: mavros' `install_geographiclib_datasets.sh` swallows a failed download and still exits 0, so the `RUN` layer succeeded either way, and `geographiclib-tools` was only ever a transitive dependency. MAVROS builds that geoid in its UAS core before any plugin loads and throws if it is missing, so `mavros_node` died at startup on affected images. `Dockerfile.robot` now pins the tool and asserts the file exists, failing the build instead
- An unrecognised `connection_type` in `natnet_config.yaml` silently fell back to `unicast`, so a typo produced a client that connected on the wrong transport and never received frames. `validate_connection_type` now throws and `natnet_ros2_node` fails at startup naming the offending value
- `natnet_ros2_node` on `robot_1` now compares the NatNet server's MODELDEF drone-body count (`Drone` / `Drone1`…`DroneN`, excluding `Target` and skeleton bones) against `NUM_ROBOTS` after the handshake and logs an error on mismatch, so a sim launch script and `natnet_config.yaml` that disagree about how many drones exist is caught at startup rather than as a robot that silently never receives frames. `NUM_ROBOTS` is forwarded into the robot container for it
- Isaac Sim PX4 never fused the mocap stream: `EKF2_EV_CTRL` defaults to 0 and the isaac compose set no PX4 parameters, so the emulator could stream perfectly while PX4 flew on sim GPS. The compose now passes the EKF2 external-vision set as `PX4_PARAM_*` (applied by PX4 SITL's `rcS` at boot), each defaulting to PX4's own default so non-mocap sims are unaffected; the mocap path opts in
- The NatNet emulator hardcoded the drone's streaming id to 1 while the client reads `NATNET_BODY_ID`, so a real Motive id desynced the two into a connected client that never published (`example_one_px4_pegasus_natnet_launch_script.py`)
- `test_optitrack_e2e.py::test_px4_fuses_vision` asserted only that `local_position/pose` publishes, which it does off GPS — the check passed with external vision disabled. It is now the pre-flight gate (an estimate exists) and the Circle flight is the actual proof of fusion
- The NatNet emulator is now installed as a Kit extension: `Dockerfile.isaac-ros` pip-installs it editable into the Isaac python and bind-mounts the repo copy over it (the same pattern as `pegasus.simulator`), and the natnet launch scripts `enable_extension` it before importing. Being on a Kit `--ext-folder` search path only makes Kit *aware* of an extension — it does not put the package on `sys.path` — so the scripts previously died with `ModuleNotFoundError: No module named 'optitrack'`

