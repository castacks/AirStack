# Design Spec: RFC #379/#380 Campaign — Modular & Heterogeneous AirStack

> Notebook entry: `notebook/002-rfc-modular-airstack/` · Date started: 2026-08-20 · Last updated: 2026-08-20 · Branch: `feature/rfc-modular-airstack`
>
> **Status: `WIP`** <!-- All implementation sections DONE (2026-08-21); remaining: release mechanics (per-phase VERSION bumps + stacked PRs — construction pending owner's force-push decision), module GitHub CI + v0.1.0 tags (needs merge + orchestrator poll-list), P3 dispatch smoke (post-merge). -->

## 1. Problem Context

Implements GitHub Discussions [#379 (Modular AirStack)](https://github.com/castacks/AirStack/discussions/379), [#380 (Heterogeneous AirStack)](https://github.com/castacks/AirStack/discussions/380) scales 1–2, and the layouts in [#385 (Directory Atlas)](https://github.com/castacks/AirStack/discussions/385). Today the only paths for optional capabilities are merging into the monolith or living in a whole-repo fork. Concrete pain: the `local_*.launch.xml` variant explosion (near-copies diverging in one node block; the MACVO variant is outright broken — wrong include arg names), the hardcoded global planner in `global.launch.xml`, `NUM_ROBOTS`/`AUTONOMY_ROLE`/`URDF_FILE`/`ISAAC_SIM_SCRIPT_NAME` env blur, MACVO's ~8–10 GB of deps baked into every robot-desktop image behind `SKIP_MACVO`, and the DFM2 disturbance library stranded in a stale fork.

Owner decisions (2026-08-20, this session):

- Scope: #379 Phases 1–3 + core of Phase 4 (registry, docs catalog, skills). Deferred: nightly canary, external dispatch test bench, org runner group, #380 Part 2 (cross-embodiment).
- Extract three modules to new castacks repos with the `asm_` prefix: `asm_dfm2_disturbances`, `asm_optitrack`, `asm_macvo`. Registry repo: `castacks/airstack-modules-index`.
- All trunk work on `feature/rfc-modular-airstack`; delivered as stacked phase PRs into `develop`, each with its own VERSION bump + CHANGELOG entry.
- Keep mkdocs current every phase; validate with unit/contract tests + local GPU system tests (RTX 5090, both sims).

Design constraints already decided in the RFCs (do not re-litigate): topology lives in plain XML launch files (no slot taxonomy, no wiring resolver — see #379's design-history comment); manifests carry deps/identity only, never wiring; checks observe rather than generate (`wiring.md` is snapshotted from the running graph); `doctor` hard-errors in exactly two places (dep conflicts at compose; control/trajectory topics in a `bridge.yaml`); pinned `.repos` only, never branch refs.

Full campaign plan (sequencing, risks, verification gates): `/home/andrew/.claude/plans/cuddly-imagining-lerdorf.md` (session artifact; key content mirrored here).

## 2. Proposed Implementation

Trunk baseline: `develop` @ `262f1267`, VERSION `0.19.0-alpha.18`. Coexistence strategy: **wrap-then-flatten** — stacks first wrap existing layer bringups (capturing a `wiring.md` baseline), then flatten with graph-equality as the merge gate. Legacy `AUTONOMY_ROLE` path stays intact until 0.21 (post-campaign).

### 2.1 P0 — Wiring snapshot tooling — `DONE` <!-- 5c6c4a41 + b2364b96, 2026-08-20; test plan (a) PASSED: liveliness+wiring green on isaacsim, golden blessed, drift gate verified on independent bring-up -->


`tests/wiring_snapshot.py` (stdlib-only, mirrors `waypoint_checker.py`): parse `ros2 node list` + batched `ros2 topic info --verbose`, render mermaid, diff observed-vs-committed with JSON verdict. `tests/system/test_wiring_snapshot.py` with new `wiring` mark (pytest.ini, `collection._MODULE_ORDER`, `run_meta.SIMULATION_MODULES`). Golden baseline of today's `AUTONOMY_ROLE=full` topology under `tests/goldens/wiring/`.

### 2.2 P1 — module.yaml schema + validation — `DONE` <!-- committed b6e4dba9, 2026-08-20; 24 unit tests green -->

`common/module_schema/module.schema.json` (incl. `hooks.host_setup`), `tools/validate_module.py`, fixture `tests/fixtures/modules/hello_module/`, `tests/meta/test_module_manifest_contract.py`, `create-module` skill stub.

### 2.3 P2 — sync/overlay + CLI module commands — `DONE` <!-- 77aa24b1 + auto-include follow-ups, 2026-08-20; 252 unit tests green; verified against real pilot module -->


`.airstack/modules/module.sh` (`airstack module add|remove|list|sync|create [--in-tree]|doctor [--drift]`), **vcs2l** sync (ros-infrastructure/vcs2l, the maintained vcstool successor — owner decision 2026-08-20; same `.repos` format/`vcs` CLI) of pinned `modules.repos` → gitignored `modules/` (`--recurse-submodules`), symlink overlay into `robot/ros_ws/src/modules/`, compose-fragment merge, isaac `launch_scripts/modules/<name>/` symlinks, host-setup hook runner (`--no-hooks`). `tests/meta/test_module_overlay_contract.py`.

### 2.4 P3 — Reusable CI workflow — `WIP` <!-- committed 11f51c53, 2026-08-20; dispatch smoke (test plan c) blocked on P2 CLI landing at airstack_ref -->


`.github/workflows/module-system-tests.yml` (`workflow_call`; inputs airstack_ref/marks/sim/num_robots/module_tests_dir; registry secrets), body lifted from `system-tests.yml` (parse block, image prep, run, honesty gate, artifacts). First-party callers only. Never rename `docker-build.yml` (cosign identity pin).

### 2.5 P4 — Docker layer composition + modules.lock — `DONE` <!-- 695ecc9c, 2026-08-20; test plan (d) PASSED: identity rule + real 2-step chain built & verified in-container (cowsay/tabulate/marker); PEP 668 fix applied; 270 unit tests green -->

### 2.6a P5-E1 — Stacks (wrap form) — `DONE` <!-- d02a3467 + 8e4d454f, 2026-08-20. Test plan (e) largely PASSED: legacy drift-clean through dispatch change; --stack full_default graph IDENTICAL to legacy golden (machine-proven); droan_cpu+macvo wiring baselines committed (macvo = first working MACVO topology); 1-robot stack THL 4/4. RESOLVED: 3-robot THL landing timeout reproduces identically on legacy (1 failed/3 passed both paths, same 45s land-action timeout on robot_1) → PRE-EXISTING trunk bug, not a dispatch regression; record in results_summary + surface to owner. E1 = DONE. Lint allowlist frozen at 19 files (real count; plan said ~10). -->


`tools/compose_module_layers.py`: tier-1 rosdep/apt/pip RUN per module, tier-2 `Dockerfile.module` (`ARG BASE_IMAGE`), tier-3 overlay; `modules.lock`; dep-conflict hard gate. **Zero-module identity rule** (no modules ⇒ byte-identical tags). Extend `SERVICE_FINGERPRINT_ROOTS`. `tests/meta/test_docker_layer_plan_contract.py`.

### 2.6 P5 — Stacks + conventions + lint + wiring + doctor — `DONE` <!-- 2026-08-21: E3a eca041a8 (all layers flat except interface-by-design; generic args prefixed w/ tested aliases; allowlist 16→14) + E3b badfde5f (lite_default + lite_offload_global split stack + bridge.yaml + gen_dds_router hard gate #2 — legacy config actually bridged set_trajectory_mode, now barred; doctor --live/--snapshot; interface_conventions.md v1.0.0; stack list|new|diff; 45+ tests; skills+index rebuilt) + 9b654664 lite baseline. GPU gates: legacy + 3 full stacks DRIFT-CLEAN through the flatten (zero re-blessing — machine-proven graph-preserving); lite_default blessed (80 nodes, no global leakage). Split live bring-up + doctor --live smoke → P6 gates. -->


E1 wrap: `stacks/{full_default,full_droan_cpu,full_macvo}/`; `robot.launch.xml` gains `stack_dir`/`stack_entry` (from `AIRSTACK_STACK_DIR`/`AIRSTACK_STACK_ENTRY`); compose adds env + `stacks/` volume; `airstack up --stack`; single-locus lint (frozen shrink-only allowlist, error-on-new); rewrite `write-launch-file` skill same commit; per-stack `wiring.md` drift gate. E2 flatten local (merge gate: empty wiring diff). E3 flatten rest; `stacks/lite_default` + split `stacks/lite_offload_global` (+`bridge.yaml` → `tools/gen_dds_router.py`); `docs/robot/autonomy/interface_conventions.md`; `airstack doctor` (`--live`, `--snapshot`, two hard gates); `airstack stack list|new|diff`; allowlist emptied+deleted.

### 2.7 P6 — airstack.yaml + vehicles + fleets + spawner — `DONE` <!-- 2026-08-21: 344fa0cc + 6 hardening commits. Test plan (g) PASSED: fleet liveliness 16/16; fleet THL 4/4 (PX4-ready 84s); heterogeneous sim_three_mixed (full+lite+split) flight-ready in 100s; split-bridge global_plan crossing verified live; doctor --live stack-scoped for heterogeneous fleets. Debug epic resolved: orphaned fleet services invisible to airstack down (legacy control experiment isolated it) — down now loads generated overlays + --remove-orphans; also fixed single-robot prim parity, watchdog settle guard, ready/doctor fleet-service discovery, campaign clamp, generated-dir mount + router auto-gen. Deferred (documented): per-robot overrides application, effective_config.yaml YAML form, config freeze, release-set registry resolution. -->


`airstack.yaml` → `airstack sync` → generated `.env`/`modules.lock`; precedence chain with leaf-values-only CLI overrides; `effective_config.yaml` + `airstack config freeze`. `config/vehicles/quad_default/`, `config/fleets/*`, gitignored `config/local/`. `tools/fleet/resolve_fleet.py` (opt-in via `FLEET_CONFIG_FILE`; legacy resolver untouched), `airstack fleet generate` (heterogeneous compose), `fleet_spawn.py` generic Isaac spawner, harness `--fleet`. Migrate `overrides/*.env`.

### 2.8 P7 — Registry + docs catalog + skills — `DONE` <!-- 2026-08-21: registry (2.8a) + docs catalog/walkthrough/fetch-loops committed; strict-build warnings 112→59; 251 unit tests -->


`castacks/airstack-modules-index` (modules/, stacks/, compat/ CI-stamped, PR validation). mkdocs Modules catalog + stack pages; docs-deploy module fetch loop + trigger paths + failure isolation. `extract-module` skill; refresh stale skills + `.agents/README.md`; update AGENTS.md.

### 2.9 M1 — asm_dfm2_disturbances (pilot, hand-built) — `DONE` <!-- 2026-08-20: repo live at castacks/asm_dfm2_disturbances; runtime-validated (ext loads, flight-ready 91s under fields, takeoff_hover_land 4/4); v0.1.0 tag waits on GitHub CI (orchestrator poll list); strobe scenario + force-on-flight-path variant tracked in module FRICTION_LOG -->


Plain copy from local fork clone (`~/Development/airstack-dfm2` @ `8c20ef2a`) with provenance; `scene_library` → Kit extension `dfm2.disturbances`; launch scripts rewritten as `PegasusApp` subclasses (`post_scene_prep`); `test_stack/`; CI via reusable workflow; friction log = RFC trip report. No trunk removal.

**Port sizing (2026-08-20):** static inspection shows `omni.isaac.dynamic_control` is used in exactly one file (`force_fields/base.py`) and only as a *preferred* branch of `_apply_force`, with a still-current `omni.physx get_physx_simulation_interface().apply_force_at_pos` fallback already implemented. Trunk pins Isaac Sim **5.1.0** (`Dockerfile.isaac-ros` ARG), where dynamic_control is removed — so the DC branch is dead code; the port = delete the DC path (`_try_apply_articulation_force`, `_dc`/`_body_handles` caches) and rely on the PhysX path for both rigid bodies and articulation links. Strobes are UsdLux (stable). Runtime smoke test still required in the isaac 5.1 container.

### 2.10 M2 — asm_optitrack — `DONE` <!-- 2026-08-20: trunk removal 3e903aba; module round-trip 6/6 GREEN (pose_alive, EV fusion, px4_ready, takeoff, Circle on mocap EKF2, landing) via pinned GitHub sync + EULA-gated SDK hook. Debug yield: natnet_config_file arg-prefix fix (global launch-config collision w/ gossip's generic config_file), flat test_stack, sync self-heal, landing-timeout margin fix (pre-existing, evidence-based). v0.1.0 tag awaits GitHub CI. -->
<!-- was: WIP --> <!-- 2026-08-20: repo live at castacks/asm_optitrack (13 commits, history preserved incl PRs #359/#374/#375/#376; emulator included per maintainer requirement; SDK guard clean). Overlay hybrid-targets fix landed in trunk. SEQUENCING NOTE: trunk-removal commit must precede module sync/CI (duplicate colcon package natnet_ros2 otherwise) — lands on the feature branch after P5-E1, validated by branch-local full suite. -->


`git filter-repo` (natnet_ros2, emulator, natnet isaac scripts, integration+e2e tests, docs, skill); `hooks.host_setup` generalizes SDK download; `server_ip` launch arg replaces env coupling; module CI `build_packages or integration or liveliness or optitrack` + weekly cron. Trunk-removal PR only after module v0.1.0 green.

### 2.8a P7 — Registry repo — `DONE` <!-- 2026-08-20: castacks/airstack-modules-index live — 6 entries (3 modules @ HEAD SHAs, 3 reference stacks), schema+validator, validate.yml gate, compat-stamp receiver (sender = trunk follow-up), governance READMEs. Docs-catalog half of P7 still pending. -->

### 2.11 M3 — asm_macvo — `DONE` <!-- 2026-08-21: trunk removal 19ec8184; DOGFOOD DATAPOINT robot-desktop 17.1GB → 6.06GB (−65%); composed image 17.3GB carries the 11.3GB torch/TensorRT/weights layer only for MACVO users. All 4 wiring baselines regen'd + drift-verified (f769eecc, 8deea5c3). Debug yield: torch upgrades numpy→2.x breaking base env compat (re-pin fix, doctor-gate candidate); sim render-pipeline node exclusion (golden flake class); ws-cache-vs-image staleness (migration note for phase PR). Module CI (GitHub) pending, tag pending. --> <!-- 2026-08-20: repo live at castacks/asm_macvo (history preserved; MAC-VO submodule @8683b532 verified MIT; weights sha256-pinned; canonical macvo.launch.xml incl. camera_info hardcode fix; TRUNK_REMOVAL.md w/ completed pip audit — only tabulate has another consumer, supplied via tests venv not the image). Reusable-workflow lock--build gap fixed in trunk (c000584c). Pending: trunk-removal commit + composed-image CI validation. -->

**E3 backlog from field debugging (2026-08-20):** (1) trunk's `interpolate_dds_router.launch.py` / gossip launch set a GENERIC global `config_file` launch configuration — ROS 2 launch configs are global across includes, so any later include reading `config_file` silently gets gossip's YAML (bit asm_optitrack: natnet loaded gossip_dds_router.yaml, zero tracked bodies, warning-only symptom). Rename trunk's generic args with prefixes in E3 + consider a lint rule (generic arg names like config_file banned in launch files included by stacks). (2) stale-colcon-cache guard landed (76f0072b). (3) stack entries must not include the dispatcher — contract test landed. (4) topic_keepalive subscribes macvo disparity with mismatched type (doctor --live candidate finding).

### 2.6b P5-E2 — Flatten local layer — `DONE` <!-- 7344a9e6; GPU gate PASSED 2026-08-20: all three stacks drift-clean vs committed wiring.md after flatten. (History split 035f51bb/3e903aba/7344a9e6 — force-push pending owner approval.) -->



`git filter-repo` on macvo_ros2 (MAC-VO stays submodule @ `8683b532`); `Dockerfile.module` absorbs SKIP_MACVO blocks; sha256-pinned `assets:` weights; tier-3 overlay published on tag; `test_stack/` fixes the broken variant wiring. Trunk-removal PR strips Dockerfile.robot (~8–10 GB win, before/after recorded).

### Affected packages

| Package / area | Change |
|---------|--------|
| `tests/` (harness, meta, system) | wiring tool + mark, 6+ new contract tests, `--fleet` option |
| `.airstack/modules/` | new `module.sh`, `stack.sh`, `doctor.sh`, `fleet.sh` |
| `robot/ros_ws/src/autonomy_bringup` | stack-dir dispatch coexistence |
| `robot/ros_ws/src/local/local_bringup` | variants flattened into stacks (E2) |
| `robot/ros_ws/src/perception/{natnet_ros2,macvo_ros2}` | extracted to asm_ repos |
| `robot/docker/` | compose env/volume, Dockerfile.robot shrink |
| `stacks/`, `config/`, `airstack.yaml` | new configuration surface |
| `.github/workflows/` | new `module-system-tests.yml`, docs-deploy fetch loop |
| `docs/`, `mkdocs.yml`, `.agents/skills/` | conventions spec, catalog, skill rewrites |
| new repos | `asm_dfm2_disturbances`, `asm_optitrack`, `asm_macvo`, `airstack-modules-index` |

### Interfaces

| Topic / Service / Param | Type | Direction | Purpose |
|-------------------------|------|-----------|---------|
| `AIRSTACK_STACK_DIR` / `AIRSTACK_STACK_ENTRY` | env | in | stack dispatch coexistence |
| `stacks/<name>/wiring.md` | artifact | out | observed topology, drift-gated |
| `module.yaml` | manifest | in | module identity/deps/hooks |
| `modules.repos` / `modules.lock` | vcstool/lock | in/out | pinned module sets |
| `FLEET_CONFIG_FILE` | env | in | opt-in fleet resolver |
| `hooks.host_setup` | manifest key | in | host-side module setup (NatNet SDK) |

## 3. Test Plan

### (a) Wiring snapshot tooling (P0)

- **What is run:** `airstack test -m unit -v` (renderer/differ unit tests); `airstack test -m "liveliness or wiring" --sim isaacsim --num-robots 1 --stress-iterations 1 -v`
- **What is measured:** snapshot captures all sentinel nodes + topics with QoS; mermaid renders; drift-vs-golden exit codes
- **Pass criteria:** wiring test green; golden baseline committed; re-run produces empty diff (deterministic)

### (b) Module manifest + overlay + CLI (P1–P2)

- **What is run:** `airstack test -m unit -v` (manifest + overlay contract tests); `airstack module add tests/fixtures/modules/hello_module && airstack test -m build_packages -v`; no-module `build_packages` parity
- **What is measured:** schema validation verdicts; symlink lifecycle; colcon sees fixture package; zero-module no-op
- **Pass criteria:** all contract tests green; build_packages green with and without fixture module

### (c) Reusable module CI (P3)

- **What is run:** `gh workflow run` dispatch smoke of `module-system-tests.yml` against the pilot module repo (marks=`build_packages or liveliness`, sim=msairsim)
- **What is measured:** end-to-end: trunk checkout @ ref → module add → test_stack bring-up → marks → artifacts
- **Pass criteria:** dispatch run green; run_meta honesty gate passes; artifacts uploaded

### (d) Docker layer composition (P4)

- **What is run:** `airstack test -m unit` (layer-plan contract); `airstack test -m build_docker -v`; fixture-module compose then `airstack test -m liveliness --sim msairsim`
- **What is measured:** lock determinism; zero-module tag identity; composed image boots the stack
- **Pass criteria:** identity rule proven byte-identical tags; liveliness green on composed image

### (e) Stacks wrap + lint + baseline (P5-E1)

- **What is run:** legacy `airstack test -m liveliness --sim isaacsim`; `airstack up --stack full_default` + `-m "liveliness or wiring"`; `-m takeoff_hover_land --sim isaacsim` on stack path; `-m unit` (lint + stack-layout contracts)
- **What is measured:** both dispatch paths green in one session; per-stack wiring.md captured; lint allowlist frozen
- **Pass criteria:** all green; wiring.md committed for 3 stacks; lint errors on any new out-of-stack remap

### (f) Flatten equivalence + split + doctor (P5-E2/E3)

- **What is run:** `-m wiring` after each flatten; full conformance `-m "liveliness or sensors or takeoff_hover_land or autonomy or waypoint_flight" --sim isaacsim` on full_default; split stack `--stack lite_offload_global -m "liveliness or wiring"`; doctor gate unit tests
- **What is measured:** graph equality vs E1 baseline; split-stack bridge topology; doctor hard gates fire correctly
- **Pass criteria:** empty wiring diffs for flattens; conformance suite green; doctor rejects control-setpoint in bridge.yaml

### (g) Fleet + airstack.yaml (P6)

- **What is run:** `airstack test --fleet sim_three_mixed -m "liveliness or wiring"`; `--fleet sim_one_default -m waypoint_flight` + `tests/parse_metrics.py` vs pre-fleet baseline; legacy `NUM_ROBOTS=2 airstack test -m liveliness`; `up --dry-run` golden equivalence per `overrides/*.env`; `-m unit` (fleet resolver parity contract)
- **What is measured:** heterogeneous bring-up; metric parity; legacy path unbroken; precedence chain
- **Pass criteria:** all green; waypoint RMSE within parse_metrics threshold vs baseline; resolver parity exact

### (h) Module extractions (M1–M3)

- **What is run:** each asm_ repo's CI via reusable workflow (DFM2: `liveliness or takeoff_hover_land` isaac w/ force field active; OptiTrack: `build_packages or integration or liveliness or optitrack`; MACVO: `build_docker or build_packages or liveliness`); trunk-removal PRs gated on module green; MACVO before/after image sizes
- **What is measured:** module CI conformance; trunk stays green post-removal; image-size delta
- **Pass criteria:** 3 repos tagged v0.1.0 with green CI; trunk full suite green after each removal; robot-desktop shrinks ≥5 GB

### (i) Docs + registry (P7, plus every phase)

- **What is run:** `mkdocs build --strict` per phase; index-repo PR validation workflow; manual new-developer walkthrough (Getting Started → stacks → module add)
- **What is measured:** strict build passes with catalog + stack pages + fetched module docs; nav coherent
- **Pass criteria:** strict build green; catalog renders 3 modules + reference stacks with embedded wiring.md
