# 006 — Diátaxis Documentation Audit

**Date:** 2026-08-25 · **Branch:** develop @ 02a12d56 · **Type:** audit + planning (no files moved or rewritten in this pass)

## Problem context

AirStack's docs site (MkDocs Material, `mkdocs.yml`, ~110 real documents plus ~25 agent skills) has grown through the monolith→modular transition (RFC #379/#380) without an explicit information-architecture model. This audit classifies every doc against the Diátaxis framework (tutorials / how-to / reference / explanation), verifies claims against the code, identifies structural problems and categorical gaps, and proposes a reorganization plan as an ordered PR sequence. Method: full inventory (see `results/inventory_raw.md`), five parallel full-read classification passes (archived under `results/a-classification/`), and a mechanical verification pass against `airstack.sh`, `.env`, compose files, `tests/pytest.ini`, and the message packages (`results/b-verification/`).

**Scope notes.** Generated files (`stacks/*/wiring.md`, `docs/modules/*.md` via `tools/gen_docs_catalog.py`, `tests/results/`) are audited as *generator outputs* — findings apply to the generator/template, not the files. Vendored third-party docs (PegasusSimulator, xdot_cpp, mav_comm) are out of scope. `.agents/skills/` (25 SKILL.md files) are agent-facing how-to documents by construction; they are treated as a parallel corpus, not site content.

---

## Executive summary

The corpus is in **better shape than typical for a robotics stack**: the modular-era docs (modules/stacks/fleets, interface conventions spec, release notes discipline) are fresh and verified against code, the `AUTONOMY_ROLE`/bringup migration left essentially zero stale live references, and two genuine tutorials exist. The problems are structural, and they cluster:

1. **The nav taxonomy is wrong on its face.** The entire Development section is labeled "Beginner/Intermediate/Advanced **Tutorials**" and contains **zero tutorials** — it is reference, how-to, and explanation organized by difficulty, an axis orthogonal to document type. The repo's own doc standards prescribe this layout, so the standards must change with the nav.
2. **One canonical reference, many decaying clones.** `interface_conventions.md` is an exemplary reference spec, but topic tables are cloned in 3 other pages (already drifted: phantom `interface/cmd_vel`), CLI flags in 3 (one phantom `--recreate`, one missing `--scene`), pytest marks in ≥4 (two missing `wiring`/`waypoint_flight`), hardware requirements in 4 (three-way numeric contradiction).
3. **A fossil stratum**: 5 orphaned pages (3 with zero inbound links, 2 still linked from live pages) documenting superseded systems — worst are `testing_frameworks.md` and the Ascent-era `scene_setup.md`/`ascent_sitl_extension.md` pair.
4. **Generated-looking filler presented as product**: `robot/configuration/index.md` (~300 of 369 lines fabricated config examples), `robot/logging/data_offloading.md` (generic rsync cookbook with invented paths that *contradicts* the real tool documented in `real_world/data_offloading/`), half of `rosbags.md`, and `system_architecture.md`'s invented performance tables.
5. **The most important missing docs are the hardware on-ramp** (the #1 "start here" page for deployment is an explicit placeholder) **and two reference surfaces that exist nowhere**: a complete `.env` schema and any `airstack_msgs` documentation (the message package has no README and no docs page; even the doc-standards template cites a message type that doesn't exist).

---

# Phase 1 — Inventory

Full table (path, lines, last-modified): `results/inventory_raw.md` (173 rows). Summary by area:

| Area | Docs | Lines (approx) | Freshest | Stalest |
|---|---|---|---|---|
| `docs/getting_started/` + `docs/tutorials/` + top-level | 9 | ~1,700 | modular_airstack.md (2026-08-24) | tutorials/index.md (orphan) |
| `docs/development/` | 26 | ~4,900 | modules/stacks/fleets (2026-08-22+) | testing_frameworks.md (2026-03-18) |
| `docs/robot/` | 30 | ~4,900 | interface_conventions.md | global/planning/index.md (never-built design) |
| `docs/simulation/` | 14 | ~2,300 | spawning_drones.md (2026-08-25) | scene_setup.md (Ascent era) |
| `docs/gcs/` | 5 | ~560 | foxglove.md (2026-08-25) | usage/user_interface.md (stub) |
| `docs/real_world/` | 5 | ~340 | index.md | HITL/index.md (pre-Foxglove era) |
| `docs/modules/` (generated) | 4 | ~220 | regenerated per deploy | n/a |
| In-nav READMEs (`robot/ros_ws/src`, `stacks/`, `common/`, `tests/`) | 22 | ~3,600 | trajectory_controller README | disparity stubs, vdb upstream README |
| Off-nav READMEs (packages, infra, config) | ~20 | ~1,600 | osmo/README.md | git-hooks READMEs (contradict CI) |
| Root: README/AGENTS/CLAUDE/CODE_OF_CONDUCT | 4 | ~1,100 | AGENTS.md (2026-08-25) | CODE_OF_CONDUCT (2024, fine) |
| `.agents/skills/` (agent corpus) | 28 | ~8,900 | several 2026-08-25 | — |

---

# Phase 2 — Classification

Legend: **T**utorial / **H**ow-to / **R**eference / **E**xplanation; conf = confidence; purity **clean** / **hybrid** (sections cited in the archived per-file reports); audience **N**ewcomer / **C**ompetent / **M**aintainer. ⚠ = verified staleness/accuracy defect (detail in Phase 3 or the archived reports).

### Onboarding corridor

| Doc | Quadrant (conf) | Purity | Aud. | Notes |
|---|---|---|---|---|
| `docs/index.md` | none — landing shell | n/a | N | content lives in `home.html` template |
| `docs/README.md` | E (low) | hybrid (features/arch=E, requirements/repo-tree=R) | N | ⚠ GPU/disk/Ubuntu numbers contradict 3 other pages; absolute `latest` docs URL |
| `docs/about.md` | E (med) | hybrid (platforms/license=R, FAQ answers=mini-H) | N | ⚠ 100GB vs 25GB disk; EOL Jetsons (Xavier NX, TX2) listed |
| `docs/getting_started/index.md` | **T (high)** | mostly clean; "Docker Images" fork + launch-variants tail = H/R intrusions | N | ⚠ "Move Robot" teaches superseded manual Foxglove import; host port is 8766 not 8765 |
| `docs/getting_started/modular_airstack.md` | **T (high)** | cleanest tutorial in repo; links out for H/R correctly | N | §1 duplicates its own prerequisite; teaches redundant `-f docker-compose.modules.yaml` |
| `docs/getting_started/tutorials_reference.md` | none — curriculum/link hub | n/a | N | mislabels E/R targets as "Tutorials"; overlaps orphaned tutorials/index.md |
| `docs/tutorials/index.md` | none — **orphan** (zero inbound links) | n/a | N | functional duplicate of tutorials_reference.md |
| `docs/tutorials/airstack_on_osmo.md` | H (med; T-ish framing, goal-directed substance) | heavy hybrid: steps=H, "Under the hood" blocks=embedded CLI R, arch/credentials=E | C | ⚠ positioning contradiction: calls itself "recommended default"; Getting Started calls it the non-Linux fallback. 13× `./airstack.sh osmo` style |
| `docs/release_notes/index.md` | R (record) (high) | clean for genre | C | the enforced home for change-relative content — model discipline |

### docs/development — nav says "Tutorials" ×3 tiers; actual tutorials: **0**

| Doc | Quadrant (conf) | Purity | Aud. | Notes |
|---|---|---|---|---|
| index.md | hub | H/R mix | N | contradicts nav (labels CLI docs "Reference Documentation") |
| beginner/key_concepts.md | E (high) | hybrid (dev-loop + env vars = H/R) | N | fresh |
| beginner/airstack-cli/index.md | R (high) | near-clean | C | ⚠ phantom `--recreate` flag (nothing implements it); missing `--scene` row |
| beginner/airstack-cli/docker_usage.md | H (med) | hybrid | N→C | ⚠ "Automated Testing" section predates pytest harness; Isaac Streaming-Client-era instructions; SSH example IP on wrong subnet |
| beginner/development_environment.md | H (med) | hybrid (requirements=R) | N | in-nav twin of orphaned top-level version |
| beginner/vscode/vscode_debug.md | H (high) | clean | N | ⚠ `.devcontainer` paths lag per-container layout; no `.devcontainer/Dockerfile` exists |
| beginner/fork_your_own_project.md | H (high) | clean | N | modules.md "researcher workflow" is unlinked continuation |
| development_environment.md | H — **orphan** (zero inbound) | dup | N | delete/redirect |
| airstack-cli/index.md | R — **orphan** (zero inbound) | dup, stale (8 commands, `./airstack.sh` era) | N | delete/redirect |
| intermediate/contributing.md | H+R (med) | hybrid (enforcement/VERSION tables=R) | C | `pip install mkdocs-material` vs `airstack docs` inconsistency |
| intermediate/documentation.md | H + heavy R (repo doc standard) | hybrid by design | C | ⚠ template example uses nonexistent `airstack_msgs/TrajectorySegment` |
| intermediate/feature_notebook.md | H (med) | light hybrid | C | fresh; links skill |
| intermediate/frame_conventions.md | R (med) | 7-line stub, no H1 | C | link into scene_setup.md anchor at risk |
| intermediate/docker-build-profiles.md | R+H (med) | managed pairing with skill | M | fresh |
| intermediate/testing/index.md | R hub (med) | hybrid | C | ⚠ marks table omits `wiring`, `waypoint_flight` |
| intermediate/testing/unit_testing.md | H (high) | hybrid (CI tables=R, principles=E) | C | coverage table rots by design |
| intermediate/testing/end_to_end_testing.md | R w/ embedded H (med, ambiguous — all four quadrants present; tables dominate → R) | heavy hybrid | C/M | ⚠ troubleshooting row claims `autonomy` mark not in pytest.ini (it is); hard-coded lab baseline numbers |
| intermediate/testing/ci_cd.md | E+R (high it's both) | heavy hybrid; two audiences interleaved | M+C | ⚠ marks table omits wiring/waypoint_flight; absolute `blob/main` links |
| intermediate/testing/testing_frameworks.md | fossil — **de-nav'd but still linked** from 2 live pages | incoherent | ? | ⚠ contradicts co-located test convention; unregistered `rostest` mark; ArduPilot GUIDED in a PX4 stack; nonexistent launch file |
| advanced/ai_agent_guide.md | R (high) | clean digest | agents | ⚠ dead `local/c_controls/...` path (real: `local/controls/`) |
| advanced/airstack-cli/architecture.md | E (high) | clean | M | "Future Enhancements" already shipped; module list omits 8 real modules |
| advanced/airstack-cli/extending.md | H (high) | mostly clean | M | fresh |
| modules.md | E+R (high it's both) | hybrid: commands=R, overlay/sync=E, workflows=H | C→M | fresh; correct Diátaxis pairing with the walkthrough tutorial |
| stacks.md | E+R (high) | hybrid; "Why stacks don't launch standalone"=textbook E | C→M | fresh, authoritative |
| fleets.md | E+R (high) | hybrid | C→M | fresh |
| module_ci.md | How-to-flavored R (med) | hybrid | M | file location (top-level) vs nav location (Intermediate→Testing) vs siblings (Advanced) disagree |

### docs/robot

| Doc | Quadrant (conf) | Purity | Aud. | Notes |
|---|---|---|---|---|
| index.md | R (high) | light hybrid | N→C | topic table = 3rd copy of the spec's data |
| configuration/index.md | H (low) | ⚠ ~300/369 lines generic or **fabricated** (config YAML matching no real files; invented validation script) | C | stack-selection paragraph is current; salvage ~40 lines |
| docker/index.md | R (high) | clean-ish | C/M | ⚠ phantom `ROBOT_LAUNCH_PACKAGE`/`ROBOT_LAUNCH_FILE` env rows (real var: `LAUNCH_PACKAGE`) |
| docker/robot_identity.md | E/R, E primary (med-high) | working hybrid — leave intact | C/M | healthiest page in tree; canonical |
| autonomy/index.md | R nav (high) | 22-line stub | N | front door with no orientation |
| autonomy/system_architecture.md | **E primary, tri-quadrant** (high) | ⚠ split candidate: node-types/task-cascade E excellent; per-layer topic lists = 4th spec copy w/ errors (`trajectory_controller/cmd_vel` doesn't exist); sequence diagram shows nonexistent CTL→BEH completion; performance tables look invented (phantom 10 Hz BT ticker) | N/C | biggest doc in tree (643 ln) |
| autonomy/integration_checklist.md | H primary (high) | hybrid: ~110-line R block restates the spec **and drifts** (`interface/cmd_vel` Twist vs spec's `interface/cmd_velocity` TwistStamped) | C/agents | core checklist is the good part |
| autonomy/interface_conventions.md | **R (very high)** | near-clean; deliberate E footnotes earn their place | C/M | the platonic reference doc; canonical source others clone |
| autonomy/tasks.md | R primary (high) | hybrid (add-executor tail=H) | C | ⚠ its action list and spec §8's action list are two different "complete" lists; 3 of 6 specs "(not yet implemented)" |
| autonomy/interface/index.md | E (med) | hybrid (extension recipe=H) | C | ⚠ `==TODO: This is not our diagram, must replace.==` + hotlinked external image; L94 TODO admits the extension instructions are wrong |
| autonomy/sensors/index.md | R (high) | clean — **model layer index** | C | fully stack-era |
| autonomy/sensors/gimbal.md | H (high) | clean | C | ⚠ Isaac Sim UI guide misfiled under robot/autonomy/sensors |
| autonomy/perception/index.md | R (med-high) | clean-ish | C | ⚠ output topics contradict spec (plain `/odometry` is v2 target; `/pose`, `/imu/data` invented) |
| autonomy/local/index.md | R stub (9 ln) | — | N | ⚠ doesn't link its own children |
| autonomy/local/world_model/index.md | E/R stub (9 ln) | good stub | N | right shape, thin |
| autonomy/local/planning/index.md | E/R stub (12 ln) | good stub | N | current |
| autonomy/local/controls/index.md | E/R stub (10 ln) | good stub | N | pid_controller has no README to link (upstream gap) |
| autonomy/global/index.md | R stub (13 ln) | — | N | ⚠ no child links, no E |
| autonomy/global/planning/index.md | E/R — **wrong content** (med) | ⚠ documents never-built Global-Manager/PlanRequest protocol (msg exists, no node implements it); two live `{==TODO==}` markers; contradicts tasks.md + spec §8 | C | most substantively stale page in the tree |
| autonomy/global/world_model/index.md | E fragment (5 ln) | emptiest stub | N | links external GitHub instead of in-tree README; calls shipped default a "placeholder" |
| autonomy/behavior/index.md | R/E stub (19 ln) | ⚠ ¶1 contradicts its own ¶2–3 (accurate: safety monitor only) | N | |
| autonomy/coordination/index.md | E/R (med) | working hybrid — arguably the model layer landing page | C | |
| autonomy/coordination/payloads.md | H (high) | clean | C | mirrors `attach-gossip-payload` skill (two maintained copies) |
| autonomy/dds_router.md | R primary + E (med-high) | working hybrid — leave intact | C/M | current (bridge.yaml, doctor gate) |
| autonomy_modes.md | R primary + H (med-high) | working hybrid | C | fully modernized; title/filename mismatch is a legacy echo |
| optitrack.md | H-pointer tombstone (9 ln) | correct by design | C | 3 OptiTrack entry points site-wide |
| static_transforms/index.md | R fragment (10 ln) | ⚠ title promises a how-to that doesn't exist | C | frame conventions triplicated; spec's copy is best |
| logging/index.md | H (high) | clean | C | complete for scope |
| logging/rosbags.md | H (med) | ⚠ ~half generic upstream `ros2 bag` tutorial; example output has wrong namespaces (`robot1` vs `robot_1`) and 2024 dates | C | |
| logging/data_offloading.md | H in form | ⚠ generic rsync/cron/systemd cookbook; invented paths (`/opt/airstack/bags`); **contradicts** the real tool at `real_world/data_offloading/` | nobody | gut to a pointer |

### docs/simulation, docs/gcs, docs/real_world, docs/modules

| Doc | Quadrant (conf) | Purity | Aud. | Notes |
|---|---|---|---|---|
| simulation/index.md | E hub (med) | 4-quadrant hybrid | N→C | env table verified current |
| simulation/scenes.md | R (high) | well-composed hybrid | C | ⚠ `fetch_scene.sh forest` — no `forest` key exists |
| isaac_sim/index.md | E (high) | minor hybrid | N | USD naming convention's **only** normative statement is buried here |
| isaac_sim/docker.md | R (med), ~40% H | "everything about the container" page; split candidate | C/M | ⚠ verbatim compose YAML rots; deprecated Omniverse-Launcher-era streaming refs; links into orphaned scene_setup.md |
| isaac_sim/pegasus_scene_setup.md | E leaning R (low-med, ambiguous) | messy hybrid | C | ⚠ its ".env" block contradicts actual `.env` on `ISAAC_SIM_USE_STANDALONE` and `ISAAC_SIM_GUI`; scene_prep helper table duplicates spawning_drones.md function-for-function |
| isaac_sim/spawning_drones.md | H (high) | intentional, strongest-written sim page | C | current |
| isaac_sim/overhead_camera.md | H (high) | minor hybrid | C | title/nav/filename disagree; satellite half is GCS content filed under simulation |
| isaac_sim/ascent_sitl_extension.md | E, deprecated stub (22 ln) | clean-but-dead | M | ⚠ still in nav and in simulation/index.md "Getting Started" list |
| isaac_sim/export_stages_from_unreal.md | H (med) | hybrid | C | ⚠ recommends discontinued Omniverse Launcher; dangling "next page" sentence |
| isaac_sim/scene_setup.md | H — **orphan, severely stale** (Ascent era) | superseded except Frame Conventions section (unique live knowledge) | C(old) | linked only from isaac_sim/docker.md |
| ms-airsim/index.md | R w/ H head (med) | hybrid | C | ⚠ ~35% overlap with its docker.md; the two disagree on tmux window ordering |
| ms-airsim/docker.md | R (med-high) | same genre | C/M | boilerplate cloned across docker pages |
| simple_sim/index.md | E (med) | well-proportioned | N/C | current |
| simple_sim/docker.md | R (high) | light hybrid — **right size for the genre** | C | |
| gcs/index.md | E hub (med) | hybrid | N→C | launch story told 3× in GCS tree |
| gcs/docker/index.md | R (high) | hybrid | C/M | current, fleet-era |
| gcs/usage/user_interface.md | stub (23 ln) | 100% duplicated content | N (promised) | the page where an operator tutorial should live |
| gcs/foxglove.md | H+R (med each) | ⚠ two audiences on one page: operator how-to + maintainer source-editing guide | C→M | excellent content otherwise |
| gcs/waypoints_and_geofences.md | **H (high) — cleanest single-quadrant page in GCS** | clean | C (operator) | |
| real_world/index.md | E overview (med) | hybrid; workflows content-free | N | stack topology section fully modernized |
| real_world/deploying_to_hardware.md | T (intended) — **explicit placeholder** (27 ln, steps in HTML comment) | n/a | N | ⚠ the nav entry AND the index's #1 recommendation |
| real_world/installation/index.md | H (high), skeletal (32 ln) | clean | C | ⚠ Orin-only (VOXL/Xavier/TX2 promised, documented nowhere); `airstack --profile l4t up` flag order unique to this page |
| real_world/HITL/index.md | H (med) — **stalest recommended page** (24 ln) | skeletal | C | ⚠ raw compose; RViz-based check from pre-Foxglove era; ignores the `gcs-real` `hitl` profile built for this case; DDS-across-LAN reduced to "ensure ping works" |
| real_world/data_offloading/index.md | H (high) | clean-ish | C | the **actual** tool (storage_tools_server/device); dangling "optional setup" cross-ref |
| modules/index.md + 3 module pages | R (high), generated | template hybrid (compat explainer ×N, registry prose verbatim) | C | fixes belong in `tools/gen_docs_catalog.py` |

### In-nav READMEs and infra (highlights; full table in `results/a-classification/scattered_readmes.md`)

| Doc | Quadrant | Notes |
|---|---|---|
| `trajectory_controller/README.md` (217) | R+E | **best module README in repo — fully meets template** |
| `droan_local_planner` (168) / `droan_gl` (164) | E+R | good; missing mermaid + Testing |
| `takeoff_landing_planner` (120) + `test/README` (171) | R / E+H | ⚠ parent documents actions, test README documents legacy service; neither notes both coexist |
| `random_walk` (99) | R+E | good |
| `exploration` (55) | H+R | ⚠ service described as toggling "the random walk planner" (copy-paste error); pre-Tasks-Panel GUI reference |
| `vdb_mapping_ros2` (167, in nav) | R (upstream) | ⚠ unmodified FZI README: Foxy links, clone-and-colcon instructions, zero AirStack integration info |
| disparity_expansion (17) / disparity_graph (9) / …cost_map (7) / trajectory_library (6) | stubs **in nav** | ⚠ "Docs TODO" served on the site; trajectory-library YAML format documented nowhere |
| `lidar_point_cloud_filter` (47, **not** in nav) | R+E | the only full-quality robot README not surfaced |
| `stacks/*/README.md` (5, in nav) | E+H | clean intent-vs-wiring division of labor — a model Diátaxis pattern |
| `bag_recorder_pid/README.md` (454, in nav) | R+H+E | bloated; duplicated tables, "License: TODO", personal-gmail maintainer |
| `tests/README.md` (665, in nav) | R+H | comprehensive, section discipline holds |
| `common/module_schema/README.md` (96, off-nav) | R | authoritative module.yaml schema — off-site |
| `config/vehicles/README.md` (48) + `config/local/README.md` (26, off-nav) | R+E | vehicle.yaml schema + calibration contract exist only here |
| `rviz_tasks_panel` (184) / `3d_waypoint_rviz2_plugin` (142, off-nav) | R+H | operator manuals living only in package READMEs |
| `osmo/README.md` (301, off-nav) | H+R+E | admin half of the OSMO story, complementary to the tutorial, invisible on site |
| `git-hooks/*/README.md` (2 files, off-nav) | H | ⚠ **actively contradict CI**: instruct a hook writing git hash into `.env` VERSION; `check-version-increment.yml` requires strictly-increasing semver |

**Ambiguous classifications (tiebreakers recorded):** `airstack_on_osmo.md` (T vs H → H: goal-directed, branching, repeat-use); `pegasus_scene_setup.md` (E vs R → E: its distinctive contribution is rationale); `end_to_end_testing.md` (all four → R: tables dominate); `docs/README.md` (marketing README → E-as-orientation); `modules/stacks/fleets.md` (E vs R → deliberately both; split only the command/table material).

---

# Phase 3 — Structural problems

## 3.1 Hybrid documents that should be split (and those that should not)

| Doc | Proposed split |
|---|---|
| `docs/robot/autonomy/system_architecture.md` | **Keep as Explanation**: layered architecture, node types (perpetual vs task executor), task cascade, data-flow narrative. **Remove**: per-layer topic lists (replace with links into `interface_conventions.md`), performance tables (delete or re-measure — numbers appear invented), "Module Integration Guidelines" (merge into `integration_checklist.md`). |
| `docs/gcs/foxglove.md` | Split by audience: operator how-to (layout use/reset, troubleshooting) stays in GCS usage; the 6-step "modify `foxglove_visualizer_node.py`" maintainer guide moves to a development how-to (or the `visualize-in-foxglove` skill becomes canonical and the page links it). |
| `docs/simulation/isaac_sim/docker.md` + `ms-airsim/docker.md` | Each splits into container **reference** (env vars, mounts, ports, services — stop quoting compose verbatim; link the file) and **how-to** (credentials, streaming/GUI access, debugging, troubleshooting). `simple_sim/docker.md` is the size/shape target. Merge the ~35% ms-airsim index/docker overlap while at it. |
| `docs/development/modules.md` / `stacks.md` / `fleets.md` | Lower priority — content is fresh and internally organized. Extract the pure command/flag tables into the CLI reference page and link back; keep the explanation cores as-is. |
| `docs/tutorials/airstack_on_osmo.md` | Keep as the how-to; extract the "Under the hood — raw `osmo …`" blocks into an `airstack osmo` section of the CLI reference; troubleshooting + "what survives" tables could join an OSMO reference page alongside the admin guide. |
| `intermediate/testing/ci_cd.md` | Split audiences: "Using CI well" + trigger how-tos → testing how-to page; architecture/security/pod-anatomy stays as Explanation. |
| **Do not split** (working hybrids): `robot_identity.md`, `dds_router.md`, `coordination/index.md`, `interface_conventions.md`, `scenes.md`, `spawning_drones.md`, stack READMEs. Each mixes quadrants in service of one reader task. |

## 3.2 Wrong-quadrant / wrong-place documents

- **Entire Development nav**: "Beginner/Intermediate/Advanced Tutorials" contain 0 tutorials (verdict per file in Phase 2). This is the single largest mislabel.
- `tutorials_reference.md` + orphaned `tutorials/index.md`: curriculum pages labeling E/R targets "Tutorials".
- `gimbal.md` (Isaac Sim UI guide) filed under `robot/autonomy/sensors/`; satellite-map half of `overhead_camera.md` (GCS feature) filed under `simulation/isaac_sim/`.
- `global/planning/index.md`: reference for a never-built protocol — not stale so much as *fictional*; contradicts tasks.md and spec §8.
- `usage/user_interface.md`: the GCS "Usage" slot filled by a duplicate-content stub while real operator content sits elsewhere.
- `module_ci.md`: file at `docs/development/` top level, nav'd under Intermediate→Testing, siblings under Advanced.

## 3.3 Duplication (canonical copy → clones)

| Content | Canonical | Drifted clones |
|---|---|---|
| Topic/interface tables | `interface_conventions.md` (declares canonicity) | `robot/index.md`, `integration_checklist.md` (phantom `interface/cmd_vel`), `system_architecture.md` (phantom `trajectory_controller/cmd_vel`), `perception/index.md` (invented outputs) |
| Task action list | spec §8 vs `tasks.md` — **the two "complete" lists disagree**; pick one canonical | |
| `airstack up` flags | `beginner/airstack-cli/index.md` (after fixing `--recreate`/`--scene`) | `docker_usage.md`, `key_concepts.md`, getting_started launch tail |
| pytest marks table | `tests/README.md` | `testing/index.md`, `ci_cd.md` (both missing `wiring`/`waypoint_flight`), AGENTS.md |
| Hardware requirements | pick ONE (suggest getting_started) | `docs/README.md` (RTX 3070/25GB), `about.md` (100GB), getting_started (RTX 4080) — three-way contradiction |
| `scene_prep.py` helpers | `spawning_drones.md` (richer) | `pegasus_scene_setup.md` (function-for-function duplicate) |
| MS-AirSim scene fetch + tmux layout | one page after index/docker merge | told 3×; tmux ordering disagrees between the two pages |
| Setup/bootstrap steps | `getting_started/index.md` | `modular_airstack.md` §1 (its own prerequisite) |
| GCS launch/startup story | `gcs/docker/index.md` | `gcs/index.md`, `foxglove.md` |
| Data offloading | `real_world/data_offloading/index.md` (real tool) | `robot/logging/data_offloading.md` — **not a duplicate, a contradiction**; gut to pointer |
| Reference-implementation table | AGENTS.md ↔ ai_agent_guide.md — both carry it; ai_agent_guide has dead `c_controls` path (AGENTS.md too) | |
| Payload workflow | `attach-gossip-payload` skill ↔ `coordination/payloads.md` | pick one, point the other |

## 3.4 Navigation / IA problems

- Types interleaved arbitrarily: quadrants are not reflected anywhere; sections mix T/H/R/E freely (Robot tab: R landing → E architecture → H checklist → R spec → generated wiring).
- **Nav depth violates the repo's own standard**: `write-mkdocs-documentation` caps nesting at 3 levels; current nav reaches 5 (Robot → Autonomy Modules → Local → World Model → DROAN Obstacle Avoidance → page).
- Orphans: 3 zero-inbound (`tutorials/index.md`, `development/development_environment.md`, `development/airstack-cli/index.md`); 2 de-nav'd but still linked from live pages (`testing_frameworks.md`, `scene_setup.md`) — the worst state: invisible in nav, one click from fresh content.
- Deprecated page (`ascent_sitl_extension.md`) still in nav and in `simulation/index.md`'s "Getting Started" list.
- 0 broken nav targets and 0 broken relative links on the six highest-traffic pages (verified) — link hygiene is good; the problems are structural, not mechanical.

## 3.5 Orphaned knowledge (ranked; full list in scattered-READMEs report)

1. **`git-hooks/` READMEs contradict the live version gate** — following them produces a red PR. Fix or delete (P0-adjacent, trivial).
2. **`config/vehicles/README.md` + `config/local/README.md`**: vehicle.yaml schema and calibration contract exist only off-site; fleets.md links out via GitHub URL.
3. **Trajectory-library YAML format**: documented nowhere; the in-nav README is a 6-line TODO; both DROAN planners consume it.
4. **RViz operator panels** (`rviz_tasks_panel`, `3d_waypoint_rviz2_plugin`): operator manuals only in package READMEs; the waypoint plugin has zero site mention.
5. **`osmo/README.md` admin half** (pool requirements, buildx amd64 pitfall, Nucleus TLS debugging): complementary to the student tutorial, invisible on site.
6. **`common/module_schema/README.md`**: authoritative module.yaml reference, off-nav.
7. **`lidar_point_cloud_filter/README.md`**: only full-quality robot README not in nav (while five stubs are); carries the raw-vs-filtered topic contract.
8. **USD file-naming convention**: only normative statement buried in `isaac_sim/index.md`.
9. AGENTS.md is otherwise well-mirrored into the site (checked topic-by-topic) — little true orphaning there, but it carries the same dead `c_controls` path and cites `airstack_msgs/TrajectoryOverride`/`TrajectorySegment`, **neither of which exists** in the package.

## 3.6 Verified accuracy defects (fix regardless of reorg)

| # | Defect | Where |
|---|---|---|
| 1 | Phantom `--recreate` flag documented; `--scene` missing from the same table | `beginner/airstack-cli/index.md:113-129` |
| 2 | Phantom env vars `ROBOT_LAUNCH_PACKAGE`/`ROBOT_LAUNCH_FILE` (real: `LAUNCH_PACKAGE`) | `robot/docker/index.md:96-113` |
| 3 | Nonexistent msg types in doc standards + agent guide (`TrajectorySegment`, `TrajectoryOverride`) | `intermediate/documentation.md:71`, AGENTS.md/CLAUDE.md |
| 4 | Dead path `local/c_controls/...` | `advanced/ai_agent_guide.md`, AGENTS.md |
| 5 | `.env` block contradicts actual `.env` (`ISAAC_SIM_USE_STANDALONE`, `ISAAC_SIM_GUI`) | `pegasus_scene_setup.md` |
| 6 | `fetch_scene.sh forest` — key doesn't exist | `simulation/scenes.md` |
| 7 | Marks tables missing `wiring`, `waypoint_flight` | `testing/index.md`, `ci_cd.md` |
| 8 | Getting Started "Move Robot" teaches superseded manual Foxglove import; wrong host port implication (8766 vs 8765) | `getting_started/index.md:109` vs `gcs/foxglove.md:16` |
| 9 | OSMO "recommended default" vs Getting Started "fallback" — positioning contradiction | `airstack_on_osmo.md` vs `getting_started/index.md` |
| 10 | Requirements three-way drift (GPU 3070/4080, disk 25GB/100GB, Ubuntu 22.04 vs 22.04/24.04) | `docs/README.md`, `about.md`, `getting_started/index.md` |
| 11 | `exploration/README.md` service described as toggling "the random walk planner" | copy-paste error |
| 12 | Task-action lists disagree | `tasks.md` vs `interface_conventions.md` §8 |
| 13 | `{==TODO==}` markers + never-built protocol | `global/planning/index.md`; `interface/index.md` (wrong diagram + wrong instructions, self-admitted) |
| 14 | git-hooks READMEs contradict `check-version-increment.yml` | `git-hooks/` |
| 15 | `end_to_end_testing.md` claims `autonomy` mark unregistered (it is registered) | troubleshooting row |
| 16 | Deprecated Omniverse Launcher / Streaming Client instructions | `export_stages_from_unreal.md`, `isaac_sim/docker.md`, `docker_usage.md` |
| 17 | `.devcontainer` layout described wrong (per-container subdirs; no Dockerfile) | `vscode_debug.md` |
| 18 | tmux window ordering disagrees between ms-airsim index and docker pages | `ms-airsim/*` |

---

# Phase 4 — Gap analysis (prioritized)

Derived from the code surface, not just existing docs. CLI surface from `airstack.sh:2289-2331` + `.airstack/modules/`; config surfaces from `.env` (18 vars), `config/vehicles/`, `config/fleets/`, `robot/docker/robot_name_map/`, `common/module_schema/`; message surface from `common/ros_packages/msgs/` (airstack_msgs: 10 msg + 4 srv, no README; task_msgs; behavior_tree_msgs); personas from the repo's own structure: **simulation user / module developer / robot deployer / GCS operator / stack maintainer / CI admin**.

| # | Gap | Quadrant | Evidence it's needed | Effort | Priority |
|---|---|---|---|---|---|
| G1 | **Deploy-to-hardware tutorial** | T | `deploying_to_hardware.md` is an explicit placeholder; it is the nav entry and `real_world/index.md`'s #1 "start here"; the TODO skeleton (l4t/voxl profiles, FCU_URL, identity verification) is already correct | L | **P0** |
| G2 | **`.env` / environment-variable schema reference** | R | 18 vars, no single schema page; the closest table carries phantom vars; `URDF_FILE`, `ISAAC_SIM_GUI`, `COMPOSE_PROFILES` coverage is one-line-in-passing | M | **P0** |
| G3 | **`airstack_msgs` (+ `behavior_tree_msgs`) interface reference** | R | package has no README and no docs page; doc standards and AGENTS.md cite message types that don't exist — nothing exists to check against | M | **P0** |
| G4 | **Resolve contradictory guidance** (data-offloading fork, git-hooks vs CI, OSMO positioning, requirements drift) | mixed | four cases where the site gives two conflicting answers; cheaper than any new doc and blocks trust | S each | **P0** |
| G5 | **Trajectory-library YAML config reference** | R | the one config a user edits to change candidate maneuvers; consumed by both DROAN planners; in-nav README is a 6-line TODO | M | **P0** |
| G6 | **HITL how-to modernization** | H | current page is pre-Foxglove, raw compose, ignores the `gcs-real` `hitl` profile that exists for exactly this; DDS-across-LAN (the hard part) undocumented | M | **P0** |
| G7 | **GCS operator tutorial** ("fly a mission from the GCS": tasks panel, waypoints, geofences) | T | GCS persona has no on-ramp; `usage/user_interface.md` is a 23-line duplicate stub; ingredients exist (waypoints page, rviz_tasks_panel README) | M | P1 |
| G8 | **Vehicle/fleet config schemas into the site** (`config/vehicles/README.md`, `config/local/README.md`, `common/module_schema/README.md` → nav) | R | first-class config schemas only reachable via GitHub URLs | S | P1 |
| G9 | **Complete CLI reference** (fix `--recreate`/`--scene`; add `osmo` group detail from tutorial's "under the hood" blocks; retire stale twin) | R | verification found the only phantom flag and the only missing flag on the page presenting itself as the reference | S | P1 |
| G10 | **Layer-index template fill** (autonomy, local, global, world_model, static_transforms, behavior) | R/E | 6 bad stubs incl. two that don't link their own children; uniform template: role (E) + child links + stack-launch note + spec-linked topics | M | P1 |
| G11 | **Module-developer on-ramp completion**: "your first module" tutorial step beyond the walkthrough, plus surfacing `module_schema` | T/R | modular AirStack is the strategic direction; walkthrough exists but stops at `module add`; `create-module` skill is agent-only | M | P1 |
| G12 | **Supported-platform matrix** (reference) | R | about.md promises Xavier NX/TX2/VOXL; installation covers only Orin; no doc says what is actually supported/tested | S | P1 |
| G13 | **Package README repairs**: vdb_mapping AirStack preamble; disparity stubs grown or collapsed to one page; exploration copy-paste fix; takeoff action-vs-service note; lidar filter + RViz panels into nav | R | five stubs and one upstream README are served in nav today | M | P1 |
| G14 | **Explanation: architecture-decision pages** — (a) why Docker-compose replicas + per-robot domain isolation, (b) why stacks/modules/fleets (standalone, no RFC references per the standalone-snapshot rule), (c) build-system design (two-tier images, cache tags) | E | rationale currently lives in RFC PRs, release notes, and half-pages (key_concepts, ci_cd); the snapshot rule forbids citing RFCs, so the rationale must be written down somewhere durable | M | P2 |
| G15 | **Sim-scene authoring path consolidation** (extract frame conventions from dead scene_setup.md; retire Ascent pair; fix Omniverse Launcher refs) | H/E | the current authoring story spans 5 pages across two eras | M | P2 |
| G16 | **Generator-template fixes** in `tools/gen_docs_catalog.py` (dedupe compat explainer, contextualize registry notes, compat-vs-checkout note) | R | template issues stamped onto every current and future module page | S | P2 |
| G17 | **Testing docs single-sourcing** (marks table lives once in tests/README; ci_cd audience split) | R/H | marks tables already drifted in 2 of 4 copies | S | P2 |

**Persona coverage after gaps close:** simulation user (T: getting_started ✓); module developer (T: modular walkthrough ✓ + G11); robot deployer (G1+G6+G12 — currently the unserved persona); GCS operator (G7); stack maintainer (stacks.md ✓ + G14); CI admin (ci_cd ✓ + orchestrator ✓).

---

# Phase 5 — Reorganization proposal

## 5.1 Target navigation (MkDocs Material tabs = quadrants)

Material's `navigation.tabs` maps naturally onto Diátaxis. Product areas (robot/sim/GCS) become *second-level groupings inside each quadrant*, which also fixes the 5-level depth violation. Existing URLs are preserved where possible; `mkdocs-redirects` (already a dependency) covers moves.

```yaml
nav:
  - Home: docs/index.md
  - Tutorials:                       # learning-oriented; numbered golden path
      - Get AirStack Flying: docs/getting_started/index.md          # clone → sim → fly
      - Modular AirStack Walkthrough: docs/getting_started/modular_airstack.md
      - Operate the GCS: docs/tutorials/gcs_first_mission.md        # NEW (G7)
      - Deploy to Hardware: docs/real_world/deploying_to_hardware.md # WRITTEN (G1)
      - What Next: docs/getting_started/tutorials_reference.md      # re-typed link hub
  - How-to Guides:
      - Development Environment: [local setup, vscode_debug, fork, OSMO remote dev]
      - Docker & Builds: [docker_usage (pruned), docker-build-profiles]
      - Simulation: [scenes: add a scene, spawning_drones, overhead_camera, export_stages, gimbal (moved here)]
      - GCS Operation: [waypoints_and_geofences, foxglove (operator half)]
      - Robot & Field: [installation, HITL (rewritten), robot identity overrides, logging, data offloading (storage-tools page)]
      - Modules & Stacks: [add a module, create a stack, module CI wiring, integration checklist]
      - Testing: [run system tests, add unit tests, use CI well (from ci_cd split)]
      - Contributing: [contributing, documentation guide, feature notebook, extend the CLI]
  - Reference:
      - CLI: beginner/airstack-cli/index.md (promoted; osmo section added)
      - Configuration: [.env schema (NEW G2), robot config (salvaged), vehicles/fleets/calibration schemas (nav-linked READMEs), module.yaml schema (nav-linked), robot name map]
      - Interfaces: [interface_conventions.md (canonical), airstack_msgs (NEW G3), task actions (tasks.md, reconciled), DDS router]
      - Containers: [robot docker, isaac-sim docker (ref half), ms-airsim docker (merged, ref half), gcs docker, simple_sim docker]
      - Stacks & Wiring: [stack READMEs + wiring.md, autonomy_modes (deployment matrix)]
      - Modules Catalog: docs/modules/* (generated)
      - Packages: [module READMEs incl. lidar filter, RViz panels; layer indexes as hubs]
      - Testing: tests/README.md (canonical marks/options)
      - Platform Matrix: NEW (G12)
      - Release Notes: docs/release_notes/index.md
  - Explanation:
      - System Architecture: system_architecture.md (post-split)
      - Key Concepts: key_concepts.md
      - Stacks, Modules & Fleets — Design: [modules.md, stacks.md, fleets.md (explanation cores)]
      - Robot Identity & Domain Isolation: robot_identity.md
      - Simulation Platforms & Tradeoffs: [simulation/index.md, isaac_sim/index.md, pegasus_scene_setup.md (rationale core), simple_sim/index.md, ms-airsim rationale]
      - CI Architecture: ci_cd.md (explanation half) + orchestrator
      - CLI Architecture: airstack-cli/architecture.md
      - Frames & Conventions: frame_conventions.md (grown; absorbs scene_setup's frame section)
  - About: docs/about.md (absorbs docs/README.md orientation content)
```

Notes: (a) this respects the ≤3-level rule; (b) tab names use plain words — "Explanation" could ship as **"Concepts"**, a common Material convention; (c) the AI-agent corpus (`.agents/skills/`, ai_agent_guide) stays outside the four tabs — it's a parallel audience; keep ai_agent_guide under Reference or Contributing.

## 5.2 Migration map (every doc → destination)

Actions: **keep** (move nav label only), **move**, **split**, **merge into**, **rewrite**, **delete** (with redirect), **new**.

| Current doc | Action → destination |
|---|---|
| getting_started/index.md | keep → Tutorials; fix Move Robot step, single-source requirements |
| getting_started/modular_airstack.md | keep → Tutorials; drop §1 duplicate, drop redundant `-f` flag |
| getting_started/tutorials_reference.md | rewrite → Tutorials "What Next" hub with quadrant-honest labels |
| tutorials/index.md | **delete** (zero inbound; superseded by hub) |
| tutorials/airstack_on_osmo.md | keep → How-to/Development Environment; normalize to `airstack osmo`; extract CLI blocks → CLI ref; resolve positioning vs Getting Started |
| docs/README.md | merge orientation content → about.md; keep file as GitHub-facing readme (already `exclude_docs`) |
| docs/about.md | keep → About; absorb README content; fix numbers, platform claims |
| release_notes/index.md | keep → Reference |
| development/index.md | rewrite → thin hub reflecting new tabs |
| beginner/key_concepts.md | keep → Explanation/Key Concepts; extract dev-loop to How-to |
| beginner/airstack-cli/index.md | keep → Reference/CLI (promoted); fix flags |
| beginner/airstack-cli/docker_usage.md | split: recipes → How-to/Docker & Builds; env-override material → .env schema; delete "Automated Testing" + dead Isaac streaming |
| beginner/development_environment.md | keep → How-to/Development Environment |
| beginner/vscode/vscode_debug.md | keep → How-to; fix .devcontainer paths |
| beginner/fork_your_own_project.md | keep → How-to; cross-link modules.md researcher workflow |
| development/development_environment.md (orphan) | **delete** + redirect to beginner version |
| development/airstack-cli/index.md (orphan) | **delete** + redirect to CLI reference |
| intermediate/contributing.md | keep → How-to/Contributing; fix mkdocs-serve inconsistency |
| intermediate/documentation.md | keep → How-to/Contributing; **update taxonomy to Diátaxis** (see 5.4); fix TrajectorySegment |
| intermediate/feature_notebook.md | keep → How-to/Contributing |
| intermediate/frame_conventions.md | grow → Explanation/Frames (absorb scene_setup frame section; add H1) |
| intermediate/docker-build-profiles.md | keep → Reference/Configuration (or How-to/Docker — it self-describes as overview; R-leaning) |
| intermediate/testing/index.md | keep → hub; marks table becomes link to tests/README |
| intermediate/testing/unit_testing.md | keep → How-to/Testing |
| intermediate/testing/end_to_end_testing.md | keep → Reference/Testing (fix stale troubleshooting row; label baselines) |
| intermediate/testing/ci_cd.md | split: "using CI" → How-to/Testing; architecture/security → Explanation/CI |
| intermediate/testing/testing_frameworks.md | **delete** (remove the 2 inbound links) |
| advanced/ai_agent_guide.md | keep → Reference (agents); fix c_controls, topic example |
| advanced/airstack-cli/architecture.md | keep → Explanation/CLI; refresh future-enhancements + module list |
| advanced/airstack-cli/extending.md | keep → How-to/Contributing |
| development/modules.md, stacks.md, fleets.md | keep → Explanation (cores); extract command tables → CLI ref |
| development/module_ci.md | keep → How-to/Modules & Stacks (move file under a consistent dir) |
| robot/index.md | keep → hub; topic table → spec links |
| robot/configuration/index.md | **rewrite** → Reference/Configuration (salvage ~40 real lines; delete fabricated examples) |
| robot/docker/index.md | keep → Reference/Containers; fix phantom env vars |
| robot/docker/robot_identity.md | keep → Explanation (canonical) |
| autonomy/index.md, local/index.md, global/index.md, global/world_model/index.md, static_transforms/index.md, behavior/index.md | **rewrite** to layer-index template (G10) |
| autonomy/system_architecture.md | **split** per 3.1 → Explanation core |
| autonomy/integration_checklist.md | keep → How-to/Modules & Stacks; replace R block with spec links |
| autonomy/interface_conventions.md | keep → Reference/Interfaces (canonical) |
| autonomy/tasks.md | keep → Reference/Interfaces; reconcile action list with spec §8; mark unimplemented prominently |
| autonomy/interface/index.md | rewrite → fix TODO diagram + wrong instructions (odometry_conversion is the real path) |
| autonomy/sensors/index.md, perception/index.md, coordination/index.md | keep (perception: fix invented topics) |
| autonomy/sensors/gimbal.md | **move** → How-to/Simulation |
| autonomy/local/{world_model,planning,controls}/index.md | keep; grow modestly |
| autonomy/global/planning/index.md | **rewrite** — delete never-built protocol; describe the real ExplorationTask/NavigateTask flow; keep example-planners section |
| autonomy/coordination/payloads.md | keep → How-to; de-dup vs skill (pick canonical) |
| autonomy/dds_router.md | keep → Reference/Interfaces |
| autonomy_modes.md | keep → Reference/Stacks & Wiring (deployment matrix); consider rename to match content |
| robot/optitrack.md | keep (tombstone) |
| robot/logging/index.md | keep → How-to/Robot & Field |
| robot/logging/rosbags.md | prune → AirStack-specific half only; link upstream for generic `ros2 bag` |
| robot/logging/data_offloading.md | **gut** → short pointer at real_world/data_offloading |
| simulation/index.md | keep → Explanation hub; remove ascent from Getting-Started list |
| simulation/scenes.md | keep → How-to+R; fix `forest` example |
| isaac_sim/index.md | keep → Explanation; extract USD naming convention → Reference |
| isaac_sim/docker.md | split per 3.1 |
| isaac_sim/pegasus_scene_setup.md | keep rationale core → Explanation; fix .env block; drop scene_prep dup (link spawning_drones) |
| isaac_sim/spawning_drones.md | keep → How-to (canonical scene_prep home) |
| isaac_sim/overhead_camera.md | split: sim half stays How-to/Simulation; satellite-map half → GCS |
| isaac_sim/ascent_sitl_extension.md + scene_setup.md | **delete** (after extracting frame conventions); remove nav entry + inbound link |
| isaac_sim/export_stages_from_unreal.md | keep → How-to; fix Launcher refs + dangling "next page" |
| ms-airsim/index.md + docker.md | **merge** → one overview+reference pair; resolve tmux disagreement |
| simple_sim/* | keep as-is (models) |
| gcs/index.md | keep → hub |
| gcs/docker/index.md | keep → Reference/Containers |
| gcs/usage/user_interface.md | **rewrite** → GCS operator tutorial or delete in favor of G7 page |
| gcs/foxglove.md | split per 3.1 |
| gcs/waypoints_and_geofences.md | keep → How-to/GCS (model page) |
| real_world/index.md | keep → hub; stop recommending the placeholder until G1 lands |
| real_world/deploying_to_hardware.md | **write** (G1) → Tutorials |
| real_world/installation/index.md | grow → How-to/Robot & Field; fix flag order + `./airstack.sh` |
| real_world/HITL/index.md | **rewrite** (G6) |
| real_world/data_offloading/index.md | keep → How-to (canonical); fix dangling cross-ref |
| docs/modules/* | keep (generated); fixes in `gen_docs_catalog.py` (G16) |
| stacks/*/README.md | keep in nav (model pattern) |
| package READMEs in nav | keep; repairs per G13 |
| + add to nav: `lidar_point_cloud_filter`, `rviz_tasks_panel`, `3d_waypoint_rviz2_plugin`, `common/module_schema`, `config/vehicles`, `config/local`, osmo admin guide (symlink pattern à la ci-cd-orchestrator) |
| git-hooks READMEs | **fix or delete** (G4) |

## 5.3 PR sequence (each independently landable)

1. **PR-1: Accuracy hotfixes** (no moves; ~20 point edits from table 3.6 + exploration/AGENTS fixes). Zero risk, immediate trust payoff. Includes the three "pick one answer" decisions that need a maintainer call: OSMO positioning, canonical requirements numbers, canonical task-action list.
2. **PR-2: Delete the fossil stratum** — 5 orphans/deprecated pages (+ git-hooks READMEs), redirects via `mkdocs-redirects`, remove inbound links, extract scene_setup frame-conventions into frame_conventions.md first.
3. **PR-3: Surface orphaned knowledge** — nav-link the 7 off-site READMEs/schemas (G8, items in 3.5); pure `mkdocs.yml` + symlink work.
4. **PR-4: Nav re-labeling to quadrants** — restructure `mkdocs.yml` into the 5.1 shape *without moving files* (Material nav is path-independent); rewrite the two hub pages (`development/index.md`, `tutorials_reference.md`). This lands the visible Diátaxis change cheaply.
5. **PR-5: De-duplication** — canonical tables + links (topic tables → spec; flags → CLI ref; marks → tests/README; scene_prep → spawning_drones; requirements → getting_started; ms-airsim merge).
6. **PR-6: Content splits** — system_architecture, ci_cd, foxglove, isaac/ms-airsim docker pages, modules/stacks/fleets table extraction. One split per commit; wiring.md-style verification not needed (docs only) but run `airstack docs` build per commit.
7. **PR-7: Rewrites of wrong content** — global/planning, interface/index, configuration/index, HITL, rosbags prune, data-offloading gut, layer-index template fill (G10).
8. **PR-8+: New gap-filling docs** (one PR each): `.env` schema (G2), airstack_msgs reference (G3, plus a package README), deploy-to-hardware tutorial (G1), GCS operator tutorial (G7), trajectory-library YAML reference (G5), platform matrix (G12), explanation pages (G14), generator-template fixes (G16).
9. **PR-final: Standards update** — CONTRIBUTING decision tree (5.5) + reconcile `documentation.md` and the `write-mkdocs-documentation` / `update-documentation` skills with the new taxonomy, so the structure stays clean.

Rationale for order: deletions and nav moves are cheap and reversible; splits change URLs/anchors so they come after redirect infrastructure is exercised; new docs last because their correct homes only exist after PR-4.

## 5.4 Conflicts with existing doc-authoring standards (flagged, not overridden)

| Standard | Conflict | Recommendation |
|---|---|---|
| `write-mkdocs-documentation` §Organization Pattern | Prescribes `development/{beginner,intermediate,advanced}/` — difficulty tiers, orthogonal to Diátaxis; this is the root cause of the "Tutorials ×3" nav | Update the skill's pattern to the quadrant layout in the same PR as the nav change (PR-4/PR-final) |
| `write-mkdocs-documentation` §Navigation | "Max 3 levels" — currently violated by the repo's own nav (5 levels under Robot) | Target nav restores compliance; no standard change needed |
| `write-mkdocs-documentation` §Motivation before mechanics | Every page must open with why-it-exists — mild tension with Diátaxis's austere reference ideal | Keep the rule but cap it at one short paragraph for Reference pages (the interface-conventions spec shows this works) |
| `documentation.md` §Documentation Types | Three-type taxonomy (Tutorials / Guides / Reference) — no Explanation quadrant; "Guides: in-depth topic coverage" conflates how-to and explanation | Replace with the four-quadrant decision tree (5.5) |
| Module README template (documentation.md + package_template) | Intentionally all-quadrant (overview+algorithm+reference+usage in one README) | **Keep** — Diátaxis governs the site IA; a package README legitimately serves as the package's complete doc. But fix the template's nonexistent `TrajectorySegment` example |
| Standalone-snapshot / Release Notes rule | No conflict — actively Diátaxis-compatible (bans change-relative prose from feature docs) | Keep; it also forces G14's rationale pages to be written standalone rather than citing RFCs |

## 5.5 Proposed CONTRIBUTING addition — "Which kind of doc am I writing?"

```markdown
## Adding documentation

Every docs page is exactly one of four kinds. Pick with this decision tree:

1. Is the reader *learning by doing*, following you step-by-step to a
   guaranteed result, with no decisions to make?
   → **Tutorial** (`nav: Tutorials`). One golden path, numbered steps,
   ends with something working. Repeating reference/how-to content is fine.
2. Is the reader *accomplishing a task* they already understand, and might
   they enter with their own context (different robot, different sim)?
   → **How-to guide** (`nav: How-to Guides`). Goal in the title
   ("Add a scene to the catalog"). Assume competence; link concepts,
   don't teach them.
3. Is the reader *looking something up* — a flag, a topic, a schema,
   a default?
   → **Reference** (`nav: Reference`). Tables over prose. Complete or
   say what's missing. Mirror the code's structure. One canonical home
   per fact — link to it, never copy it.
4. Is the reader trying to *understand why* the system is shaped this way?
   → **Explanation** (`nav: Explanation`). No steps, no tables of options.
   Written standalone (see the Release Notes rule — no RFC/PR references).

If your draft does two of these, it's two pages. The most common failure:
a how-to that stops to explain design rationale (move it to Explanation
and link), or a concept page that accumulates setup steps (move them to
a how-to). Package READMEs are exempt: they intentionally combine all
four for one package (use the template).
```

---

## Validation plan (for the migration PRs, not this audit)

- **a-build:** `airstack docs` builds clean per PR; `mkdocs build --strict` catches broken nav/links.
- **b-redirects:** every deleted/moved path has a `redirect_maps` entry; spot-check old URLs on the built site.
- **c-accuracy:** re-run the mechanical checks from `results/b-verification/` after PR-1 (grep for the 18 defects — all should be gone).
- **d-single-source:** grep proves each hot table (topics, flags, marks, requirements) exists in exactly one file.

## Artifacts

- `results/inventory_raw.md` — full 173-row inventory (lines + git dates)
- `results/a-classification/` — five verbatim/condensed classifier reports (getting-started, development, robot, sim/gcs/real-world/modules, scattered READMEs)
- `results/b-verification/code_verification.md` — mechanical verification (nav coverage, CLI flags, stale-concept sweep, env vars, links, msgs)
