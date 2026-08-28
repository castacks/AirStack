# Results Summary — Diátaxis Docs Overhaul

**Branch:** `feature/diataxis-docs-overhaul` (off develop @ 02a12d56) · 9 commits, intended for squash-merge into develop.
**Validation:** `mkdocs build --strict` passes after every commit and at HEAD; regression greps for all 18 audited defects return zero hits; working tree clean.

## Commits (implements design_spec.md §5.3, PR-sequence collapsed to one branch)

| # | Commit | Plan phase | Net effect |
|---|---|---|---|
| 1 | `6d0bf18d` fix 18 verified accuracy defects | PR-1 | phantom flags/env vars/types gone; requirements & OSMO positioning reconciled; spec → v1.0.1 (all 8 task actions) |
| 2 | `31f2ffc2` remove fossil pages, add redirects, deprecate git-hooks | PR-2 | 6 pages + 6 orphaned images deleted with `mkdocs-redirects` entries; frame_conventions.md rewritten from live facts (the map_FLU story matched nothing in code) |
| 3 | `20a6b1f6` surface orphaned knowledge | PR-3 | 7 off-site READMEs/schemas nav-linked (vehicles, calibration, module.yaml, lidar filter, 2 RViz panels, OSMO admin) |
| 4 | `6197dd8c` restructure nav into quadrant tabs | PR-4 | Tutorials / How-to / Reference / Concepts tabs; no file moves; ≤3-level rule restored; both hub pages rewritten |
| 5 | `d2dbe79c` de-duplicate to canonical homes | PR-5 | −131 lines of clones; MS-AirSim pair merged — both pages had tmux ordering backwards (fixed from entrypoint.sh) |
| 6 | `25ccbd84` split hybrid pages | PR-6 | system_architecture 643→375 (fabricated perf tables + wrong data-flow arrow removed); 3 new pages: using_ci, extending_foxglove, container_workflows |
| 7 | `1b988c99` rewrite wrong/filler content + layer stubs | PR-7 | −600 lines invented material (never-built Global-Manager protocol, fabricated configs, rsync cookbook); HITL modernized; 6 layer indexes filled |
| 8 | `4c3ccf3d` fill P0/P1 gaps | PR-8 | new: .env schema, airstack_msgs reference, trajectory-library YAML reference, platform matrix, Deploy-to-Hardware tutorial, Operating-the-GCS guide |
| 9 | `65e3930d` standards + version | PR-9 | Diátaxis decision tree in doc standards + both mkdocs skills; VERSION → 0.20.0-alpha.15; release-notes entry |

## Maintainer decisions made (flag for review at merge)

1. **OSMO positioning**: local Linux+GPU stays the golden path; OSMO reframed as "recommended *remote* development path".
2. **Requirements canonicalized**: RTX 3070 min / 4080+ recommended; ~25 GB images, 100 GB free disk recommended; Ubuntu 22.04/24.04.
3. **Task-action canon**: the 8 `task_msgs` actions; spec §8 and tasks.md both list all 8 with honest implementation status.
4. **about.md platform list**: Xavier NX / TX2 removed (zero repo evidence); Orin + VOXL 2 per compose profiles.

## Extra defects found & fixed during implementation (beyond the audit's 18)

- Both MS-AirSim pages had the tmux window ordering backwards; MAVLink-port troubleshooting bullet wrong twice over (entrypoint.sh + settings.json.j2 are the ground truth).
- `random_walk` declares the `global_plan_toggle` remap but never serves it — toggle now attributed only to the exploration planner.
- scene_setup.md's frame section (the piece the audit wanted preserved) was itself stale: no `map_FLU` anywhere in code; live truth is the `world → map` identity TF in robot.launch.xml:52-55.
- `docs/robot/docker/index.md` base-image rows lagged compose (ubuntu22.04 → ubuntu24.04, l4t-jetpack → dustynv jazzy).
- `PlanToWaypoint.srv` exists but is missing from `rosidl_generate_interfaces` (documented as "defined but not built").
- Foxglove robot-commands panel has no airborne gating (RViz panel does) — documented accurately instead of claiming parity.

## Follow-ups deliberately NOT done (out of docs scope or deferred)

- **Code change**: remove the obsolete `airstack config git-hooks` CLI path (config.sh:86-122 still installs the hook that the semver gate rejects; READMEs now warn about it).
- **G16**: `tools/gen_docs_catalog.py` template fixes (dedupe the per-page compat explainer, contextualize registry notes).
- **G14**: standalone architecture-decision explanation pages (containers/replicas rationale, build-system design) — partial coverage exists in key_concepts/ci_cd.
- modules/stacks/fleets command-table extraction into the CLI reference (audit rated it low priority; pages are fresh).
- `PlanToWaypoint.srv` build omission and the pid_controller missing README are upstream code/package gaps.

## Amendment round (commit 10, `c98a3d1e`)

Per user follow-up: six new how-to guides — Adding a State Estimator, Adding a Planner, Creating a Multi-Agent Coordination Algorithm (all grouped under a new **How-to → Autonomy** section, which also absorbed the Integration Checklist and Coordination Payloads), Creating a Custom Stack Topology (Modules & Stacks), Adding a Vehicle Type/Unit/Platform (Robot & Field), and Getting the Most out of Your Coding Agent (Contributing). Each presents the in-tree-package vs `airstack module create --in-tree` module-scaffolding choice per the user's mid-flight addition. The **Concepts tab moved to directly after Tutorials**. The UE→Isaac export tutorial now embeds the new walkthrough video (cMjO7Sb7Zmo), instructs Z-up + meters export (contra the video), and warns that UE Decals (paint markings, dirt, puddles) don't export.

Accuracy notes from this round: the planner guide verifies flights with `-m waypoint_flight` (the `autonomy` mark never exercises `tasks/navigate`); the estimator guide documents that `odometry_conversion` is launched unconditionally and owns the map→base_link TF, so the supported swap is the `interface_odometry_in_topic` launch arg; the coordination guide recommends `coordination/peer_registry` (RELIABLE/TRANSIENT_LOCAL) over raw domain-99 `/gossip/peers` and lists the platform's honest gaps (no relay, no payload hashing, no peer eviction).

## Tutorial round (commit 11, `d73d44e7`)

Five beginner tutorials added, completing the Tutorials-tab learning path: Fly a Mission from the GCS → Change a Parameter → (Modular Walkthrough) → Write Your First Module → Your First Fleet → Build and Fly Your Own Scene → (Deploy to Hardware). Several steps were **executed, not just read**: the module scaffold was generated for ground truth (and its launch stub's double-namespace gotcha documented), the two-robot fleet YAML validated via resolve_fleet.py, and the scene-catalog resolution check run against a scratch catalog. The parameter tutorial verified the edit→relaunch loop needs no rebuild (symlink-install from the bind-mounted source) and that the panel's velocity=0 falls back to the config value (takeoff_landing_task.cpp:216).

How-to additions: **Adding a Controller** (verified chain: trajectory_controller `tracking_point` → pid_controller → `interface/cmd_roll_pitch_yawrate_thrust`; swap-the-PID vs own-the-§5-surface paths with enumerated obligations) and **Adding a Planner** renamed/expanded to **Adding a World Model and Planner** (local WM↔planner is a matched pair wired per-stack — not a spec interchange — vs the spec'd §3 global map; full_droan_cpu as the pair-swap example).
