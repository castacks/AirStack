# Agent report E — docs outside docs/: package READMEs, infra docs, orphaned knowledge

(Condensed archive of classifier output.)

## Package READMEs in robot/ros_ws/src (16 found)

Stubs IN NAV: trajectory_library (6 lines, "Docs TODO" — the trajectory-library YAML config format that both DROAN planners load is documented NOWHERE); disparity_graph_cost_map (7); disparity_graph (9); disparity_expansion (17, Bitbucket-era personal-gmail contact). mav_comm (3, vendored, not in nav — acceptable).

Substantial: trajectory_controller README (217) FULLY meets template — best module README in repo. droan_local_planner (168) + droan_gl (164) good E+R, missing mermaid/Testing. takeoff_landing_planner (120) pure R; its test/README (171) describes the LEGACY service interface while parent documents action servers — neither notes both coexist. random_walk (99) good. exploration (55) STALE/ERRONEOUS: `~/global_plan_toggle` described as toggling "the random walk planner" (copy-paste error); references pre-Tasks-Panel GUI. vdb_mapping_ros2 (167) upstream FZI README unmodified — in nav but ZERO AirStack integration info (Foxy links, clone-and-colcon irrelevant in containers). lidar_point_cloud_filter (47) fresh + full-quality but NOT in nav (only full-quality robot README not surfaced; raw-vs-filtered point_cloud contract lives there).

## Stacks READMEs (all in nav)
Clean division of labor: READMEs = intent/rationale (E), generated wiring.md = observed graph (R), each pointing at the other. One of the better Diátaxis-conformant patterns in the repo. lite_offload_global strongest (safety invariant articulated); notes own wiring.md not committed yet.

## Tests
tests/README.md (665) comprehensive R+H, fresh, section discipline holds. ci-cd-orchestrator.md = symlink to .github/orchestrator/README.md (in nav ✓). meta/integration/robot/sim sub-READMEs fresh, linked from tests/README — fine off-nav.

## common/
- module_schema/README.md (96): authoritative module.yaml field/validation reference — NOT in nav (only linked from modules.md).
- coordination/README.md (76): fresh, covered by docs pages — low orphan risk.
- rviz_tasks_panel/README.md (184): operator manual for the 8 task tabs lives ONLY here; tasks.md only name-drops.
- 3d_waypoint_rviz2_plugin/README.md (142): ZERO mention anywhere in docs/.
- bag_recorder_pid/README.md (454, IN NAV): bloated — duplicated param tables, 80 lines trivial pub/sub samples, "License: TODO", personal-gmail maintainer, "Humble or later" header. Needs 50% cut.

## Infra (not in site)
- .github/orchestrator/README.md: in nav via symlink ✓.
- osmo/README.md (301): admin half (pool requirements, buildx amd64 pitfall, Nucleus TLS debugging) invisible on site; complementary to student tutorial, zero overlap — worth nav entry/symlink.
- .airstack/README.md (67): per-source-file CLI command map only here.
- git-hooks/README.md + docker-versioning/README.md: STALE AND CONTRADICTORY — instruct installing a pre-commit hook writing the git hash into .env VERSION, but check-version-increment.yml requires semver strictly greater than base; following them = red PR gate. Fix or delete (near-duplicate copies).
- config/vehicles/README.md (48): vehicle.yaml schema documented ONLY here; fleets.md links out via GitHub tree URL, not same-dir nav.
- config/local/README.md (26): CALIBRATION_DIR unit-calibration contract only here + vehicles README.

## AGENTS.md vs docs site
Well-mirrored; little truly orphaned. Residuals: full per-skill catalog, capture-discovered-knowledge meta-workflow. STALE detail: AGENTS.md reference table says `local/c_controls/trajectory_controller` — actual path is `local/controls/trajectory_controller`.

## Orphaned-knowledge ranked findings
1. git-hooks READMEs contradict live version gate — fix/delete.
2. vehicle.yaml schema nowhere in docs/ — add config/vehicles/README.md (+config/local) to nav.
3. Trajectory-library YAML format undocumented anywhere (in-nav 6-line stub).
4. RViz operator GUIs (rviz_tasks_panel, 3d_waypoint plugin) documented only in package READMEs — nav-link them.
5. osmo/README.md admin half off-site — nav entry or symlink.
6. common/module_schema/README.md off-site — nav-link.
7. lidar_point_cloud_filter README not in nav while five stubs are.
8. vdb_mapping_ros2 in-nav page is upstream-only — needs AirStack preamble.
9. Stale in-nav stubs (3 disparity + exploration's wrong service description) — fix copy-paste error; grow or collapse the disparity entries.
10. Minor: takeoff_landing action-vs-service interface confusion; bag_recorder "License: TODO".
