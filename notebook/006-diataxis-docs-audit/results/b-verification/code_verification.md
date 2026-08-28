# Agent report F — mechanical verification (docs vs code)

## 1. Nav coverage
- docs/README.md — intentionally excluded via exclude_docs (mkdocs.yml:7-10). Not a gap.
- TRUE ORPHANS (zero inbound links): docs/development/airstack-cli/index.md, docs/development/development_environment.md, docs/tutorials/index.md.
- Not in nav but link-reachable from live pages: docs/development/intermediate/testing/testing_frameworks.md (from testing/index.md:89, unit_testing.md:214), docs/simulation/isaac_sim/scene_setup.md (from isaac_sim/docker.md:82).
- Nav entries pointing at nonexistent files: NONE.

## 2. CLI accuracy
Source: repo-root airstack.sh (commands at :2289-2331) + .airstack/modules/*.sh.
- `airstack up` actual flags (parse_launch_intent, airstack.sh:1143-1161): --sim, --robots, --headless, --play, --no-play, --no-autolaunch, --stack, --fleet, --scene, --wait, --dry-run (+compose passthrough).
- PHANTOM FLAG: docs/development/beginner/airstack-cli/index.md:129 documents `--recreate` — does not exist in airstack.sh or docker compose (only --force-recreate/--no-recreate). Would fail.
- MISSING FROM CLI REF: `up --scene` absent from the flags table (documented only in simulation/scenes.md).
- Effectively-complete CLI reference exists: beginner/airstack-cli/index.md (only gaps above). Top-level airstack-cli/index.md is the stale orphaned predecessor.
- All other checked flags documented somewhere (module --no-hooks, doctor --strict, lock --check-conflicts/--build, pytest opts, ready --json).

## 3. Stale concept sweep
- AUTONOMY_ROLE: all 10 hits are "removed/preflight error" statements — acceptable. No doc instructs setting it.
- ./airstack.sh: legitimate bootstrap uses OK; airstack_on_osmo.md uses `./airstack.sh osmo ...` style in 13 places (should be `airstack osmo`); stale orphan CLI page built around it; extending.md:170 uses it in a worked example.
- Bringup packages: autonomy_bringup, global_bringup, interface_bringup, perception_bringup STILL EXIST (docs referencing them are accurate); local_bringup/sensors_bringup/behavior_bringup removed — zero live doc references outside release notes. desktop_bringup exists in common/.
- robot_bringup: zero hits. Humble: one historical release-note hit only.

## 4. Env var reference
.env has 18 uncommented vars. NO single complete .env schema page. Partial tables: docs/robot/docker/index.md:96-113 (10 rows, robot-only, includes PHANTOM ROBOT_LAUNCH_PACKAGE/FILE) and docs/robot/configuration/index.md:19-33 (4 vars). Every var documented somewhere, but thin: URDF_FILE (one fleet table row), ISAAC_SIM_GUI/ISAAC_SIM_USE_STANDALONE (sim docker pages only), RECORD_BAGS, COMPOSE_PROFILES (never schema'd).

## 5. Dead links
0 broken relative-link file targets across the six checked high-traffic pages (anchors not validated).

## 6. Message/interface reference
- airstack_msgs at common/ros_packages/msgs/airstack_msgs: 10 .msg, 4 .srv, 0 .action; NO README.md; NO docs reference page. Partial type coverage in interface_conventions.md:117-121,150,173 and integration_checklist.md:82-85. task_msgs actions well documented in tasks.md. behavior_tree_msgs documented nowhere.
- STALE TYPE: docs/development/intermediate/documentation.md:71 uses `airstack_msgs/TrajectorySegment` in its README template example — no such message (real: TrajectoryXYZVYaw). AGENTS.md/CLAUDE.md standard-topics table cites TrajectoryOverride and TrajectorySegment — NEITHER exists in the package.
