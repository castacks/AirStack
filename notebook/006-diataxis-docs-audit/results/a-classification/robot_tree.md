# Agent report C — docs/robot tree (30 files)

(Condensed archive of classifier output. T=Tutorial H=How-to R=Reference E=Explanation.)

## Per-file classification

| File | Lines | Quadrant | Key findings |
|---|---|---|---|
| robot/index.md | 121 | R (high) | Topic table is a 3rd copy of interface_conventions data (drift hazard); `global_bringup` mention is ACCURATE (package still exists) |
| configuration/index.md | 369 | H (low) | ~300/369 lines generic or FABRICATED config examples (camera/IMU/planner YAML matches no real files; invented validation script); stack-selection para is current; useful core ~40 lines |
| docker/index.md | 113 | R (high) | STALE: `ROBOT_LAUNCH_PACKAGE`/`ROBOT_LAUNCH_FILE` env rows — real compose var is `LAUNCH_PACKAGE`; profile table overlaps autonomy_modes.md (sync hazard) |
| docker/robot_identity.md | 158 | E/R hybrid (E primary) | Healthiest doc in tree; canonical copy, correct pointers elsewhere |
| autonomy/index.md | 22 | R nav stub | Front door to autonomy docs, offers no orientation |
| autonomy/system_architecture.md | 643 | E primary, tri-quadrant | Node-types & task-cascade E is excellent; per-layer topic lists = 4th copy of interface data with errors (`trajectory_controller/cmd_vel` doesn't exist); sequence diagram shows nonexistent CTL→BEH completion flow; "Performance Characteristics" numbers look invented (phantom 10Hz BT ticker); "Module Integration Guidelines" duplicates checklist at lower fidelity |
| autonomy/integration_checklist.md | 553 | H primary | Core 10-step checklist good; ~110-line R block restates spec AND drifts (`interface/cmd_vel` Twist — spec/code say `interface/cmd_velocity` TwistStamped); `.agents/skills` relative links likely break on built site |
| autonomy/interface_conventions.md | 244 | R (very high) | Platonic reference doc: versioned, semver'd, observed-graph-wins; canonical source others clone from; freshest doc |
| autonomy/tasks.md | 278 | R primary | 3 of 6 action specs "(not yet implemented)"; tasks.md and spec §8 give TWO DIFFERENT "complete" task lists (spec: takeoff/land/fixed_trajectory/navigate/exploration/semantic_search; tasks.md: fixed_trajectory/navigate/exploration/coverage/semantic_search/chat) |
| autonomy/interface/index.md | 112 | E (med) hybrid | WORST cosmetic staleness: `==TODO: This is not our diagram, must replace.==` + hotlinked 404warehouse.net image; L94 TODO admits "Broadcast State" instructions are wrong (real path is odometry_conversion node) |
| autonomy/sensors/index.md | 70 | R (high) | Model layer index — what others should look like; fully stack-era |
| autonomy/sensors/gimbal.md | 33 | H (high) | Isaac Sim UI guide misplaced under robot/autonomy/sensors — belongs in docs/simulation/isaac_sim/; fragile Google Drive iframe |
| autonomy/perception/index.md | 54 | R (med-high) | Outputs list contradicts spec (plain `/odometry` is v2 target, v1 is `odometry_conversion/odometry`; `/pose`,`/imu/data` invented); `perception_bringup` mention accurate |
| autonomy/local/index.md | 9 | R stub (bad) | Doesn't link its own children (world_model/planning/controls) |
| autonomy/local/world_model/index.md | 9 | E/R stub (good) | Thin but accurate; right shape |
| autonomy/local/planning/index.md | 12 | E/R stub (good) | Current (full_default vs full_droan_cpu selection verified) |
| autonomy/local/controls/index.md | 10 | E/R stub (good) | pid_controller HAS NO README to link (upstream gap) |
| autonomy/global/index.md | 13 | R stub (bad) | No links to children; no E |
| autonomy/global/planning/index.md | 100 | E/R — WRONG CONTENT | Most substantively stale doc: documents never-built Global-Manager/PlanRequest protocol (PlanRequest.msg exists but NO node implements it); two live `{==TODO==}` markers; contradicts tasks.md/spec §8/task cascade |
| autonomy/global/world_model/index.md | 5 | E fragment — emptiest stub | Links external GitHub instead of in-tree vdb_mapping_ros2 README; calls shipped default a "placeholder" |
| autonomy/behavior/index.md | 19 | R/E stub | Para 1 stale generic claim contradicts its own paras 2-3 (accurate: safety monitor only, tasks moved to GCS) |
| autonomy/coordination/index.md | 66 | E/R hybrid (model) | Arguably the model layer index |
| autonomy/coordination/payloads.md | 111 | H (high) | Deliberately mirrors attach-gossip-payload skill — two maintained copies of one workflow |
| autonomy/dds_router.md | 170 | R primary + E | Current (RFC-379 split-stack, bridge.yaml); launch-tool section could stand alone as feature reference |
| autonomy_modes.md | 181 | R primary + H | Fully modernized; title/filename mismatch is legacy echo; profile table overlaps docker/index.md |
| optitrack.md | 9 | H-pointer (tombstone) | Correct extraction tombstone; but 3 OptiTrack entry points exist (here, perception/index, docs/modules/optitrack.md) |
| static_transforms/index.md | 10 | R fragment — real gap | Title promises how-to that doesn't exist; frame conventions triplicated (system_architecture, spec — spec's is best) |
| logging/index.md | 30 | H (high) | Complete for scope |
| logging/rosbags.md | 209 | H (med) | ~Half is generic upstream `ros2 bag` tutorial; example output mixes 2026 filename with Mar 2024 dates and wrong `/robot1/` namespaces (real: robot_1) |
| logging/data_offloading.md | 322 | H in form — generic Linux rsync/cron/systemd cookbook | Invented paths (/opt/airstack/bags — real: robot/bags→/bags or /media/airlab/Storage); NOT a duplicate of docs/real_world/data_offloading — DIVERGENCE: real_world page documents the actual storage_tools_server/device workflow; this page ignores it. Gut to a pointer |

## Synthesis

1. **Stub epidemic, two diseases**: good stubs (local/world_model, local/planning, local/controls, optitrack, logging/index) need modest growth; bad stubs (autonomy/index, local/index, global/index, global/world_model, static_transforms, behavior/index) fail as navigation/orientation. Fix: uniform layer-index template (role E, children links, stack-launch note, interchange topics as spec links).
2. **One canonical reference, three decaying clones**: interface_conventions.md is canonical; robot/index.md, integration_checklist.md, system_architecture.md carry drifting topic-table copies. Highest-leverage cleanup: replace cloned tables with spec links.
3. **Fabricated/wrong-quadrant content in three files**: configuration/index.md, logging/data_offloading.md, half of logging/rosbags.md — cut to AirStack-specific cores + pointers.
4. **Stack/fleet migration mostly landed** (no local_bringup or ./airstack.sh refs in docs/robot; AUTONOMY_ROLE story consistent). Surviving rot is architectural: global/planning/index.md (never-built protocol), interface/index.md (TODO/wrong diagram), system_architecture.md (invented perf numbers).
5. **Split only system_architecture.md** (keep E core; push topic lists to spec links; delete/measure perf tables; merge integration guidelines into checklist). Leave working hybrids alone (robot_identity, dds_router, coordination/index, interface_conventions).
