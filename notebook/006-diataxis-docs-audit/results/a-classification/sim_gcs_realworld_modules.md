# Agent report D — docs/simulation, docs/gcs, docs/real_world, docs/modules

(Condensed archive of classifier output.)

## Simulation

| File | Lines | Quadrant | Key findings |
|---|---|---|---|
| simulation/index.md | 201 | E hub (med) | 4-quadrant hybrid; env table matches .env; config table + troubleshooting duplicated in isaac_sim/docker.md |
| simulation/scenes.md | 213 | R (high) | Well-composed hybrid; BUG: `fetch_scene.sh forest` — `forest` is not a valid key; MS-AirSim scene fetch story told 3× |
| isaac_sim/index.md | 57 | E (high) | USD naming convention (*.prop/*.stage/*.robot/*.scene.usd) is the ONLY normative statement of that convention, buried here |
| isaac_sim/docker.md | 467 | R (med), 40% how-to | "Everything about the container" page; verbatim compose YAML rots silently; version-pinned "Isaac-Sim Full/5.1" path in prose; deprecated Omniverse Launcher-era streaming refs; links to orphaned scene_setup.md |
| isaac_sim/pegasus_scene_setup.md | 196 | E leaning R (ambiguous) | STALE: its ".env" block shows ISAAC_SIM_USE_STANDALONE=false + omniverse:// GUI path — actual .env says true + local simple_pegasus.scene.usd; scene_prep.py helper table duplicates spawning_drones.md function-for-function |
| isaac_sim/spawning_drones.md | 213 | H (high) | Strongest-written sim page; current (fleet_spawn.py, asm_optitrack post_spawn) |
| isaac_sim/overhead_camera.md | 135 | H (high) | Title/nav/filename disagree; half the page (satellite Map panel) is GCS content filed under simulation |
| isaac_sim/ascent_sitl_extension.md | 22 | E, deprecated stub | Self-declared deprecated but still in nav and in simulation/index.md "Getting Started" list |
| isaac_sim/export_stages_from_unreal.md | 37 | H (med) | Recommends discontinued Omniverse Launcher; "next page" sentence written for old ordering |
| isaac_sim/scene_setup.md | 51 | H — SEVERELY STALE | NOT in nav (orphan) but linked from isaac_sim/docker.md; documents pre-Pegasus Ascent pipeline; ONLY its Frame Conventions section (map_FLU→map TF) is unique live knowledge — extract before archiving with ascent_sitl_extension.md |
| ms-airsim/index.md | 187 | R w/ how-to head (med) | ~35% overlap with ms-airsim/docker.md; the two DISAGREE on tmux window ordering |
| ms-airsim/docker.md | 339 | R (med-high) | Same everything-page genre; GPU/imaging/networking boilerplate cloned across all docker pages |
| simple_sim/index.md | 67 | E (med) | Clean, current; names a specific maintainer (rot-prone) |
| simple_sim/docker.md | 75 | R (high) | The right size for the docker-page genre — model to converge on |

## GCS

| File | Lines | Quadrant | Key findings |
|---|---|---|---|
| gcs/index.md | 123 | E hub (med) | Current; launch story told 3× (index, docker/index, foxglove) |
| gcs/docker/index.md | 166 | R (high) | Current, fleet-era; SSH root password published (flag for gcs-real field profile on host network) |
| gcs/usage/user_interface.md | 23 | stub (no real quadrant) | 100% duplicated content; the page where a real operator tutorial should live and doesn't |
| gcs/foxglove.md | 160 | H+R (med) | Excellent but two audiences on one page (operator how-to + maintainer guide to editing foxglove_visualizer_node.py); .agents/skills link may break on built site |
| gcs/waypoints_and_geofences.md | 87 | H (high) | Cleanest single-quadrant GCS page; closest thing to a GCS tutorial |

**GCS IA inverts Diátaxis**: "Usage" section holds only a 23-line stub while real operational knowledge lives in foxglove.md/waypoints page. No GCS tutorial exists.

## Real World — weakest area by a wide margin

| File | Lines | Quadrant | Key findings |
|---|---|---|---|
| real_world/index.md | 173 | E overview (med) | Stack-topology section fully modernized; but fans out to 1 placeholder + 2 stubs |
| deploying_to_hardware.md | 27 | Tutorial (intended) — EXPLICIT PLACEHOLDER | Steps live in an HTML TODO comment; is the nav entry AND index's #1 "start here" recommendation |
| installation/index.md | 32 | H, skeletal | Only covers Orin (VOXL/Xavier/TX2 promised by index, documented nowhere); leads with ./airstack.sh setup; `airstack --profile l4t up` flag order inconsistent with everything else |
| HITL/index.md | 24 | H — stalest recommended page in the audit | Raw docker compose; RViz-based verification from pre-Foxglove era; ignores the gcs-real `hitl` profile that exists for this exact case; cross-network DDS (the hard part) reduced to "ensure ping works"; Google Drive /view iframes likely refuse to embed |
| data_offloading/index.md | 80 | H (high) | The ACTUAL tool (castacks storage_tools_server/device); dangling "optional setup" cross-ref; admin password published |

**Data-offloading fork**: robot/logging/data_offloading.md (323-line generic rsync/systemd cookbook, paths that don't exist in repo, presented as product) vs real_world/data_offloading/index.md (the real tool). Contradictory guidance; make storage-tools canonical.

## Modules catalog (generated — verified: tools/gen_docs_catalog.py exists, all 4 pages carry do-not-edit header)

- modules/index.md (52): clean Reference catalog.
- dfm2_disturbances/macvo/optitrack (56 each): template-uniform Reference; issues are GENERATOR-TEMPLATE design choices: identical 8-line compat explainer stamped on every page; "Registry notes" quotes maintainer-voice registry prose verbatim; per-module substance deferred to GitHub READMEs at pinned SHAs; all three declare compat <0.20.0 while checkout is 0.20.0-alpha.14 (accurate but reader-surprising, uncontextualized).
