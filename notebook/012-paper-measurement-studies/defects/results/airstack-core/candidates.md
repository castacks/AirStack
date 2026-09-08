# Candidate commits

Every commit in the window whose message matches the fix-keyword regex. Read the diff before classifying; the message alone is `low` confidence.

## 9804274fb3 — 2026-04-29 — Andrew Jong — KEYWORD bug,fix

**Pin empy version to fix ROS2 jazzy version bug**

https://github.com/castacks/AirStack/commit/9804274fb386f96c2c32302c6d904e77595d0cad

+1 / −1 in 1 files:

- `robot/docker/Dockerfile.robot`

## 92708d54bc — 2026-04-30 — John Liu — KEYWORD error,fixes

**Johnliu/px4 cpu optimization (#348)**

https://github.com/castacks/AirStack/commit/92708d54bc104bca7726f0aba7c2b108b9274b8d

```
* added option for physics step frequency

* reverted example launch script

* patches PX4 simulation startup script and fixes robot DDS version

* set default physics Hz for PX4 to be 100Hz which is the minimum.

* reverted simulation changes

* updated docs

* Better error logging for ci/cd orchestrator

* Add check system resources before spawning server; if resources not available, report back and try again later

* added option for physics step frequency

* added option for physics step frequency

* removed physics frequency from .env and set working PX4 values in docker-compose defaults.

* removed unnecessary benchmarking from AirStack launch scripts.

---------

Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
```

+195 / −10 in 10 files:

- `.env`
- `docs/simulation/isaac_sim/docker.md`
- `docs/simulation/isaac_sim/pegasus_scene_setup.md`
- `robot/docker/Dockerfile.robot`
- `simulation/isaac-sim/docker/Dockerfile.isaac-ros`
- `simulation/isaac-sim/docker/docker-compose.yaml`
- `simulation/isaac-sim/extensions/PegasusSimulator`
- `simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/utils/scene_prep.py`

## 08ac171718 — 2026-04-30 — Andrew Jong — KEYWORD bug,fix

**Try fix another bug**

https://github.com/castacks/AirStack/commit/08ac1717180ae5776c9a62c9287b4aadb5c9d0c3

+8 / −7 in 1 files:

- `.github/workflows/system-tests.yml`

## 9fc5580ed8 — 2026-04-30 — Andrew Jong — KEYWORD bug,fix

**Fix bug**

https://github.com/castacks/AirStack/commit/9fc5580ed8620e36084010c34ff380f803bba00d

+22 / −17 in 1 files:

- `.github/workflows/system-tests.yml`

## e37e136de4 — 2026-04-30 — Andrew Jong — KEYWORD fix

**Fix finding baseline metrics**

https://github.com/castacks/AirStack/commit/e37e136de415a0dbd6aa26646f1e15ed1b6f2d7a

+9 / −5 in 1 files:

- `.github/workflows/system-tests.yml`

## 3373e252b5 — 2026-04-30 — Andrew Jong — KEYWORD error

**Better error logging for ci/cd orchestrator**

https://github.com/castacks/AirStack/commit/3373e252b5a810ce501350cbc39d88b12940739c

+91 / −0 in 2 files:

- `.github/orchestrator/config.example.yaml`
- `.github/orchestrator/orchestrator.py`

## 6a0f9a7fd9 — 2026-05-08 — John Liu — KEYWORD bug,fix,fixed,issue

**Johnliu/rtx lidar update (#351)**

https://github.com/castacks/AirStack/commit/6a0f9a7fd9f96b9f068662a7fa803813b60504ef

```
* Update PegasusSim lidar to new rtx lidar and optional min_sensor_range parameter to vdb model to avoid self-detection.

* removed deprecated ouster lidar. Completely integrated new rtx lidar

* renaming frame id back to ouster

* Added node to filter near and invalid lidar points

* reconciled topic names for lidar point cloud

* fixed example scripts to use rtx lidar api

* fixed tmux closing and rclpy path issue

* uses add_rtx in multi px4 script

* bumping version index

* docs added

* unit testing and documentation updates

* cleaning code from copilot suggestions

* docs(tests): fix pytest marker example for running liveliness and sensors

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/7ce7609a-a7f3-414d-9d42-0c9999d0459f

Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>

* docs(tests): fix marker semantics in test_sensors module docstring

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/bdf00f6f-1d9f-4597-bf57-b96f99421646

Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>

* addressing github copilot concerns

* docs(bridge): remove stale camera topics comment

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/2d5718ac-20e3-4f10-a12e-05d601cf000c

Co-authored-by: JohnYanxinLiu <63010779+JohnYanxinLiu@users.noreply.github.com>

* addressing copilot concerns

* removing debug print statement from reading point cloud

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fix(isaac-sim): align drone1 lidar prim path with spawned prim

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/fbad2b9c-1761-45b1-b464-3e874511255c

Co-authored-by: JohnYanxinLiu <63010779+JohnYanxinLiu@users.noreply.github.com>

* more succint comment in sim bashrc

* resolving discrepant comments in ros bridge yaml

* removed bug allocated new copy of point cloud array

* logs lidaar test with boolean instead of hz

* and --> or for marks

---------

Co-authored-by: copilot-swe-agent[bot] <198982749+Copilot@users.noreply.github.com>
Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
```

+1580 / −489 in 43 files:

- `.agents/skills/run-system-tests/SKILL.md`
- `.agents/skills/use-airstack-cli/SKILL.md`
- `.agents/skills/write-isaac-sim-scene/SKILL.md`
- `.env`
- `AGENTS.md`
- `common/ros_packages/desktop_bringup/rviz/robot.rviz`
- `common/ros_packages/robot_descriptions/iris/urdf/iris_with_sensors.pegasus.robot.urdf`
- `docs/development/intermediate/testing/index.md`
- `docs/robot/autonomy/sensors/index.md`
- `docs/simulation/isaac_sim/docker.md`
- `docs/simulation/isaac_sim/index.md`
- `docs/simulation/isaac_sim/pegasus_scene_setup.md`
- `robot/ros_ws/src/global/global_bringup/config/vdb_params.yaml`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/README.md`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/config/lidar_point_cloud_filter.yaml`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/launch/lidar_point_cloud_filter.launch.xml`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/lidar_point_cloud_filter/__init__.py`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/lidar_point_cloud_filter/lidar_point_cloud_filter_node.py`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/package.xml`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/resource/lidar_point_cloud_filter`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/scripts/validate_lidar_filter_clouds.py`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/setup.cfg`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/setup.py`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/test/test_copyright.py`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/test/test_flake8.py`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/test/test_pep257.py`
- `robot/ros_ws/src/sensors/sensors_bringup/launch/sensors.launch.xml`
- `robot/ros_ws/src/sensors/sensors_bringup/package.xml`
- `simulation/isaac-sim/assets/scenes/simple_pegasus.scene.usd`
- `simulation/isaac-sim/config/sim_to_robot_bridge.yaml`
- `simulation/isaac-sim/docker/.bashrc`
- `simulation/isaac-sim/docker/docker-compose.yaml`
- `simulation/isaac-sim/extensions/PegasusSimulator`
- `simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/launch_scripts/two_drone_scene_import.py`
- `tests/README.md`
- `tests/conftest.py`
- `tests/parse_metrics.py`
- `tests/pytest.ini`
- … 3 more

## ae64f97031 — 2026-05-11 — Krrish Jain — KEYWORD bug,bugs,fix,fixed,fixes,fixing,revert

**Krrish/coord pr (#350)**

https://github.com/castacks/AirStack/commit/ae64f970316df8893b071bbd8f365acd7aa6d4d1

```
* Fixed multi-drone global plan

* added sep files for fire and retro

* added robot2 relative pos; diff rviz files; bridge for rayfronts topics

* added sharing of semantic rays

* changed rviz for both drones

* added target sharing

* changed drone start pos

* gossip layer w/o relay

* added global coords under /{ROBOT_NAME}/interface/mavros/global_position/raw/fix(not my topic, it was already publishing to that)

* gossip, threedrone,peerprofile

* multi drone vis in foxglove, odom doesn't work in foxglove yet

* multi drone vis in foxglove works with odom

* global plan added

* added image, vdb markers(not transformed yet)

* fixed state estimation flickering and vdb transform

* added custom foxglove buttons for commands

* added modular payloads to peerprofile, foxglove reads the payloads and vizualizes it,currently works for rayfronts

* fixing the rotation of payload

* syncing devices

* fixed gossip + translate

* added skill for foxglove/coordination

* removed VDB ENV

* rebase with main

* updated docs

* fixed launch files so they have play start on sim. scene_prep utils: added non-world prims to save in flattened manner

* created raven_nav package

* moved coordination to common

* fixed gcs<->robot dds

* added hitl functionality

* fixes to dds

* put dds hitl under gcs

* fixes to robot hitl

* syncing both computers

* mimiced robot-l4t for dataflow

* fixed path to ddsrouter_yaml

* fixed dds server

* fixed two_drone_fire

* rayfronts is now a ros package

* added feedback, it's sending success too early though

* fixed raven behavior

* foxglove panel with working executors

* random walk fixed

* fixed random walk bringup. Added saves and viz for multiple waypoints and polygons

* fixed bounds for exploration task, combined waypoint/polygon editor into task panel

* made waypoint/polygon gui larger

* added 2d map to foxglove

* WIP: pre-merge snapshot

* added changes from main

* WIP: pre-branch-split snapshot

* PR for foxglove+multi-robot

* merged with main

* PR cleanup: revert unrelated changes and drop extra files

- Restore main's robot.rviz (drop redundant robot_1/robot_2.rviz)
- Restore ms-airsim include in root docker-compose.yaml
- Restore airsim sections in docs/simulation/index.md
- Restore docs/gcs/docker/index.md (VERSION env name)
- Restore robot/docker/{.bashrc, Dockerfile.robot} to main
- Restore SIM_IP in robot/docker/docker-compose.yaml
- Restore takeoff_landing_planner takeoff_height: 8.0
- Drop docs/action_bridging.md (internal design memo)
- Drop personal launch scripts (two_drone_fire*, three_drone_scene_import, two_drone_RetroNeighbourhood)
- Trim verbose comments in gps_utils.py and example_multi_drone_scene_import.py

* Trim noisy inline comments in PR-added Python files

* Pin vdb_mapping_ros2 to public main (was at unpushed 68fe8dde)

* fixed launch script

* fixed foxglove bugs, added dynamic fg layout, updated docs

* fixed bugs found by copilot. Removed rviz by adding a node

* fixed comment

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fixed path in skill

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fixes from copilot

* Fix Pegasus submodule pointer after merge

Advance to 8e01d013 (main's pointer) which contains spawn_rtx_lidar.py,
required by example_one_px4_pegasus_launch_script.py and the multi
script after the rtx-lidar update merged from main.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* fix(coordination): align gossip with steady clock + manifest hygiene

- gossip_node: swap startup log + outgoing-stamp clock to STEADY_TIME so
  the dedup-by-stamp invariant survives /clock pauses; subscribe to
  /global_position/global to match foxglove_visualizer and action_relay
- gossip_node docstring: drop the false "waypoint triggers immediate
  publish" claim
- coordination README: rename peer_registry node block to the actual
  per-robot registry topic; "wall-clock" -> "steady"
- package.xml: add missing exec/depend rules
  - coordination_bringup -> autonomy_bringup
  - autonomy_bringup -> coordination_bringup
  - desktop_bringup -> coordination_bringup, gcs_visualizer
  - gcs_visualizer -> std_msgs, coordination_msgs, coordination_bringup
- task_msgs: replace TODO license with BSD-3-Clause
- gcs.launch.xml: comment had `--no-sandbox` (`--` is illegal inside an
  XML comment and crashed the ROS launch parser)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* fix(gcs+autonomy): drop dead BT panel, lint payload imports, name-map override

- payload_visualizer_node: remove unused PointCloud2 / transform_point_cloud2
  imports (F401), collapse Marker/MarkerArray
- action_relay launch: ROBOT_RELAY_MAP env override for non-default
  robot_name -> domain mappings (default behavior unchanged)
- desktop_bringup robot.rviz: drop BehaviorTreePanel entry pointing at
  /behavior/behavior_tree_graphviz (publisher package was removed)
- autonomy_bringup domain_bridge: bridge /global_position/global to match
  the dds_router and the rest of the stack (was /raw/fix)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* fix(foxglove): clean panel-id stacking, atomic render, drop dead .foxe

- render_layout: regex now strips every trailing _r<n> (was: only the
  last one), fixes _r1_r1_r1... stacking on repeated runs
- render_layout: atomic write via tmp + os.replace so a partial
  json.dump doesn't corrupt the layout file
- airstack_default.json: re-render with fixed stripper to commit a
  clean source template (no stacked _r1 suffixes)
- install.sh -> install.py: file is Python, shebang is python3
- install.py: slugify publisher into the on-disk extension dir name
  so "AirLab CMU" doesn't produce a directory with a space
- drop robot-commands/robot-commands.foxe (duplicate; canonical is at
  foxglove_extensions/robot-commands.foxe) and the .foxe.bak

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* bug fixes

* bug fixes

* version

* reverted env

* updated gitignore and docs

* updated foxglove viz + consistent spellings across repo

* Move layout file to /root/ so it's immediately accessible, also fix template path

* Change so that file name reflects NUM_ROBOTS

* Add a DEBUG_RVIZ flag to launch robot rviz if needed

---------

Co-authored-by: krrishj18 <krrishj18@users.noreply.github.com>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: Claude Opus 4.7 <noreply@anthropic.com>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
```

+9065 / −457 in 128 files:

- `.agents/skills/add-ros2-package/SKILL.md`
- `.agents/skills/add-task-executor/SKILL.md`
- `.agents/skills/attach-gossip-payload/SKILL.md`
- `.agents/skills/configure-multi-robot/SKILL.md`
- `.agents/skills/use-airstack-cli/SKILL.md`
- `.agents/skills/visualize-in-foxglove/SKILL.md`
- `.agents/skills/write-isaac-sim-scene/SKILL.md`
- `.agents/skills/write-launch-file/SKILL.md`
- `.agents/skills/write-mkdocs-documentation/SKILL.md`
- `.env`
- `.gitignore`
- `.gitmodules`
- `AGENTS.md`
- `airstack.sh`
- `common/ros_packages/coordination/README.md`
- `common/ros_packages/coordination/coordination_bringup/config/gcs_gossip_dds_router.yaml`
- `common/ros_packages/coordination/coordination_bringup/config/gossip_dds_router.yaml`
- `common/ros_packages/coordination/coordination_bringup/config/gossip_payloads.yaml`
- `common/ros_packages/coordination/coordination_bringup/coordination_bringup/__init__.py`
- `common/ros_packages/coordination/coordination_bringup/coordination_bringup/frame_utils.py`
- `common/ros_packages/coordination/coordination_bringup/coordination_bringup/gossip_node.py`
- `common/ros_packages/coordination/coordination_bringup/coordination_bringup/peer_profile.py`
- `common/ros_packages/coordination/coordination_bringup/coordination_bringup/peer_registry_monitor.py`
- `common/ros_packages/coordination/coordination_bringup/launch/gcs_gossip_bridge.launch.py`
- `common/ros_packages/coordination/coordination_bringup/launch/gossip.launch.xml`
- `common/ros_packages/coordination/coordination_bringup/package.xml`
- `common/ros_packages/coordination/coordination_bringup/resource/coordination_bringup`
- `common/ros_packages/coordination/coordination_bringup/scripts/gossip_node`
- `common/ros_packages/coordination/coordination_bringup/scripts/peer_registry_monitor`
- `common/ros_packages/coordination/coordination_bringup/setup.py`
- `common/ros_packages/coordination/coordination_msgs/CMakeLists.txt`
- `common/ros_packages/coordination/coordination_msgs/msg/PeerProfile.msg`
- `common/ros_packages/coordination/coordination_msgs/msg/PeerProfilePayload.msg`
- `common/ros_packages/coordination/coordination_msgs/package.xml`
- `common/ros_packages/desktop_bringup/launch/gcs.launch.xml`
- `common/ros_packages/desktop_bringup/launch/robot.launch.xml`
- `common/ros_packages/desktop_bringup/package.xml`
- `common/ros_packages/desktop_bringup/params/domain_bridge.yaml`
- `common/ros_packages/desktop_bringup/rviz/robot.rviz`
- `common/ros_packages/gui/rviz/rviz_tasks_panel/README.md`
- … 88 more

## a26cb2541f — 2026-05-20 — Krrish Jain — KEYWORD bug,fix,fixed,fixes

**Scene prep bug fix (#354)**

https://github.com/castacks/AirStack/commit/a26cb2541ffcd84e4411c35f7a69a121b6dd1f34

```
* fixes to scene_prep_utils.py

* edited docs

* clean launch script

* updated version

* fixed comments inconsistency and typos

* formatting fix

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* bug in gossip if payload is empty

* fixed omni_pass.env file creation bug from CICD guest default profile

* fixed depth topic naming in foxglove gcs

* changed gps topic

* removed redundant exntentions

---------

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: airlab <johnliuchs2022@gmail.com>
```

+290 / −76 in 9 files:

- `.airstack/modules/config.sh`
- `.env`
- `common/ros_packages/coordination/coordination_bringup/coordination_bringup/gossip_node.py`
- `docs/simulation/isaac_sim/overhead_camera.md`
- `docs/simulation/isaac_sim/spawning_drones.md`
- `gcs/foxglove_extensions/airstack_default.json`
- `simulation/isaac-sim/launch_scripts/barebones_pegasus_launch.py`
- `simulation/isaac-sim/launch_scripts/example_multi_drone_scene_import.py`
- `simulation/isaac-sim/utils/scene_prep.py`

## 25d52da575 — 2026-05-22 — Sebastian Scherer — KEYWORD bugs,error,fail,failed,fails,failure,fix,fixes,patch,typo,wrong

**feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO (#352)**

https://github.com/castacks/AirStack/commit/25d52da5759eb731a34f90e9b5e731bb80926418

```
* feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO

Adds a privileged Docker-in-Docker workspace task that lets a developer
run the full AirStack docker-compose stack on OSMO and attach an IDE
over SSH, with Isaac Sim WebRTC livestream + Foxglove websocket exposed
via osmo port-forward.

Components:
- osmo/workspace/{Dockerfile,entrypoint.sh,sshd_config}: airstack-osmo-workspace
  image. Ubuntu 24.04 + sshd (pubkey-only) + Docker CE + Docker Compose +
  nvidia-container-toolkit + fuse-overlayfs (DinD-on-overlayfs needs it,
  otherwise dockerd falls back to vfs which bloats AirStack images ~10x).
- osmo/workflows/airstack-dev.yaml: single privileged GPU task. Materializes
  Nucleus + airlab-docker secrets from OSMO credentials, clones AirStack,
  starts inner dockerd, runs `airstack up` with desktop + isaac-sim-livestream
  Compose profiles.
- simulation/isaac-sim: isaac-sim-livestream Compose service that runs
  Pegasus standalone with --/app/livestream/enabled=true and exposes
  WebRTC port ranges 47995-48012 / 49000-49007 / 49100; launch script
  gates headless+livestream extension on ISAAC_SIM_LIVESTREAM env var.
- .airstack/modules/osmo.sh: airstack osmo:{up,ide,foxglove,webrtc,logs,down}
  CLI wrappers around `osmo workflow submit` / `port-forward` / `cancel`.
  Persists the active workflow id and validates it's still running before
  each command (prevents the stale-state 410 error).
- airstack.sh: bash 4+ re-exec bootstrap (macOS ships 3.2; the CLI uses
  `declare -A`).
- osmo/README.md + docs/tutorials/airstack_on_osmo.md: admin pool setup
  (privileged_allowed) + per-user credentials (airlab-docker-login,
  airlab-nucleus) + student-facing IDE attach + WebRTC/Foxglove flow.

Pool requirements: privileged_allowed: true, GPU pool with
nvidia-container-toolkit on the host, ample node ephemeral storage
(AirStack images extracted are ~50-100Gi via fuse-overlayfs; vfs needs
~500Gi+).

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): harden CLI + workspace image against stale-state, port-forward race, and cursor-server install hangs

Four bugs that bit the first end-to-end runs (airstack-dev-10 → -13):

- _osmo_wf_id: validate saved workflow id against `osmo workflow query`
  before returning. Without this, the state file at ~/.airstack/osmo-state
  outlives the workflow it points at and every subsequent osmo:webrtc /
  osmo:foxglove / osmo:ide call surfaces the same confusing
  "Workflow airstack-dev-N is not running! (status 410)" instead of the
  obvious "run airstack osmo:up to launch a fresh workflow".

- cmd_osmo_up: `osmo workflow submit --set-env` is variadic. Passing two
  separate `--set-env A=1 --set-env B=2` silently drops the first one —
  this is what made airstack-dev-11 fail with "ERROR: SSH_PUB_KEY not set"
  when --branch was passed alongside the pubkey. Collapse the K=V pairs
  into a single --set-env.

- cmd_osmo_ide: previously launched the IDE before starting the
  port-forward, so Cursor/VS Code would try to SSH localhost:2200 a few
  hundred ms before the tunnel listener existed and fail with
  "connect to host localhost port 2200: Connection refused". Now: detect
  an existing forward and reuse it (also avoids the "Address already in
  use" if osmo:foxglove was started in parallel), otherwise spawn the
  forward in the background, wait up to 30s for it to bind, then launch
  the IDE. Ctrl+C tears down the spawned forward cleanly via a trap.

- workspace image / entrypoint: Cursor Remote-SSH hung indefinitely
  on airstack-dev-13 because (a) cursor-server's installer fell back to
  wget when curl timed out and wget was not in the image, and (b) a
  /tmp/cursor-remote-lock.* file left behind by the first crashed
  install blocked every silent retry. Add wget to the apt install list
  and rm -f the stale Cursor / VS Code remote lock files at the very
  top of entrypoint.sh so each fresh pod starts from a clean slate.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): correct osmo:logs CLI invocation; install Foxglove extensions locally on osmo:foxglove

osmo:logs was invoking `osmo workflow logs <id> workspace --follow`, but
the real CLI takes the task via `-t TASK` (not positionally) and has no
`--follow` flag at all — so the command failed immediately with
"unrecognized arguments: workspace --follow". Replace with a polling loop
that uses `-t workspace -n <N>` on a short interval, prints only the
suffix that appeared since the previous fetch (find-the-last-seen-line
trick; degrades to "reprint tail" with a warning if the cursor outruns
-n), and exits cleanly once the workflow reaches a terminal state.
Tunables: OSMO_LOGS_TASK / OSMO_LOGS_TAIL / OSMO_LOGS_INTERVAL.

osmo:foxglove now installs the AirStack Foxglove extensions
(robot-commands / waypoint-editor / polygon-editor) into the laptop's
local Foxglove user-extensions directory before opening the
port-forward. Without this, custom panels show up as "Unknown panel
type: robot-commands.Robot Tasks" in the laptop's Foxglove Desktop
because it has no way to discover the extension folders that live
inside the GCS container. To avoid duplicating the install logic, the
existing gcs/foxglove_extensions/install.py is refactored to read
FOXGLOVE_EXT_SRC / FOXGLOVE_EXT_DST env vars (the in-container call
already in gcs/docker/gcs-base-docker-compose.yaml keeps working
unchanged via defaults). The wrapper sets those vars to
${PROJECT_ROOT}/gcs/foxglove_extensions and
~/.foxglove-studio/extensions respectively, overridable with
OSMO_FOXGLOVE_EXT_DIR / skippable with OSMO_FOXGLOVE_SKIP_EXTENSIONS=1.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): pin Kit livestream UDP media port to 49099 so osmo:webrtc actually shows pixels

Kit 107's WebRTC livestream picks a UDP media port dynamically. The
documented `omni.services.livestream.nvcf` defaults (minHostPort=47998
maxHostPort=48020 fixedHostPort=0) are ignored by the stock standalone
Kit binary — on airstack-dev-13 it bound to UDP 49042, outside both the
Compose-published range AND the default `osmo:webrtc --udp` forward of
`47995-48012,49000-49007`. Result: TCP signaling on 49100 worked, the
WebRTC Streaming Client window opened, but every SRTP media packet was
dropped → black viewport plus the recurring
`NVST_CCE_DISCONNECTED when m_connectionCount 0 != 1` underflow in Kit's log.

Pin the media port via three `app.livestream.*` settings set on
`SimulationApp` before `omni.kit.livestream.webrtc` is enabled, so
whichever code path the carb.livestream-rtc.plugin consults lands on the
same port:

    app.livestream.fixedHostPort = 49099
    app.livestream.minHostPort   = 49099
    app.livestream.maxHostPort   = 49099

49099 is a deliberate one-off from the 49100 TCP signaling port — same
neighborhood, easy to remember. Verified live on airstack-dev-13 after
`docker compose up -d --force-recreate isaac-sim-livestream`: Kit binds
UDP 49099 (`/proc/net/udp` hex BFCB on 0.0.0.0) and docker-proxy
publishes it from the pod host network.

Knock-on cleanups:
- `simulation/isaac-sim/docker/docker-compose.yaml` shrinks the
  isaac-sim-livestream `ports:` from 27 forwarded ports
  (`47995-48012, 49000-49007 TCP+UDP, 49100 TCP`) to just two:
  `49100/tcp` + `49099/udp`.
- `.airstack/modules/osmo.sh` shrinks `OSMO_WEBRTC_TCP` to `49100` and
  `OSMO_WEBRTC_UDP` to `49099`, so `airstack osmo:webrtc` spawns two
  port-forwards instead of thirty.
- `.gitignore` ignores `.DS_Store` so working from a Mac doesn't leak
  Finder metadata.

After pulling this commit into a running pod: `docker compose up -d
--force-recreate isaac-sim-livestream` to apply the new port mapping;
then re-run `airstack osmo:webrtc` on the laptop to pick up the new
forward ranges. The standalone WebRTC Streaming Client connects to
`localhost` (same address as before) and now actually receives frames.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): render Kit GUI in WebRTC stream; document SSH agent forward for in-pod git push

Two paper-cuts that bit airstack-dev-13 after the WebRTC media port pin
landed (commit 2d9b1611):

(1) The WebRTC stream showed only the bare 3D viewport — no menu bar,
    no toolbar, no panels, no console. Cause: SimulationApp's default
    when `headless=True` is to also hide the UI (`hide_ui=True`). The
    NVIDIA reference at
    `simulation/isaac-sim/standalone_examples/api/isaacsim.simulation_app/livestream.py`
    explicitly opts back into UI rendering plus picks explicit window
    sizing and `display_options=3286` to keep the default grid/axes
    visible. Mirror that config in `example_one_px4_pegasus_launch_script.py`
    when `ISAAC_SIM_LIVESTREAM=true` (local desktop dev keeps the
    minimal `headless=False` path unchanged).

(2) The pod has no SSH private key, only an `authorized_keys` for
    inbound connections from the user's laptop. As a result, `git push`
    from inside the Cursor / VS Code Remote-SSH session inside the pod
    fails with "Permission denied (publickey)". sshd inside the
    workspace image already has `AllowAgentForwarding yes` baked in via
    `osmo/workspace/sshd_config`; the missing piece is purely on the
    Mac side. Update the `~/.ssh/config` block in the tutorial to
    include `ForwardAgent yes` (so the local agent's keys are exposed
    in the pod), `AddKeysToAgent yes` (auto-load on first push), and
    `UseKeychain yes` (macOS-only Keychain unlock without passphrase
    prompts; ignored on Linux). Adds an `ssh-add -l` smoke-test note.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): make osmo:setup idempotent + paste-safe; document Nucleus auth-debug path

osmo:setup hit two failure modes that wasted a debug session each:

- `osmo credential set` is not an upsert for GENERIC creds — re-running
  setup (e.g. to rotate a Nucleus API token) failed with `400 duplicate
  key value violates unique constraint "credential_pkey"` and bailed
  before reaching the airlab-nucleus credential. Delete-then-set each
  credential so re-running is idempotent.
- Bracket-paste mode and cross-OS clipboards routinely smuggle invisible
  bytes around long pastes. Nucleus's auth endpoint silently DENIES a
  token with one extra trailing byte, with no actionable error from the
  client side. _osmo_prompt now strips leading/trailing whitespace and
  CR/NUL bytes via a new _osmo_trim helper, and warns when bytes were
  stripped. cmd_osmo_setup additionally JWT-shape-checks the Nucleus
  token (must be eyJ.<dot>.<dot>.) before submitting it, so a wrong
  paste fails at setup time instead of silently DENIED at pod boot.

Also documents how to debug the "Login Required: Unable to connect
server omniverse://airlab-nucleus..." popup: SSH the Nucleus host and
tail base_stack-nucleus-auth-1 for InternalCredentials.auth status:
DENIED. Adds a "Nucleus connectivity from OSMO" section to the admin
README clarifying that Nucleus over HTTPS uses a single 443 (no need
to open the native 3009-3180 range from the OSMO cluster), per
NVIDIA's TLS docs.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): use Nucleus API-token auth, with double-dollar to survive compose parser

The OSMO entrypoint was writing OMNI_USER=<andrew_id> alongside an API
token JWT in OMNI_PASS, which routes the JWT through the password-
verification path. Nucleus silently DENIES — visible only in
base_stack-nucleus-auth-1 as `InternalCredentials.auth … 'username':
'<andrew>' … status: DENIED` (no Tokens.auth_with_api_token call). Kit
then pops "Login Required: Unable to connect server omniverse://...".

omniclient expects the literal sentinel username `$omni-api-token` paired
with the JWT as the password. The entrypoint now detects a JWT-shaped
OMNI_PASS (header starts with `eyJ`) and emits OMNI_USER=$$omni-api-token
into omni_pass.env. The `$$` is intentional: docker-compose v2
interpolates env_file values, and a single `$` would be eaten by the
parser (`OMNI_USER=$omni-api-token` becomes `OMNI_USER=-api-token` after
${omni}- expansion to empty). The container ultimately sees
OMNI_USER=$omni-api-token, which is the correct sentinel.

Also note for the next debugger: `docker compose restart` does NOT
re-read env_file. Use `docker compose up -d <svc>` to recreate the
container after editing omni_pass.env.

Updates omni_pass_TEMPLATE.env header to document the API-token pattern
explicitly (with the $$ caveat), and adds a troubleshooting row that
distinguishes "wrong auth path" (DENIED with no Tokens.auth_with_api_token
call) from "bad/expired token" (Tokens.auth_with_api_token: DENIED).

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(osmo): make OSMO the recommended dev path, single clone-the-repo flow

Reposition the OSMO tutorial as AirStack's recommended day-to-day
development path (not just a fallback for laptops without GPUs) and
collapse it onto a single recipe: clone the repo, then drive everything
through the airstack osmo:* wrappers in .airstack/modules/osmo.sh.

- docs/tutorials/airstack_on_osmo.md
  - Retitle + rewrite the intro to lead with five concrete advantages
    (pooled GPUs, no local CUDA/Docker/driver maintenance, same image as
    CI + field robots, one-command onboarding, hardware bigger than your
    laptop). Demote the Linux+GPU-desktop path to an escape hatch.
  - Drop the Mac/Windows/no-GPU framing in 'Who is this for?' and the
    mermaid laptop subgraph label.
  - Add 'a local clone of AirStack' to Prerequisites; remove it from the
    'do not need' list.
  - Replace Option A/B credential split with a single
    ./airstack.sh osmo:setup recipe; move the three raw osmo credential
    set calls into a collapsible 'Under the hood' footnote.
  - Replace each step's raw osmo workflow ... command with the
    corresponding airstack osmo:up/logs/ide/webrtc/foxglove/down wrapper;
    preserve the raw form in 'Under the hood' footnotes that cross-link
    cmd_osmo_* in .airstack/modules/osmo.sh.
  - Drop the export WF=... paragraph — the wrappers read the id from
    ~/.airstack/osmo-state automatically; AIRSTACK_OSMO_WF overrides
    per-invocation. \$WF now only appears inside the raw-form footnotes.
  - Sweep Troubleshooting + What-survives tables: redirect raw
    port-forward fixes to the airstack osmo:* equivalents and rename the
    section to 'What survives airstack osmo:down?'.
  - Fix WebRTC edge label (49100/tcp + 49099/udp) to match the pinned
    ports the workflow actually uses today.

Companion cleanups now that the privileged_allowed flip is automatic on
the OSMO autosync side (synchronize_osmo_team_pools.py forces
privileged_allowed: true on every platform of every pool, so students
never see the 'platform does not have privileged flag enabled' error):

- osmo/README.md: drop the 'Most common blocker' privileged warning, the
  privileged_allowed row from the pool-requirements table, and the
  'privileged GPU pod' / '(privileged, GPU)' descriptors in the
  architecture summary. Simplify the validation-stage SSH-failure hint.
- osmo/workflows/airstack-dev.yaml: trim the long DinD-requires-privileged
  comment to a one-liner (the privileged: true directive itself stays).
- .airstack/modules/osmo.sh: remove the special-case 'privileged flag
  enabled' error branch in cmd_osmo_up — it should never fire now.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): make osmo:logs actually stream + survive pod host-key churn

osmo:logs was silent because cmd_osmo_logs wrapped osmo workflow logs in
$( ... ) on the assumption that -n LAST_N_LINES exits after dumping the
tail. Empirically the CLI keeps the stream open as new lines arrive (it
already behaves like tail -f, despite --help advertising only -n), so
command substitution waited forever and printed nothing. Drop the polling
loop and just exec the command directly.

Each fresh OSMO pod also ships a new sshd host key, so every osmo:up
trips StrictHostKeyChecking against the previous workflow's fingerprint
and SSH/Cursor abort with "Host key for [localhost]:2200 has changed".
Switch the recommended ~/.ssh/config block (and osmo/README.md) to the
ephemeral-host pattern (StrictHostKeyChecking no + UserKnownHostsFile
/dev/null + LogLevel ERROR), and have cmd_osmo_ide ssh-keygen -R the
stale loopback entry on every run so users on the old config get
unblocked automatically.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): auto-pin --branch to local checkout + clean error UX when workflow dies

The pod's entrypoint clones AirStack fresh from GitHub on every workflow
start (the pod fs is ephemeral). It defaulted to `main`, so any developer
testing branch-only OSMO changes silently ran their pod against stale
`main` code — most visibly: COMPOSE_PROFILES=desktop,isaac-sim-livestream
resolved to "desktop" alone on `main` because the isaac-sim-livestream
service only exists on the feature branch, so isaac-sim never came up
and `airstack osmo:webrtc` showed a blank stream.

  - cmd_osmo_up now defaults --branch to the local repo's current
    branch (git rev-parse --abbrev-ref HEAD). Detached HEAD or
    non-git checkouts fall back to `main` cleanly. Pass --branch
    explicitly to override.
  - New _osmo_check_branch_pushed warns up-front when the about-to-
    submit branch has no upstream, is ahead of origin, or has an
    uncommitted working tree. The pod doesn't see your laptop's edits.

Separately, when an OSMO workflow gets canceled mid-flight (osmo:down
in another shell, or OSMO timing it out), the in-flight port-forward
and logs streams raise OSMOUserError("Workflow X is not running!")
from inside an asyncio Task. The CLI prints "Task exception was never
retrieved" + a multi-line Traceback that buries the actual one-line
cause. New _osmo_pf_filter awk script collapses that into a single
[ERROR] line pointing at `airstack osmo:up`. Wired into webrtc,
foxglove, and logs. webrtc also gains a cleanup trap that kills the
backgrounded UDP port-forward on EXIT/INT/TERM so we don't leak it
against a dead workflow.

Tutorial Step 2 documents the new --branch default and the
"pod-clones-from-GitHub-not-your-laptop" gotcha.

Co-authored-by: Cursor <cursoragent@cursor.com>

* perf(osmo): bump inner dockerd concurrency to saturate 10 GbE pulls

dockerd's defaults of --max-concurrent-downloads=3 / --max-concurrent
-uploads=5 cap a fresh airstack-dev pod's image-pull at ~300 MiB/s
against the airlab-backup-10g registry — single-stream TLS tops out
around 300-500 MiB/s per core, and three parallel streams of unevenly
sized blobs serialize down to that ceiling. Ceph (1014 TiB, 92 OSDs,
SSD pools) and 10 GbE both have far more headroom than that. Bump to
10/10 to overlap enough blob downloads to saturate the pipe.

Threaded through the DOCKERD_MAX_DOWNLOADS / DOCKERD_MAX_UPLOADS env
vars so a pool can be tuned at submit time without rebuilding the
workspace image.

Workspace image needs a rebuild + push for this to take effect:
  cd osmo/workspace
  docker build -t airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest .
  docker push   airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(osmo): require buildx --platform linux/amd64 for workspace image

A plain `docker build && docker push` on an Apple Silicon Mac silently
produces a linux/arm64-only `latest` manifest. OSMO workers are amd64,
so every subsequent workflow fails at the outer pod-image pull with
"no match for platform in manifest" before the entrypoint even runs —
a confusing failure mode whose root cause lives entirely in the push,
not in the workflow yaml or the entrypoint.

Switch the README and the Dockerfile docstring to the buildx form,
explain the why, and document the post-push manifest check.

Co-authored-by: Cursor <cursoragent@cursor.com>

* perf(osmo): move dockerd data-root to /osmo/run for native overlay2

The OSMO pod's `/` is itself a containerd overlay snapshot, and Linux
refuses to stack a second overlayfs on top of an overlay rootfs — which
is why the inner dockerd was falling through to fuse-overlayfs. That
costs a kernel↔userspace FUSE round-trip on every `creat()` during
layer extraction, which murders throughput on apt/pip/ROS layers
(measured: 32-50 MB/s for small-file-heavy layers vs 480 MB/s for
big-file layers in the same pull).

Pointing dockerd at /osmo/run/docker (the kubelet emptyDir backed by
ext4 on /dev/vda3) lets the existing overlay2-first fallback chain
actually succeed on its first try, restoring kernel-overlay extraction
performance. emptyDir lifetime matches the workflow lifetime, so the
docker layer cache gets the right scope automatically.

Falls back to /var/lib/docker if /osmo/run isn't present so the image
still works in non-OSMO test contexts.

Co-authored-by: Cursor <cursoragent@cursor.com>

* updated version

* added virtual display for GL context

* added virtual display for droan_gl

* droan_gl patch

* run Xvfb in its own tmux session

* updated dockerfile + version

* typo in docs

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in osmo logs, renamed airstack-isaac-sim to just isaac-sim

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in container name for isaac-sim-livestream

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* airstack-dev version overwrite removed

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

---------

Co-authored-by: Cursor <cursoragent@cursor.com>
Co-authored-by: krrishj18 <krrishj@andrew.cmu.edu>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
```

+2383 / −21 in 19 files:

- `.airstack/modules/osmo.sh`
- `.env`
- `.gitignore`
- `airstack.sh`
- `docs/getting_started/index.md`
- `docs/tutorials/airstack_on_osmo.md`
- `docs/tutorials/index.md`
- `gcs/foxglove_extensions/install.py`
- `mkdocs.yml`
- `osmo/README.md`
- `osmo/workflows/airstack-dev.yaml`
- `osmo/workspace/Dockerfile`
- `osmo/workspace/entrypoint.sh`
- `osmo/workspace/sshd_config`
- `robot/docker/Dockerfile.robot`
- `robot/docker/docker-compose.yaml`
- `simulation/isaac-sim/docker/docker-compose.yaml`
- `simulation/isaac-sim/docker/omni_pass_TEMPLATE.env`
- `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`

## dff3dc6f76 — 2026-05-28 — Andrew Jong — KEYWORD fix,fixing

**fix(isaac-sim): pegasus drone retains PX4 state across Stop/Play (#363)**

https://github.com/castacks/AirStack/commit/dff3dc6f765ec23ca3fbeb324ef83358b989dabc

```
* Update submodule to point to pegasus fix fixing start/stop behavior

* Bump VERSION to 0.19.0-alpha.2

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>

---------

Co-authored-by: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
```

+2 / −2 in 2 files:

- `.env`
- `simulation/isaac-sim/extensions/PegasusSimulator`

## 8b927e465c — 2026-05-29 — John Liu — KEYWORD bug,error,failing,fix,fixed,fixes,issue

**Johnliu/optitrack autonomy (#359)**

https://github.com/castacks/AirStack/commit/8b927e465c164039fd00c1b9525bb6e7e36629d1

```
* incremented version tag

* docker image builds on l4t with generalizability features for other ros and linux versions

* documentation and claude skills for developing a new profile.

* initial natnet implementation

* deployment to jetson with ros2 jazzy now fixed

* unit testing dependency fix

* added optitrack perception to launch

* put tag version back in

* added instructions for Agents to run tests

* attempt at completely custom Optitrack Parser (Not working)

* fully implemented NatNetSDK natnet ros2 wrapper natively in AirStack. Hand test in mocap room successful

* unit test restructuring

* reorganized natnet logic for unit-testability

* unit testing restructuring to have unit tests in src and proxies in test. Unit tests workflows created

* reupdated documentation for current state of testing

* change unit tests to occur with system tests so that environment is builtgit status

* generalizes natnet parameters and disables natnet automatically for launch

* natnet client adaptor now references correct error code from NatNet SDK 4.4.0.0

* increment version tag

* bug fixes to natnet launching from env file

* fixed failing systems test due to depends issue and specifying unit tests via yaml

* Use NatNet callback context instead of thread-local dispatch

* addressing Krrish' documentation comments

* incrementing version tag after osmo PR merge

* documentation corrections

* Bump VERSION Update .env

---------

Co-authored-by: copilot-swe-agent[bot] <198982749+Copilot@users.noreply.github.com>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
```

+4599 / −159 in 73 files:

- `.agents/skills/add-ros2-package/assets/package_template/setup.py`
- `.agents/skills/add-unit-tests/SKILL.md`
- `.agents/skills/configure-multi-robot/SKILL.md`
- `.agents/skills/docker-build-profiles/SKILL.md`
- `.agents/skills/run-system-tests/SKILL.md`
- `.env`
- `AGENTS.md`
- `airstack.sh`
- `common/ros_packages/msgs/airstack_msgs/package.xml`
- `common/ros_packages/msgs/task_msgs/package.xml`
- `docs/development/beginner/airstack-cli/docker_usage.md`
- `docs/development/index.md`
- `docs/development/intermediate/docker-build-profiles.md`
- `docs/development/intermediate/testing/index.md`
- `docs/development/intermediate/testing/unit_testing.md`
- `docs/robot/autonomy/perception/index.md`
- `docs/robot/docker/index.md`
- `mkdocs.yml`
- `robot/docker/Dockerfile.l4t-stack-base`
- `robot/docker/Dockerfile.robot`
- `robot/docker/docker-compose.yaml`
- `robot/docker/zed/Dockerfile.zed-l4t`
- `robot/ros_ws/src/local/controls/pid_controller_msgs/package.xml`
- `robot/ros_ws/src/perception/natnet_ros2/.gitignore`
- `robot/ros_ws/src/perception/natnet_ros2/CMakeLists.txt`
- `robot/ros_ws/src/perception/natnet_ros2/README.md`
- `robot/ros_ws/src/perception/natnet_ros2/config/natnet_config.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/config/vision_pose_converter.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/env-hooks/natnet_library_path.dsv.in`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/natnet_client_adapter.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/natnet_logic.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/launch/natnet_ros2.launch.py`
- `robot/ros_ws/src/perception/natnet_ros2/launch/vision_pose_converter.launch.xml`
- `robot/ros_ws/src/perception/natnet_ros2/package.xml`
- `robot/ros_ws/src/perception/natnet_ros2/scripts/download-natnet-sdk.sh`
- `robot/ros_ws/src/perception/natnet_ros2/src/natnet_client_adapter.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/natnet_ros2_node.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/vision_pose_converter_node.py`
- `robot/ros_ws/src/perception/natnet_ros2/test/fake_natnet_client.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_logic.cpp`
- … 33 more

## 6279bea8be — 2026-07-06 — Krrish Jain — KEYWORD fix

**Fix/camera init (#368)**

https://github.com/castacks/AirStack/commit/6279bea8bea7d3c5e4ea26fb9dde70eda58ff4a5

```
* re-ordered initialization of stereo render product node to only initialize after right camera is initialized, ensuring camera is initialized as stereo (left camera is assumed, but right is optional)
---------

Co-authored-by: John <johnliuchs2022@gmail.com>
```

+4 / −2 in 3 files:

- `.env`
- `robot/ros_ws/src/perception/perception_bringup/launch/perception.launch.xml`
- `simulation/isaac-sim/extensions/PegasusSimulator`

## fa990f4dc0 — 2026-07-10 — pvkumara — KEYWORD bug,bugs,error,fail,failed,fails,failure,fix,fixed,fixes,fixing,issue,patch,revert,typo,wrong

**Add fixed-trajectory system tests with cross-track error metrics (#365)**

https://github.com/castacks/AirStack/commit/fa990f4dc03e8bee48d76aa73b1d6285b8bd16f9

```
* Add fixed-trajectory evaluation tests

New tests/test_fixed_trajectory.py evaluates drone performance on Circle,
Figure8, Racetrack, and Line trajectories: takeoff -> execute -> land with
cross-track error, path RMSE, execution time, and success metrics recorded
to metrics.json for baseline comparison.

- Python ideal-path generators mirror fixed_trajectory_task.cpp equations
- Cross-track error uses robot pose snapshot at dispatch to transform
  base_link ideal path to world frame for odom comparison
- 5m loose tolerance documents the known circle failure without stranding drone
- conftest.py gains --trajectory-types CLI option and generalised phase-order
  sorting/ID-rewriting for both autonomy test modules
- tests/README.md documents the new module, all 11 metrics, and run commands

Made-with: Cursor

* Remove module docstring from test_fixed_trajectory.py

Made-with: Cursor

* Aj/GitHub ci cd (#347)

* Add link to PAT

* Change to new orchestrator instance workflow

* Add availability zone

* Bump version to 0.18.0-alpha.7

* Add fix for boot volume size blocking orchestrator

* Add floating IPs to CI/CD

* Bump gh runner_version to latest

* Update cicd defaults

* Rename integration-tests.yml to system-tests.yml

* Add debugging tips and add to mkdocs

* Use venv instead of pip3 to fix error: externally-managed-environment

* Explicitly fail autonomy test if images not yet built

* Enable using docker cache from docker registry to speed up docker image build tests for ci/cd

* Fix bug

* Update docs and change docker image build/push to also run on self-hosted runner

* Enable trigger docker build workflow on via manual dispatch

* Increase instance volume size so that space doesn't run out when building docker images

* Update to always try build all images

* Create dummy file for docker compose push to pass

* Add omni_pass.env with guest access to AirLab nucleus

* Update ci/cd tests to make sure image is present before running tests

* Make sure images for profiles get built

* Update system tests to not build images if pull available

* Make build/pull quiet

* Pin empy version to fix ROS2 jazzy version bug

* Switch image to desktop so that tests run successfully

* Add docker image signing to workflow

* Change pytest mark 'autonomy' to 'takeoff_hover_land'

* update comments on workflow

* Recurisve checkout of airstack

* Log more to GitHub

* Better error logging for ci/cd orchestrator

* Add check system resources before spawning server; if resources not available, report back and try again later

* Make it so that pytest no longer triggers from pushes on PR; make it so we can manually trigger pytest by commenting /pytest

* Update PR template

* Update AGENTS.md

* Fix finding baseline metrics

* Update workflow to comment instead of react

* Fix bug

* Try fix another bug

* Update omni_pass_TEMPLATE.env to use 'guest'; update default on system tests to include build_packages

* Auto prepend 'build_packages' mark to ensure code is built before tests

* Lower default stress-iterations to 1 and single takeoff-velocity to 0.5

* Johnliu/px4 cpu optimization (#348)

* added option for physics step frequency

* reverted example launch script

* patches PX4 simulation startup script and fixes robot DDS version

* set default physics Hz for PX4 to be 100Hz which is the minimum.

* reverted simulation changes

* updated docs

* Better error logging for ci/cd orchestrator

* Add check system resources before spawning server; if resources not available, report back and try again later

* added option for physics step frequency

* added option for physics step frequency

* removed physics frequency from .env and set working PX4 values in docker-compose defaults.

* removed unnecessary benchmarking from AirStack launch scripts.

---------

Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>

* Add new skills

* Revise pull request template for clarity and detail

Update pull request template with versioning guidelines

Added guidelines for versioning in the pull request template.

Update pull request template for media uploads

Clarified instructions for adding videos and images in the PR template.

* Johnliu/rtx lidar update (#351)

* Update PegasusSim lidar to new rtx lidar and optional min_sensor_range parameter to vdb model to avoid self-detection.

* removed deprecated ouster lidar. Completely integrated new rtx lidar

* renaming frame id back to ouster

* Added node to filter near and invalid lidar points

* reconciled topic names for lidar point cloud

* fixed example scripts to use rtx lidar api

* fixed tmux closing and rclpy path issue

* uses add_rtx in multi px4 script

* bumping version index

* docs added

* unit testing and documentation updates

* cleaning code from copilot suggestions

* docs(tests): fix pytest marker example for running liveliness and sensors

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/7ce7609a-a7f3-414d-9d42-0c9999d0459f

Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>

* docs(tests): fix marker semantics in test_sensors module docstring

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/bdf00f6f-1d9f-4597-bf57-b96f99421646

Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>

* addressing github copilot concerns

* docs(bridge): remove stale camera topics comment

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/2d5718ac-20e3-4f10-a12e-05d601cf000c

Co-authored-by: JohnYanxinLiu <63010779+JohnYanxinLiu@users.noreply.github.com>

* addressing copilot concerns

* removing debug print statement from reading point cloud

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fix(isaac-sim): align drone1 lidar prim path with spawned prim

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/fbad2b9c-1761-45b1-b464-3e874511255c

Co-authored-by: JohnYanxinLiu <63010779+JohnYanxinLiu@users.noreply.github.com>

* more succint comment in sim bashrc

* resolving discrepant comments in ros bridge yaml

* removed bug allocated new copy of point cloud array

* logs lidaar test with boolean instead of hz

* and --> or for marks

---------

Co-authored-by: copilot-swe-agent[bot] <198982749+Copilot@users.noreply.github.com>
Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>

* Krrish/coord pr (#350)

* Fixed multi-drone global plan

* added sep files for fire and retro

* added robot2 relative pos; diff rviz files; bridge for rayfronts topics

* added sharing of semantic rays

* changed rviz for both drones

* added target sharing

* changed drone start pos

* gossip layer w/o relay

* added global coords under /{ROBOT_NAME}/interface/mavros/global_position/raw/fix(not my topic, it was already publishing to that)

* gossip, threedrone,peerprofile

* multi drone vis in foxglove, odom doesn't work in foxglove yet

* multi drone vis in foxglove works with odom

* global plan added

* added image, vdb markers(not transformed yet)

* fixed state estimation flickering and vdb transform

* added custom foxglove buttons for commands

* added modular payloads to peerprofile, foxglove reads the payloads and vizualizes it,currently works for rayfronts

* fixing the rotation of payload

* syncing devices

* fixed gossip + translate

* added skill for foxglove/coordination

* removed VDB ENV

* rebase with main

* updated docs

* fixed launch files so they have play start on sim. scene_prep utils: added non-world prims to save in flattened manner

* created raven_nav package

* moved coordination to common

* fixed gcs<->robot dds

* added hitl functionality

* fixes to dds

* put dds hitl under gcs

* fixes to robot hitl

* syncing both computers

* mimiced robot-l4t for dataflow

* fixed path to ddsrouter_yaml

* fixed dds server

* fixed two_drone_fire

* rayfronts is now a ros package

* added feedback, it's sending success too early though

* fixed raven behavior

* foxglove panel with working executors

* random walk fixed

* fixed random walk bringup. Added saves and viz for multiple waypoints and polygons

* fixed bounds for exploration task, combined waypoint/polygon editor into task panel

* made waypoint/polygon gui larger

* added 2d map to foxglove

* WIP: pre-merge snapshot

* added changes from main

* WIP: pre-branch-split snapshot

* PR for foxglove+multi-robot

* merged with main

* PR cleanup: revert unrelated changes and drop extra files

- Restore main's robot.rviz (drop redundant robot_1/robot_2.rviz)
- Restore ms-airsim include in root docker-compose.yaml
- Restore airsim sections in docs/simulation/index.md
- Restore docs/gcs/docker/index.md (VERSION env name)
- Restore robot/docker/{.bashrc, Dockerfile.robot} to main
- Restore SIM_IP in robot/docker/docker-compose.yaml
- Restore takeoff_landing_planner takeoff_height: 8.0
- Drop docs/action_bridging.md (internal design memo)
- Drop personal launch scripts (two_drone_fire*, three_drone_scene_import, two_drone_RetroNeighbourhood)
- Trim verbose comments in gps_utils.py and example_multi_drone_scene_import.py

* Trim noisy inline comments in PR-added Python files

* Pin vdb_mapping_ros2 to public main (was at unpushed 68fe8dde)

* fixed launch script

* fixed foxglove bugs, added dynamic fg layout, updated docs

* fixed bugs found by copilot. Removed rviz by adding a node

* fixed comment

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fixed path in skill

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fixes from copilot

* Fix Pegasus submodule pointer after merge

Advance to 8e01d013 (main's pointer) which contains spawn_rtx_lidar.py,
required by example_one_px4_pegasus_launch_script.py and the multi
script after the rtx-lidar update merged from main.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* fix(coordination): align gossip with steady clock + manifest hygiene

- gossip_node: swap startup log + outgoing-stamp clock to STEADY_TIME so
  the dedup-by-stamp invariant survives /clock pauses; subscribe to
  /global_position/global to match foxglove_visualizer and action_relay
- gossip_node docstring: drop the false "waypoint triggers immediate
  publish" claim
- coordination README: rename peer_registry node block to the actual
  per-robot registry topic; "wall-clock" -> "steady"
- package.xml: add missing exec/depend rules
  - coordination_bringup -> autonomy_bringup
  - autonomy_bringup -> coordination_bringup
  - desktop_bringup -> coordination_bringup, gcs_visualizer
  - gcs_visualizer -> std_msgs, coordination_msgs, coordination_bringup
- task_msgs: replace TODO license with BSD-3-Clause
- gcs.launch.xml: comment had `--no-sandbox` (`--` is illegal inside an
  XML comment and crashed the ROS launch parser)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* fix(gcs+autonomy): drop dead BT panel, lint payload imports, name-map override

- payload_visualizer_node: remove unused PointCloud2 / transform_point_cloud2
  imports (F401), collapse Marker/MarkerArray
- action_relay launch: ROBOT_RELAY_MAP env override for non-default
  robot_name -> domain mappings (default behavior unchanged)
- desktop_bringup robot.rviz: drop BehaviorTreePanel entry pointing at
  /behavior/behavior_tree_graphviz (publisher package was removed)
- autonomy_bringup domain_bridge: bridge /global_position/global to match
  the dds_router and the rest of the stack (was /raw/fix)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* fix(foxglove): clean panel-id stacking, atomic render, drop dead .foxe

- render_layout: regex now strips every trailing _r<n> (was: only the
  last one), fixes _r1_r1_r1... stacking on repeated runs
- render_layout: atomic write via tmp + os.replace so a partial
  json.dump doesn't corrupt the layout file
- airstack_default.json: re-render with fixed stripper to commit a
  clean source template (no stacked _r1 suffixes)
- install.sh -> install.py: file is Python, shebang is python3
- install.py: slugify publisher into the on-disk extension dir name
  so "AirLab CMU" doesn't produce a directory with a space
- drop robot-commands/robot-commands.foxe (duplicate; canonical is at
  foxglove_extensions/robot-commands.foxe) and the .foxe.bak

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* bug fixes

* bug fixes

* version

* reverted env

* updated gitignore and docs

* updated foxglove viz + consistent spellings across repo

* Move layout file to /root/ so it's immediately accessible, also fix template path

* Change so that file name reflects NUM_ROBOTS

* Add a DEBUG_RVIZ flag to launch robot rviz if needed

---------

Co-authored-by: krrishj18 <krrishj18@users.noreply.github.com>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: Claude Opus 4.7 <noreply@anthropic.com>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>

* Scene prep bug fix (#354)

* fixes to scene_prep_utils.py

* edited docs

* clean launch script

* updated version

* fixed comments inconsistency and typos

* formatting fix

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* bug in gossip if payload is empty

* fixed omni_pass.env file creation bug from CICD guest default profile

* fixed depth topic naming in foxglove gcs

* changed gps topic

* removed redundant exntentions

---------

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: airlab <johnliuchs2022@gmail.com>

* Add workflows to (1) enforce correct branch merge convention (2) update develop from main

* Update docs on branches

* Update workflow to handle develop version increment

* Release 0.18.0

* Bump VERSION to  after sync from main

* feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO (#352)

* feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO

Adds a privileged Docker-in-Docker workspace task that lets a developer
run the full AirStack docker-compose stack on OSMO and attach an IDE
over SSH, with Isaac Sim WebRTC livestream + Foxglove websocket exposed
via osmo port-forward.

Components:
- osmo/workspace/{Dockerfile,entrypoint.sh,sshd_config}: airstack-osmo-workspace
  image. Ubuntu 24.04 + sshd (pubkey-only) + Docker CE + Docker Compose +
  nvidia-container-toolkit + fuse-overlayfs (DinD-on-overlayfs needs it,
  otherwise dockerd falls back to vfs which bloats AirStack images ~10x).
- osmo/workflows/airstack-dev.yaml: single privileged GPU task. Materializes
  Nucleus + airlab-docker secrets from OSMO credentials, clones AirStack,
  starts inner dockerd, runs `airstack up` with desktop + isaac-sim-livestream
  Compose profiles.
- simulation/isaac-sim: isaac-sim-livestream Compose service that runs
  Pegasus standalone with --/app/livestream/enabled=true and exposes
  WebRTC port ranges 47995-48012 / 49000-49007 / 49100; launch script
  gates headless+livestream extension on ISAAC_SIM_LIVESTREAM env var.
- .airstack/modules/osmo.sh: airstack osmo:{up,ide,foxglove,webrtc,logs,down}
  CLI wrappers around `osmo workflow submit` / `port-forward` / `cancel`.
  Persists the active workflow id and validates it's still running before
  each command (prevents the stale-state 410 error).
- airstack.sh: bash 4+ re-exec bootstrap (macOS ships 3.2; the CLI uses
  `declare -A`).
- osmo/README.md + docs/tutorials/airstack_on_osmo.md: admin pool setup
  (privileged_allowed) + per-user credentials (airlab-docker-login,
  airlab-nucleus) + student-facing IDE attach + WebRTC/Foxglove flow.

Pool requirements: privileged_allowed: true, GPU pool with
nvidia-container-toolkit on the host, ample node ephemeral storage
(AirStack images extracted are ~50-100Gi via fuse-overlayfs; vfs needs
~500Gi+).

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): harden CLI + workspace image against stale-state, port-forward race, and cursor-server install hangs

Four bugs that bit the first end-to-end runs (airstack-dev-10 → -13):

- _osmo_wf_id: validate saved workflow id against `osmo workflow query`
  before returning. Without this, the state file at ~/.airstack/osmo-state
  outlives the workflow it points at and every subsequent osmo:webrtc /
  osmo:foxglove / osmo:ide call surfaces the same confusing
  "Workflow airstack-dev-N is not running! (status 410)" instead of the
  obvious "run airstack osmo:up to launch a fresh workflow".

- cmd_osmo_up: `osmo workflow submit --set-env` is variadic. Passing two
  separate `--set-env A=1 --set-env B=2` silently drops the first one —
  this is what made airstack-dev-11 fail with "ERROR: SSH_PUB_KEY not set"
  when --branch was passed alongside the pubkey. Collapse the K=V pairs
  into a single --set-env.

- cmd_osmo_ide: previously launched the IDE before starting the
  port-forward, so Cursor/VS Code would try to SSH localhost:2200 a few
  hundred ms before the tunnel listener existed and fail with
  "connect to host localhost port 2200: Connection refused". Now: detect
  an existing forward and reuse it (also avoids the "Address already in
  use" if osmo:foxglove was started in parallel), otherwise spawn the
  forward in the background, wait up to 30s for it to bind, then launch
  the IDE. Ctrl+C tears down the spawned forward cleanly via a trap.

- workspace image / entrypoint: Cursor Remote-SSH hung indefinitely
  on airstack-dev-13 because (a) cursor-server's installer fell back to
  wget when curl timed out and wget was not in the image, and (b) a
  /tmp/cursor-remote-lock.* file left behind by the first crashed
  install blocked every silent retry. Add wget to the apt install list
  and rm -f the stale Cursor / VS Code remote lock files at the very
  top of entrypoint.sh so each fresh pod starts from a clean slate.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): correct osmo:logs CLI invocation; install Foxglove extensions locally on osmo:foxglove

osmo:logs was invoking `osmo workflow logs <id> workspace --follow`, but
the real CLI takes the task via `-t TASK` (not positionally) and has no
`--follow` flag at all — so the command failed immediately with
"unrecognized arguments: workspace --follow". Replace with a polling loop
that uses `-t workspace -n <N>` on a short interval, prints only the
suffix that appeared since the previous fetch (find-the-last-seen-line
trick; degrades to "reprint tail" with a warning if the cursor outruns
-n), and exits cleanly once the workflow reaches a terminal state.
Tunables: OSMO_LOGS_TASK / OSMO_LOGS_TAIL / OSMO_LOGS_INTERVAL.

osmo:foxglove now installs the AirStack Foxglove extensions
(robot-commands / waypoint-editor / polygon-editor) into the laptop's
local Foxglove user-extensions directory before opening the
port-forward. Without this, custom panels show up as "Unknown panel
type: robot-commands.Robot Tasks" in the laptop's Foxglove Desktop
because it has no way to discover the extension folders that live
inside the GCS container. To avoid duplicating the install logic, the
existing gcs/foxglove_extensions/install.py is refactored to read
FOXGLOVE_EXT_SRC / FOXGLOVE_EXT_DST env vars (the in-container call
already in gcs/docker/gcs-base-docker-compose.yaml keeps working
unchanged via defaults). The wrapper sets those vars to
${PROJECT_ROOT}/gcs/foxglove_extensions and
~/.foxglove-studio/extensions respectively, overridable with
OSMO_FOXGLOVE_EXT_DIR / skippable with OSMO_FOXGLOVE_SKIP_EXTENSIONS=1.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): pin Kit livestream UDP media port to 49099 so osmo:webrtc actually shows pixels

Kit 107's WebRTC livestream picks a UDP media port dynamically. The
documented `omni.services.livestream.nvcf` defaults (minHostPort=47998
maxHostPort=48020 fixedHostPort=0) are ignored by the stock standalone
Kit binary — on airstack-dev-13 it bound to UDP 49042, outside both the
Compose-published range AND the default `osmo:webrtc --udp` forward of
`47995-48012,49000-49007`. Result: TCP signaling on 49100 worked, the
WebRTC Streaming Client window opened, but every SRTP media packet was
dropped → black viewport plus the recurring
`NVST_CCE_DISCONNECTED when m_connectionCount 0 != 1` underflow in Kit's log.

Pin the media port via three `app.livestream.*` settings set on
`SimulationApp` before `omni.kit.livestream.webrtc` is enabled, so
whichever code path the carb.livestream-rtc.plugin consults lands on the
same port:

    app.livestream.fixedHostPort = 49099
    app.livestream.minHostPort   = 49099
    app.livestream.maxHostPort   = 49099

49099 is a deliberate one-off from the 49100 TCP signaling port — same
neighborhood, easy to remember. Verified live on airstack-dev-13 after
`docker compose up -d --force-recreate isaac-sim-livestream`: Kit binds
UDP 49099 (`/proc/net/udp` hex BFCB on 0.0.0.0) and docker-proxy
publishes it from the pod host network.

Knock-on cleanups:
- `simulation/isaac-sim/docker/docker-compose.yaml` shrinks the
  isaac-sim-livestream `ports:` from 27 forwarded ports
  (`47995-48012, 49000-49007 TCP+UDP, 49100 TCP`) to just two:
  `49100/tcp` + `49099/udp`.
- `.airstack/modules/osmo.sh` shrinks `OSMO_WEBRTC_TCP` to `49100` and
  `OSMO_WEBRTC_UDP` to `49099`, so `airstack osmo:webrtc` spawns two
  port-forwards instead of thirty.
- `.gitignore` ignores `.DS_Store` so working from a Mac doesn't leak
  Finder metadata.

After pulling this commit into a running pod: `docker compose up -d
--force-recreate isaac-sim-livestream` to apply the new port mapping;
then re-run `airstack osmo:webrtc` on the laptop to pick up the new
forward ranges. The standalone WebRTC Streaming Client connects to
`localhost` (same address as before) and now actually receives frames.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): render Kit GUI in WebRTC stream; document SSH agent forward for in-pod git push

Two paper-cuts that bit airstack-dev-13 after the WebRTC media port pin
landed (commit 2d9b1611):

(1) The WebRTC stream showed only the bare 3D viewport — no menu bar,
    no toolbar, no panels, no console. Cause: SimulationApp's default
    when `headless=True` is to also hide the UI (`hide_ui=True`). The
    NVIDIA reference at
    `simulation/isaac-sim/standalone_examples/api/isaacsim.simulation_app/livestream.py`
    explicitly opts back into UI rendering plus picks explicit window
    sizing and `display_options=3286` to keep the default grid/axes
    visible. Mirror that config in `example_one_px4_pegasus_launch_script.py`
    when `ISAAC_SIM_LIVESTREAM=true` (local desktop dev keeps the
    minimal `headless=False` path unchanged).

(2) The pod has no SSH private key, only an `authorized_keys` for
    inbound connections from the user's laptop. As a result, `git push`
    from inside the Cursor / VS Code Remote-SSH session inside the pod
    fails with "Permission denied (publickey)". sshd inside the
    workspace image already has `AllowAgentForwarding yes` baked in via
    `osmo/workspace/sshd_config`; the missing piece is purely on the
    Mac side. Update the `~/.ssh/config` block in the tutorial to
    include `ForwardAgent yes` (so the local agent's keys are exposed
    in the pod), `AddKeysToAgent yes` (auto-load on first push), and
    `UseKeychain yes` (macOS-only Keychain unlock without passphrase
    prompts; ignored on Linux). Adds an `ssh-add -l` smoke-test note.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): make osmo:setup idempotent + paste-safe; document Nucleus auth-debug path

osmo:setup hit two failure modes that wasted a debug session each:

- `osmo credential set` is not an upsert for GENERIC creds — re-running
  setup (e.g. to rotate a Nucleus API token) failed with `400 duplicate
  key value violates unique constraint "credential_pkey"` and bailed
  before reaching the airlab-nucleus credential. Delete-then-set each
  credential so re-running is idempotent.
- Bracket-paste mode and cross-OS clipboards routinely smuggle invisible
  bytes around long pastes. Nucleus's auth endpoint silently DENIES a
  token with one extra trailing byte, with no actionable error from the
  client side. _osmo_prompt now strips leading/trailing whitespace and
  CR/NUL bytes via a new _osmo_trim helper, and warns when bytes were
  stripped. cmd_osmo_setup additionally JWT-shape-checks the Nucleus
  token (must be eyJ.<dot>.<dot>.) before submitting it, so a wrong
  paste fails at setup time instead of silently DENIED at pod boot.

Also documents how to debug the "Login Required: Unable to connect
server omniverse://airlab-nucleus..." popup: SSH the Nucleus host and
tail base_stack-nucleus-auth-1 for InternalCredentials.auth status:
DENIED. Adds a "Nucleus connectivity from OSMO" section to the admin
README clarifying that Nucleus over HTTPS uses a single 443 (no need
to open the native 3009-3180 range from the OSMO cluster), per
NVIDIA's TLS docs.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): use Nucleus API-token auth, with double-dollar to survive compose parser

The OSMO entrypoint was writing OMNI_USER=<andrew_id> alongside an API
token JWT in OMNI_PASS, which routes the JWT through the password-
verification path. Nucleus silently DENIES — visible only in
base_stack-nucleus-auth-1 as `InternalCredentials.auth … 'username':
'<andrew>' … status: DENIED` (no Tokens.auth_with_api_token call). Kit
then pops "Login Required: Unable to connect server omniverse://...".

omniclient expects the literal sentinel username `$omni-api-token` paired
with the JWT as the password. The entrypoint now detects a JWT-shaped
OMNI_PASS (header starts with `eyJ`) and emits OMNI_USER=$$omni-api-token
into omni_pass.env. The `$$` is intentional: docker-compose v2
interpolates env_file values, and a single `$` would be eaten by the
parser (`OMNI_USER=$omni-api-token` becomes `OMNI_USER=-api-token` after
${omni}- expansion to empty). The container ultimately sees
OMNI_USER=$omni-api-token, which is the correct sentinel.

Also note for the next debugger: `docker compose restart` does NOT
re-read env_file. Use `docker compose up -d <svc>` to recreate the
container after editing omni_pass.env.

Updates omni_pass_TEMPLATE.env header to document the API-token pattern
explicitly (with the $$ caveat), and adds a troubleshooting row that
distinguishes "wrong auth path" (DENIED with no Tokens.auth_with_api_token
call) from "bad/expired token" (Tokens.auth_with_api_token: DENIED).

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(osmo): make OSMO the recommended dev path, single clone-the-repo flow

Reposition the OSMO tutorial as AirStack's recommended day-to-day
development path (not just a fallback for laptops without GPUs) and
collapse it onto a single recipe: clone the repo, then drive everything
through the airstack osmo:* wrappers in .airstack/modules/osmo.sh.

- docs/tutorials/airstack_on_osmo.md
  - Retitle + rewrite the intro to lead with five concrete advantages
    (pooled GPUs, no local CUDA/Docker/driver maintenance, same image as
    CI + field robots, one-command onboarding, hardware bigger than your
    laptop). Demote the Linux+GPU-desktop path to an escape hatch.
  - Drop the Mac/Windows/no-GPU framing in 'Who is this for?' and the
    mermaid laptop subgraph label.
  - Add 'a local clone of AirStack' to Prerequisites; remove it from the
    'do not need' list.
  - Replace Option A/B credential split with a single
    ./airstack.sh osmo:setup recipe; move the three raw osmo credential
    set calls into a collapsible 'Under the hood' footnote.
  - Replace each step's raw osmo workflow ... command with the
    corresponding airstack osmo:up/logs/ide/webrtc/foxglove/down wrapper;
    preserve the raw form in 'Under the hood' footnotes that cross-link
    cmd_osmo_* in .airstack/modules/osmo.sh.
  - Drop the export WF=... paragraph — the wrappers read the id from
    ~/.airstack/osmo-state automatically; AIRSTACK_OSMO_WF overrides
    per-invocation. \$WF now only appears inside the raw-form footnotes.
  - Sweep Troubleshooting + What-survives tables: redirect raw
    port-forward fixes to the airstack osmo:* equivalents and rename the
    section to 'What survives airstack osmo:down?'.
  - Fix WebRTC edge label (49100/tcp + 49099/udp) to match the pinned
    ports the workflow actually uses today.

Companion cleanups now that the privileged_allowed flip is automatic on
the OSMO autosync side (synchronize_osmo_team_pools.py forces
privileged_allowed: true on every platform of every pool, so students
never see the 'platform does not have privileged flag enabled' error):

- osmo/README.md: drop the 'Most common blocker' privileged warning, the
  privileged_allowed row from the pool-requirements table, and the
  'privileged GPU pod' / '(privileged, GPU)' descriptors in the
  architecture summary. Simplify the validation-stage SSH-failure hint.
- osmo/workflows/airstack-dev.yaml: trim the long DinD-requires-privileged
  comment to a one-liner (the privileged: true directive itself stays).
- .airstack/modules/osmo.sh: remove the special-case 'privileged flag
  enabled' error branch in cmd_osmo_up — it should never fire now.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): make osmo:logs actually stream + survive pod host-key churn

osmo:logs was silent because cmd_osmo_logs wrapped osmo workflow logs in
$( ... ) on the assumption that -n LAST_N_LINES exits after dumping the
tail. Empirically the CLI keeps the stream open as new lines arrive (it
already behaves like tail -f, despite --help advertising only -n), so
command substitution waited forever and printed nothing. Drop the polling
loop and just exec the command directly.

Each fresh OSMO pod also ships a new sshd host key, so every osmo:up
trips StrictHostKeyChecking against the previous workflow's fingerprint
and SSH/Cursor abort with "Host key for [localhost]:2200 has changed".
Switch the recommended ~/.ssh/config block (and osmo/README.md) to the
ephemeral-host pattern (StrictHostKeyChecking no + UserKnownHostsFile
/dev/null + LogLevel ERROR), and have cmd_osmo_ide ssh-keygen -R the
stale loopback entry on every run so users on the old config get
unblocked automatically.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): auto-pin --branch to local checkout + clean error UX when workflow dies

The pod's entrypoint clones AirStack fresh from GitHub on every workflow
start (the pod fs is ephemeral). It defaulted to `main`, so any developer
testing branch-only OSMO changes silently ran their pod against stale
`main` code — most visibly: COMPOSE_PROFILES=desktop,isaac-sim-livestream
resolved to "desktop" alone on `main` because the isaac-sim-livestream
service only exists on the feature branch, so isaac-sim never came up
and `airstack osmo:webrtc` showed a blank stream.

  - cmd_osmo_up now defaults --branch to the local repo's current
    branch (git rev-parse --abbrev-ref HEAD). Detached HEAD or
    non-git checkouts fall back to `main` cleanly. Pass --branch
    explicitly to override.
  - New _osmo_check_branch_pushed warns up-front when the about-to-
    submit branch has no upstream, is ahead of origin, or has an
    uncommitted working tree. The pod doesn't see your laptop's edits.

Separately, when an OSMO workflow gets canceled mid-flight (osmo:down
in another shell, or OSMO timing it out), the in-flight port-forward
and logs streams raise OSMOUserError("Workflow X is not running!")
from inside an asyncio Task. The CLI prints "Task exception was never
retrieved" + a multi-line Traceback that buries the actual one-line
cause. New _osmo_pf_filter awk script collapses that into a single
[ERROR] line pointing at `airstack osmo:up`. Wired into webrtc,
foxglove, and logs. webrtc also gains a cleanup trap that kills the
backgrounded UDP port-forward on EXIT/INT/TERM so we don't leak it
against a dead workflow.

Tutorial Step 2 documents the new --branch default and the
"pod-clones-from-GitHub-not-your-laptop" gotcha.

Co-authored-by: Cursor <cursoragent@cursor.com>

* perf(osmo): bump inner dockerd concurrency to saturate 10 GbE pulls

dockerd's defaults of --max-concurrent-downloads=3 / --max-concurrent
-uploads=5 cap a fresh airstack-dev pod's image-pull at ~300 MiB/s
against the airlab-backup-10g registry — single-stream TLS tops out
around 300-500 MiB/s per core, and three parallel streams of unevenly
sized blobs serialize down to that ceiling. Ceph (1014 TiB, 92 OSDs,
SSD pools) and 10 GbE both have far more headroom than that. Bump to
10/10 to overlap enough blob downloads to saturate the pipe.

Threaded through the DOCKERD_MAX_DOWNLOADS / DOCKERD_MAX_UPLOADS env
vars so a pool can be tuned at submit time without rebuilding the
workspace image.

Workspace image needs a rebuild + push for this to take effect:
  cd osmo/workspace
  docker build -t airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest .
  docker push   airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(osmo): require buildx --platform linux/amd64 for workspace image

A plain `docker build && docker push` on an Apple Silicon Mac silently
produces a linux/arm64-only `latest` manifest. OSMO workers are amd64,
so every subsequent workflow fails at the outer pod-image pull with
"no match for platform in manifest" before the entrypoint even runs —
a confusing failure mode whose root cause lives entirely in the push,
not in the workflow yaml or the entrypoint.

Switch the README and the Dockerfile docstring to the buildx form,
explain the why, and document the post-push manifest check.

Co-authored-by: Cursor <cursoragent@cursor.com>

* perf(osmo): move dockerd data-root to /osmo/run for native overlay2

The OSMO pod's `/` is itself a containerd overlay snapshot, and Linux
refuses to stack a second overlayfs on top of an overlay rootfs — which
is why the inner dockerd was falling through to fuse-overlayfs. That
costs a kernel↔userspace FUSE round-trip on every `creat()` during
layer extraction, which murders throughput on apt/pip/ROS layers
(measured: 32-50 MB/s for small-file-heavy layers vs 480 MB/s for
big-file layers in the same pull).

Pointing dockerd at /osmo/run/docker (the kubelet emptyDir backed by
ext4 on /dev/vda3) lets the existing overlay2-first fallback chain
actually succeed on its first try, restoring kernel-overlay extraction
performance. emptyDir lifetime matches the workflow lifetime, so the
docker layer cache gets the right scope automatically.

Falls back to /var/lib/docker if /osmo/run isn't present so the image
still works in non-OSMO test contexts.

Co-authored-by: Cursor <cursoragent@cursor.com>

* updated version

* added virtual display for GL context

* added virtual display for droan_gl

* droan_gl patch

* run Xvfb in its own tmux session

* updated dockerfile + version

* typo in docs

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in osmo logs, renamed airstack-isaac-sim to just isaac-sim

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in container name for isaac-sim-livestream

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* airstack-dev version overwrite removed

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

---------

Co-authored-by: Cursor <cursoragent@cursor.com>
Co-authored-by: krrishj18 <krrishj@andrew.cmu.edu>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* Add fixed-trajectory evaluation tests

New tests/test_fixed_trajectory.py evaluates drone performance on Circle,
Figure8, Racetrack, and Line trajectories: takeoff -> execute -> land with
cross-track error, path RMSE, execution time, and success metrics recorded
to metrics.json for baseline comparison.

- Python ideal-path generators mirror fixed_trajectory_task.cpp equations
- Cross-track error uses robot pose snapshot at dispatch to transform
  base_link ideal path to world frame for odom comparison
- 5m loose tolerance documents the known circle failure without stranding drone
- conftest.py gains --trajectory-types CLI option and generalised phase-order
  sorting/ID-rewriting for both autonomy test modules
- tests/README.md documents the new module, all 11 metrics, and run commands

Made-with: Cursor

* Spherical lookahead bug that fixed the circle test and caused the circle test to pass

* Added in code that consolidated all the results code so the user can easily see their results in one file without having to wade through a ton of log files to get what they need

* Results for 10 tries headless summary statistics

* Fixed the logging files so now it only outputs one summary file and it doesn't inundate the user with a ton of log files for no reason

* deleted cleanup_old_results.sh which was a local tool for cleaning up everything

* Added preliminary docs to explain changes made

* Changed .env to say 0.19.0-alpha.4

* Resolved all the merge conflicts that are in this file

* Revert sphere_radius to 1.0; velocity_sphere_radius_multiplier=1.0 makes the fixed value inert

Co-authored-by: Cursor <cursoragent@cursor.com>

* Remove internal branch reference from baseline; note AirStation hardware

Co-authored-by: Cursor <cursoragent@cursor.com>

* Remove parameter tuning bullet from docs after reverting sphere_radius

Co-authored-by: Cursor <cursoragent@cursor.com>

* Move system-test prerequisites to index.md and reference it from fixed-trajectory doc

Co-authored-by: Cursor <cursoragent@cursor.com>

* Remove path tracker bug fixes section from docs (covered in PR description)

Co-authored-by: Cursor <cursoragent@cursor.com>

* Trim duplicated stack bring-up from manual usage; link to Getting Started

Co-authored-by: Cursor <cursoragent@cursor.com>

* Reframe fixed-trajectory doc as end-to-end testing guide

Rename fixed_trajectory_testing.md to end_to_end_testing.md (history preserved), add e2e intro and future-work note, fix stale test path to tests/system, and update mkdocs nav, testing index, and tests/README references.

Co-authored-by: Cursor <cursoragent@cursor.com>

* removed stale test_sensors file

* incremented version tag

* Fixed the summary.txt file after it broke after a ton of commits were completed.

* resyncing Pegasus module to fixed camera initialization fix

---------

Co-authored-by: pvkumara <pkumara@andrew.cmu.edu>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
Co-authored-by: John Liu <63010779+JohnYanxinLiu@users.noreply.github.com>
Co-authored-by: copilot-swe-agent[bot] <198982749+Copilot@users.noreply.github.com>
Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: Krrish Jain <krrishj@andrew.cmu.edu>
Co-authored-by: krrishj18 <krrishj18@users.noreply.github.com>
Co-authored-by: Claude Opus 4.7 <noreply@anthropic.com>
Co-authored-by: airlab <johnliuchs2022@gmail.com>
Co-authored-by: Andrew Jong <andrewjong@fieldai.com>
Co-authored-by: github-actions[bot] <41898282+github-actions[bot]@users.noreply.github.com>
Co-authored-by: Sebastian Scherer <basti@andrew.cmu.edu>
Co-authored-by: Cursor <cursoragent@cursor.com>
```

+1813 / −524 in 23 files:

- `--gui`
- `--num-robots`
- `--sim`
- `--stress-iterations`
- `--trajectory-types`
- `-v`
- `.agents/skills/run-system-tests/SKILL.md`
- `.env`
- `AGENTS.md`
- `docs/development/intermediate/testing/end_to_end_testing.md`
- `docs/development/intermediate/testing/index.md`
- `git-hooks/docker-versioning/update-docker-image-tag_BACKUP_3660135.pre-commit`
- `git-hooks/docker-versioning/update-docker-image-tag_BASE_3660135.pre-commit`
- `git-hooks/docker-versioning/update-docker-image-tag_LOCAL_3660135.pre-commit`
- `git-hooks/docker-versioning/update-docker-image-tag_REMOTE_3660135.pre-commit`
- `mkdocs.yml`
- `robot/ros_ws/src/local/controls/trajectory_controller/src/trajectory_controller.cpp`
- `robot/ros_ws/src/local/planners/trajectory_library/src/trajectory_library.cpp`
- `tests/README.md`
- `tests/conftest.py`
- `tests/pytest.ini`
- `tests/run_summary.py`
- `tests/system/test_fixed_trajectory.py`

## c476db32a4 — 2026-07-21 — John Liu — KEYWORD fixes

**General robot deployment infra: aarch64 build args + robot-name resolution fixes (#370)**

https://github.com/castacks/AirStack/commit/c476db32a452962d0200a99cda3881aa5ff2783e

```
Foundational real-robot deployment fixes extracted from the OptiTrack
emulation PR (#367) so they can be reviewed and merged first; #367 will be
rebased on top afterward, shrinking its diff.

Docker / ARM build:
- Add TARGET_ARCH build arg (default x86_64) to Dockerfile.robot and use it to
  parametrize LD_LIBRARY_PATH, so the aarch64 (Jetson/l4t, voxl) images link
  against the correct arch triplet.
- docker-compose.yaml passes TARGET_ARCH: aarch64 to the voxl and l4t image
  builds.
- Install ros-${ROS_DISTRO}-mavros-extras (generic dep; also provides the
  vision_pose plugin used by external-pose deployments).

Robot name resolution:
- .bashrc now follows a pre-set ROBOT_NAME (e.g. injected by docker compose)
  instead of always overriding it from the container/hostname mapping. The
  bws() flock build lock is retained.
- default_robot_name_map.yaml catch-all fallback maps to unknown_robot (valid
  ROS namespace token) instead of unknown-robot.

Version bumped 0.19.0-alpha.5 -> 0.19.0-alpha.6 for the version-increment gate.

Note: the trajectory_controller/trajectory_library robustness fixes originally
listed for extraction are already present on develop (PR #365), so they are not
included here.

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
```

+47 / −30 in 6 files:

- `.env`
- `CHANGELOG.md`
- `robot/docker/.bashrc`
- `robot/docker/Dockerfile.robot`
- `robot/docker/docker-compose.yaml`
- `robot/docker/robot_name_map/default_robot_name_map.yaml`

## 1a25d60b43 — 2026-07-22 — John Liu — KEYWORD fix,fixes

**l4t deployment fixes: make the Jetson profile build + boot on real hardware (#371)**

https://github.com/castacks/AirStack/commit/1a25d60b439e4e9cc98d2f8b1f1116f5fab51a54

```
* feat(l4t): make robot-l4t deployment knobs overridable + document name resolution

Parametrize the robot-l4t compose service so a single service covers real
deployments without editing compose:
- AUTONOMY_ROLE and FCU_URL are now ${VAR:-default} overridable (and FCU_URL is
  unquoted so the literal serial path reaches mavros).
- Rosbag output path is BAG_STORAGE_PATH-overridable.

Update the configure-multi-robot skill to reflect the honor-pre-set-ROBOT_NAME
guard (#370): document pinning ROBOT_NAME in an override for a single real robot,
the never-on-the-shared-service caveat, and the unknown_robot fallback fixes by
topology.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* feat(l4t): add site-agnostic l4t-px4-realrobot override template

Deployment override for a single real PX4 robot on a Jetson (aarch64/l4t).
Surfaces the common knobs at the top with sensible defaults: ROBOT_NAME pinned
directly (single-robot shortcut honored by .bashrc), FCU_URL, AUTONOMY_ROLE,
BAG_STORAGE_PATH, and RECORD_BAGS. Mocap-agnostic — NatNet/external-vision
settings are added by a separate optitrack override.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* fix(l4t): entrypoint passthrough + ZED SDK 5.2; document build gotchas

Two real-hardware build fixes for the Jetson profile:
- Dockerfile.l4t-stack-base: overwrite dustynv's /ros_entrypoint.sh with an
  `exec "$@"` passthrough. Its prebuilt source-ROS libs (fastcdr 2.2.5) were
  shadowing the apt Jazzy (2.2.7) that Dockerfile.robot layers on, crashing
  apt-built nodes like mavros with symbol-lookup errors under tmux autolaunch.
- zed/Dockerfile.zed-l4t: bump ZED SDK 4.2 -> 5.2 and move the coupled ROS deps
  together (zed_msgs 5.2.1, point_cloud_transport(_plugins) 4.x, add backward_ros).

Document both gotchas in the docker-build-profiles skill, and correct the stale
unknown-robot -> unknown_robot in the robot_identity reference doc.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.7

Version-increment gate: bump above develop's 0.19.0-alpha.6 and record the l4t
deployment changes in the changelog.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

---------

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
```

+89 / −14 in 9 files:

- `.agents/skills/configure-multi-robot/SKILL.md`
- `.agents/skills/docker-build-profiles/SKILL.md`
- `.env`
- `CHANGELOG.md`
- `docs/robot/docker/robot_identity.md`
- `overrides/l4t-px4-realrobot.env`
- `robot/docker/Dockerfile.l4t-stack-base`
- `robot/docker/docker-compose.yaml`
- `robot/docker/zed/Dockerfile.zed-l4t`

## 47f8c798d5 — 2026-07-31 — John Liu — KEYWORD break,failing,fails,fix,fixes,regression

**Test infra rework: YAML-driven unit-test collection + integration tier (#372)**

https://github.com/castacks/AirStack/commit/47f8c798d591b99bc79c152ede75755b582bdbcb

```
* feat(l4t): make robot-l4t deployment knobs overridable + document name resolution

Parametrize the robot-l4t compose service so a single service covers real
deployments without editing compose:
- AUTONOMY_ROLE and FCU_URL are now ${VAR:-default} overridable (and FCU_URL is
  unquoted so the literal serial path reaches mavros).
- Rosbag output path is BAG_STORAGE_PATH-overridable.

Update the configure-multi-robot skill to reflect the honor-pre-set-ROBOT_NAME
guard (#370): document pinning ROBOT_NAME in an override for a single real robot,
the never-on-the-shared-service caveat, and the unknown_robot fallback fixes by
topology.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* feat(l4t): add site-agnostic l4t-px4-realrobot override template

Deployment override for a single real PX4 robot on a Jetson (aarch64/l4t).
Surfaces the common knobs at the top with sensible defaults: ROBOT_NAME pinned
directly (single-robot shortcut honored by .bashrc), FCU_URL, AUTONOMY_ROLE,
BAG_STORAGE_PATH, and RECORD_BAGS. Mocap-agnostic — NatNet/external-vision
settings are added by a separate optitrack override.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* fix(l4t): entrypoint passthrough + ZED SDK 5.2; document build gotchas

Two real-hardware build fixes for the Jetson profile:
- Dockerfile.l4t-stack-base: overwrite dustynv's /ros_entrypoint.sh with an
  `exec "$@"` passthrough. Its prebuilt source-ROS libs (fastcdr 2.2.5) were
  shadowing the apt Jazzy (2.2.7) that Dockerfile.robot layers on, crashing
  apt-built nodes like mavros with symbol-lookup errors under tmux autolaunch.
- zed/Dockerfile.zed-l4t: bump ZED SDK 4.2 -> 5.2 and move the coupled ROS deps
  together (zed_msgs 5.2.1, point_cloud_transport(_plugins) 4.x, add backward_ros).

Document both gotchas in the docker-build-profiles skill, and correct the stale
unknown-robot -> unknown_robot in the robot_identity reference doc.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.7

Version-increment gate: bump above develop's 0.19.0-alpha.6 and record the l4t
deployment changes in the changelog.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* test(infra): collect co-located unit tests via the package list + integration tier

Unit tests are defined by tests/colcon_unit_test_packages.yaml: conftest.py resolves
each listed package to its <pkg>/test dir and collects the non-linter test_*.py files
under --import-mode=importlib (set in pytest.ini), marking each `unit` by path. ament
lint files are skipped (they run under colcon test). Removes two now-unnecessary files
under tests/robot/; the package test/ dirs are collected directly.

Also add an integration test tier: tests/integration/ + `integration` mark + a shared
robot_autonomy_stack fixture (robot-desktop container, no sim/GPU), slotted into
_MODULE_ORDER between build_packages and the sim tiers.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* docs(testing): describe unit tests as co-located and listed in the package YAML

Update the add-unit-tests and run-system-tests skills, AGENTS.md, and the unit-testing
docs: adding a unit test is "list the package in colcon_unit_test_packages.yaml", and the
source lives in the package's own test/ dir. Document the `integration` mark/tier.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.8

Version-increment gate: bump above develop (0.19.0-alpha.6); alpha.7 is taken by the
l4t-deployment-fix PR. Record the test-infra changes in the changelog.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* refactor(tests): split unit-test discovery + session state into tests/harness/

Begin modularizing conftest.py (959 lines) by concern. Extract two self-contained
pieces into a new tests/harness/ package:
- harness/session.py: session-scoped mutable state (results dir, current pytest item,
  last subprocess output, logger) with setter/getter accessors. Hooks write it; helpers
  read it, so helper modules no longer reach into conftest globals.
- harness/discovery.py: unit-test discovery driven by colcon_unit_test_packages.yaml
  (repo_path, load_colcon_unit_test_config, colcon_test_robot_command, unit_test_dirs,
  unit_test_files, _is_unit_item).

conftest.py imports from harness and its hooks delegate to the session accessors; it
re-exports AIRSTACK_ROOT / colcon_test_robot_command / load_colcon_unit_test_config /
logger so existing `from conftest import ...` in the system tests keeps working
unchanged. Behavior-preserving (host-validated): `-m unit` still 14 passed / 152
deselected, 166 collected, same order. Follow-on: the commands/containers/metrics/sim
helpers and collection ordering move out the same way.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* refactor(tests): extract commands/containers/metrics/sim helpers into tests/harness/

Continue modularizing conftest.py. Move the subprocess/ros2 command helpers
(harness/commands.py), docker container + compute-usage + image helpers
(harness/containers.py), MetricsRecorder + get_metrics/current_test_id
(harness/metrics.py), and the sim target configs + ros2 topic sampling
(harness/sim.py) out of conftest.py.

conftest.py drops from 836 to 360 lines and re-exports the harness helper API
(`from harness import *`) so `from conftest import <name>` in the system tests +
sensor_probes keeps working unchanged. Behavior-preserving: -m unit still 14 passed /
152 deselected, 166 collected, same order. Remaining in conftest: pytest hooks,
collection ordering, and the airstack_env / robot_autonomy_stack fixtures.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* refactor(tests): extract collection ordering into tests/harness/collection.py

Final step of the conftest.py modularization: move test ordering — _MODULE_ORDER, the
per-module phase chains, _module_key, and the parametrize-id rewrite — into
harness/collection.py. conftest's pytest_collection_modifyitems hook now delegates to
collection.modify_items(items).

conftest.py is now 246 lines (from 959): pytest hooks + the airstack_env /
robot_autonomy_stack fixtures. All helpers live in tests/harness/ by concern (session,
discovery, commands, containers, metrics, sim, collection). Behavior-preserving: -m unit
still 14 passed / 152 deselected, 166 collected, unit → build → integration → sim order
unchanged.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

Also sync docs/skills to the tests/harness/ layout (AGENTS.md, tests/README.md,
tests/integration/README.md, run-system-tests + add-unit-tests skills, unit_testing +
end_to_end_testing docs): helpers, MetricsRecorder, the workspace globs, and _MODULE_ORDER
now point at tests/harness/ instead of conftest.py (still re-exported via conftest).

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(robot): pin pytest to 7.4.* so apt launch_pytest stays compatible

The builder-stage pip block pulled pytest >=8 transitively into /usr/local (copied
into the runtime image), shadowing Jazzy's apt python3-pytest 7.4. pytest 8 removed
the `path` argument from pytest_pycollect_makemodule, which apt's launch_pytest plugin
still declares — so every pytest invocation in the robot container aborted at plugin
registration. This broke `colcon test` for ament_python packages (e.g.
lidar_point_cloud_filter in test_colcon_test_robot), while ament_cmake gtest packages
were unaffected.

Pin pytest to Jazzy's version so the container is internally consistent and
launch_testing / launch_pytest remain usable for future launch-based tests. The test
runner (tests/docker) is a separate interpreter and keeps its newer pytest.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* fix(isaac-sim): clear LD_LIBRARY_PATH for PX4 ubuntu.sh so ca-certificates configures

The global ENV LD_LIBRARY_PATH puts isaac-sim's bundled libs
(.../isaacsim.ros2.bridge/jazzy/lib) on the linker path. Its older libcrypto.so.3
shadows the system one, so when the updated ca-certificates (20240203 →
20260601~24.04.1) runs its postinst `openssl`, it fails with
`version 'OPENSSL_3.0.9' not found`, aborting the apt transaction and failing the
isaac-sim image build (PX4 Tools/setup/ubuntu.sh, exit 100).

Clear LD_LIBRARY_PATH for that RUN only so apt/openssl use the system libcrypto;
the global ENV still applies to every other layer. Environmental break (new
ca-certificates × isaac-sim's stale bundled openssl) — not a code regression.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

---------

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
Co-authored-by: Cursor <cursoragent@cursor.com>
```

+1145 / −916 in 27 files:

- `.agents/skills/add-unit-tests/SKILL.md`
- `.agents/skills/docker-build-profiles/SKILL.md`
- `.agents/skills/run-system-tests/SKILL.md`
- `.env`
- `AGENTS.md`
- `CHANGELOG.md`
- `docs/development/intermediate/testing/end_to_end_testing.md`
- `docs/development/intermediate/testing/index.md`
- `docs/development/intermediate/testing/unit_testing.md`
- `robot/docker/Dockerfile.robot`
- `simulation/isaac-sim/docker/Dockerfile.isaac-ros`
- `tests/README.md`
- `tests/colcon_unit_test_packages.yaml`
- `tests/conftest.py`
- `tests/harness/__init__.py`
- `tests/harness/collection.py`
- `tests/harness/commands.py`
- `tests/harness/containers.py`
- `tests/harness/discovery.py`
- `tests/harness/metrics.py`
- `tests/harness/session.py`
- `tests/harness/sim.py`
- `tests/integration/README.md`
- `tests/pytest.ini`
- `tests/robot/README.md`
- `tests/robot/perception/natnet_ros2/test_natnet_ros2.py`
- `tests/robot/sensors/lidar_point_cloud_filter/test_validation_core.py`

## 55d9b887d9 — 2026-08-04 — Andrew Jong — KEYWORD error,fixes

**Add waypoint_flight system test judged by a standalone track checker (#378)**

https://github.com/castacks/AirStack/commit/55d9b887d983c4837e350b1102543897f307c4cf

```
* Add waypoint_flight system test judged by standalone track checker

New end-to-end acceptance test for planner integration/swaps:
takeoff -> ordered waypoint route -> land, per (sim, num_robots, iter).

- tests/system/test_waypoint_flight.py (mark: waypoint_flight): after
  takeoff, sends the route to the local planner's NavigateTask action
  as a nav_msgs/Path and captures odometry throughout; reuses the
  flight-cycle workers from test_fixed_trajectory.py (chain guard,
  takeoff/land, odom CSV capture).
- tests/waypoint_checker.py: standalone stdlib-only judge — the
  odometry track must pass within --waypoint-tolerance of every
  waypoint IN ORDER, each within --waypoint-timeout of the previous
  arrival. Success is defined purely on the odometry track (not the
  action result), so swapping the global or local planner leaves the
  judgment unchanged; the checker also runs outside the harness on
  any ros2 `topic echo --csv` odometry dump.
- Waypoints are relative to the robot pose at dispatch (x forward
  along heading, z up), so routes are spawn/sim agnostic. Default:
  10 m square at takeoff altitude.
- New pytest options: --waypoints, --waypoint-tolerance,
  --waypoint-timeout; mark registered in pytest.ini; docs in
  tests/README.md and AGENTS.md; VERSION 0.19.0-alpha.9 + CHANGELOG.

Metrics recorded per robot: waypoint_success, waypoints_reached,
navigate_action_success, route_time_sim_s, worst_closest_approach_m.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Calibrate waypoint_flight to validated stock behavior in Isaac Sim

Validated end-to-end against Isaac Sim + the stock stack (4/4 phases
pass in 3m20s; corners cut 3.75/5.13 m, final goal error 0.63 m).
Fixes found by flying:

- Path header frame: an empty frame_id crashed droan_gl (uncaught
  tf2::InvalidArgumentException in its plan TF transform); the goal now
  carries the frame from the odometry snapshot (fallback "map").
- Dense plan dispatch: sparse poses get corner-skipped by the local
  planner's distance-walking look-ahead; the route is now interpolated
  at 1 m from the current pose (mirrors real global-planner output).
- Route/tolerance semantics: the stack's contract is "reach the goal
  precisely, follow the corridor loosely" (droan_gl cost =
  deviation - path_distance cuts corners ~4-7 m). Split tolerances:
  intermediate corridor 15 m, final goal 2.5 m (new --goal-tolerance;
  NavigateTask's 1.5 m + tracking lag). Default route is now an open
  30 m square — NavigateTask succeeds on distance to the FINAL pose,
  so closed loops succeed instantly without flying (documented).
- Settle capture: the action succeeds on the tracking point, which
  leads the drone by up to the look-ahead distance (~10 m); capture
  now continues until the drone is stationary (max 30 s) so the goal
  approach is recorded. New metric: final_goal_error_m.
- waypoint_checker: closest_approach now reports the true minimum over
  the remaining track instead of the tolerance-boundary crossing
  (arrival stays first-crossing, ordering semantics unchanged).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Raise default waypoint route +10m to clear scene clutter

Validated on both sim backends with the identical default config
(open 30 m square climbing to ~20 m AGL):
- Isaac Sim: corners 5.67/5.72 m, final goal 0.28 m, 4/4 phases
- ms-airsim (Blocks): corners 5.93/5.67 m, final goal 0.89 m, 4/4

At the old takeoff-altitude route the drone collided with a Blocks
obstacle (disparity was streaming, so DROAN had perception — the
corner-cut diagonals leave the forward stereo's coverage). This test
judges route-following, not obstacle avoidance, so the default route
flies above the clutter; documented in the option help and README.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Add waypoint_flight screenshots from validation runs

Captured mid-route during the validated flights: Isaac Sim viewport
with the drone on the square route, ms-airsim Blocks with the drone
clearing the obstacle field (collision count 0), and the Foxglove GCS
dashboard showing the planned path, expanded obstacle voxels, robot
task panel, and live stereo feed. Embedded in the waypoint section of
tests/README.md.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+687 / −2 in 11 files:

- `.env`
- `AGENTS.md`
- `CHANGELOG.md`
- `tests/README.md`
- `tests/assets/waypoint_flight_foxglove.png`
- `tests/assets/waypoint_flight_isaac.jpg`
- `tests/assets/waypoint_flight_msairsim_blocks.jpg`
- `tests/conftest.py`
- `tests/pytest.ini`
- `tests/system/test_waypoint_flight.py`
- `tests/waypoint_checker.py`

## 234587aa05 — 2026-08-05 — John Liu — KEYWORD fails,failure,fix,fixed,fixes

**Robot deployment fixes: bag recording + adding warning for robot-identity failure (#377)**

https://github.com/castacks/AirStack/commit/234587aa05b1011d5f20bf8c0a2857433eedf3fc

```
* make RECORD_BAGS actually reach the bag recorder

LOG_CONFIG selects which topic set in logging_bringup/config to record, default
log.yaml.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* warn when the robot identity fails to resolve

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.12

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fixed comments and documentation

* fix the bag recording status bridge direction

It was bridged gcs -> robot, the same direction as the command it answers, so
status never reached the GCS and the rqt Recording: label stayed blank.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fix the exclude flag so the main bag section records

ros2 bag record renamed --exclude to --exclude-regex, and the old name is now an
ambiguous prefix of four options, so argparse rejected the command and any section
using exclude: recorded nothing.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* restore the bags .gitignore files

#318 dropped robot/bags/.gitignore and gcs/bags/.gitignore while moving a dozen
others; nothing has covered recorded bags since.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Opus 5 <noreply@anthropic.com>
```

+120 / −31 in 12 files:

- `.agents/skills/configure-multi-robot/SKILL.md`
- `.env`
- `CHANGELOG.md`
- `common/ros_packages/logging/bag_recorder_pid/bag_record_pid/bag_record_node.py`
- `common/ros_packages/logging/logging_bringup/launch/logging.launch.xml`
- `docs/robot/docker/robot_identity.md`
- `gcs/bags/.gitignore`
- `overrides/l4t-px4-realrobot.env`
- `robot/bags/.gitignore`
- `robot/docker/.bashrc`
- `robot/docker/robot-base-docker-compose.yaml`
- `robot/ros_ws/src/autonomy_bringup/onboard_all/config/domain_bridge.yaml`

## 5cf595523e — 2026-08-14 — pvkumara — KEYWORD failed,fails,failure,fix,regression,repair

**ci: land OSMO ephemeral runners and system-test harness on develop (#382)**

https://github.com/castacks/AirStack/commit/5cf595523e3ebec1b7b7fd8ad3c20c271c25af3a

```
* ci(orchestrator): migrate ephemeral CI runners from OpenStack to NVIDIA OSMO

Replace the OpenStack-Nova spawn/reap backend with OSMO workflow submission. The GitHub side is unchanged (self-hosted/airstack-ephemeral labels, single-use JIT runner tokens, same-repo fork guard) and the one-job-per-worker destroy-after model is preserved; only the spawn target moved from creating a Nova VM to submitting an OSMO workflow.

orchestrator.py: submit/query/cancel/list via the osmo CLI, job_id -> workflow_id state, re-login-on-auth-failure, orphan sweep via osmo workflow list; drop floating-IP/boot-volume/placement/keypair/security-group logic.

runner.Dockerfile + runner-entrypoint.sh + runner-workflow.yaml.j2: prebaked privileged docker-in-docker + GPU GitHub runner image/task (replaces cloud-init.yaml.j2).

config.example.yaml, setup.sh, airstack-orchestrator.service, requirements.txt: OSMO service-account token auth, install the osmo CLI, drop openstacksdk. Docs (AGENTS.md, tests/README.md, orchestrator README) updated to OSMO.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci(orchestrator): pin AirLab OSMO JSON keys and runner image path

Resolve uuid/live name after submit (OSMO returns name-only + suffix),
default config to the Keycloak-backed airstack pool and Harbor runner
image, and add scripts to build/push airstack-ci-runner on OSMO DinD.

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(ci): document the OSMO-backed CI/CD pipeline

Fills in the empty ci_cd.md stub with an end-to-end guide to how CI runs
the full AirStack stack on ephemeral OSMO GPU pods: architecture and job
lifecycle diagrams, runner pod anatomy, the three trigger paths, what
each pytest mark catches, the metrics regression gate, the security
model, and layer-by-layer troubleshooting.

Adds the page to the mkdocs nav (it was previously unreachable) and
cross-links it from tests/README.md and the testing index.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): repair Docker builds on OSMO ephemeral runners

Every build_docker and build_packages test failed on the OSMO backend
because the inner dockerd kept its data-root on the pod's overlayfs
rootfs. Linux rejects a directory on overlayfs as an overlay upperdir,
so image pulls still succeeded -- containerd unpacks layers with plain
writes -- while every build step needing a real mount died with
"mount source: overlay ... err: invalid argument", surfacing as
unrelated-looking apt-get and WORKDIR failures.

runner-entrypoint.sh now picks a storage backend by attempting a real
overlay mount rather than trusting the filesystem type, preferring a
loopback ext4 data-root (real overlay2, sparse, dies with the pod) and
falling back to a pod-mounted filesystem, fuse-overlayfs, then vfs.
vfs is a last resort only: it copies the whole filesystem per layer and
would exhaust the storage request on the sim images.

Also bumps the GitHub Actions runner to 2.336.0, since 2.334.0 stops
being able to run jobs on 2026-08-10.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): seed PR Docker builds from a floating cache tag

Versioned cache_from entries always miss on PRs because VERSION is forced
up; add a stable cache_* tag published only by docker-build.yml so system
tests can reuse layers without writing the shared cache.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci(docker-build): retag unchanged images on VERSION bump

Skip full compose rebuilds when a service's content fingerprint matches
the previous versioned image label; registry-retag instead and only
rebuild services whose Docker inputs changed.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): parse quoted .env values before inline comments

docker_image_plan was feeding NUM_ROBOTS with a trailing comment into
compose config, which broke strconv.Atoi for deploy.replicas.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci(docker-build): build/push services sequentially

Publish successful images even when a sibling (e.g. isaac-sim) fails, and
still cosign whatever was retagged or pushed in the same run.

Co-authored-by: Cursor <cursoragent@cursor.com>

* chore: bump VERSION to 0.19.0-alpha.8 for retag validation

Seeded gcs/ms-airsim/robot images carry content-fingerprint labels; this
bump should registry-retag those digests without rebuilding.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): unblock isaac-sim PX4 apt and robot colcon pytest

Isaac's PX4 ubuntu.sh fails dpkg configure on the NVIDIA base; pre-fix
ca-certificates, drop software-properties-common, and skip NuttX/Gazebo
like ms-airsim. Pin pytest<8.1 and disable launch_testing for colcon
unit tests so ROS Jazzy's outdated pytest hook no longer aborts CI.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): pass colcon --pytest-args as separate tokens

A single quoted blob made pytest treat "-p no:launch_testing" as part of
the -m expression, which broke lidar_point_cloud_filter colcon tests.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): quote colcon pytest args through bash -ic

Nested single quotes around 'not linter' terminated the outer bash -ic
string early, so pytest saw 'not' as a path. Use shlex.quote for the
whole command and list-form pytest_args in the YAML.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): pass colcon pytest flags via PYTEST_ADDOPTS

colcon --pytest-args is a single nargs='*' option, so repeating it
dropped -p and pytest treated no:launch_testing as a file path.
Set PYTEST_ADDOPTS with docker exec -e instead.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): rename helper so pytest does not treat it as a hook

conftest functions named pytest_* are registered as hooks.
pytest_addopts_env caused PluginValidationError and exit code 3.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci: skip image-build for build_packages reruns

Pull and retag cache_* images instead of baking isaac/airsim on every
colcon/pytest iteration. /pytest --no-image-build does the same for
other marks. compose up --no-build when AIRSTACK_NO_IMAGE_BUILD=1.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): disable pytest plugin autoload for colcon tests

-p no:launch_testing is applied after setuptools entrypoints load, so
pytest 8.1+ still crashes on launch_testing's path= hook. Set
PYTEST_DISABLE_PLUGIN_AUTOLOAD so cache_* robot images (unpinned pytest)
can run lidar tests without a rebuild.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): skip lidar ament linters in package pytest config

PYTEST_ADDOPTS -m not linter never reached ament pytest, so copyright /
flake8 / pep257 still ran after the unit tests passed. Ignore those
modules in setup.cfg and collect_ignore.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci: default system tests to isaacsim only

PR-open and bare /pytest were sweeping both sims. Default --sim to
isaacsim; msairsim is opt-in via --sim msairsim.

Co-authored-by: Cursor <cursoragent@cursor.com>

---------

Co-authored-by: pvkumara <pkumara@andrew.cmu.edu>
Co-authored-by: Cursor <cursoragent@cursor.com>
```

+2708 / −862 in 41 files:

- `.agents/skills/bump-version-and-release/SKILL.md`
- `.agents/skills/run-system-tests/SKILL.md`
- `.env`
- `.github/orchestrator/README.md`
- `.github/orchestrator/airstack-orchestrator.service`
- `.github/orchestrator/build-and-push.sh`
- `.github/orchestrator/build-runner-on-osmo.yaml`
- `.github/orchestrator/cloud-init.yaml.j2`
- `.github/orchestrator/config.example.yaml`
- `.github/orchestrator/orchestrator.py`
- `.github/orchestrator/requirements.txt`
- `.github/orchestrator/runner-entrypoint.sh`
- `.github/orchestrator/runner-workflow.yaml.j2`
- `.github/orchestrator/runner.Dockerfile`
- `.github/orchestrator/setup.sh`
- `.github/workflows/docker-build.yml`
- `.github/workflows/scripts/docker_image_plan.py`
- `.github/workflows/system-tests.yml`
- `.gitignore`
- `AGENTS.md`
- `CHANGELOG.md`
- `airstack.sh`
- `docs/development/intermediate/testing/ci_cd.md`
- `docs/development/intermediate/testing/end_to_end_testing.md`
- `docs/development/intermediate/testing/index.md`
- `gcs/docker/gcs-base-docker-compose.yaml`
- `mkdocs.yml`
- `robot/docker/Dockerfile.robot`
- `robot/docker/docker-compose.yaml`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/setup.cfg`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/test/conftest.py`
- `simulation/isaac-sim/docker/Dockerfile.isaac-ros`
- `simulation/isaac-sim/docker/docker-compose.yaml`
- `simulation/ms-airsim/docker/docker-compose.yaml`
- `tests/README.md`
- `tests/colcon_unit_test_packages.yaml`
- `tests/conftest.py`
- `tests/harness/__init__.py`
- `tests/harness/commands.py`
- `tests/harness/discovery.py`
- … 1 more

## 19402c2223 — 2026-08-15 — John Liu — KEYWORD break,error,fix

**OptiTrack (2/3): NatNet server emulator + host integration tests (#375)**

https://github.com/castacks/AirStack/commit/19402c2223812c9f454c1ffe0dd2d854b48d3e93

```
* feat(sim): add NatNet server emulator (protocol core) + register unit tests

The pure-Python NatNet server that emulates an OptiTrack Motive server so
natnet_ros2 can be driven without hardware. USD/Isaac-free — this is the protocol
+ server core (unicast server, data/model/server types, serializers, default
catalogs). The Isaac wrapper that maps a USD scene onto this server lands next.

Registers the emulator package's co-located unit tests via a `sim:` entry in
tests/colcon_unit_test_packages.yaml (base's simulation/**/<pkg>/test glob). The
root conftest now puts each unit-test package's import root on sys.path so
co-located tests import their package without a per-package conftest.py.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* test(natnet): host integration tests — emulator server → natnet_ros2

Drive the real natnet_ros2 client from the host NatNet server emulator and check
the drone pose reaches ROS at rate (single-body and multi-body profiles). No sim,
no GPU — uses the base's `robot_autonomy_stack` fixture + `integration` mark. The
Isaac-wrapper variant lands with the Isaac wrapper PR.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.15

* pack frame sections through one helper

* fix the labeled-marker struct format that raised on every pack

sMarker.pack used '<i5fhf', which expects five floats between ID and
params but is only given four (x, y, z, size), so every call raised
struct.error. Nothing hit it because the emulator streams rigid bodies
only, leaving nLabeledMarkers at 0 and the section never packed.

* ignore the editable-install egg-info in the emulator extension

* comment trim

* put helper-module dirs on sys.path for co-located unit tests

The emulator's tests import natnet_test_helpers from their own test/ dir,
which only resolved because test_natnet_integration.py inserts that path at
import time and pytest imports it during collection. Narrowing the run
(-m unit --ignore=integration) dropped that side effect and broke collection.

The dir is added only when it ships no conftest.py, so lidar_point_cloud_filter
keeps its parent-only path — putting its test/ dir on sys.path would shadow this
conftest as module `conftest` and break every `from conftest import`.

---------

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
```

+2653 / −1 in 25 files:

- `.env`
- `CHANGELOG.md`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/.gitignore`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/README.md`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/__init__.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/__init__.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/__init__.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/defaults.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/server/__init__.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/server/natnet_common.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/server/natnet_data_types.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/server/natnet_model_types.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/server/natnet_server.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/server/natnet_server_types.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/server/natnet_unicast_server.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/setup.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/natnet_test_helpers.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_defaults.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_serializers.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_server_catalog.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_unicast_protocol.py`
- `tests/colcon_unit_test_packages.yaml`
- `tests/conftest.py`
- `tests/integration/natnet/README.md`
- `tests/integration/natnet/test_natnet_integration.py`

## 0ae96fe8c4 — 2026-08-15 — John Liu — KEYWORD broken,error,fail,failed,fails,failure,fix,fixed,fixes,wrong

**OptiTrack (1/3): robot-side NatNet client + PX4 external-vision fusion (#374)**

https://github.com/castacks/AirStack/commit/0ae96fe8c40474dfb7b107be7a2a736d50976b11

```
* feat(perception): bring natnet_ros2 client up to the optitrack_emulation baseline

Take the natnet_ros2 package from #367 onto the reworked base: the C++ NatNet
client (natnet_ros2_node + client adapter + natnet_logic seam), the base
mavros_gp_origin and vision_pose_converter nodes, per-robot natnet_config profiles,
launch files, and the co-located C++/Python unit tests. natnet_ros2 is already
listed in tests/colcon_unit_test_packages.yaml, so the base's YAML-driven collection
picks up the updated unit tests directly — no proxy files.

Real-robot PX4 external-vision fusion (px4_param_setter, geoid-corrected origin,
EV-pose bounds) is layered on next.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* feat(natnet): real-robot PX4 external-vision fusion (mocap → EKF2)

Layer the Hummingbird real-robot fusion pipeline onto natnet_ros2 so an
OptiTrack-only drone (no GNSS/mag/baro) fuses mocap pose into PX4 EKF2:
- mavros_gp_origin_node: publishes a guarded synthetic GPS origin. On real HW,
  use_geoid_altitude feeds the egm96-5 geoid undulation (N ≈ 54 m at Lisbon) so
  mavros's ellipsoidal→AMSL conversion cancels and local z == OptiTrack z (fixes
  the ~36 m = 90 − 54 boot offset; see docs). Auto-skipped in sim.
- vision_pose_converter_node: rate-limited mocap → MAVROS vision_pose bridge.
- px4_params.yaml: the external-vision EKF2 param set.
- natnet_ros2.launch.py wires the bridges when a robot's vision_pose block is on.

px4_param_setter reworked into a **checker** (R3): auto_set=false by default — it
reads and *flags* FCU params that differ from the desired set instead of writing
them; on_mismatch=warn|halt (default warn). Set the params in QGroundControl; the
node is the pre-flight safety net. auto_set=true restores the legacy enforce path.

Excludes the duplicate vendored NatNet SDK (sensors/natnet_ros2) and deployment
override .envs.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* docs(natnet): PX4 external-vision setup guide + height-datum explainer

Move the PX4 external-vision setup guide into docs/ (was a repo-root markdown) and
wire it into the mkdocs nav under Perception. Adapt it to the reworked param
checker (auto_set default off; check-and-flag, not enforce), and add a "height
datum" section explaining the ~36 m local_z offset: AirStack's 90.0 ellipsoidal
world datum minus the egm96-5 geoid undulation (N ≈ 54 m at Lisbon) = 36 m; fixed by
publishing the geoid-corrected origin altitude so mavros's conversion cancels.
Documents why it's invisible in sim and why the shared 90.0 datum must not be
changed globally.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* feat(perception): point natnet launch include at the natnet_config schema

Refine the perception bringup comment on the LAUNCH_NATNET include so it points at
the per-robot natnet_config.yaml schema parsed by natnet_ros2.launch.py.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.14

* fix(natnet): make the NatNet client actually reachable + correct EV tuning

Three defects that together meant the OptiTrack client could never connect to
anything, in sim or on a real robot.

1. NATNET_SERVER_IP was unreachable config. natnet_config.yaml resolves it via
   $(env ...), but docker compose only injects variables named in a service's
   `environment:` block and no service declared it — not the compose files, not
   .env, not tests/system/test_optitrack_e2e.py. The client therefore always fell
   back to its hardcoded default (192.168.123.199), which is neither the in-sim
   emulator (172.31.0.200) nor any Motive host. Forwarded in
   robot-base-docker-compose.yaml, defaulting to the emulator so the sim path
   works unconfigured.

2. The tracked rigid body could never match. robot_1 pinned "Hummingbird" id
   1146 while the emulator streams "Drone" id 1, and the NatNet client filters
   incoming frames by NUMERIC id — a mismatch yields a connected client that
   silently never publishes. Body name/id now accept $(env ...) (expanded in
   _build_node_params, with the id still coerced to int) and default to the
   emulator's body; sites override via NATNET_BODY_NAME / NATNET_BODY_ID.

3. EV tuning was not the deployment-validated set. EKF2_EV_DELAY 8.0 -> 7.0 and
   EKF2_EVP_NOISE 0.01 -> 0.05. EKF2_EVP_NOISE is not marker precision: it also
   sets the innovation gate at EKF2_EVP_GATE (default 5) sigma, so 0.01 gave a
   5 cm gate that rejected legitimate mocap updates and refused to arm. 0.05 is
   a 25 cm gate, still far tighter than PX4's 0.1 default.

px4_params.yaml keeps the evidence inline, including two results that are
expensive to rediscover: raising EKF2_EV_DELAY to 50.0 measurably degrades
tracking (the negative best-fit time shift shows the estimate running ahead of
truth), and the drift-and-snap excursions were a 90 deg body-yaw offset in the
Motive rigid-body definition, not a gate problem — so the fix belongs in Motive,
never as yaw compensation in code.

Adds two unit tests covering body-field env expansion and the emulator-matching
defaults (natnet_ros2: 14 -> 16 passing).

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* add a real-robot OptiTrack deployment override

Mocap counterpart to l4t-px4-realrobot.env: same Jetson stack, plus the NatNet
server/body settings and LAUNCH_NATNET.

Carries the two things that are easy to get wrong and produce no error. The body id
must match Motive's streaming id, since the client filters frames numerically and a
mismatch just never publishes. And nothing writes the EKF2 external-vision parameters
to a real FCU — px4_param_setter only reads them back and warns — so they have to be
set once in QGroundControl.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* config bodies per robot profile; trim comments to the docs

The rigid body a robot tracks is now set only in its natnet_config.yaml profile,
keyed by ROBOT_NAME. NATNET_BODY_NAME / NATNET_BODY_ID are gone: a single global
env var cannot express per-robot values, so it blocked the multi-robot case the
profiles already handle. NATNET_SERVER_IP stays in the environment — one Motive
host serves every robot.

Comments across the package are cut back to what is not evident from the code.
The EKF2 tuning results that were buried in px4_params.yaml move into
docs/robot/px4_external_vision.md, which also had stale values (EV_DELAY 15.0,
EVP_NOISE 0.01) contradicting the config: that raising EV_DELAY measurably hurts
tracking, and that drift-and-snap was a Motive rigid-body yaw offset rather than
a gate problem.

Kept: the license header, and the note on why the SDK needs a reachability
pre-check before Connect().

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* put the mocap floor at the shared world datum

desired_floor_amsl 0.0 -> 36.0, the world datum (90 m ellipsoidal) expressed in
AMSL, so a mocap robot's reported global altitude agrees with sim and the GCS
instead of sitting at sea level. The published ellipsoidal origin works out to
~90 m, the datum itself.

local_position.z equals the OptiTrack height for any value of this parameter — it
only moves the global altitude. Reasoning lives in the external-vision doc, which
also now records that GeoPoint.altitude is ellipsoidal by contract, so AMSL must
not be sent here.

Not yet confirmed on hardware.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fail the build when the geoid dataset is missing

MAVROS constructs the egm96-5 geoid in its UAS core, before any plugin loads, and
throws std::invalid_argument if the dataset is absent — mavros_node terminates at
startup, so there is no MAVROS at all, GPS or mocap.

The image could ship without it. mavros' install_geographiclib_datasets.sh sends the
downloader's output to /dev/null and, on failure, prints "Error while installing" and
returns without a non-zero exit, so the RUN layer succeeded regardless. The tool it
calls, geographiclib-get-geoids, was also only a transitive dependency of ros-mavros
rather than something we pinned.

Now pins geographiclib-tools and asserts the file landed, so a failed download fails
the build. Verified against the shipped image: with the downloader broken the script
still exits 0, and the new test -f returns non-zero.

This is the dependency the OptiTrack external-vision path needs — mavros_gp_origin
resolves the geoid undulation with the same egm96-5 model — hence landing it here.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* abbreviated Dockerfile comment on geographic lib installation

* fix repo-root doc links in the external-vision guide

They resolved relative to docs/robot/, so mkdocs looked for
docs/robot/robot/ros_ws/... and warned on every one. Prefixed with ../../;
the file now builds warning-free.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* comment trim

* point the companion-link section at the PX4 docs

Section 3 documented MAVLink serial setup at length — MAV_n_CONFIG / SER_TEL2_BAUD
tables, wiring, USB-vs-TELEM2 comparison — all of which is standard PX4 setup that
PX4 documents better and keeps current. Replaced with links to the companion
computer, MAVLink peripherals, and serial configuration pages.

Kept the part PX4 does not cover: the Cube Orange USB CDC-ACM stall, which starves
EKF2 of vision updates and is why the companion link belongs on TELEM2. Four other
sections and the troubleshooting table point here for that symptom.

65 lines -> 19.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* frame section 4 around mavros_gp_origin, demote the 36 m note

Section 4 now leads with what mavros_gp_origin does — inject a synthetic global
position so PX4 will arm in modes that need one without GNSS — rather than
presenting the height datum as a peer topic.

The ~36 m offset becomes a note under it, scoped to real deployments and ending
with why sim never sees it (the geoid path is skipped under use_sim_time, and
sim's synthetic GPS is self-consistent with the spawn). Section 4b is gone; it
had no inbound references.

Dropped the "don't change the 90.0 globally" warning.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* reject an unknown connection_type instead of defaulting to unicast

validate_connection_type returned "unicast" for anything it did not recognise, so
"mutlicast" or "Unicast" produced a client that connected on the wrong transport
and then never received a frame — with only a warning to show for it.

It now throws std::invalid_argument naming the offending value, and the node turns
that into a fatal startup error rather than a warning it flies past. Case-sensitivity
is deliberate: accepting "Unicast" would mean the config silently disagrees with
itself.

Tests updated from fallback to throw, plus one asserting the message names the bad
value. 60 gtests pass.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* px4 external vision docs trim

* trim natnet node comments; note the latency figure is an estimate

Comment trims in natnet_ros2_node.cpp (no code change).

Records what cube_orange_latency_ms actually is: an estimate of the FCU hop, added
to a logged total and never fused. Only the transport half of EKF2_EV_DELAY is
measured, and that measurement starts at the NatNet server transmit, so Motive's
own capture pipeline is not in it either.

Also notes, for whoever retunes next, that the node stamps poses with its receive
time — so delay after that stamp does not belong in EKF2_EV_DELAY, which points
lower than 7.0 and matches the negative best-fit shift already recorded. Not chased
down; 7.0 flies. CameraMidExposureTimestamp would replace the estimate with a
measurement if it ever matters.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* trim the external-vision tuning notes

Replaces the two long tuning write-ups with a short troubleshooting tip (check the
Motive rigid-body definition first — x forward, z up) and cuts the latency section
back to what is measured versus estimated.

Fixed a dangling "see below" in the EKF2_EV_DELAY table row, which pointed at the
removed tuning result; the warning it carried is now stated inline.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
```

+1956 / −333 in 28 files:

- `.env`
- `CHANGELOG.md`
- `docs/robot/px4_external_vision.md`
- `mkdocs.yml`
- `overrides/l4t-optitrack-realrobot.env`
- `robot/docker/Dockerfile.robot`
- `robot/docker/robot-base-docker-compose.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/CMakeLists.txt`
- `robot/ros_ws/src/perception/natnet_ros2/README.md`
- `robot/ros_ws/src/perception/natnet_ros2/config/mavros_gp_origin.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/config/natnet_config.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/config/px4_params.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/config/vision_pose_converter.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/natnet_client_adapter.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/natnet_logic.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/launch/mavros_gp_origin.launch.xml`
- `robot/ros_ws/src/perception/natnet_ros2/launch/natnet_ros2.launch.py`
- `robot/ros_ws/src/perception/natnet_ros2/launch/px4_param_setter.launch.xml`
- `robot/ros_ws/src/perception/natnet_ros2/launch/vision_pose_converter.launch.xml`
- `robot/ros_ws/src/perception/natnet_ros2/package.xml`
- `robot/ros_ws/src/perception/natnet_ros2/src/mavros_gp_origin_node.py`
- `robot/ros_ws/src/perception/natnet_ros2/src/natnet_client_adapter.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/natnet_ros2_node.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/px4_param_setter_node.py`
- `robot/ros_ws/src/perception/natnet_ros2/src/vision_pose_converter_node.py`
- `robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_logic.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_ros2.py`
- `robot/ros_ws/src/perception/perception_bringup/launch/perception.launch.xml`

## 1c41f8c029 — 2026-08-17 — John Liu — KEYWORD error,failed,fails,failure,fix,wrong

**OptiTrack (3/3): Isaac wrapper, mocap EV fusion in sim, and a Circle-trajectory e2e (#376)**

https://github.com/castacks/AirStack/commit/1c41f8c029a6b579fa3910e8c69c0dcc02e78c22

```
* feat(sim): Isaac wrapper for the NatNet emulator (USD scene → server)

The Isaac integration layer that maps a live USD scene onto the NatNet server:
catalog/config/frames/manager/scene_setup/ui_extension/usd_bindings, the extension
manifest (config/), and the USD schema. Adds the natnet Pegasus launch scripts that
spawn the emulator alongside PX4 in Isaac Sim, the isaac unit tests (incl. a
float-tolerance loosen on the pose round-trip for float32/USD noise), and the
Isaac-wrapper host integration test. scipy + usd-core added for the emulator's
USD/pose-sampling tests.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* test(natnet): dedicated OptiTrack sim e2e (optitrack mark)

One dedicated Isaac bring-up (example_one_px4_pegasus_natnet_launch_script +
LAUNCH_NATNET=true) that asserts the full NatNet chain: emulator → natnet_ros2
pose_cov >= 5 Hz, then PX4 local_position alive (EKF2 fusing the vision). Its own
`optitrack` mark + _MODULE_ORDER slot — deliberately NOT a third parametrized sim,
so the generic liveliness/sensors/flight suites aren't re-run under NatNet.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* docs(natnet): emulator sim doc + optitrack-development skill

Add the NatNet emulator Isaac Sim documentation (docs/simulation/isaac_sim/
natnet_emulator.md) and the optitrack-development agent skill covering the emulator,
natnet_ros2, and the NatNet wire-protocol handshake.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.16

* fix(sim): register the NatNet emulator via the Kit ext-folder

The Isaac launch scripts import `optitrack.natnet.emulator`, but Kit was only
pointed at the shared exts dir (`~/.local/share/ov/data/documents/Kit/shared/exts`),
where Dockerfile.isaac-ros installs pegasus.simulator at image build. The emulator
lives in the repo at simulation/isaac-sim/extensions/ and is never copied there, so
it was not a registered extension and the import depended on ambient sys.path.

Kit accepts repeated --ext-folder, so both standalone commands now pass the repo's
extensions dir as a second search root. Chosen over copying the extension into the
shared dir at build time because the repo tree is bind-mounted: emulator edits take
effect on relaunch instead of requiring an image rebuild.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* make the sim actually fuse the mocap stream

EKF2_EV_CTRL defaults to 0, and the isaac compose set no PX4 params at all, so PX4
discarded the vision entirely and flew on sim GPS. The emulator could stream
perfectly and change nothing.

PX4 SITL's rcS applies any PX4_PARAM_<NAME> env var at boot and Pegasus passes the
container env through, so no new mechanism is needed. Each entry defaults to PX4's
own default, read out of the firmware in this image — unset is an explicit no-op and
non-mocap sims are unaffected. They cannot be defined-but-empty: the rcS loop has no
empty-value guard.

Also hooks NATNET_BODY_ID in the single-drone launch script. The emulator hardcoded
streaming id 1 while the client reads the env var, so a real Motive id would desync
the two into a connected client that never publishes.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fly a circle on mocap fusion instead of asserting a topic exists

test_px4_fuses_vision claimed to prove EKF2 fused the external vision but only
waited for local_position/pose, which publishes off GPS regardless — it passed with
vision disabled.

The stack now comes up with GPS, baro and range aiding off, so mocap is the vehicle's
only position source, and the module flies the Circle trajectory. Sustained lateral
motion is where a wrong EV delay or a too-tight innovation gate shows up; a hover
would not reveal either. Cross-track error is scored by the same helpers the autonomy
benchmark uses, imported rather than reimplemented.

test_px4_fuses_vision is kept as the pre-flight gate — it now establishes only that an
estimate exists, and says so.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* enforce only the mocap circle flight on PR open

The pull_request branch passed no args, so opening a PR ran pytest's defaults: every
mark, both sims, all four trajectory types. Now it runs the one end-to-end flight that
covers the whole chain.

Every other suite is unchanged and still reachable on demand — /pytest comments,
workflow_dispatch inputs, and local airstack test.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* add an isaac natnet mocap override

Brings up the emulator plus PX4 on external-vision fusion in one command — the same
configuration test_optitrack_e2e.py uses, so the test environment is reproducible by
hand.

Sets PLAY_SIM_ON_START explicitly because the root .env ships it false: the scene then
loads paused, /clock never ticks, and every use_sim_time node sits frozen while the
stack looks healthy.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* install the natnet emulator as a real Kit extension

The natnet launch scripts died with ModuleNotFoundError: No module named 'optitrack'.
Pointing Kit's --ext-folder at the repo extensions dir was not enough — that only makes
Kit aware of an extension, it does not put the package on sys.path.

Handle it the same way pegasus.simulator already is: bake a copy into the Kit shared
exts dir and pip-install it editable, then bind-mount the repo copy over it so edits
stay live. The scripts now enable_extension() before importing, which registers the
extension and its omni.isaac.core / omni.usd dependencies.

The repo-extensions --ext-folder flag is dropped; the extension now lives in the dir the
image already searches.

Verified in a running container: extension starts, emulator serves on 172.31.0.200
:1510/:1511, and the robot sees /robot_1/perception/optitrack/drone/pose_cov at ~101 Hz
feeding vision_pose and PX4 local_position at ~32 Hz.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* set the streamed body in the script, not the environment

The emulator read NATNET_BODY_NAME / NATNET_BODY_ID from the environment to stay in
sync with the client. The client now takes its bodies from its per-robot profile in
natnet_config.yaml, so the env hook was asymmetric and, being global, could not
describe a multi-robot scene anyway.

Both are now constants in the launch scripts, with the pairing spelled out inline,
in the emulator sim doc, and in the optitrack-development skill — including that a
mismatched id fails silently: the client connects and never publishes.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* comment trim on isaac-sim docker compose

* point the isaac-sim env blocks at their documentation

* comment trim on editable installation of natnet emulator

* keep the full default test run on PR open

Narrowing the PR gate to `-m 'build_packages or optitrack'` also dropped the
unit tier — 155 tests, including the emulator's own suite, which colcon test
does not cover (it runs only the robot workspace packages).

The optitrack e2e needs no gate of its own: with no -m filter it is collected
like everything else, and it brings up its own mocap-EV stack via _E2E_ENV.
Only the heavy-mark classification stays, so /pytest -m optitrack still builds
sim images instead of taking the pull-only path.

* wait for a converged estimate before arming in the optitrack e2e

Gate on local_position/odom instead of /pose. odom goes live only once EKF2 has
converged and home is set, which is what PX4's arming preflight requires; /pose
fires earlier, and the takeoff dispatched in that window returned "failed to arm".
Both autonomy suites already gate on odom for this reason (test_px4_ready).

The gate alone is not sufficient under external vision: with GPS, baro and range
aiding off, PX4's heading and horizontal-position stability checks settle after
odom starts publishing — measured at ~26s past the gate. TakeoffTask does not
retry its own ARM, so retry here first.

* comment trim on optitrack e2e collection ordering

* comment trim on the PR-open test args

* rename the isaac natnet override to isaac-optitrack-simulation.env

* select PX4 SITL parameters with a named env_file

The isaac-sim service listed eleven PX4_PARAM_* entries, each defaulting to a
hardcoded copy of PX4's own default so that an unset value stayed a no-op — rcS
has no empty-value guard. Those copies can drift from firmware silently.

Replaced with env_file: ./px4-params/${PX4_PARAM_SET:-default}.env. default.env
is empty, so an unselected run injects nothing and PX4 keeps its firmware
defaults; external-vision.env holds the mocap set. An unknown name fails the
compose config rather than falling back.

Also corrects the natnet_emulator doc table, which described three robots, the
multi-drone script, and a SITL_PARAM_PROFILE variable that exists nowhere.

* comment trim in compose file

* trim verbose comments in the natnet sources

Shorten multi-line inline comments that explained rationale or compared the
chosen approach against alternatives. The longer explanations already live in
docs/simulation/isaac_sim/natnet_emulator.md, so the comments now state what
the code does and point there.

Limited to files this PR adds: the natnet launch scripts and the emulator's
isaac/ modules. The env files and the pre-existing launch script keep their
original comments.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* removed comment change

* drop the GPS origin change from the baseline pegasus launch script

example_one_px4_pegasus_launch_script.py is a pre-existing non-mocap script and
does not need to change for the NatNet emulator work, so restore it to develop.

The set_gps_origins call was also inert here: for a single drone spawned at the
world origin it computes (38.736832, -9.137977, 90.07), which is the Lisbon
default gps_utils already documents, and nothing in the Pegasus submodule reads
the PX4_HOME_LAT_<domain_id> vars it writes.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* assert the external-vision params actually reached the FCU

The rest of this module assumes PX4_PARAM_SET=external-vision took effect. If it
silently does not, EKF2_EV_CTRL stays 0 and EKF2_GPS_CTRL stays 7, the vehicle
flies the Circle on sim GPS, and every test still passes — the proof-by-elimination
in test_px4_fuses_vision collapses because the elimination never happened.

Read EKF2_EV_CTRL and EKF2_GPS_CTRL back off the FCU through the MAVROS param
plugin, so the check covers the whole chain: compose env_file -> container env ->
Pegasus -> PX4 rcS -> FCU. Runs before the flight tests so a param failure
short-circuits in seconds instead of after two 2400s timeouts.

Two params, not the full set: if these are right, PX4_PARAM_SET demonstrably
applied and the rest came with it. Matching is on the printed value line, not the
exit code — an unpulled param prints "Parameter not set." and still exits 0.

Verified against a live sim: passes on the real config, fails with distinct
messages for a wrong value and for a param that never appears.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* docs(natnet): publish the emulator page and correct the setup examples

Add the emulator doc to the nav as "MoCap Emulator" — it built and served but
was orphaned, so it was only reachable by knowing the URL, and the "See
docs/..." pointers in the code led somewhere unnavigable.

Fix the launch-script examples in the doc and the extension README. Both omitted
enable_extension(), which is the actual prerequisite: the package imports fine
because Dockerfile.isaac-ros pip-installs it, but the emulator's modules pull
omni.usd / omni.physx lazily, so Kit has to have the extension registered. The
doc also carried a sys.path.insert pointing at ../utils (where scene_prep lives)
that had nothing to do with the optitrack import. The README targeted
/World/drone1/base_link rather than the /body child the launch scripts stream.

Document that client registration does not survive a server restart: restart the
robot container after Stop/Start Server. Stopping and starting the simulation is
unaffected — frames are sampled on the physics step.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* feat(natnet): the extension owns the server, tied to the sim timeline

The Kit extension is the single owner of the NatNet server. It builds one from
the /World/NatNetInterface prim on Play and shuts it down on Stop, so the
server's lifetime matches the simulation and the panel reports its state rather
than controlling it.

Launch scripts author the interface prim before starting the timeline;
author_drone_natnet_interface writes the prim and returns the authored config.
Because the server is constructed on each Play, serverIp/ports/mode — bound into
the socket at construction — pick up whatever is authored at that point. Bodies,
up-axis and pose noise are re-read while running and need no rebuild.

The panel opens on the interface authored on the stage, so Save writes back what
is there; author_interface replaces the whole body set.

A client registers with the server instance it connects to, and natnet_ros2
handshakes only until its first success, so a client from an earlier run is
unknown to the server built by the next Play. Restart the robot container after
each Stop -> Play cycle; documented in natnet_emulator.md.

Not exercised against a live panel yet.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* docs trim

---------

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
```

+4689 / −11 in 41 files:

- `.agents/skills/optitrack-development/SKILL.md`
- `.env`
- `.github/workflows/system-tests.yml`
- `CHANGELOG.md`
- `docs/simulation/isaac_sim/natnet_emulator.md`
- `mkdocs.yml`
- `overrides/isaac-optitrack-simulation.env`
- `simulation/isaac-sim/docker/Dockerfile.isaac-ros`
- `simulation/isaac-sim/docker/docker-compose.yaml`
- `simulation/isaac-sim/docker/px4-params/default.env`
- `simulation/isaac-sim/docker/px4-params/external-vision.env`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/README.md`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/config/extension.toml`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/__init__.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/catalog.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/config.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/frames.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/manager.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/scene_setup.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/ui_extension.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/usd_bindings.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/schema/schema.usda`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_catalog.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_discovery.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_frames.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_interface_authoring.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_interface_config.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_pose_sampling.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_pose_streaming.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_scene_setup.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_server_from_config.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_server_lifecycle.py`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_target_resolution.py`
- `simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_natnet_launch_script.py`
- `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_natnet_launch_script.py`
- `tests/harness/collection.py`
- `tests/integration/natnet/README.md`
- `tests/integration/natnet/test_natnet_integration.py`
- `tests/pytest.ini`
- `tests/requirements.txt`
- … 1 more

## f4697265e4 — 2026-08-18 — pvkumara — KEYWORD bug,failure,fix,fixes,wrong

**CI/CD Tuning PR - pytest collection bug fix (#384)**

https://github.com/castacks/AirStack/commit/f4697265e40d65a39ba0127593adf172ee51f840

```
* docs(tests): align unit-test docs with the co-located layout

Unit test source moved into <package>/test/ and is collected from
colcon_unit_test_packages.yaml, but the surrounding documentation still described
the mirror-directory-and-proxy scheme that replaced. Six per-layer stubs under
tests/robot/ told authors to add tests in directories tests no longer live in, and
tests/sim/motive_emulator/README.md proposed a NatNet emulator that was built at
simulation/isaac-sim/extensions/optitrack.natnet.emulator/ instead. Remove them and
rewrite the two tree READMEs as signposts.

Correct the add-unit-tests and run-system-tests skills, which future agents read to
work in this area, on four points they had wrong:

- Running them. `pytest tests/` does not collect co-located unit tests — the
  injection in conftest.pytest_configure is skipped whenever a path is given on the
  command line. It reports "no tests collected" and exits 5, which reads as a
  failure but means nothing ran. `airstack test -m unit` and `cd tests && pytest -m
  unit` are the working forms; verified 155 passed vs exit 5.
- CI. No workflow runs unit tests. system-tests.yml invokes `pytest tests/`, and
  fires only on PR-open, /pytest, or workflow_dispatch.
- The mark. pytest_itemcollected applies @pytest.mark.unit by file location, so
  test sources should not declare it. The skill previously said "always decorate",
  which is where the redundant declarations came from.
- colcon. It runs only what a package's CMakeLists registers. natnet_ros2 has
  ament_add_gtest but no ament_add_pytest_test, so its Python tests run only under
  the root harness.

Also fixes a pytest_args example that would silently do nothing (`-m not linter`;
ament's pytest runner ignores -m via PYTEST_ADDOPTS, and the real value is []), and
the same stale layout claim in the testing docs and the emulator README.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* docs(tests): record how C++ and Python unit tests reach CI

C++ gtests run under colcon test, which CI executes inside the robot container via
the build_packages mark (test_build_packages.py::test_colcon_test_robot). Python unit
tests run under the root harness, which no workflow invokes.

Whether colcon test also picks up a package's Python tests depends on its build type:
lidar_point_cloud_filter is ament_python and exposes them via setup.cfg
(testpaths = test), so they run in both places; natnet_ros2 is ament_cmake and
registers only ament_add_gtest, so its Python tests run nowhere in CI.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fix(tests): collect co-located unit tests when the run is not narrowed

Unit-test source lives outside tests/, so pytest_configure appends it to the
collection args. That injection was gated on args_source != ARGS, which pytest sets
for any positional path — including `tests/`. The intent was that
`pytest tests/system/foo.py` should not drag in 155 unrelated tests, but the guard
could not tell narrowing from naming the whole suite, so CI's `pytest tests/`
collected 97 of 252 items and the Python unit tests ran nowhere.

Decide on the paths instead: a positional is broad when it names tests/ itself or an
ancestor, narrow otherwise. `pytest tests/` and `pytest .` inject; `pytest tests/system`,
a single file, and a node id do not. Node ids are split on `::` first, since only the
part before it addresses the filesystem.

`any` rather than `all` is deliberate — pytest_configure appends the co-located files
(narrow, absolute) to config.args, so `all` would flip the answer for anything
re-deriving it after that mutation. The decision is also stashed on config for the
contract test to read.

tests/meta/test_collection_contract.py pins the behaviour: a table over broad/narrow
invocations, a check that the command in system-tests.yml is classified broad (the
test that would have caught this), and a check that every discovered file produced
collected items. It lives under tests/ on purpose — co-located, it would stop being
collected at the same moment it stopped guarding anything.

Verified: `pytest tests/ -m unit` 0 -> 170 passed; `cd tests && pytest -m unit`
unchanged at 170; `pytest tests/system/test_liveliness.py` still collects 16.

Unit tests now run with every system-tests.yml invocation. That workflow's triggers
are unchanged and intentional — PR open, /pytest, workflow_dispatch — since the same
run drives the GPU system tests.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* docs(tests): explain why C++ and Python unit tests use different runners

The split was documented as a fact without its reason. A gtest is a binary compiled
against the package's headers and rclcpp, so it can only run where the ROS toolchain
is — colcon test inside the robot container, which build_packages reaches after
building with -DBUILD_TESTING=ON. Python unit tests stub ROS at the import boundary
and touch no ROS runtime, so they need neither a build nor a container, which is what
keeps the suite under a second.

State the invariant that follows: a Python test needing a live ROS node belongs in
tests/integration/ or tests/system/, not in a package test/ dir.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* test: run the collection contract tests with the fast tier

They are hermetic and they guard the collection of everything above them, so
running them after the GPU sim suites is backwards — a hung flight test would mean
they never execute. Rank them in _MODULE_ORDER right after the co-located unit
tests, ahead of system.test_build_docker.

Also drop the `from conftest import repo_path` in favour of harness.discovery,
which the module already imports from — one less thing between the test and the
function it needs.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fix(ci): make PR validation and metrics trustworthy

Run fast unit checks automatically, constrain host collection, and distinguish infrastructure failures from comparable simulation results.

---------

Co-authored-by: John <johnliuchs2022@gmail.com>
Co-authored-by: Claude Opus 5 <noreply@anthropic.com>
Co-authored-by: Pranav Kumara <pkumara@andrew.cmu.edu>
```

+1377 / −303 in 34 files:

- `.agents/skills/add-unit-tests/SKILL.md`
- `.agents/skills/bump-version-and-release/SKILL.md`
- `.agents/skills/run-system-tests/SKILL.md`
- `.env`
- `.github/workflows/system-tests.yml`
- `.github/workflows/unit-tests.yml`
- `AGENTS.md`
- `CHANGELOG.md`
- `docs/development/intermediate/testing/ci_cd.md`
- `docs/development/intermediate/testing/index.md`
- `docs/development/intermediate/testing/unit_testing.md`
- `osmo/README.md`
- `simulation/isaac-sim/extensions/optitrack.natnet.emulator/README.md`
- `tests/README.md`
- `tests/conftest.py`
- `tests/harness/__init__.py`
- `tests/harness/collection.py`
- `tests/harness/discovery.py`
- `tests/harness/run_meta.py`
- `tests/harness/test_ids.py`
- `tests/integration/natnet/README.md`
- `tests/meta/test_collection_contract.py`
- `tests/meta/test_metrics_reporting_contract.py`
- `tests/parse_metrics.py`
- `tests/robot/README.md`
- `tests/robot/behavior/README.md`
- `tests/robot/global/README.md`
- `tests/robot/interface/README.md`
- `tests/robot/local/README.md`
- `tests/robot/perception/README.md`
- `tests/robot/sensors/README.md`
- `tests/run_summary.py`
- `tests/sim/README.md`
- `tests/sim/motive_emulator/README.md`

## 3852ae2d31 — 2026-08-20 — Andrew Jong — KEYWORD failed,failure

**P7(rfc-379): marketplace catalog, module-docs fetch, new-developer walkthrough**

https://github.com/castacks/AirStack/commit/3852ae2d316616c65ae859fdc86e15eae0246729

```
- tools/gen_docs_catalog.py: deterministic docs/modules/ generator from the
  airstack-modules-index registry (catalog table, per-module pages with
  pinned install snippets, DECLARED-vs-VERIFIED compat notes); --check drift
  mode; committed pages regenerate from a hermetic registry fixture
- docs deploy workflows: shallow-clone registry + registered module repos at
  their registered refs before mike deploy, with RFC §9 failure isolation
  (unreachable repo = skip + stub, never a failed deploy); weekly cron +
  dispatch on develop; triggers extended with stacks/**
- mkdocs: top-level Modules tab (catalog + module pages + the five reference
  stacks with rendered wiring.md); fetched checkouts excluded from the site
  (strict-build warnings 112 → 59, zero new)
- docs/getting_started/modular_airstack.md: the end-to-end new-developer
  walkthrough (stack up → wiring map → module add → own stack → fleet →
  doctor); AGENTS.md updated (RFC subsection, workflow table, CLI)
- 14 new contract tests; unit suite 252 passed

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+1332 / −8 in 23 files:

- `.github/workflows/deploy_docs_from_develop.yaml`
- `.github/workflows/deploy_docs_from_main.yaml`
- `.github/workflows/deploy_docs_from_release.yaml`
- `AGENTS.md`
- `docs/getting_started/index.md`
- `docs/getting_started/modular_airstack.md`
- `docs/modules/dfm2_disturbances.md`
- `docs/modules/index.md`
- `docs/modules/macvo.md`
- `docs/modules/optitrack.md`
- `mkdocs.yml`
- `stacks/full_default/README.md`
- `stacks/full_droan_cpu/README.md`
- `stacks/full_macvo/README.md`
- `stacks/lite_default/README.md`
- `tests/meta/fixtures/modules_index/modules/dfm2_disturbances.yaml`
- `tests/meta/fixtures/modules_index/modules/macvo.yaml`
- `tests/meta/fixtures/modules_index/modules/optitrack.yaml`
- `tests/meta/fixtures/modules_index/stacks/full_default.yaml`
- `tests/meta/fixtures/modules_index/stacks/full_droan_cpu.yaml`
- `tests/meta/fixtures/modules_index/stacks/full_macvo.yaml`
- `tests/meta/test_docs_catalog_contract.py`
- `tools/gen_docs_catalog.py`

## b0541c15a6 — 2026-08-20 — Andrew Jong — KEYWORD fail

**P2(rfc-379): sync self-heals stale partial module checkouts**

https://github.com/castacks/AirStack/commit/b0541c15a67b5330d124b24eb4daf4d59ee00750

```
A managed modules/<name> dir without .git (interrupted clone or partial
remove) made vcs import fail with 'destination path already exists'; sync now
clears such dirs before importing.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+18 / −0 in 1 files:

- `.airstack/modules/module.sh`

## b408ee7193 — 2026-08-20 — Andrew Jong — KEYWORD failed,fix

**fix(tests): wait for the state-estimate watchdog before commanding takeoff**

https://github.com/castacks/AirStack/commit/b408ee7193b465b0793bdc972d929c2b72a9a765

```
px4_ready proves EKF/MAVROS signals, but sending takeoff the instant it
passes races the drone_safety_monitor's watchdog after slow sim loads —
observed as 'Goal was rejected' and 'failed to arm' (three distinct race
signatures on fleet runs; manual takeoff on the settled stack accepted fine).
Wait up to 45s for state_estimate_timed_out=false; proceed with a warning if
unconfirmed.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+26 / −0 in 1 files:

- `tests/system/test_takeoff_hover_land.py`

## 9cc7b3383a — 2026-08-20 — Andrew Jong — KEYWORD fixes

**P5-E3b(rfc-379): split stack + bridge.yaml, doctor, interface conventions spec, stack CLI**

https://github.com/castacks/AirStack/commit/9cc7b3383ac544417f7d24b5939bcaf061a93a8e

```
- stacks/lite_default (onboard-lite) + stacks/lite_offload_global — the first
  SPLIT stack: onboard/offboard entry files + bridge.yaml explicitly listing
  every boundary crossing (seeded from the legacy dds_router allowlist;
  set_trajectory_mode deliberately NOT bridged — the legacy config bridged it,
  violating what is now doctor hard gate #2)
- tools/gen_dds_router.py: bridge.yaml → deterministic DDS-router config;
  --check enforces hard gate #2 (control/trajectory topics in a bridge)
- airstack doctor: compose-time checks (manifests, overlay, dep-conflict gate,
  stack anatomy, bridge gate) — observe-and-report except the two RFC-
  enumerated hard gates; --live diffs the running graph vs wiring.md + flags
  unblessed control-setpoint publishers; --snapshot writes hardware-observed
  wiring.md with unverified-in-CI provenance
- airstack stack list|new|diff (diff compares generated wiring, not XML)
- docs/robot/autonomy/interface_conventions.md v1.0.0: the versioned narrow-
  waist spec (names/types/QoS/frames/rate/placement per interchange point,
  verified against observed wiring; fixes stale PointStamped claims)
- 45+ new unit tests; integrate-module-into-layer rewritten for stacks;
  create-stack skill; .agents README index rebuilt (was 8 rows, now 22)

Split-stack live two-host bring-up deferred to P6 (needs hosts: placement).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+3342 / −423 in 27 files:

- `.agents/README.md`
- `.agents/skills/create-stack/SKILL.md`
- `.agents/skills/integrate-module-into-layer/SKILL.md`
- `.airstack/modules/doctor.sh`
- `.airstack/modules/stack.sh`
- `airstack.sh`
- `docs/development/stacks.md`
- `docs/robot/autonomy/integration_checklist.md`
- `docs/robot/autonomy/interface_conventions.md`
- `mkdocs.yml`
- `stacks/lite_default/README.md`
- `stacks/lite_default/docker-compose.yaml`
- `stacks/lite_default/launch/stack.launch.xml`
- `stacks/lite_default/modules.repos`
- `stacks/lite_offload_global/README.md`
- `stacks/lite_offload_global/bridge.yaml`
- `stacks/lite_offload_global/docker-compose.yaml`
- `stacks/lite_offload_global/launch/offboard.launch.xml`
- `stacks/lite_offload_global/launch/onboard.launch.xml`
- `stacks/lite_offload_global/modules.repos`
- `tests/meta/test_bridge_contract.py`
- `tests/meta/test_doctor_contract.py`
- `tests/meta/test_stack_layout_contract.py`
- `tools/doctor/__init__.py`
- `tools/doctor/checks.py`
- `tools/gen_dds_router.py`
- `tools/stack_diff.py`

## 5efffa3d1b — 2026-08-20 — Andrew Jong — KEYWORD broken

**P5-E1(rfc-379): observed wiring.md for all three reference stacks**

https://github.com/castacks/AirStack/commit/5efffa3d1b9c867212835d4d509e6782ded3c9dd

```
Captured from live isaacsim bring-ups @ d02a3467:
- full_default: 83 nodes / 405 edges — diff vs the legacy AUTONOMY_ROLE=full
  golden is IDENTICAL (machine-proven wrap-form equivalence)
- full_droan_cpu: 84 nodes / 419 edges (CPU droan + live disparity_expansion)
- full_macvo: 84 nodes / 407 edges — first working MACVO topology: droan's
  disparity_expander subscribes /perception/macvo/disparity (the wiring the
  broken legacy variant never achieved)

These are the E2 flatten gates: the refactor must reproduce these graphs.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+23160 / −0 in 3 files:

- `stacks/full_default/wiring.md`
- `stacks/full_droan_cpu/wiring.md`
- `stacks/full_macvo/wiring.md`

## 4a76935c04 — 2026-08-20 — Andrew Jong — KEYWORD break,fails

**P4(rfc-379): docker module-layer composition, layer plan, modules.lock**

https://github.com/castacks/AirStack/commit/4a76935c04b5c91427265114c8981d6a9030d2f2

```
- tools/compose_module_layers.py: three dep tiers per RFC #379 §6 — tier-1
  rosdep/apt/pip (one RUN per module per package manager → per-module layer
  cache; pip uses --no-cache-dir --break-system-packages per trunk's PEP 668
  house pattern), tier-2 Dockerfile.module chained via ARG BASE_IMAGE, tier-3
  prebuilt overlay (used as-is only when sole docker-relevant module on the
  host; fragment is source of truth otherwise). Deterministic plan
  (.airstack/generated/layer_plan.json), gitignored modules.lock (dep hashes +
  plan hash, byte-identical for identical inputs), --check-conflicts static
  apt/pip pin-conflict gate (doctor hard gate #1 — sync fails on conflict),
  --build executes the chain and points robot services at the composed tag
- ZERO-MODULE IDENTITY RULE: no docker-relevant modules ⇒ every host keeps
  today's exact image tag; no image: overrides emitted. Published trunk images
  are untouched (composed -m<hash> tags are per-checkout artifacts), so
  docker_image_plan.py needs no change
- validated end-to-end locally: heavy_module fixture built a real 2-step chain
  on v0.19.0-alpha.18_robot-x86-64_dev (apt cowsay + pip tabulate + tier-2
  marker verified in-container); identity restored on removal
- 18 contract tests; full unit suite 270 passed

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+1236 / −2 in 8 files:

- `.airstack/modules/module.sh`
- `.gitignore`
- `docs/development/modules.md`
- `tests/fixtures/modules/heavy_module/Dockerfile.module`
- `tests/fixtures/modules/heavy_module/module.yaml`
- `tests/meta/test_docker_layer_plan_contract.py`
- `tests/meta/test_module_overlay_contract.py`
- `tools/compose_module_layers.py`

## 6f436d8798 — 2026-08-20 — Andrew Jong — KEYWORD bug,fixes

**P5-E3a(rfc-379): flatten perception/sensors/global/behavior into stacks; prefix generic launch args**

https://github.com/castacks/AirStack/commit/6f436d879842abb28e1ca6faa3f6af2bc87abdec

```
- canonical module launch files (prefixed described args, set_remap in
  namespace groups, params via yaml): stereo_image_proc, topic_keepalive,
  lidar_point_cloud_filter, random_walk_planner (also fixes the old orphan's
  name= bug), drone_safety_monitor; stack entries wire every layer flat
  except interface (stays wrapped BY DESIGN until the RFC #380 platform-module
  refactor — it is the safety boundary) and logging (already self-contained)
- generic launch-config names killed (the global-collision class that bit
  asm_optitrack): interpolate_dds_router config_file/args →
  dds_router_config_file/dds_router_args, gossip publish_rate →
  gossip_publish_rate, domain_bridge likewise — deprecated aliases kept and
  execution-tested; all trunk callers updated
- allowlist 16→14 (deleted two dead orphan launches); add-ros2-package
  template no longer teaches remap-in-module patterns
- frozen legacy files untouched except graph-neutral arg-name call sites

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+644 / −230 in 26 files:

- `.agents/skills/add-ros2-package/SKILL.md`
- `.agents/skills/add-ros2-package/assets/package_template/README.md`
- `.agents/skills/add-ros2-package/assets/package_template/launch/template.launch.xml`
- `.agents/skills/update-documentation/SKILL.md`
- `common/ros_packages/coordination/coordination_bringup/launch/gossip.launch.xml`
- `docs/robot/autonomy/coordination/index.md`
- `docs/robot/autonomy/dds_router.md`
- `robot/ros_ws/src/autonomy_bringup/launch/interpolate_dds_router.launch.py`
- `robot/ros_ws/src/autonomy_bringup/launch/interpolate_domain_bridge.launch.py`
- `robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml`
- `robot/ros_ws/src/behavior/drone_safety_monitor/CMakeLists.txt`
- `robot/ros_ws/src/behavior/drone_safety_monitor/launch/drone_safety_monitor.launch.xml`
- `robot/ros_ws/src/global/planners/random_walk/launch/random_walk_launch.xml`
- `robot/ros_ws/src/global/planners/random_walk/launch/random_walk_planner.launch.xml`
- `robot/ros_ws/src/local/world_models/disparity_expansion/launch/disparity_pcd.launch.xml`
- `robot/ros_ws/src/perception/perception_bringup/launch/stereo_image_proc.launch.xml`
- `robot/ros_ws/src/perception/perception_bringup/launch/topic_keepalive.launch.xml`
- `robot/ros_ws/src/perception/perception_bringup/package.xml`
- `robot/ros_ws/src/sensors/lidar_point_cloud_filter/launch/lidar_point_cloud_filter.launch.xml`
- `stacks/full_default/README.md`
- `stacks/full_default/launch/stack.launch.xml`
- `stacks/full_droan_cpu/README.md`
- `stacks/full_droan_cpu/launch/stack.launch.xml`
- `stacks/full_macvo/README.md`
- `stacks/full_macvo/launch/stack.launch.xml`
- `tests/meta/launch_lint_allowlist.txt`

## 76af860239 — 2026-08-20 — Andrew Jong — KEYWORD fails

**P2(rfc-379): warn on stale colcon caches for extracted module packages**

https://github.com/castacks/AirStack/commit/76af860239fb895a0601b1524fe78eba39bd0021

```
A package moved from trunk into a module keeps its old build/<pkg> CMake
cache in bind-mounted checkouts; colcon then fails with 'source does not
match cache'. Sync now detects and prints the exact (root-owned-safe)
cleanup command. Observe-and-report, never deletes.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+40 / −0 in 1 files:

- `tools/module_overlay.py`

## 19c4adc4e2 — 2026-08-20 — Andrew Jong — KEYWORD broken,fixes

**P5-E1(rfc-379): reference stack folders (wrap form), --stack dispatch, single-locus lint**

https://github.com/castacks/AirStack/commit/19c4adc4e289686716c58533e1d208bc30ad1ab2

```
- stacks/{full_default,full_droan_cpu,full_macvo}: self-contained stack
  folders (modules.repos w/ airstack_compat, flat wrap-form stack.launch.xml,
  compose stub, README). full_macvo FIXES the broken
  local_macvo_obstacle_avoidance.launch.xml wiring in the stack file
  (correct local_-prefixed args, real perception/macvo/disparity topic,
  launch_macvo actually enabled) — collapses the local_* variant explosion
  into named stacks differing by include lines
- robot.launch.xml: stack_dir/stack_entry env-default args; stack group
  replaces role groups when AIRSTACK_STACK_DIR set; legacy role path
  byte-preserved. Compose: stacks/ volume + env passthrough in robot-base
- airstack up --stack <name>[:<entry>]: host-side validation, container-path
  exports, effective-config lines; AUTONOMY_ROLE deprecation warning
- single-locus launch lint (unit mark): no remaps outside stacks/*/launch/,
  frozen shrink-only allowlist (19 grandfathered files), description=
  required on stack args. write-launch-file skill rewritten same commit
  (canonical-default args, never remap in module files)
- wiring mark is stack-aware: --stack makes the golden stacks/<name>/wiring.md
- docs/development/stacks.md + nav; 8 new launch-intent contract tests

Unit suite: 294 passed.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+1170 / −299 in 24 files:

- `.agents/skills/write-launch-file/SKILL.md`
- `airstack.sh`
- `docs/development/stacks.md`
- `mkdocs.yml`
- `robot/docker/robot-base-docker-compose.yaml`
- `robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml`
- `stacks/full_default/README.md`
- `stacks/full_default/docker-compose.yaml`
- `stacks/full_default/launch/stack.launch.xml`
- `stacks/full_default/modules.repos`
- `stacks/full_droan_cpu/README.md`
- `stacks/full_droan_cpu/docker-compose.yaml`
- `stacks/full_droan_cpu/launch/stack.launch.xml`
- `stacks/full_droan_cpu/modules.repos`
- `stacks/full_macvo/README.md`
- `stacks/full_macvo/docker-compose.yaml`
- `stacks/full_macvo/launch/stack.launch.xml`
- `stacks/full_macvo/modules.repos`
- `tests/conftest.py`
- `tests/meta/launch_lint_allowlist.txt`
- `tests/meta/test_launch_intent_contract.py`
- `tests/meta/test_launch_single_locus.py`
- `tests/meta/test_stack_layout_contract.py`
- `tests/system/test_wiring_snapshot.py`

## a9f56b0a10 — 2026-08-20 — Andrew Jong — KEYWORD fix

**P6(rfc-380): doctor --live handles heterogeneous fleets**

https://github.com/castacks/AirStack/commit/a9f56b0a10d002ece6490fd3249ac54dc1ac9af1

```
- discover fleet-generated robot services (same pattern fix as ready.sh);
  ground-host tenants excluded
- --live --stack <name> captures only robots whose AIRSTACK_STACK_DIR names
  that stack — one wiring.md describes one stack, not the fleet union
  (validated live against the running sim_three_mixed fleet: full_default
  diff shrank from robot_3's whole graph to a single honest environmental
  finding — the GCS action_relay client count differs between 1-robot and
  3-robot deployments; host-grouped wiring per RFC #380 is the refinement)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+39 / −9 in 1 files:

- `tools/doctor/checks.py`

## efb7c86860 — 2026-08-20 — Andrew Jong — KEYWORD failed,fix

**P6(rfc-380): fix fleet-service discovery in ready.sh; clamp harness campaigns to fleet size**

https://github.com/castacks/AirStack/commit/efb7c868607089c4168cf6463be3e0976216489e

```
- ready.sh matched only compose replicas (-robot-); fleet-generated services
  (airstack-robot_N-1) were invisible, so heterogeneous bring-ups timed out at
  'robot containers running' with all containers actually up. Ground-host
  tenants (gcs-robot_N) are excluded — they never run MAVROS/PX4
- pytest_generate_tests now implements what the --fleet docstring promised:
  campaigns run at exactly the fleet's robot count instead of the
  --num-robots matrix (a 3-robot campaign against a 1-robot fleet failed on
  'no odometry' for robots the fleet never declared)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+16 / −2 in 2 files:

- `.airstack/modules/ready.sh`
- `tests/conftest.py`

## b951fd30ab — 2026-08-20 — Andrew Jong — KEYWORD fix,fixes

**M3(rfc-379): remove MACVO from trunk — extracted to castacks/asm_macvo**

https://github.com/castacks/AirStack/commit/b951fd30abb191ec6a228d4357d10580bf2b204b

```
The dogfood case for Docker dep tiers: macvo_ros2 (+ the 116MB MAC-VO
submodule) moves to the module, and Dockerfile.robot sheds the SKIP_MACVO
payload — TensorRT apt blocks, torch/torchvision/onnx/tensorrt pips, the 14
MACVO-only pips (audited: only tabulate has another consumer, supplied by the
tests venv, never the image), ~220MB model weights, and the
huggingface/matplotlib fix block. numpy~=1.26 pin kept (own-risk change,
deferred). Behavior note: matplotlib now stays in published images (the fix
block previously uninstalled it).

- perception.launch.xml: launch_macvo gate removed; the module ships the
  canonical macvo.launch.xml (also fixes the node's hardcoded camera_info sub)
- stacks/full_macvo: includes the module launch under the perception
  namespace (graph-identical to the committed wiring.md); modules.repos pins
  asm_macvo; README documents 'airstack module add asm_macvo'
- topic_keepalive: macvo rows removed
- compose SKIP_MACVO/SKIP_TENSORRT args dropped; docs/skills re-pointed

Image-size measurement and wiring.md regeneration follow in the next commits.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+81 / −1371 in 35 files:

- `.agents/skills/docker-build-profiles/SKILL.md`
- `.devcontainer/robot/launch.json`
- `.gitmodules`
- `docs/development/intermediate/docker-build-profiles.md`
- `docs/development/stacks.md`
- `docs/robot/autonomy/integration_checklist.md`
- `docs/robot/autonomy/system_architecture.md`
- `docs/robot/docker/index.md`
- `mkdocs.yml`
- `robot/docker/Dockerfile.robot`
- `robot/docker/docker-compose.yaml`
- `robot/ros_ws/src/perception/macvo_ros2/.gitignore`
- `robot/ros_ws/src/perception/macvo_ros2/README.md`
- `robot/ros_ws/src/perception/macvo_ros2/config/MACVO_fast_for_orin.yaml`
- `robot/ros_ws/src/perception/macvo_ros2/config/interface_config.yaml`
- `robot/ros_ws/src/perception/macvo_ros2/config/rviz_macvo.rviz`
- `robot/ros_ws/src/perception/macvo_ros2/launch/macvo_ros2.launch.xml`
- `robot/ros_ws/src/perception/macvo_ros2/macvo_ros2/DispartyPublisher.py`
- `robot/ros_ws/src/perception/macvo_ros2/macvo_ros2/MessageFactory.py`
- `robot/ros_ws/src/perception/macvo_ros2/macvo_ros2/__init__.py`
- `robot/ros_ws/src/perception/macvo_ros2/macvo_ros2/config/zedcam_config.yaml`
- `robot/ros_ws/src/perception/macvo_ros2/macvo_ros2/config/zedcam_macvo.yaml`
- `robot/ros_ws/src/perception/macvo_ros2/macvo_ros2/macvo`
- `robot/ros_ws/src/perception/macvo_ros2/macvo_ros2/macvo_node.py`
- `robot/ros_ws/src/perception/macvo_ros2/macvo_ros2/model/README.md`
- `robot/ros_ws/src/perception/macvo_ros2/package.xml`
- `robot/ros_ws/src/perception/macvo_ros2/resource/macvo_ros2`
- `robot/ros_ws/src/perception/macvo_ros2/setup.cfg`
- `robot/ros_ws/src/perception/macvo_ros2/setup.py`
- `robot/ros_ws/src/perception/perception_bringup/launch/perception.launch.xml`
- `robot/ros_ws/src/perception/perception_bringup/scripts/topic_keepalive_node.py`
- `stacks/full_macvo/README.md`
- `stacks/full_macvo/docker-compose.yaml`
- `stacks/full_macvo/launch/stack.launch.xml`
- `stacks/full_macvo/modules.repos`

## aa578de902 — 2026-08-20 — Andrew Jong — KEYWORD fix

**fix(tests): widen landing phase-timeout margin — evidence-based**

https://github.com/castacks/AirStack/commit/aa578de902220f1ec43c2dd18ace5d6b06904318

```
2026-08-20 runs show v=0.5 landings completing at 45.1-45.2s against a 45s
send_goal cap (TARGET_ALTITUDE/v + 15 + 10): a coin flip producing spurious
landing timeouts on 3-robot campaigns (legacy AND stack identically) and the
optitrack e2e (v=1.0, 40s cap). The constant now covers the
velocity-independent overhead (touchdown detection, land-detector dwell,
disarm): max(45, alt/v + 35).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+9 / −2 in 1 files:

- `tests/system/test_takeoff_hover_land.py`

## b2364b96ca — 2026-08-20 — Andrew Jong — KEYWORD fixes

**P0(rfc-379): first observed wiring golden + snapshot robustness fixes**

https://github.com/castacks/AirStack/commit/b2364b96ca540f2af9f1cf376f2af7f7b781afb4

```
- exclude _CREATED_BY_BARE_DDS_APP_ phantom participants (bare-DDS apps: sim
  bridges, uXRCE agents) from the normalized graph
- source-sha provenance falls back to reading .git/HEAD directly (tests
  container has no git binary)
- bless tests/goldens/wiring/full_default.isaacsim.1robot.md, captured from a
  live isaacsim bring-up @ 5c6c4a41 (83 nodes) and verified drift-clean
  against a second independent bring-up

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+7666 / −4 in 3 files:

- `tests/goldens/wiring/full_default.isaacsim.1robot.md`
- `tests/system/test_wiring_snapshot.py`
- `tests/wiring_snapshot.py`

## 05db6c5f9e — 2026-08-20 — Andrew Jong — KEYWORD fix

**fix(cli): PID-suffix effective-config run dirs to avoid same-second collisions**

https://github.com/castacks/AirStack/commit/05db6c5f9e6176499601244f55fe36ddbe1f278c

```
Back-to-back 'airstack up --dry-run' calls within one second landed in the same
.airstack/runs/<ts>/ dir, making test_effective_config_dump_written flaky.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+2 / −1 in 1 files:

- `airstack.sh`

## 262f12679a — 2026-08-20 — Andrew Jong — KEYWORD error,fails,failure,fix,fixed,fixes,fixing,wrong

**Pre-RFC workflow cleanup: intent-based launch, readiness gates, launch-script dedup, truthful logs (#386)**

https://github.com/castacks/AirStack/commit/262f12679af7c67cb4df40c074e8d2133d79f226

```
* refactor(isaac): dedupe launch scripts into shared PegasusApp base

The six launch scripts were 80-90% copy-pasted boilerplate (extension
enabling, wait_for_stage, scene prep, spawn calls, run loop) that had
already drifted: livestream existed only in the *_one_* scripts (so the
isaac-sim-livestream service silently black-screened with multi scripts),
ISAAC_SIM_HEADLESS was honored only by the *_multi_* scripts, and
barebones_pegasus_launch.py (the documented template) crashed with a
NameError (os never imported).

pegasus_app.py now owns the skeleton once: create_simulation_app()
(livestream + headless env handling, uniform across all scripts),
extension enabling, world/env loading, scene prep, drone/sensor spawning
from config dicts, and the run loop. Scripts reduce to scenario
declarations plus hooks (pre_scene_prep/post_scene_prep/post_spawn).

Behavior preserved per script (spawn poses, prim/node names, sensor
offsets, NatNet bodies, GPS origins), with three deliberate fixes:
- ISAAC_SIM_HEADLESS and ISAAC_SIM_LIVESTREAM now work in every script
- barebones template runs again
- NATNET_BODY_NAME/NATNET_TARGET_NAME env overrides now work as the
  one-drone natnet script's docstring already claimed

example_multi_drone_scene_import keeps its historical ZED offset
[0.21, 0, 0.05] (drift vs the canonical [0.2, 0, -0.05] — now visible
and annotated instead of buried).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* feat(cli): intent flags on 'up', resolved-value preflight, and 'airstack ready'

airstack up learns intent flags that derive the coordinated env-var sets
users previously had to know by heart (they export leaf values only —
compose interpolation gives shell env precedence, so .env is untouched):

  --sim isaac|airsim   swap simulator profile + matching URDF
  --robots N           NUM_ROBOTS + auto-select one/multi Isaac script
                       (also natnet pair; warns on custom scripts)
  --headless           ISAAC_SIM_HEADLESS + MS_AIRSIM_HEADLESS + QT offscreen
  --play/--no-play     PLAY_SIM_ON_START
  --no-autolaunch      AUTOLAUNCH=false
  --wait               chain into 'airstack ready' after compose up
  --dry-run            print + validate the resolved config, start nothing

Every up prints the resolved launch config and dumps it to
.airstack/runs/<ts>/effective_config.env (gitignored; best-effort on
read-only checkouts).

Preflight now validates RESOLVED values (env > --env-file > .env),
fixing the historical guard bypass where 'up --env-file overrides/...'
was checked against .env only. New checks: NUM_ROBOTS>1 with the
single-drone Isaac script (previously a silent 3-containers-1-drone
failure) is a hard error; missing images are listed by name with an
image-pull hint before compose starts a multi-GB implicit build; missing
omni_pass.env / empty Pegasus submodule / docker<29 name-resolution are
surfaced on the host instead of dying invisibly inside tmux.
AIRSTACK_SKIP_PREFLIGHT=1 downgrades errors to warnings.

'airstack ready' (and 'up --wait') answers "can I press Takeoff yet?":
staged gates mirroring the system-test budgets — containers (120s) →
sim /clock (600s) → per-robot sentinel nodes (300s) → PX4 MAVROS
connected + local_position/odom streaming (300s, the EKF-converged
armable signal; connected alone fires ~25s early). --json for scripts;
per-gate failures name the container/tmux window to inspect.

tests/meta/test_launch_intent_contract.py pins the flag derivations,
guard behavior, and exit codes (runs under the unit mark).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* feat(docker): tee tmux pane output to container stdout

Every service runs its real workload inside tmux, so 'docker logs' /
'airstack logs' were empty by construction — colcon build failures,
Pegasus import errors, and scene downloads all landed in panes nobody
attaches to. tmux hooks in the shared .tmux.conf (mounted into robot,
gcs, isaac-sim, and ms-airsim containers) now pipe-pane every created
session/window/split to /proc/1/fd/1, making container logs truthful.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: fix launch-workflow drift against actual code behavior

Corrects statements the audit found wrong, and teaches the new flags:
- getting_started: sim comes up PAUSED by default (PLAY_SIM_ON_START=false
  in .env, docs claimed auto-play), operator UI is Foxglove not RViz
  (DEBUG_RVIZ=false by default), adds 'airstack ready' / --wait and
  --sim/--robots variants
- simulation index + isaac docker.md + key_concepts + docker_usage:
  ISAAC_SIM_SCENE does not exist — scene selection is
  ISAAC_SIM_SCRIPT_NAME (standalone) or ISAAC_SIM_GUI (USD path, non-
  standalone); defaults table now matches .env/compose (AUTOLAUNCH=true,
  PLAY_SIM_ON_START=false, ISAAC_SIM_USE_STANDALONE=true, 100 Hz physics)
- simulation index: NUM_ROBOTS=3 alone does NOT put 3 drones in Isaac —
  documents --robots (auto script switch) and the preflight guard
- docker_usage: the test service is robot-test, not autotest
- gcs user_interface: gcs service is not in the deploy profile (gcs-real is)
- ms-airsim: MAVROS connects on 14540+domain (24540+i is AirSim's own PX4
  channel), camera FOV default is 90 not 110, vehicles are robot_<i> not
  drone<i>
- AGENTS.md: airstack stop/build are not registered commands (down /
  image-build); documents the new up flags and ready
- .env: correct usage comment; PLAY_SIM_ON_START paused-by-default note

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* chore(release): bump VERSION to 0.19.0-alpha.18 and update CHANGELOG

Image inputs are unchanged (all edits are bind-mounted or host-side), so
docker-build should registry-retag rather than rebuild on merge.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs(sim): document PegasusApp launch-script authoring; re-teach stale skills

spawning_drones.md now documents the pegasus_app.PegasusApp base class as
the way to write a launch script: import-order contract, constructor
kwargs, the drone-config dict (incl. prim/node_name/sensor overrides),
hooks (pre_scene_prep/post_scene_prep/post_spawn), and which reference
subclass to study for scene-import and NatNet scenarios.
pegasus_scene_setup.md points at it and drops the false 'PLAY_SIM_ON_START
not supported in standalone mode' claim. docker_usage.md gains a 'Launch
flags and readiness' section (--sim/--robots/--headless/--play/--wait/
--dry-run, effective-config dumps, airstack ready).

The write-isaac-sim-scene skill was re-taught from scratch: it prescribed
copy-pasting a ~240-line skeleton whose API had drifted to non-runnable
(wrong add_zed_stereo_camera_subgraph signature, nonexistent
SIMULATION_ENVIRONMENTS keys, low-level Multirotor API no shipped script
uses). It now teaches scenario declaration on PegasusApp with an explicit
'do not copy-paste' rule. Other skills fixed where the old guidance became
wrong or footgun-inducing: integrate-module-into-layer ('airstack stop' is
not a command), test-in-simulation and configure-multi-robot (bare
NUM_ROBOTS=N up now fails preflight with the single-drone script — use
--robots), use-airstack-cli (new flags + ready in the reference),
optitrack-development (single-drone NatNet body names are env-overridable
now).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+1617 / −1753 in 30 files:

- `.agents/skills/configure-multi-robot/SKILL.md`
- `.agents/skills/integrate-module-into-layer/SKILL.md`
- `.agents/skills/optitrack-development/SKILL.md`
- `.agents/skills/test-in-simulation/SKILL.md`
- `.agents/skills/use-airstack-cli/SKILL.md`
- `.agents/skills/write-isaac-sim-scene/SKILL.md`
- `.airstack/modules/ready.sh`
- `.env`
- `.gitignore`
- `AGENTS.md`
- `CHANGELOG.md`
- `airstack.sh`
- `common/.tmux.conf`
- `docs/development/beginner/airstack-cli/docker_usage.md`
- `docs/development/beginner/key_concepts.md`
- `docs/gcs/usage/user_interface.md`
- `docs/getting_started/index.md`
- `docs/simulation/index.md`
- `docs/simulation/isaac_sim/docker.md`
- `docs/simulation/isaac_sim/pegasus_scene_setup.md`
- `docs/simulation/isaac_sim/spawning_drones.md`
- `docs/simulation/ms-airsim/index.md`
- `simulation/isaac-sim/launch_scripts/barebones_pegasus_launch.py`
- `simulation/isaac-sim/launch_scripts/example_multi_drone_scene_import.py`
- `simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_natnet_launch_script.py`
- `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_natnet_launch_script.py`
- `simulation/isaac-sim/launch_scripts/pegasus_app.py`
- `tests/meta/test_launch_intent_contract.py`

## dbead2a748 — 2026-08-21 — Andrew Jong — KEYWORD broken,fix,fixed,wrong

**audit(docs): link integrity 81->4, truthful CHANGELOG, root README, beginner funnel**

https://github.com/castacks/AirStack/commit/dbead2a748c3e3e7972fc60ff5d707aa7b6746e8

```
From the four-domain rot audit (fix-now tier, docs/tests/skills):
- repo-wide broken links 81 -> 4 (all intentional placeholders); nav 404s
  fixed and made unrepeatable by a nav-existence contract test
- CHANGELOG [Unreleased]: five entries advertising extracted/deleted things
  removed; the campaign's actual user-visible changes written up
  (stacks-only dispatch, module/fleet/doctor CLI, removals, image slimming)
- repo-root README.md created (there was none); docs/README architecture
  refreshed post-RFC
- beginner funnel taught the new canon: key_concepts gains
  stacks/modules/fleets, the CLI reference regenerated from real
  registrations (~35 commands), tutorials_reference repaired
- tests/README: wiring + tests/meta + --stack/--fleet documented, wrong
  defaults fixed; tests/meta/README added
- skills: use-airstack-cli + run-system-tests refreshed; frontmatter added
  to the two headless skills; NEW extract-module skill distilled from the
  three completed extractions; catalog sentence fixed + pages regenerated

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+999 / −282 in 46 files:

- `.agents/README.md`
- `.agents/skills/add-ros2-package/assets/package_template/README.md`
- `.agents/skills/add-unit-tests/SKILL.md`
- `.agents/skills/attach-gossip-payload/SKILL.md`
- `.agents/skills/debug-module/SKILL.md`
- `.agents/skills/docker-build-profiles/SKILL.md`
- `.agents/skills/extract-module/SKILL.md`
- `.agents/skills/run-system-tests/SKILL.md`
- `.agents/skills/test-in-simulation/SKILL.md`
- `.agents/skills/use-airstack-cli/SKILL.md`
- `.github/orchestrator/README.md`
- `AGENTS.md`
- `CHANGELOG.md`
- `README.md`
- `docs/README.md`
- `docs/about.md`
- `docs/development/advanced/ai_agent_guide.md`
- `docs/development/beginner/airstack-cli/index.md`
- `docs/development/beginner/key_concepts.md`
- `docs/development/development_environment.md`
- `docs/development/index.md`
- `docs/development/intermediate/documentation.md`
- `docs/development/intermediate/frame_conventions.md`
- `docs/development/module_ci.md`
- `docs/getting_started/tutorials_reference.md`
- `docs/modules/index.md`
- `docs/real_world/HITL/index.md`
- `docs/real_world/deploying_to_hardware.md`
- `docs/real_world/index.md`
- `docs/robot/autonomy/dds_router.md`
- `docs/robot/autonomy/perception/index.md`
- `docs/robot/autonomy/sensors/index.md`
- `docs/robot/autonomy/system_architecture.md`
- `docs/robot/autonomy_modes.md`
- `docs/robot/configuration/index.md`
- `docs/robot/index.md`
- `docs/robot/logging/index.md`
- `docs/robot/logging/rosbags.md`
- `docs/tutorials/index.md`
- `mkdocs.yml`
- … 6 more

## 0b7b33d85b — 2026-08-21 — Andrew Jong — KEYWORD fix,wrong

**audit(gcs/sim): fleet-aware GCS, distinct GPS homes, sshd fix, dead-file purge**

https://github.com/castacks/AirStack/commit/0b7b33d85bbcf4b4179582ff8a4a285e8f1e4693

```
From the four-domain rot audit (fix-now tier, GCS + simulation):
- action_relay is fleet-aware: roster from FLEET_CONFIG_FILE (mounted
  config/ + tools/fleet), ROBOT_RELAY_MAP env override, legacy NUM_ROBOTS
  byte-identical fallback; gcs container gains the fleet env + mounts
- multi-drone runs get distinct PX4 GPS homes (PegasusApp anchors
  world_gps_origin for >1 drone; single-drone byte-unchanged) — verified
  live: three distinct coordinates matching the fleet spawn offsets; the
  GCS map no longer stacks the fleet on one point
- sshd actually starts (gcs/simple-sim/zed services ran 'ssh service
  restart'); isaac .bashrc no longer clobbers persisted shell history
- deleted: the retired domain-100 sim_to_robot_bridge.yaml, the inert
  gcs/docker/.env (wrong values), superseded robot-commands.foxe v1, unused
  fastrtps profile, committed bash histories teaching a deleted package,
  ms-airsim's bypassed bridge launch, simple-sim's stale duplicate launch
- fleet_spawn honors per-vehicle camera flags; ENU-origin constants
  cross-referenced in all four copies; AIRSIM_* knobs plumbed through
  compose; devcontainer/gcs unbroken; gcs docs rewritten against reality;
  user_TEMPLATE asset_root 4.5 -> 5.1; .gitignore footguns scoped

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+3815 / −674 in 221 files:

- `.devcontainer/gcs/devcontainer.json`
- `.devcontainer/gcs/launch.json`
- `.gitignore`
- `common/ros_packages/coordination/coordination_bringup/coordination_bringup/frame_utils.py`
- `docs/gcs/docker/index.md`
- `docs/gcs/index.md`
- `docs/simulation/isaac_sim/pegasus_scene_setup.md`
- `docs/simulation/isaac_sim/scene_setup.md`
- `docs/simulation/isaac_sim/spawning_drones.md`
- `docs/simulation/ms-airsim/index.md`
- `gcs/docker/.bash_history`
- `gcs/docker/.env`
- `gcs/docker/docker-compose.yaml`
- `gcs/docker/gcs-base-docker-compose.yaml`
- `gcs/docker/resources/fastrtps-profile.xml`
- `gcs/foxglove_extensions/render_layout.py`
- `gcs/foxglove_extensions/robot-commands.foxe`
- `gcs/ros_ws/src/action_relay/action_relay/relay_node.py`
- `gcs/ros_ws/src/action_relay/launch/action_relay.launch.py`
- `gcs/ros_ws/src/gcs_visualizer/gcs_visualizer/gcs_utils.py`
- `simulation/isaac-sim/.gitignore`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/.collect.mapping.json`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_Barcode_0001.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_CeilingA_06b.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_CratePlasticE_01.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_Floor_01.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_FrameA_01.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_LampCeilingA.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_PaperNotes_01.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_PushcartA_01.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_RackShield_01.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_SignB.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MI_WallB_01.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/M_AisleSign.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/M_Glow.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/M_TrafficCone.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/M_WallBoard_01.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/M_WetFloorSign.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/MaterialInstanceDynamic_1220.mdl`
- `simulation/isaac-sim/assets/scenes/local/1bfe5688f17c/SubUSDs/materials/OmniUe4Base.mdl`
- … 181 more

## 8242ae85cb — 2026-08-21 — Andrew Jong — KEYWORD fix,fixed

**audit(robot): index-collision fix, live plugin param, honest baselines, orphan purge**

https://github.com/castacks/AirStack/commit/8242ae85cbb550b17cb0dabf9be6786eba4bea34

```
From the four-domain rot audit (fix-now tier, robot domain):
- robot_name_map regex: robots >=9 no longer collapse onto another robot's
  namespace/domain (robot 12 -> robot_2 before; contract-tested now)
- robot_interface_node honors its 'interface' plugin parameter (was
  hardcoded to MAVROSInterface, silently discarding the param)
- wiring baselines: JSON trailers compacted (30,406 -> 1,935 committed
  lines), full_droan_cpu's hand-copied title fixed + title==stack asserted,
  golden-node wait added (settle proves the graph stopped growing, not that
  it is complete — MAC-VO's model load raced the capture); all five
  baselines re-blessed drift-clean on GPU
- full_macvo modules.repos: real HTTPS pin replaces the phantom v0.1.0 SSH
  placeholder
- 13 orphan files deleted (ROS1 leftovers, byte-identical config dups, the
  caller-less domain-bridge machinery, .vscode dup); exploration package
  repaired (ament_package order, format-3 manifest, real deps) with its
  invalid ROS1 launch deleted
- stack READMEs/headers: bootstrap-instruction contradictions removed,
  set_remap wording corrected in 10 module launch headers + the lint
  docstring, Dockerfile diagnostic block deleted + lying stage comments
  fixed, robot-test runs the whole workspace, .bashrc identity resolution
  deduped, rviz MACVO ghosts removed

GPU-gated: default + 4 stack wiring drift-clean; heterogeneous fleet
flight-ready 111s.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+332 / −31593 in 62 files:

- `common/README.md`
- `common/ros_packages/desktop_bringup/config/mavros_config.yaml`
- `common/ros_packages/desktop_bringup/launch/gcs.launch.xml`
- `common/ros_packages/desktop_bringup/launch/static_transforms.launch.xml`
- `common/ros_packages/desktop_bringup/rviz/robot.rviz`
- `robot/docker/.bashrc`
- `robot/docker/.vscode/c_cpp_properties.json`
- `robot/docker/.vscode/extensions.json`
- `robot/docker/.vscode/launch.json`
- `robot/docker/.vscode/settings.json`
- `robot/docker/.vscode/tasks.json`
- `robot/docker/Dockerfile.robot`
- `robot/docker/docker-compose.yaml`
- `robot/docker/robot-base-docker-compose.yaml`
- `robot/docker/robot_name_map/default_robot_name_map.yaml`
- `robot/ros_ws/src/autonomy_bringup/CMakeLists.txt`
- `robot/ros_ws/src/autonomy_bringup/config/dds_router_echo.yaml`
- `robot/ros_ws/src/autonomy_bringup/config/dds_router_vanilla.yaml`
- `robot/ros_ws/src/autonomy_bringup/config/domain_bridge.yaml`
- `robot/ros_ws/src/autonomy_bringup/config/mavros_config.yaml`
- `robot/ros_ws/src/autonomy_bringup/launch/interpolate_dds_router.launch.py`
- `robot/ros_ws/src/autonomy_bringup/launch/interpolate_domain_bridge.launch.py`
- `robot/ros_ws/src/behavior/drone_safety_monitor/launch/drone_safety_monitor.launch.xml`
- `robot/ros_ws/src/global/global_bringup/config/vdb_remote_params.yaml`
- `robot/ros_ws/src/global/planners/exploration/CMakeLists.txt`
- `robot/ros_ws/src/global/planners/exploration/launch/exploration_launch.xml`
- `robot/ros_ws/src/global/planners/exploration/launch/random_walk_launch.xml`
- `robot/ros_ws/src/global/planners/exploration/package.xml`
- `robot/ros_ws/src/global/planners/random_walk/launch/random_walk_planner.launch.xml`
- `robot/ros_ws/src/interface/interface_bringup/CMakeLists.txt`
- `robot/ros_ws/src/interface/interface_bringup/{launch => config}/px4_config.yaml`
- `robot/ros_ws/src/interface/interface_bringup/launch/README.md`
- `robot/ros_ws/src/interface/interface_bringup/launch/mavros_px4.launch.xml`
- `robot/ros_ws/src/interface/robot_interface/src/robot_interface_node.cpp`
- `robot/ros_ws/src/local/controls/pid_controller/launch/pid_controller.launch.xml`
- `robot/ros_ws/src/local/controls/trajectory_controller/launch/fixed_trajectory_task.launch.xml`
- `robot/ros_ws/src/local/controls/trajectory_controller/launch/trajectory_controller.launch.xml`
- `robot/ros_ws/src/local/controls/trajectory_controller/launch/trajectory_controller_bag.launch`
- `robot/ros_ws/src/local/planners/droan_gl/launch/droan_gl.launch.xml`
- `robot/ros_ws/src/local/planners/droan_local_planner/launch/droan_local_planner.launch.xml`
- … 22 more

## 0a13f04c9e — 2026-08-21 — Andrew Jong — KEYWORD error,fix,fixed

**audit(cli): truthful help/README, one fleet pipeline, shared lib, hardened commands**

https://github.com/castacks/AirStack/commit/0a13f04c9eda0d9588a5387b3b8e453513a2c91a

```
From the four-domain rot audit (fix-now tier, CLI/toolchain):
- .airstack/README.md rewritten — it documented a containerized-compose CLI
  that never existed (rebuild-cli, Dockerfile.airstack-cli)
- ONE fleet pipeline: bridge-router generation moved into
  generate_fleet_compose.py (resolve-aware, covers <alias>/<stack> external
  stacks); airstack up --fleet and fleet generate share it; --dry-run no
  longer writes files
- set -e dead error paths fixed (connect/logs/config); rmi rewritten off
  positional docker-images parsing; ready --json emits pure JSON on stdout;
  ready gates built from the resolved robot NAME (non-robot_N fleets no
  longer 10-min false-timeout)
- help arms regenerated from reality (up/test/module/ready/sync/rmi) + a
  help-truth contract test; lint wired to real checks; format unregistered
- .airstack/modules/_lib.sh: the 4 duplicated shell primitives deduped
  (python/yaml check ×5, container identity ×3, env-value ×4, discovery ×2)
- .airstack/runs/ pruned to newest 50 (969-dir backlog cleared)
- module sync warns when plan-only regeneration drops lock --build image
  overrides (containers silently ran the base image without module layers —
  bit the macvo wiring gate); deprecation shims annotated 'remove in 0.21.0'

Unit suite: 273 passed.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+852 / −736 in 20 files:

- `.airstack/README.md`
- `.airstack/modules/_lib.sh`
- `.airstack/modules/config.sh`
- `.airstack/modules/dev.sh`
- `.airstack/modules/doctor.sh`
- `.airstack/modules/fleet.sh`
- `.airstack/modules/module.sh`
- `.airstack/modules/osmo.sh`
- `.airstack/modules/ready.sh`
- `.airstack/modules/stack.sh`
- `.airstack/modules/sync.sh`
- `airstack.sh`
- `tests/meta/test_cli_help_contract.py`
- `tests/meta/test_docker_layer_plan_contract.py`
- `tests/meta/test_fleet_contract.py`
- `tests/meta/test_module_overlay_contract.py`
- `tools/doctor/checks.py`
- `tools/fleet/generate_fleet_compose.py`
- `tools/fleet/resolve_fleet.py`
- `tools/gen_docs_catalog.py`

## c9e3cf6eab — 2026-08-21 — Andrew Jong — KEYWORD error,failed

**P2(rfc-379): module remove survives root-owned container artifacts**

https://github.com/castacks/AirStack/commit/c9e3cf6eabed2d84d4c043bf05214baaddd63d7a

```
Containers drop root-owned __pycache__/build debris into mounted module
checkouts; plain rm then failed and left a partial dir that broke the next
vcs import. Retry via a throwaway container, with the manual command in the
error path.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+13 / −1 in 1 files:

- `.airstack/modules/module.sh`

## 46c499bd19 — 2026-08-21 — Andrew Jong — KEYWORD error

**Remove the legacy AUTONOMY_ROLE dispatch — stacks are the only launch path**

https://github.com/castacks/AirStack/commit/46c499bd19758f165de2290a2b96134b4db54a4a

```
Owner ruling: no legacy-hardware compatibility required; hardware launch
workflows redefinable at will. Evidence basis: full ≡ full_default is
machine-proven (graph equality, held through every flatten); the desktop
profile's 'onboard' role was unreachable anyway (compose hardcoded
AUTONOMY_ROLE=full); lite_offload_global's generated bridge config
deliberately improves on the legacy split (drops the set_trajectory_mode
crossing — doctor hard gate #2).

- robot.launch.xml: shared preamble + one stack include; no-env default =
  stacks/full_default. role arg, three role groups, gossip/gcs args gone
- deleted: onboard_all/ + onboard_local_offboard_global/ (shared DDS configs
  moved to autonomy_bringup/config/ first), the five frozen layer launch
  files, and the now-empty behavior_bringup/sensors_bringup/local_bringup
  packages; lite stacks flattened to match (their only remaining wraps)
- compose: role env/args gone everywhere; desktop_split/voxl/l4t services
  select stacks via AIRSTACK_STACK_DIR (redefinable); base default
  full_default; l4t override rewritten stack-form
- CLI: AUTONOMY_ROLE now a hard preflight error with migration guidance;
  effective config always names the stack
- tests: legacy wiring golden retired (default golden = the stack baseline);
  contract tests assert the removal error; lint allowlist down to 4
  principled entries (playback utility, vendored zed, exploration pending
  rewrite, interface safety boundary)
- docs/skills sweep: 15+ pages and 7 skills re-pointed at stacks

GPU-gated: flagless default drift-clean vs stacks/full_default/wiring.md;
lite_default drift-clean through its flatten; fleet liveliness 8/8.
Module follow-up: asm_optitrack/asm_macvo test_stacks still wrap the deleted
layer files and need the same flatten in their repos.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+578 / −9965 in 83 files:

- `.agents/skills/add-task-executor/SKILL.md`
- `.agents/skills/configure-multi-robot/SKILL.md`
- `.agents/skills/integrate-module-into-layer/SKILL.md`
- `.agents/skills/update-documentation/SKILL.md`
- `.agents/skills/visualize-in-foxglove/SKILL.md`
- `.agents/skills/write-launch-file/SKILL.md`
- `AGENTS.md`
- `airstack.sh`
- `common/ros_packages/desktop_bringup/launch/gcs.launch.xml`
- `common/ros_packages/desktop_bringup/launch/robot.launch.xml`
- `common/ros_packages/logging/logging_bringup/launch/logging.launch.xml`
- `common/ros_packages/robot_descriptions/launch/robot_state_publisher.launch.py`
- `docs/development/fleets.md`
- `docs/development/intermediate/testing/end_to_end_testing.md`
- `docs/development/stacks.md`
- `docs/gcs/foxglove.md`
- `docs/getting_started/modular_airstack.md`
- `docs/real_world/index.md`
- `docs/robot/autonomy/behavior/index.md`
- `docs/robot/autonomy/dds_router.md`
- `docs/robot/autonomy/global/index.md`
- `docs/robot/autonomy/local/index.md`
- `docs/robot/autonomy/perception/index.md`
- `docs/robot/autonomy/sensors/index.md`
- `docs/robot/autonomy_modes.md`
- `docs/robot/configuration/index.md`
- `docs/robot/index.md`
- `docs/tutorials/index.md`
- `overrides/l4t-px4-realrobot.env`
- `robot/docker/docker-compose.yaml`
- `robot/docker/robot-base-docker-compose.yaml`
- `robot/ros_ws/src/autonomy_bringup/CMakeLists.txt`
- `robot/ros_ws/src/autonomy_bringup/{onboard_all => }/config/dds_router.yaml`
- `robot/ros_ws/src/autonomy_bringup/{onboard_all => }/config/dds_router_echo.yaml`
- `robot/ros_ws/src/autonomy_bringup/{onboard_all => }/config/dds_router_vanilla.yaml`
- `robot/ros_ws/src/autonomy_bringup/{onboard_all => }/config/domain_bridge.yaml`
- `robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml`
- `robot/ros_ws/src/autonomy_bringup/onboard_all/launch/onboard_autonomy_all.launch.xml`
- `robot/ros_ws/src/autonomy_bringup/onboard_local_offboard_global/config/dds_router.yaml`
- `robot/ros_ws/src/autonomy_bringup/onboard_local_offboard_global/config/domain_bridge.yaml`
- … 43 more

## 1ccbb20244 — 2026-08-21 — Andrew Jong — KEYWORD wrong

**chore(launch): legibility sweep — delete dead launch files, purge dead blocks, header discipline**

https://github.com/castacks/AirStack/commit/1ccbb2024472c4aa7c567ae703415d633d5c89ed

```
Graph-neutrality machine-proven: colcon build green and the legacy wiring
golden + all four stack wiring.md baselines drift-clean after the sweep.

Deleted (zero references, evidence per file in the sweep audit):
- the legacy Gazebo parallel-bringup tree (8 files, dead env indirection)
- static_transforms dup, gst2ros orphan, mavros_connection_poll,
  odometry_conversion (pointed at the wrong package), the unwired
  px4_interface uXRCE launch (package + plugin kept), the empty
  global_planner.launch.xml stub
- dead <?ignore?> blocks in local/sensors/gcs launch files (alternates live
  in stacks now) and the consumerless local_depth_in_topic arg

Legibility: 2-6 line status headers on 22 launch files (stack entry /
canonical module launch / LEGACY graph-frozen until 0.21 / utility);
description= added to 19 args across 5 files; 6 wrong type-claims corrected
(tracking_point/look_ahead are airstack_msgs/Odometry per the conventions
spec). Lint allowlist shrinks 14 -> 8.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+176 / −1543 in 47 files:

- `common/ros_packages/airstack_common/launch/playback.launch.xml`
- `common/ros_packages/coordination/coordination_bringup/launch/gcs_gossip_bridge.launch.py`
- `common/ros_packages/desktop_bringup/launch/gcs.launch.xml`
- `common/ros_packages/desktop_bringup/launch/robot.launch.xml`
- `common/ros_packages/desktop_bringup/launch/static_transforms.launch.xml`
- `common/ros_packages/logging/bag_recorder_pid/launch/bag_record_pid.launch.py`
- `common/ros_packages/logging/bag_recorder_pid/launch/bag_record_pid_namespaced.launch.py`
- `common/ros_packages/logging/bag_recorder_pid/launch/bag_record_pid_node.launch.py`
- `common/ros_packages/logging/logging_bringup/launch/logging.launch.xml`
- `common/ros_packages/robot_descriptions/launch/robot_state_publisher.launch.py`
- `robot/ros_ws/src/autonomy_bringup/onboard_all/launch/onboard_autonomy_all.launch.xml`
- `robot/ros_ws/src/autonomy_bringup/onboard_all/launch/static_transforms.launch.xml`
- `robot/ros_ws/src/autonomy_bringup/onboard_local_offboard_global/launch/offboard_autonomy_global.launch.xml`
- `robot/ros_ws/src/autonomy_bringup/onboard_local_offboard_global/launch/onboard_autonomy_local.launch.xml`
- `robot/ros_ws/src/behavior/behavior_bringup/launch/behavior.launch.xml`
- `robot/ros_ws/src/global/global_bringup/launch/global.launch.xml`
- `robot/ros_ws/src/global/global_bringup/launch/global_planner.launch.xml`
- `robot/ros_ws/src/global/planners/exploration/launch/exploration_launch.xml`
- `robot/ros_ws/src/global/planners/exploration/launch/robot_launch_gazebo.xml`
- `robot/ros_ws/src/global/planners/exploration/launch/robot_launch_gazebo/gazebo_vis.rviz`
- `robot/ros_ws/src/global/planners/exploration/launch/robot_launch_gazebo/gz_autonomy_launch.xml`
- `robot/ros_ws/src/global/planners/exploration/launch/robot_launch_gazebo/gz_behavior_launch.xml`
- `robot/ros_ws/src/global/planners/exploration/launch/robot_launch_gazebo/gz_domain_bridge.yaml`
- `robot/ros_ws/src/global/planners/exploration/launch/robot_launch_gazebo/gz_global_launch.xml`
- `robot/ros_ws/src/global/planners/exploration/launch/robot_launch_gazebo/gz_interface_launch.xml`
- `robot/ros_ws/src/global/planners/exploration/launch/robot_launch_gazebo/gz_local_launch.xml`
- `robot/ros_ws/src/global/planners/exploration/launch/robot_launch_gazebo/gz_static_transforms.launch.xml`
- `robot/ros_ws/src/interface/interface_bringup/launch/interface.launch.py`
- `robot/ros_ws/src/interface/interface_bringup/launch/mavros_px4.launch.xml`
- `robot/ros_ws/src/interface/mavros_interface/CMakeLists.txt`
- `robot/ros_ws/src/interface/mavros_interface/launch/README.md`
- `robot/ros_ws/src/interface/mavros_interface/launch/mavros_connection_poll.launch.py`
- `robot/ros_ws/src/interface/px4_interface/CMakeLists.txt`
- `robot/ros_ws/src/interface/px4_interface/launch/px4_interface.launch.xml`
- `robot/ros_ws/src/interface/robot_interface/CMakeLists.txt`
- `robot/ros_ws/src/interface/robot_interface/launch/odometry_conversion.xml`
- `robot/ros_ws/src/local/controls/pid_controller/launch/pid_controller.launch.xml`
- `robot/ros_ws/src/local/local_bringup/launch/local.launch.xml`
- `robot/ros_ws/src/local/planners/droan_gl/launch/droan_gl.launch.xml`
- `robot/ros_ws/src/local/planners/droan_local_planner/launch/droan_local_planner.launch.xml`
- … 7 more

## 03ef065647 — 2026-08-22 — Andrew Jong — KEYWORD fix

**fix(.env): restore the audited env comments and the 0.20.0-alpha.9 bump**

https://github.com/castacks/AirStack/commit/03ef06564756c2bad8a54a5ca07e954d1d40f7db

```
A botched release-prep attempt committed develop's pre-audit .env (old
prebuilt comment block, VERSION 0.19.0) onto this branch by accident;
this restores the stack's intended state. CHANGELOG.md, which the same
stray commit resurrected, is already re-deleted in the merge above.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+5 / −3 in 1 files:

- `.env`

## f14ad6d6da — 2026-08-22 — Andrew Jong — KEYWORD break,broken,bug,bugs,error,fail,failed,failing,fails,failure,fix,fixed,fixes,fixing,issue,patch,regression,repair,revert,typo,wrong

**Release 0.19.0 (#398)**

https://github.com/castacks/AirStack/commit/f14ad6d6dad92707963ee97230e1c43f819ea26f

```
* Bump VERSION to  after sync from main

* feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO (#352)

* feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO

Adds a privileged Docker-in-Docker workspace task that lets a developer
run the full AirStack docker-compose stack on OSMO and attach an IDE
over SSH, with Isaac Sim WebRTC livestream + Foxglove websocket exposed
via osmo port-forward.

Components:
- osmo/workspace/{Dockerfile,entrypoint.sh,sshd_config}: airstack-osmo-workspace
  image. Ubuntu 24.04 + sshd (pubkey-only) + Docker CE + Docker Compose +
  nvidia-container-toolkit + fuse-overlayfs (DinD-on-overlayfs needs it,
  otherwise dockerd falls back to vfs which bloats AirStack images ~10x).
- osmo/workflows/airstack-dev.yaml: single privileged GPU task. Materializes
  Nucleus + airlab-docker secrets from OSMO credentials, clones AirStack,
  starts inner dockerd, runs `airstack up` with desktop + isaac-sim-livestream
  Compose profiles.
- simulation/isaac-sim: isaac-sim-livestream Compose service that runs
  Pegasus standalone with --/app/livestream/enabled=true and exposes
  WebRTC port ranges 47995-48012 / 49000-49007 / 49100; launch script
  gates headless+livestream extension on ISAAC_SIM_LIVESTREAM env var.
- .airstack/modules/osmo.sh: airstack osmo:{up,ide,foxglove,webrtc,logs,down}
  CLI wrappers around `osmo workflow submit` / `port-forward` / `cancel`.
  Persists the active workflow id and validates it's still running before
  each command (prevents the stale-state 410 error).
- airstack.sh: bash 4+ re-exec bootstrap (macOS ships 3.2; the CLI uses
  `declare -A`).
- osmo/README.md + docs/tutorials/airstack_on_osmo.md: admin pool setup
  (privileged_allowed) + per-user credentials (airlab-docker-login,
  airlab-nucleus) + student-facing IDE attach + WebRTC/Foxglove flow.

Pool requirements: privileged_allowed: true, GPU pool with
nvidia-container-toolkit on the host, ample node ephemeral storage
(AirStack images extracted are ~50-100Gi via fuse-overlayfs; vfs needs
~500Gi+).

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): harden CLI + workspace image against stale-state, port-forward race, and cursor-server install hangs

Four bugs that bit the first end-to-end runs (airstack-dev-10 → -13):

- _osmo_wf_id: validate saved workflow id against `osmo workflow query`
  before returning. Without this, the state file at ~/.airstack/osmo-state
  outlives the workflow it points at and every subsequent osmo:webrtc /
  osmo:foxglove / osmo:ide call surfaces the same confusing
  "Workflow airstack-dev-N is not running! (status 410)" instead of the
  obvious "run airstack osmo:up to launch a fresh workflow".

- cmd_osmo_up: `osmo workflow submit --set-env` is variadic. Passing two
  separate `--set-env A=1 --set-env B=2` silently drops the first one —
  this is what made airstack-dev-11 fail with "ERROR: SSH_PUB_KEY not set"
  when --branch was passed alongside the pubkey. Collapse the K=V pairs
  into a single --set-env.

- cmd_osmo_ide: previously launched the IDE before starting the
  port-forward, so Cursor/VS Code would try to SSH localhost:2200 a few
  hundred ms before the tunnel listener existed and fail with
  "connect to host localhost port 2200: Connection refused". Now: detect
  an existing forward and reuse it (also avoids the "Address already in
  use" if osmo:foxglove was started in parallel), otherwise spawn the
  forward in the background, wait up to 30s for it to bind, then launch
  the IDE. Ctrl+C tears down the spawned forward cleanly via a trap.

- workspace image / entrypoint: Cursor Remote-SSH hung indefinitely
  on airstack-dev-13 because (a) cursor-server's installer fell back to
  wget when curl timed out and wget was not in the image, and (b) a
  /tmp/cursor-remote-lock.* file left behind by the first crashed
  install blocked every silent retry. Add wget to the apt install list
  and rm -f the stale Cursor / VS Code remote lock files at the very
  top of entrypoint.sh so each fresh pod starts from a clean slate.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): correct osmo:logs CLI invocation; install Foxglove extensions locally on osmo:foxglove

osmo:logs was invoking `osmo workflow logs <id> workspace --follow`, but
the real CLI takes the task via `-t TASK` (not positionally) and has no
`--follow` flag at all — so the command failed immediately with
"unrecognized arguments: workspace --follow". Replace with a polling loop
that uses `-t workspace -n <N>` on a short interval, prints only the
suffix that appeared since the previous fetch (find-the-last-seen-line
trick; degrades to "reprint tail" with a warning if the cursor outruns
-n), and exits cleanly once the workflow reaches a terminal state.
Tunables: OSMO_LOGS_TASK / OSMO_LOGS_TAIL / OSMO_LOGS_INTERVAL.

osmo:foxglove now installs the AirStack Foxglove extensions
(robot-commands / waypoint-editor / polygon-editor) into the laptop's
local Foxglove user-extensions directory before opening the
port-forward. Without this, custom panels show up as "Unknown panel
type: robot-commands.Robot Tasks" in the laptop's Foxglove Desktop
because it has no way to discover the extension folders that live
inside the GCS container. To avoid duplicating the install logic, the
existing gcs/foxglove_extensions/install.py is refactored to read
FOXGLOVE_EXT_SRC / FOXGLOVE_EXT_DST env vars (the in-container call
already in gcs/docker/gcs-base-docker-compose.yaml keeps working
unchanged via defaults). The wrapper sets those vars to
${PROJECT_ROOT}/gcs/foxglove_extensions and
~/.foxglove-studio/extensions respectively, overridable with
OSMO_FOXGLOVE_EXT_DIR / skippable with OSMO_FOXGLOVE_SKIP_EXTENSIONS=1.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): pin Kit livestream UDP media port to 49099 so osmo:webrtc actually shows pixels

Kit 107's WebRTC livestream picks a UDP media port dynamically. The
documented `omni.services.livestream.nvcf` defaults (minHostPort=47998
maxHostPort=48020 fixedHostPort=0) are ignored by the stock standalone
Kit binary — on airstack-dev-13 it bound to UDP 49042, outside both the
Compose-published range AND the default `osmo:webrtc --udp` forward of
`47995-48012,49000-49007`. Result: TCP signaling on 49100 worked, the
WebRTC Streaming Client window opened, but every SRTP media packet was
dropped → black viewport plus the recurring
`NVST_CCE_DISCONNECTED when m_connectionCount 0 != 1` underflow in Kit's log.

Pin the media port via three `app.livestream.*` settings set on
`SimulationApp` before `omni.kit.livestream.webrtc` is enabled, so
whichever code path the carb.livestream-rtc.plugin consults lands on the
same port:

    app.livestream.fixedHostPort = 49099
    app.livestream.minHostPort   = 49099
    app.livestream.maxHostPort   = 49099

49099 is a deliberate one-off from the 49100 TCP signaling port — same
neighborhood, easy to remember. Verified live on airstack-dev-13 after
`docker compose up -d --force-recreate isaac-sim-livestream`: Kit binds
UDP 49099 (`/proc/net/udp` hex BFCB on 0.0.0.0) and docker-proxy
publishes it from the pod host network.

Knock-on cleanups:
- `simulation/isaac-sim/docker/docker-compose.yaml` shrinks the
  isaac-sim-livestream `ports:` from 27 forwarded ports
  (`47995-48012, 49000-49007 TCP+UDP, 49100 TCP`) to just two:
  `49100/tcp` + `49099/udp`.
- `.airstack/modules/osmo.sh` shrinks `OSMO_WEBRTC_TCP` to `49100` and
  `OSMO_WEBRTC_UDP` to `49099`, so `airstack osmo:webrtc` spawns two
  port-forwards instead of thirty.
- `.gitignore` ignores `.DS_Store` so working from a Mac doesn't leak
  Finder metadata.

After pulling this commit into a running pod: `docker compose up -d
--force-recreate isaac-sim-livestream` to apply the new port mapping;
then re-run `airstack osmo:webrtc` on the laptop to pick up the new
forward ranges. The standalone WebRTC Streaming Client connects to
`localhost` (same address as before) and now actually receives frames.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): render Kit GUI in WebRTC stream; document SSH agent forward for in-pod git push

Two paper-cuts that bit airstack-dev-13 after the WebRTC media port pin
landed (commit 2d9b1611):

(1) The WebRTC stream showed only the bare 3D viewport — no menu bar,
    no toolbar, no panels, no console. Cause: SimulationApp's default
    when `headless=True` is to also hide the UI (`hide_ui=True`). The
    NVIDIA reference at
    `simulation/isaac-sim/standalone_examples/api/isaacsim.simulation_app/livestream.py`
    explicitly opts back into UI rendering plus picks explicit window
    sizing and `display_options=3286` to keep the default grid/axes
    visible. Mirror that config in `example_one_px4_pegasus_launch_script.py`
    when `ISAAC_SIM_LIVESTREAM=true` (local desktop dev keeps the
    minimal `headless=False` path unchanged).

(2) The pod has no SSH private key, only an `authorized_keys` for
    inbound connections from the user's laptop. As a result, `git push`
    from inside the Cursor / VS Code Remote-SSH session inside the pod
    fails with "Permission denied (publickey)". sshd inside the
    workspace image already has `AllowAgentForwarding yes` baked in via
    `osmo/workspace/sshd_config`; the missing piece is purely on the
    Mac side. Update the `~/.ssh/config` block in the tutorial to
    include `ForwardAgent yes` (so the local agent's keys are exposed
    in the pod), `AddKeysToAgent yes` (auto-load on first push), and
    `UseKeychain yes` (macOS-only Keychain unlock without passphrase
    prompts; ignored on Linux). Adds an `ssh-add -l` smoke-test note.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): make osmo:setup idempotent + paste-safe; document Nucleus auth-debug path

osmo:setup hit two failure modes that wasted a debug session each:

- `osmo credential set` is not an upsert for GENERIC creds — re-running
  setup (e.g. to rotate a Nucleus API token) failed with `400 duplicate
  key value violates unique constraint "credential_pkey"` and bailed
  before reaching the airlab-nucleus credential. Delete-then-set each
  credential so re-running is idempotent.
- Bracket-paste mode and cross-OS clipboards routinely smuggle invisible
  bytes around long pastes. Nucleus's auth endpoint silently DENIES a
  token with one extra trailing byte, with no actionable error from the
  client side. _osmo_prompt now strips leading/trailing whitespace and
  CR/NUL bytes via a new _osmo_trim helper, and warns when bytes were
  stripped. cmd_osmo_setup additionally JWT-shape-checks the Nucleus
  token (must be eyJ.<dot>.<dot>.) before submitting it, so a wrong
  paste fails at setup time instead of silently DENIED at pod boot.

Also documents how to debug the "Login Required: Unable to connect
server omniverse://airlab-nucleus..." popup: SSH the Nucleus host and
tail base_stack-nucleus-auth-1 for InternalCredentials.auth status:
DENIED. Adds a "Nucleus connectivity from OSMO" section to the admin
README clarifying that Nucleus over HTTPS uses a single 443 (no need
to open the native 3009-3180 range from the OSMO cluster), per
NVIDIA's TLS docs.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): use Nucleus API-token auth, with double-dollar to survive compose parser

The OSMO entrypoint was writing OMNI_USER=<andrew_id> alongside an API
token JWT in OMNI_PASS, which routes the JWT through the password-
verification path. Nucleus silently DENIES — visible only in
base_stack-nucleus-auth-1 as `InternalCredentials.auth … 'username':
'<andrew>' … status: DENIED` (no Tokens.auth_with_api_token call). Kit
then pops "Login Required: Unable to connect server omniverse://...".

omniclient expects the literal sentinel username `$omni-api-token` paired
with the JWT as the password. The entrypoint now detects a JWT-shaped
OMNI_PASS (header starts with `eyJ`) and emits OMNI_USER=$$omni-api-token
into omni_pass.env. The `$$` is intentional: docker-compose v2
interpolates env_file values, and a single `$` would be eaten by the
parser (`OMNI_USER=$omni-api-token` becomes `OMNI_USER=-api-token` after
${omni}- expansion to empty). The container ultimately sees
OMNI_USER=$omni-api-token, which is the correct sentinel.

Also note for the next debugger: `docker compose restart` does NOT
re-read env_file. Use `docker compose up -d <svc>` to recreate the
container after editing omni_pass.env.

Updates omni_pass_TEMPLATE.env header to document the API-token pattern
explicitly (with the $$ caveat), and adds a troubleshooting row that
distinguishes "wrong auth path" (DENIED with no Tokens.auth_with_api_token
call) from "bad/expired token" (Tokens.auth_with_api_token: DENIED).

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(osmo): make OSMO the recommended dev path, single clone-the-repo flow

Reposition the OSMO tutorial as AirStack's recommended day-to-day
development path (not just a fallback for laptops without GPUs) and
collapse it onto a single recipe: clone the repo, then drive everything
through the airstack osmo:* wrappers in .airstack/modules/osmo.sh.

- docs/tutorials/airstack_on_osmo.md
  - Retitle + rewrite the intro to lead with five concrete advantages
    (pooled GPUs, no local CUDA/Docker/driver maintenance, same image as
    CI + field robots, one-command onboarding, hardware bigger than your
    laptop). Demote the Linux+GPU-desktop path to an escape hatch.
  - Drop the Mac/Windows/no-GPU framing in 'Who is this for?' and the
    mermaid laptop subgraph label.
  - Add 'a local clone of AirStack' to Prerequisites; remove it from the
    'do not need' list.
  - Replace Option A/B credential split with a single
    ./airstack.sh osmo:setup recipe; move the three raw osmo credential
    set calls into a collapsible 'Under the hood' footnote.
  - Replace each step's raw osmo workflow ... command with the
    corresponding airstack osmo:up/logs/ide/webrtc/foxglove/down wrapper;
    preserve the raw form in 'Under the hood' footnotes that cross-link
    cmd_osmo_* in .airstack/modules/osmo.sh.
  - Drop the export WF=... paragraph — the wrappers read the id from
    ~/.airstack/osmo-state automatically; AIRSTACK_OSMO_WF overrides
    per-invocation. \$WF now only appears inside the raw-form footnotes.
  - Sweep Troubleshooting + What-survives tables: redirect raw
    port-forward fixes to the airstack osmo:* equivalents and rename the
    section to 'What survives airstack osmo:down?'.
  - Fix WebRTC edge label (49100/tcp + 49099/udp) to match the pinned
    ports the workflow actually uses today.

Companion cleanups now that the privileged_allowed flip is automatic on
the OSMO autosync side (synchronize_osmo_team_pools.py forces
privileged_allowed: true on every platform of every pool, so students
never see the 'platform does not have privileged flag enabled' error):

- osmo/README.md: drop the 'Most common blocker' privileged warning, the
  privileged_allowed row from the pool-requirements table, and the
  'privileged GPU pod' / '(privileged, GPU)' descriptors in the
  architecture summary. Simplify the validation-stage SSH-failure hint.
- osmo/workflows/airstack-dev.yaml: trim the long DinD-requires-privileged
  comment to a one-liner (the privileged: true directive itself stays).
- .airstack/modules/osmo.sh: remove the special-case 'privileged flag
  enabled' error branch in cmd_osmo_up — it should never fire now.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): make osmo:logs actually stream + survive pod host-key churn

osmo:logs was silent because cmd_osmo_logs wrapped osmo workflow logs in
$( ... ) on the assumption that -n LAST_N_LINES exits after dumping the
tail. Empirically the CLI keeps the stream open as new lines arrive (it
already behaves like tail -f, despite --help advertising only -n), so
command substitution waited forever and printed nothing. Drop the polling
loop and just exec the command directly.

Each fresh OSMO pod also ships a new sshd host key, so every osmo:up
trips StrictHostKeyChecking against the previous workflow's fingerprint
and SSH/Cursor abort with "Host key for [localhost]:2200 has changed".
Switch the recommended ~/.ssh/config block (and osmo/README.md) to the
ephemeral-host pattern (StrictHostKeyChecking no + UserKnownHostsFile
/dev/null + LogLevel ERROR), and have cmd_osmo_ide ssh-keygen -R the
stale loopback entry on every run so users on the old config get
unblocked automatically.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): auto-pin --branch to local checkout + clean error UX when workflow dies

The pod's entrypoint clones AirStack fresh from GitHub on every workflow
start (the pod fs is ephemeral). It defaulted to `main`, so any developer
testing branch-only OSMO changes silently ran their pod against stale
`main` code — most visibly: COMPOSE_PROFILES=desktop,isaac-sim-livestream
resolved to "desktop" alone on `main` because the isaac-sim-livestream
service only exists on the feature branch, so isaac-sim never came up
and `airstack osmo:webrtc` showed a blank stream.

  - cmd_osmo_up now defaults --branch to the local repo's current
    branch (git rev-parse --abbrev-ref HEAD). Detached HEAD or
    non-git checkouts fall back to `main` cleanly. Pass --branch
    explicitly to override.
  - New _osmo_check_branch_pushed warns up-front when the about-to-
    submit branch has no upstream, is ahead of origin, or has an
    uncommitted working tree. The pod doesn't see your laptop's edits.

Separately, when an OSMO workflow gets canceled mid-flight (osmo:down
in another shell, or OSMO timing it out), the in-flight port-forward
and logs streams raise OSMOUserError("Workflow X is not running!")
from inside an asyncio Task. The CLI prints "Task exception was never
retrieved" + a multi-line Traceback that buries the actual one-line
cause. New _osmo_pf_filter awk script collapses that into a single
[ERROR] line pointing at `airstack osmo:up`. Wired into webrtc,
foxglove, and logs. webrtc also gains a cleanup trap that kills the
backgrounded UDP port-forward on EXIT/INT/TERM so we don't leak it
against a dead workflow.

Tutorial Step 2 documents the new --branch default and the
"pod-clones-from-GitHub-not-your-laptop" gotcha.

Co-authored-by: Cursor <cursoragent@cursor.com>

* perf(osmo): bump inner dockerd concurrency to saturate 10 GbE pulls

dockerd's defaults of --max-concurrent-downloads=3 / --max-concurrent
-uploads=5 cap a fresh airstack-dev pod's image-pull at ~300 MiB/s
against the airlab-backup-10g registry — single-stream TLS tops out
around 300-500 MiB/s per core, and three parallel streams of unevenly
sized blobs serialize down to that ceiling. Ceph (1014 TiB, 92 OSDs,
SSD pools) and 10 GbE both have far more headroom than that. Bump to
10/10 to overlap enough blob downloads to saturate the pipe.

Threaded through the DOCKERD_MAX_DOWNLOADS / DOCKERD_MAX_UPLOADS env
vars so a pool can be tuned at submit time without rebuilding the
workspace image.

Workspace image needs a rebuild + push for this to take effect:
  cd osmo/workspace
  docker build -t airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest .
  docker push   airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(osmo): require buildx --platform linux/amd64 for workspace image

A plain `docker build && docker push` on an Apple Silicon Mac silently
produces a linux/arm64-only `latest` manifest. OSMO workers are amd64,
so every subsequent workflow fails at the outer pod-image pull with
"no match for platform in manifest" before the entrypoint even runs —
a confusing failure mode whose root cause lives entirely in the push,
not in the workflow yaml or the entrypoint.

Switch the README and the Dockerfile docstring to the buildx form,
explain the why, and document the post-push manifest check.

Co-authored-by: Cursor <cursoragent@cursor.com>

* perf(osmo): move dockerd data-root to /osmo/run for native overlay2

The OSMO pod's `/` is itself a containerd overlay snapshot, and Linux
refuses to stack a second overlayfs on top of an overlay rootfs — which
is why the inner dockerd was falling through to fuse-overlayfs. That
costs a kernel↔userspace FUSE round-trip on every `creat()` during
layer extraction, which murders throughput on apt/pip/ROS layers
(measured: 32-50 MB/s for small-file-heavy layers vs 480 MB/s for
big-file layers in the same pull).

Pointing dockerd at /osmo/run/docker (the kubelet emptyDir backed by
ext4 on /dev/vda3) lets the existing overlay2-first fallback chain
actually succeed on its first try, restoring kernel-overlay extraction
performance. emptyDir lifetime matches the workflow lifetime, so the
docker layer cache gets the right scope automatically.

Falls back to /var/lib/docker if /osmo/run isn't present so the image
still works in non-OSMO test contexts.

Co-authored-by: Cursor <cursoragent@cursor.com>

* updated version

* added virtual display for GL context

* added virtual display for droan_gl

* droan_gl patch

* run Xvfb in its own tmux session

* updated dockerfile + version

* typo in docs

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in osmo logs, renamed airstack-isaac-sim to just isaac-sim

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in container name for isaac-sim-livestream

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* airstack-dev version overwrite removed

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

---------

Co-authored-by: Cursor <cursoragent@cursor.com>
Co-authored-by: krrishj18 <krrishj@andrew.cmu.edu>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fix(isaac-sim): pegasus drone retains PX4 state across Stop/Play (#363)

* Update submodule to point to pegasus fix fixing start/stop behavior

* Bump VERSION to 0.19.0-alpha.2

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>

---------

Co-authored-by: Claude Opus 4.7 (1M context) <noreply@anthropic.com>

* Johnliu/optitrack autonomy (#359)

* incremented version tag

* docker image builds on l4t with generalizability features for other ros and linux versions

* documentation and claude skills for developing a new profile.

* initial natnet implementation

* deployment to jetson with ros2 jazzy now fixed

* unit testing dependency fix

* added optitrack perception to launch

* put tag version back in

* added instructions for Agents to run tests

* attempt at completely custom Optitrack Parser (Not working)

* fully implemented NatNetSDK natnet ros2 wrapper natively in AirStack. Hand test in mocap room successful

* unit test restructuring

* reorganized natnet logic for unit-testability

* unit testing restructuring to have unit tests in src and proxies in test. Unit tests workflows created

* reupdated documentation for current state of testing

* change unit tests to occur with system tests so that environment is builtgit status

* generalizes natnet parameters and disables natnet automatically for launch

* natnet client adaptor now references correct error code from NatNet SDK 4.4.0.0

* increment version tag

* bug fixes to natnet launching from env file

* fixed failing systems test due to depends issue and specifying unit tests via yaml

* Use NatNet callback context instead of thread-local dispatch

* addressing Krrish' documentation comments

* incrementing version tag after osmo PR merge

* documentation corrections

* Bump VERSION Update .env

---------

Co-authored-by: copilot-swe-agent[bot] <198982749+Copilot@users.noreply.github.com>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>

* Fix/camera init (#368)

* re-ordered initialization of stereo render product node to only initialize after right camera is initialized, ensuring camera is initialized as stereo (left camera is assumed, but right is optional)
---------

Co-authored-by: John <johnliuchs2022@gmail.com>

* Add fixed-trajectory system tests with cross-track error metrics (#365)

* Add fixed-trajectory evaluation tests

New tests/test_fixed_trajectory.py evaluates drone performance on Circle,
Figure8, Racetrack, and Line trajectories: takeoff -> execute -> land with
cross-track error, path RMSE, execution time, and success metrics recorded
to metrics.json for baseline comparison.

- Python ideal-path generators mirror fixed_trajectory_task.cpp equations
- Cross-track error uses robot pose snapshot at dispatch to transform
  base_link ideal path to world frame for odom comparison
- 5m loose tolerance documents the known circle failure without stranding drone
- conftest.py gains --trajectory-types CLI option and generalised phase-order
  sorting/ID-rewriting for both autonomy test modules
- tests/README.md documents the new module, all 11 metrics, and run commands

Made-with: Cursor

* Remove module docstring from test_fixed_trajectory.py

Made-with: Cursor

* Aj/GitHub ci cd (#347)

* Add link to PAT

* Change to new orchestrator instance workflow

* Add availability zone

* Bump version to 0.18.0-alpha.7

* Add fix for boot volume size blocking orchestrator

* Add floating IPs to CI/CD

* Bump gh runner_version to latest

* Update cicd defaults

* Rename integration-tests.yml to system-tests.yml

* Add debugging tips and add to mkdocs

* Use venv instead of pip3 to fix error: externally-managed-environment

* Explicitly fail autonomy test if images not yet built

* Enable using docker cache from docker registry to speed up docker image build tests for ci/cd

* Fix bug

* Update docs and change docker image build/push to also run on self-hosted runner

* Enable trigger docker build workflow on via manual dispatch

* Increase instance volume size so that space doesn't run out when building docker images

* Update to always try build all images

* Create dummy file for docker compose push to pass

* Add omni_pass.env with guest access to AirLab nucleus

* Update ci/cd tests to make sure image is present before running tests

* Make sure images for profiles get built

* Update system tests to not build images if pull available

* Make build/pull quiet

* Pin empy version to fix ROS2 jazzy version bug

* Switch image to desktop so that tests run successfully

* Add docker image signing to workflow

* Change pytest mark 'autonomy' to 'takeoff_hover_land'

* update comments on workflow

* Recurisve checkout of airstack

* Log more to GitHub

* Better error logging for ci/cd orchestrator

* Add check system resources before spawning server; if resources not available, report back and try again later

* Make it so that pytest no longer triggers from pushes on PR; make it so we can manually trigger pytest by commenting /pytest

* Update PR template

* Update AGENTS.md

* Fix finding baseline metrics

* Update workflow to comment instead of react

* Fix bug

* Try fix another bug

* Update omni_pass_TEMPLATE.env to use 'guest'; update default on system tests to include build_packages

* Auto prepend 'build_packages' mark to ensure code is built before tests

* Lower default stress-iterations to 1 and single takeoff-velocity to 0.5

* Johnliu/px4 cpu optimization (#348)

* added option for physics step frequency

* reverted example launch script

* patches PX4 simulation startup script and fixes robot DDS version

* set default physics Hz for PX4 to be 100Hz which is the minimum.

* reverted simulation changes

* updated docs

* Better error logging for ci/cd orchestrator

* Add check system resources before spawning server; if resources not available, report back and try again later

* added option for physics step frequency

* added option for physics step frequency

* removed physics frequency from .env and set working PX4 values in docker-compose defaults.

* removed unnecessary benchmarking from AirStack launch scripts.

---------

Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>

* Add new skills

* Revise pull request template for clarity and detail

Update pull request template with versioning guidelines

Added guidelines for versioning in the pull request template.

Update pull request template for media uploads

Clarified instructions for adding videos and images in the PR template.

* Johnliu/rtx lidar update (#351)

* Update PegasusSim lidar to new rtx lidar and optional min_sensor_range parameter to vdb model to avoid self-detection.

* removed deprecated ouster lidar. Completely integrated new rtx lidar

* renaming frame id back to ouster

* Added node to filter near and invalid lidar points

* reconciled topic names for lidar point cloud

* fixed example scripts to use rtx lidar api

* fixed tmux closing and rclpy path issue

* uses add_rtx in multi px4 script

* bumping version index

* docs added

* unit testing and documentation updates

* cleaning code from copilot suggestions

* docs(tests): fix pytest marker example for running liveliness and sensors

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/7ce7609a-a7f3-414d-9d42-0c9999d0459f

Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>

* docs(tests): fix marker semantics in test_sensors module docstring

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/bdf00f6f-1d9f-4597-bf57-b96f99421646

Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>

* addressing github copilot concerns

* docs(bridge): remove stale camera topics comment

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/2d5718ac-20e3-4f10-a12e-05d601cf000c

Co-authored-by: JohnYanxinLiu <63010779+JohnYanxinLiu@users.noreply.github.com>

* addressing copilot concerns

* removing debug print statement from reading point cloud

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fix(isaac-sim): align drone1 lidar prim path with spawned prim

Agent-Logs-Url: https://github.com/castacks/AirStack/sessions/fbad2b9c-1761-45b1-b464-3e874511255c

Co-authored-by: JohnYanxinLiu <63010779+JohnYanxinLiu@users.noreply.github.com>

* more succint comment in sim bashrc

* resolving discrepant comments in ros bridge yaml

* removed bug allocated new copy of point cloud array

* logs lidaar test with boolean instead of hz

* and --> or for marks

---------

Co-authored-by: copilot-swe-agent[bot] <198982749+Copilot@users.noreply.github.com>
Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>

* Krrish/coord pr (#350)

* Fixed multi-drone global plan

* added sep files for fire and retro

* added robot2 relative pos; diff rviz files; bridge for rayfronts topics

* added sharing of semantic rays

* changed rviz for both drones

* added target sharing

* changed drone start pos

* gossip layer w/o relay

* added global coords under /{ROBOT_NAME}/interface/mavros/global_position/raw/fix(not my topic, it was already publishing to that)

* gossip, threedrone,peerprofile

* multi drone vis in foxglove, odom doesn't work in foxglove yet

* multi drone vis in foxglove works with odom

* global plan added

* added image, vdb markers(not transformed yet)

* fixed state estimation flickering and vdb transform

* added custom foxglove buttons for commands

* added modular payloads to peerprofile, foxglove reads the payloads and vizualizes it,currently works for rayfronts

* fixing the rotation of payload

* syncing devices

* fixed gossip + translate

* added skill for foxglove/coordination

* removed VDB ENV

* rebase with main

* updated docs

* fixed launch files so they have play start on sim. scene_prep utils: added non-world prims to save in flattened manner

* created raven_nav package

* moved coordination to common

* fixed gcs<->robot dds

* added hitl functionality

* fixes to dds

* put dds hitl under gcs

* fixes to robot hitl

* syncing both computers

* mimiced robot-l4t for dataflow

* fixed path to ddsrouter_yaml

* fixed dds server

* fixed two_drone_fire

* rayfronts is now a ros package

* added feedback, it's sending success too early though

* fixed raven behavior

* foxglove panel with working executors

* random walk fixed

* fixed random walk bringup. Added saves and viz for multiple waypoints and polygons

* fixed bounds for exploration task, combined waypoint/polygon editor into task panel

* made waypoint/polygon gui larger

* added 2d map to foxglove

* WIP: pre-merge snapshot

* added changes from main

* WIP: pre-branch-split snapshot

* PR for foxglove+multi-robot

* merged with main

* PR cleanup: revert unrelated changes and drop extra files

- Restore main's robot.rviz (drop redundant robot_1/robot_2.rviz)
- Restore ms-airsim include in root docker-compose.yaml
- Restore airsim sections in docs/simulation/index.md
- Restore docs/gcs/docker/index.md (VERSION env name)
- Restore robot/docker/{.bashrc, Dockerfile.robot} to main
- Restore SIM_IP in robot/docker/docker-compose.yaml
- Restore takeoff_landing_planner takeoff_height: 8.0
- Drop docs/action_bridging.md (internal design memo)
- Drop personal launch scripts (two_drone_fire*, three_drone_scene_import, two_drone_RetroNeighbourhood)
- Trim verbose comments in gps_utils.py and example_multi_drone_scene_import.py

* Trim noisy inline comments in PR-added Python files

* Pin vdb_mapping_ros2 to public main (was at unpushed 68fe8dde)

* fixed launch script

* fixed foxglove bugs, added dynamic fg layout, updated docs

* fixed bugs found by copilot. Removed rviz by adding a node

* fixed comment

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fixed path in skill

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* fixes from copilot

* Fix Pegasus submodule pointer after merge

Advance to 8e01d013 (main's pointer) which contains spawn_rtx_lidar.py,
required by example_one_px4_pegasus_launch_script.py and the multi
script after the rtx-lidar update merged from main.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* fix(coordination): align gossip with steady clock + manifest hygiene

- gossip_node: swap startup log + outgoing-stamp clock to STEADY_TIME so
  the dedup-by-stamp invariant survives /clock pauses; subscribe to
  /global_position/global to match foxglove_visualizer and action_relay
- gossip_node docstring: drop the false "waypoint triggers immediate
  publish" claim
- coordination README: rename peer_registry node block to the actual
  per-robot registry topic; "wall-clock" -> "steady"
- package.xml: add missing exec/depend rules
  - coordination_bringup -> autonomy_bringup
  - autonomy_bringup -> coordination_bringup
  - desktop_bringup -> coordination_bringup, gcs_visualizer
  - gcs_visualizer -> std_msgs, coordination_msgs, coordination_bringup
- task_msgs: replace TODO license with BSD-3-Clause
- gcs.launch.xml: comment had `--no-sandbox` (`--` is illegal inside an
  XML comment and crashed the ROS launch parser)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* fix(gcs+autonomy): drop dead BT panel, lint payload imports, name-map override

- payload_visualizer_node: remove unused PointCloud2 / transform_point_cloud2
  imports (F401), collapse Marker/MarkerArray
- action_relay launch: ROBOT_RELAY_MAP env override for non-default
  robot_name -> domain mappings (default behavior unchanged)
- desktop_bringup robot.rviz: drop BehaviorTreePanel entry pointing at
  /behavior/behavior_tree_graphviz (publisher package was removed)
- autonomy_bringup domain_bridge: bridge /global_position/global to match
  the dds_router and the rest of the stack (was /raw/fix)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* fix(foxglove): clean panel-id stacking, atomic render, drop dead .foxe

- render_layout: regex now strips every trailing _r<n> (was: only the
  last one), fixes _r1_r1_r1... stacking on repeated runs
- render_layout: atomic write via tmp + os.replace so a partial
  json.dump doesn't corrupt the layout file
- airstack_default.json: re-render with fixed stripper to commit a
  clean source template (no stacked _r1 suffixes)
- install.sh -> install.py: file is Python, shebang is python3
- install.py: slugify publisher into the on-disk extension dir name
  so "AirLab CMU" doesn't produce a directory with a space
- drop robot-commands/robot-commands.foxe (duplicate; canonical is at
  foxglove_extensions/robot-commands.foxe) and the .foxe.bak

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>

* bug fixes

* bug fixes

* version

* reverted env

* updated gitignore and docs

* updated foxglove viz + consistent spellings across repo

* Move layout file to /root/ so it's immediately accessible, also fix template path

* Change so that file name reflects NUM_ROBOTS

* Add a DEBUG_RVIZ flag to launch robot rviz if needed

---------

Co-authored-by: krrishj18 <krrishj18@users.noreply.github.com>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: Claude Opus 4.7 <noreply@anthropic.com>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>

* Scene prep bug fix (#354)

* fixes to scene_prep_utils.py

* edited docs

* clean launch script

* updated version

* fixed comments inconsistency and typos

* formatting fix

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* bug in gossip if payload is empty

* fixed omni_pass.env file creation bug from CICD guest default profile

* fixed depth topic naming in foxglove gcs

* changed gps topic

* removed redundant exntentions

---------

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: airlab <johnliuchs2022@gmail.com>

* Add workflows to (1) enforce correct branch merge convention (2) update develop from main

* Update docs on branches

* Update workflow to handle develop version increment

* Release 0.18.0

* Bump VERSION to  after sync from main

* feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO (#352)

* feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO

Adds a privileged Docker-in-Docker workspace task that lets a developer
run the full AirStack docker-compose stack on OSMO and attach an IDE
over SSH, with Isaac Sim WebRTC livestream + Foxglove websocket exposed
via osmo port-forward.

Components:
- osmo/workspace/{Dockerfile,entrypoint.sh,sshd_config}: airstack-osmo-workspace
  image. Ubuntu 24.04 + sshd (pubkey-only) + Docker CE + Docker Compose +
  nvidia-container-toolkit + fuse-overlayfs (DinD-on-overlayfs needs it,
  otherwise dockerd falls back to vfs which bloats AirStack images ~10x).
- osmo/workflows/airstack-dev.yaml: single privileged GPU task. Materializes
  Nucleus + airlab-docker secrets from OSMO credentials, clones AirStack,
  starts inner dockerd, runs `airstack up` with desktop + isaac-sim-livestream
  Compose profiles.
- simulation/isaac-sim: isaac-sim-livestream Compose service that runs
  Pegasus standalone with --/app/livestream/enabled=true and exposes
  WebRTC port ranges 47995-48012 / 49000-49007 / 49100; launch script
  gates headless+livestream extension on ISAAC_SIM_LIVESTREAM env var.
- .airstack/modules/osmo.sh: airstack osmo:{up,ide,foxglove,webrtc,logs,down}
  CLI wrappers around `osmo workflow submit` / `port-forward` / `cancel`.
  Persists the active workflow id and validates it's still running before
  each command (prevents the stale-state 410 error).
- airstack.sh: bash 4+ re-exec bootstrap (macOS ships 3.2; the CLI uses
  `declare -A`).
- osmo/README.md + docs/tutorials/airstack_on_osmo.md: admin pool setup
  (privileged_allowed) + per-user credentials (airlab-docker-login,
  airlab-nucleus) + student-facing IDE attach + WebRTC/Foxglove flow.

Pool requirements: privileged_allowed: true, GPU pool with
nvidia-container-toolkit on the host, ample node ephemeral storage
(AirStack images extracted are ~50-100Gi via fuse-overlayfs; vfs needs
~500Gi+).

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): harden CLI + workspace image against stale-state, port-forward race, and cursor-server install hangs

Four bugs that bit the first end-to-end runs (airstack-dev-10 → -13):

- _osmo_wf_id: validate saved workflow id against `osmo workflow query`
  before returning. Without this, the state file at ~/.airstack/osmo-state
  outlives the workflow it points at and every subsequent osmo:webrtc /
  osmo:foxglove / osmo:ide call surfaces the same confusing
  "Workflow airstack-dev-N is not running! (status 410)" instead of the
  obvious "run airstack osmo:up to launch a fresh workflow".

- cmd_osmo_up: `osmo workflow submit --set-env` is variadic. Passing two
  separate `--set-env A=1 --set-env B=2` silently drops the first one —
  this is what made airstack-dev-11 fail with "ERROR: SSH_PUB_KEY not set"
  when --branch was passed alongside the pubkey. Collapse the K=V pairs
  into a single --set-env.

- cmd_osmo_ide: previously launched the IDE before starting the
  port-forward, so Cursor/VS Code would try to SSH localhost:2200 a few
  hundred ms before the tunnel listener existed and fail with
  "connect to host localhost port 2200: Connection refused". Now: detect
  an existing forward and reuse it (also avoids the "Address already in
  use" if osmo:foxglove was started in parallel), otherwise spawn the
  forward in the background, wait up to 30s for it to bind, then launch
  the IDE. Ctrl+C tears down the spawned forward cleanly via a trap.

- workspace image / entrypoint: Cursor Remote-SSH hung indefinitely
  on airstack-dev-13 because (a) cursor-server's installer fell back to
  wget when curl timed out and wget was not in the image, and (b) a
  /tmp/cursor-remote-lock.* file left behind by the first crashed
  install blocked every silent retry. Add wget to the apt install list
  and rm -f the stale Cursor / VS Code remote lock files at the very
  top of entrypoint.sh so each fresh pod starts from a clean slate.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): correct osmo:logs CLI invocation; install Foxglove extensions locally on osmo:foxglove

osmo:logs was invoking `osmo workflow logs <id> workspace --follow`, but
the real CLI takes the task via `-t TASK` (not positionally) and has no
`--follow` flag at all — so the command failed immediately with
"unrecognized arguments: workspace --follow". Replace with a polling loop
that uses `-t workspace -n <N>` on a short interval, prints only the
suffix that appeared since the previous fetch (find-the-last-seen-line
trick; degrades to "reprint tail" with a warning if the cursor outruns
-n), and exits cleanly once the workflow reaches a terminal state.
Tunables: OSMO_LOGS_TASK / OSMO_LOGS_TAIL / OSMO_LOGS_INTERVAL.

osmo:foxglove now installs the AirStack Foxglove extensions
(robot-commands / waypoint-editor / polygon-editor) into the laptop's
local Foxglove user-extensions directory before opening the
port-forward. Without this, custom panels show up as "Unknown panel
type: robot-commands.Robot Tasks" in the laptop's Foxglove Desktop
because it has no way to discover the extension folders that live
inside the GCS container. To avoid duplicating the install logic, the
existing gcs/foxglove_extensions/install.py is refactored to read
FOXGLOVE_EXT_SRC / FOXGLOVE_EXT_DST env vars (the in-container call
already in gcs/docker/gcs-base-docker-compose.yaml keeps working
unchanged via defaults). The wrapper sets those vars to
${PROJECT_ROOT}/gcs/foxglove_extensions and
~/.foxglove-studio/extensions respectively, overridable with
OSMO_FOXGLOVE_EXT_DIR / skippable with OSMO_FOXGLOVE_SKIP_EXTENSIONS=1.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): pin Kit livestream UDP media port to 49099 so osmo:webrtc actually shows pixels

Kit 107's WebRTC livestream picks a UDP media port dynamically. The
documented `omni.services.livestream.nvcf` defaults (minHostPort=47998
maxHostPort=48020 fixedHostPort=0) are ignored by the stock standalone
Kit binary — on airstack-dev-13 it bound to UDP 49042, outside both the
Compose-published range AND the default `osmo:webrtc --udp` forward of
`47995-48012,49000-49007`. Result: TCP signaling on 49100 worked, the
WebRTC Streaming Client window opened, but every SRTP media packet was
dropped → black viewport plus the recurring
`NVST_CCE_DISCONNECTED when m_connectionCount 0 != 1` underflow in Kit's log.

Pin the media port via three `app.livestream.*` settings set on
`SimulationApp` before `omni.kit.livestream.webrtc` is enabled, so
whichever code path the carb.livestream-rtc.plugin consults lands on the
same port:

    app.livestream.fixedHostPort = 49099
    app.livestream.minHostPort   = 49099
    app.livestream.maxHostPort   = 49099

49099 is a deliberate one-off from the 49100 TCP signaling port — same
neighborhood, easy to remember. Verified live on airstack-dev-13 after
`docker compose up -d --force-recreate isaac-sim-livestream`: Kit binds
UDP 49099 (`/proc/net/udp` hex BFCB on 0.0.0.0) and docker-proxy
publishes it from the pod host network.

Knock-on cleanups:
- `simulation/isaac-sim/docker/docker-compose.yaml` shrinks the
  isaac-sim-livestream `ports:` from 27 forwarded ports
  (`47995-48012, 49000-49007 TCP+UDP, 49100 TCP`) to just two:
  `49100/tcp` + `49099/udp`.
- `.airstack/modules/osmo.sh` shrinks `OSMO_WEBRTC_TCP` to `49100` and
  `OSMO_WEBRTC_UDP` to `49099`, so `airstack osmo:webrtc` spawns two
  port-forwards instead of thirty.
- `.gitignore` ignores `.DS_Store` so working from a Mac doesn't leak
  Finder metadata.

After pulling this commit into a running pod: `docker compose up -d
--force-recreate isaac-sim-livestream` to apply the new port mapping;
then re-run `airstack osmo:webrtc` on the laptop to pick up the new
forward ranges. The standalone WebRTC Streaming Client connects to
`localhost` (same address as before) and now actually receives frames.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): render Kit GUI in WebRTC stream; document SSH agent forward for in-pod git push

Two paper-cuts that bit airstack-dev-13 after the WebRTC media port pin
landed (commit 2d9b1611):

(1) The WebRTC stream showed only the bare 3D viewport — no menu bar,
    no toolbar, no panels, no console. Cause: SimulationApp's default
    when `headless=True` is to also hide the UI (`hide_ui=True`). The
    NVIDIA reference at
    `simulation/isaac-sim/standalone_examples/api/isaacsim.simulation_app/livestream.py`
    explicitly opts back into UI rendering plus picks explicit window
    sizing and `display_options=3286` to keep the default grid/axes
    visible. Mirror that config in `example_one_px4_pegasus_launch_script.py`
    when `ISAAC_SIM_LIVESTREAM=true` (local desktop dev keeps the
    minimal `headless=False` path unchanged).

(2) The pod has no SSH private key, only an `authorized_keys` for
    inbound connections from the user's laptop. As a result, `git push`
    from inside the Cursor / VS Code Remote-SSH session inside the pod
    fails with "Permission denied (publickey)". sshd inside the
    workspace image already has `AllowAgentForwarding yes` baked in via
    `osmo/workspace/sshd_config`; the missing piece is purely on the
    Mac side. Update the `~/.ssh/config` block in the tutorial to
    include `ForwardAgent yes` (so the local agent's keys are exposed
    in the pod), `AddKeysToAgent yes` (auto-load on first push), and
    `UseKeychain yes` (macOS-only Keychain unlock without passphrase
    prompts; ignored on Linux). Adds an `ssh-add -l` smoke-test note.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): make osmo:setup idempotent + paste-safe; document Nucleus auth-debug path

osmo:setup hit two failure modes that wasted a debug session each:

- `osmo credential set` is not an upsert for GENERIC creds — re-running
  setup (e.g. to rotate a Nucleus API token) failed with `400 duplicate
  key value violates unique constraint "credential_pkey"` and bailed
  before reaching the airlab-nucleus credential. Delete-then-set each
  credential so re-running is idempotent.
- Bracket-paste mode and cross-OS clipboards routinely smuggle invisible
  bytes around long pastes. Nucleus's auth endpoint silently DENIES a
  token with one extra trailing byte, with no actionable error from the
  client side. _osmo_prompt now strips leading/trailing whitespace and
  CR/NUL bytes via a new _osmo_trim helper, and warns when bytes were
  stripped. cmd_osmo_setup additionally JWT-shape-checks the Nucleus
  token (must be eyJ.<dot>.<dot>.) before submitting it, so a wrong
  paste fails at setup time instead of silently DENIED at pod boot.

Also documents how to debug the "Login Required: Unable to connect
server omniverse://airlab-nucleus..." popup: SSH the Nucleus host and
tail base_stack-nucleus-auth-1 for InternalCredentials.auth status:
DENIED. Adds a "Nucleus connectivity from OSMO" section to the admin
README clarifying that Nucleus over HTTPS uses a single 443 (no need
to open the native 3009-3180 range from the OSMO cluster), per
NVIDIA's TLS docs.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): use Nucleus API-token auth, with double-dollar to survive compose parser

The OSMO entrypoint was writing OMNI_USER=<andrew_id> alongside an API
token JWT in OMNI_PASS, which routes the JWT through the password-
verification path. Nucleus silently DENIES — visible only in
base_stack-nucleus-auth-1 as `InternalCredentials.auth … 'username':
'<andrew>' … status: DENIED` (no Tokens.auth_with_api_token call). Kit
then pops "Login Required: Unable to connect server omniverse://...".

omniclient expects the literal sentinel username `$omni-api-token` paired
with the JWT as the password. The entrypoint now detects a JWT-shaped
OMNI_PASS (header starts with `eyJ`) and emits OMNI_USER=$$omni-api-token
into omni_pass.env. The `$$` is intentional: docker-compose v2
interpolates env_file values, and a single `$` would be eaten by the
parser (`OMNI_USER=$omni-api-token` becomes `OMNI_USER=-api-token` after
${omni}- expansion to empty). The container ultimately sees
OMNI_USER=$omni-api-token, which is the correct sentinel.

Also note for the next debugger: `docker compose restart` does NOT
re-read env_file. Use `docker compose up -d <svc>` to recreate the
container after editing omni_pass.env.

Updates omni_pass_TEMPLATE.env header to document the API-token pattern
explicitly (with the $$ caveat), and adds a troubleshooting row that
distinguishes "wrong auth path" (DENIED with no Tokens.auth_with_api_token
call) from "bad/expired token" (Tokens.auth_with_api_token: DENIED).

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(osmo): make OSMO the recommended dev path, single clone-the-repo flow

Reposition the OSMO tutorial as AirStack's recommended day-to-day
development path (not just a fallback for laptops without GPUs) and
collapse it onto a single recipe: clone the repo, then drive everything
through the airstack osmo:* wrappers in .airstack/modules/osmo.sh.

- docs/tutorials/airstack_on_osmo.md
  - Retitle + rewrite the intro to lead with five concrete advantages
    (pooled GPUs, no local CUDA/Docker/driver maintenance, same image as
    CI + field robots, one-command onboarding, hardware bigger than your
    laptop). Demote the Linux+GPU-desktop path to an escape hatch.
  - Drop the Mac/Windows/no-GPU framing in 'Who is this for?' and the
    mermaid laptop subgraph label.
  - Add 'a local clone of AirStack' to Prerequisites; remove it from the
    'do not need' list.
  - Replace Option A/B credential split with a single
    ./airstack.sh osmo:setup recipe; move the three raw osmo credential
    set calls into a collapsible 'Under the hood' footnote.
  - Replace each step's raw osmo workflow ... command with the
    corresponding airstack osmo:up/logs/ide/webrtc/foxglove/down wrapper;
    preserve the raw form in 'Under the hood' footnotes that cross-link
    cmd_osmo_* in .airstack/modules/osmo.sh.
  - Drop the export WF=... paragraph — the wrappers read the id from
    ~/.airstack/osmo-state automatically; AIRSTACK_OSMO_WF overrides
    per-invocation. \$WF now only appears inside the raw-form footnotes.
  - Sweep Troubleshooting + What-survives tables: redirect raw
    port-forward fixes to the airstack osmo:* equivalents and rename the
    section to 'What survives airstack osmo:down?'.
  - Fix WebRTC edge label (49100/tcp + 49099/udp) to match the pinned
    ports the workflow actually uses today.

Companion cleanups now that the privileged_allowed flip is automatic on
the OSMO autosync side (synchronize_osmo_team_pools.py forces
privileged_allowed: true on every platform of every pool, so students
never see the 'platform does not have privileged flag enabled' error):

- osmo/README.md: drop the 'Most common blocker' privileged warning, the
  privileged_allowed row from the pool-requirements table, and the
  'privileged GPU pod' / '(privileged, GPU)' descriptors in the
  architecture summary. Simplify the validation-stage SSH-failure hint.
- osmo/workflows/airstack-dev.yaml: trim the long DinD-requires-privileged
  comment to a one-liner (the privileged: true directive itself stays).
- .airstack/modules/osmo.sh: remove the special-case 'privileged flag
  enabled' error branch in cmd_osmo_up — it should never fire now.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): make osmo:logs actually stream + survive pod host-key churn

osmo:logs was silent because cmd_osmo_logs wrapped osmo workflow logs in
$( ... ) on the assumption that -n LAST_N_LINES exits after dumping the
tail. Empirically the CLI keeps the stream open as new lines arrive (it
already behaves like tail -f, despite --help advertising only -n), so
command substitution waited forever and printed nothing. Drop the polling
loop and just exec the command directly.

Each fresh OSMO pod also ships a new sshd host key, so every osmo:up
trips StrictHostKeyChecking against the previous workflow's fingerprint
and SSH/Cursor abort with "Host key for [localhost]:2200 has changed".
Switch the recommended ~/.ssh/config block (and osmo/README.md) to the
ephemeral-host pattern (StrictHostKeyChecking no + UserKnownHostsFile
/dev/null + LogLevel ERROR), and have cmd_osmo_ide ssh-keygen -R the
stale loopback entry on every run so users on the old config get
unblocked automatically.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(osmo): auto-pin --branch to local checkout + clean error UX when workflow dies

The pod's entrypoint clones AirStack fresh from GitHub on every workflow
start (the pod fs is ephemeral). It defaulted to `main`, so any developer
testing branch-only OSMO changes silently ran their pod against stale
`main` code — most visibly: COMPOSE_PROFILES=desktop,isaac-sim-livestream
resolved to "desktop" alone on `main` because the isaac-sim-livestream
service only exists on the feature branch, so isaac-sim never came up
and `airstack osmo:webrtc` showed a blank stream.

  - cmd_osmo_up now defaults --branch to the local repo's current
    branch (git rev-parse --abbrev-ref HEAD). Detached HEAD or
    non-git checkouts fall back to `main` cleanly. Pass --branch
    explicitly to override.
  - New _osmo_check_branch_pushed warns up-front when the about-to-
    submit branch has no upstream, is ahead of origin, or has an
    uncommitted working tree. The pod doesn't see your laptop's edits.

Separately, when an OSMO workflow gets canceled mid-flight (osmo:down
in another shell, or OSMO timing it out), the in-flight port-forward
and logs streams raise OSMOUserError("Workflow X is not running!")
from inside an asyncio Task. The CLI prints "Task exception was never
retrieved" + a multi-line Traceback that buries the actual one-line
cause. New _osmo_pf_filter awk script collapses that into a single
[ERROR] line pointing at `airstack osmo:up`. Wired into webrtc,
foxglove, and logs. webrtc also gains a cleanup trap that kills the
backgrounded UDP port-forward on EXIT/INT/TERM so we don't leak it
against a dead workflow.

Tutorial Step 2 documents the new --branch default and the
"pod-clones-from-GitHub-not-your-laptop" gotcha.

Co-authored-by: Cursor <cursoragent@cursor.com>

* perf(osmo): bump inner dockerd concurrency to saturate 10 GbE pulls

dockerd's defaults of --max-concurrent-downloads=3 / --max-concurrent
-uploads=5 cap a fresh airstack-dev pod's image-pull at ~300 MiB/s
against the airlab-backup-10g registry — single-stream TLS tops out
around 300-500 MiB/s per core, and three parallel streams of unevenly
sized blobs serialize down to that ceiling. Ceph (1014 TiB, 92 OSDs,
SSD pools) and 10 GbE both have far more headroom than that. Bump to
10/10 to overlap enough blob downloads to saturate the pipe.

Threaded through the DOCKERD_MAX_DOWNLOADS / DOCKERD_MAX_UPLOADS env
vars so a pool can be tuned at submit time without rebuilding the
workspace image.

Workspace image needs a rebuild + push for this to take effect:
  cd osmo/workspace
  docker build -t airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest .
  docker push   airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(osmo): require buildx --platform linux/amd64 for workspace image

A plain `docker build && docker push` on an Apple Silicon Mac silently
produces a linux/arm64-only `latest` manifest. OSMO workers are amd64,
so every subsequent workflow fails at the outer pod-image pull with
"no match for platform in manifest" before the entrypoint even runs —
a confusing failure mode whose root cause lives entirely in the push,
not in the workflow yaml or the entrypoint.

Switch the README and the Dockerfile docstring to the buildx form,
explain the why, and document the post-push manifest check.

Co-authored-by: Cursor <cursoragent@cursor.com>

* perf(osmo): move dockerd data-root to /osmo/run for native overlay2

The OSMO pod's `/` is itself a containerd overlay snapshot, and Linux
refuses to stack a second overlayfs on top of an overlay rootfs — which
is why the inner dockerd was falling through to fuse-overlayfs. That
costs a kernel↔userspace FUSE round-trip on every `creat()` during
layer extraction, which murders throughput on apt/pip/ROS layers
(measured: 32-50 MB/s for small-file-heavy layers vs 480 MB/s for
big-file layers in the same pull).

Pointing dockerd at /osmo/run/docker (the kubelet emptyDir backed by
ext4 on /dev/vda3) lets the existing overlay2-first fallback chain
actually succeed on its first try, restoring kernel-overlay extraction
performance. emptyDir lifetime matches the workflow lifetime, so the
docker layer cache gets the right scope automatically.

Falls back to /var/lib/docker if /osmo/run isn't present so the image
still works in non-OSMO test contexts.

Co-authored-by: Cursor <cursoragent@cursor.com>

* updated version

* added virtual display for GL context

* added virtual display for droan_gl

* droan_gl patch

* run Xvfb in its own tmux session

* updated dockerfile + version

* typo in docs

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in comments

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in osmo logs, renamed airstack-isaac-sim to just isaac-sim

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* typo in container name for isaac-sim-livestream

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* airstack-dev version overwrite removed

Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

---------

Co-authored-by: Cursor <cursoragent@cursor.com>
Co-authored-by: krrishj18 <krrishj@andrew.cmu.edu>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>

* Add fixed-trajectory evaluation tests

New tests/test_fixed_trajectory.py evaluates drone performance on Circle,
Figure8, Racetrack, and Line trajectories: takeoff -> execute -> land with
cross-track error, path RMSE, execution time, and success metrics recorded
to metrics.json for baseline comparison.

- Python ideal-path generators mirror fixed_trajectory_task.cpp equations
- Cross-track error uses robot pose snapshot at dispatch to transform
  base_link ideal path to world frame for odom comparison
- 5m loose tolerance documents the known circle failure without stranding drone
- conftest.py gains --trajectory-types CLI option and generalised phase-order
  sorting/ID-rewriting for both autonomy test modules
- tests/README.md documents the new module, all 11 metrics, and run commands

Made-with: Cursor

* Spherical lookahead bug that fixed the circle test and caused the circle test to pass

* Added in code that consolidated all the results code so the user can easily see their results in one file without having to wade through a ton of log files to get what they need

* Results for 10 tries headless summary statistics

* Fixed the logging files so now it only outputs one summary file and it doesn't inundate the user with a ton of log files for no reason

* deleted cleanup_old_results.sh which was a local tool for cleaning up everything

* Added preliminary docs to explain changes made

* Changed .env to say 0.19.0-alpha.4

* Resolved all the merge conflicts that are in this file

* Revert sphere_radius to 1.0; velocity_sphere_radius_multiplier=1.0 makes the fixed value inert

Co-authored-by: Cursor <cursoragent@cursor.com>

* Remove internal branch reference from baseline; note AirStation hardware

Co-authored-by: Cursor <cursoragent@cursor.com>

* Remove parameter tuning bullet from docs after reverting sphere_radius

Co-authored-by: Cursor <cursoragent@cursor.com>

* Move system-test prerequisites to index.md and reference it from fixed-trajectory doc

Co-authored-by: Cursor <cursoragent@cursor.com>

* Remove path tracker bug fixes section from docs (covered in PR description)

Co-authored-by: Cursor <cursoragent@cursor.com>

* Trim duplicated stack bring-up from manual usage; link to Getting Started

Co-authored-by: Cursor <cursoragent@cursor.com>

* Reframe fixed-trajectory doc as end-to-end testing guide

Rename fixed_trajectory_testing.md to end_to_end_testing.md (history preserved), add e2e intro and future-work note, fix stale test path to tests/system, and update mkdocs nav, testing index, and tests/README references.

Co-authored-by: Cursor <cursoragent@cursor.com>

* removed stale test_sensors file

* incremented version tag

* Fixed the summary.txt file after it broke after a ton of commits were completed.

* resyncing Pegasus module to fixed camera initialization fix

---------

Co-authored-by: pvkumara <pkumara@andrew.cmu.edu>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
Co-authored-by: John Liu <63010779+JohnYanxinLiu@users.noreply.github.com>
Co-authored-by: copilot-swe-agent[bot] <198982749+Copilot@users.noreply.github.com>
Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: Krrish Jain <krrishj@andrew.cmu.edu>
Co-authored-by: krrishj18 <krrishj18@users.noreply.github.com>
Co-authored-by: Claude Opus 4.7 <noreply@anthropic.com>
Co-authored-by: airlab <johnliuchs2022@gmail.com>
Co-authored-by: Andrew Jong <andrewjong@fieldai.com>
Co-authored-by: github-actions[bot] <41898282+github-actions[bot]@users.noreply.github.com>
Co-authored-by: Sebastian Scherer <basti@andrew.cmu.edu>
Co-authored-by: Cursor <cursoragent@cursor.com>

* General robot deployment infra: aarch64 build args + robot-name resolution fixes (#370)

Foundational real-robot deployment fixes extracted from the OptiTrack
emulation PR (#367) so they can be reviewed and merged first; #367 will be
rebased on top afterward, shrinking its diff.

Docker / ARM build:
- Add TARGET_ARCH build arg (default x86_64) to Dockerfile.robot and use it to
  parametrize LD_LIBRARY_PATH, so the aarch64 (Jetson/l4t, voxl) images link
  against the correct arch triplet.
- docker-compose.yaml passes TARGET_ARCH: aarch64 to the voxl and l4t image
  builds.
- Install ros-${ROS_DISTRO}-mavros-extras (generic dep; also provides the
  vision_pose plugin used by external-pose deployments).

Robot name resolution:
- .bashrc now follows a pre-set ROBOT_NAME (e.g. injected by docker compose)
  instead of always overriding it from the container/hostname mapping. The
  bws() flock build lock is retained.
- default_robot_name_map.yaml catch-all fallback maps to unknown_robot (valid
  ROS namespace token) instead of unknown-robot.

Version bumped 0.19.0-alpha.5 -> 0.19.0-alpha.6 for the version-increment gate.

Note: the trajectory_controller/trajectory_library robustness fixes originally
listed for extraction are already present on develop (PR #365), so they are not
included here.

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>

* l4t deployment fixes: make the Jetson profile build + boot on real hardware (#371)

* feat(l4t): make robot-l4t deployment knobs overridable + document name resolution

Parametrize the robot-l4t compose service so a single service covers real
deployments without editing compose:
- AUTONOMY_ROLE and FCU_URL are now ${VAR:-default} overridable (and FCU_URL is
  unquoted so the literal serial path reaches mavros).
- Rosbag output path is BAG_STORAGE_PATH-overridable.

Update the configure-multi-robot skill to reflect the honor-pre-set-ROBOT_NAME
guard (#370): document pinning ROBOT_NAME in an override for a single real robot,
the never-on-the-shared-service caveat, and the unknown_robot fallback fixes by
topology.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* feat(l4t): add site-agnostic l4t-px4-realrobot override template

Deployment override for a single real PX4 robot on a Jetson (aarch64/l4t).
Surfaces the common knobs at the top with sensible defaults: ROBOT_NAME pinned
directly (single-robot shortcut honored by .bashrc), FCU_URL, AUTONOMY_ROLE,
BAG_STORAGE_PATH, and RECORD_BAGS. Mocap-agnostic — NatNet/external-vision
settings are added by a separate optitrack override.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* fix(l4t): entrypoint passthrough + ZED SDK 5.2; document build gotchas

Two real-hardware build fixes for the Jetson profile:
- Dockerfile.l4t-stack-base: overwrite dustynv's /ros_entrypoint.sh with an
  `exec "$@"` passthrough. Its prebuilt source-ROS libs (fastcdr 2.2.5) were
  shadowing the apt Jazzy (2.2.7) that Dockerfile.robot layers on, crashing
  apt-built nodes like mavros with symbol-lookup errors under tmux autolaunch.
- zed/Dockerfile.zed-l4t: bump ZED SDK 4.2 -> 5.2 and move the coupled ROS deps
  together (zed_msgs 5.2.1, point_cloud_transport(_plugins) 4.x, add backward_ros).

Document both gotchas in the docker-build-profiles skill, and correct the stale
unknown-robot -> unknown_robot in the robot_identity reference doc.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.7

Version-increment gate: bump above develop's 0.19.0-alpha.6 and record the l4t
deployment changes in the changelog.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

---------

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>

* Test infra rework: YAML-driven unit-test collection + integration tier (#372)

* feat(l4t): make robot-l4t deployment knobs overridable + document name resolution

Parametrize the robot-l4t compose service so a single service covers real
deployments without editing compose:
- AUTONOMY_ROLE and FCU_URL are now ${VAR:-default} overridable (and FCU_URL is
  unquoted so the literal serial path reaches mavros).
- Rosbag output path is BAG_STORAGE_PATH-overridable.

Update the configure-multi-robot skill to reflect the honor-pre-set-ROBOT_NAME
guard (#370): document pinning ROBOT_NAME in an override for a single real robot,
the never-on-the-shared-service caveat, and the unknown_robot fallback fixes by
topology.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* feat(l4t): add site-agnostic l4t-px4-realrobot override template

Deployment override for a single real PX4 robot on a Jetson (aarch64/l4t).
Surfaces the common knobs at the top with sensible defaults: ROBOT_NAME pinned
directly (single-robot shortcut honored by .bashrc), FCU_URL, AUTONOMY_ROLE,
BAG_STORAGE_PATH, and RECORD_BAGS. Mocap-agnostic — NatNet/external-vision
settings are added by a separate optitrack override.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* fix(l4t): entrypoint passthrough + ZED SDK 5.2; document build gotchas

Two real-hardware build fixes for the Jetson profile:
- Dockerfile.l4t-stack-base: overwrite dustynv's /ros_entrypoint.sh with an
  `exec "$@"` passthrough. Its prebuilt source-ROS libs (fastcdr 2.2.5) were
  shadowing the apt Jazzy (2.2.7) that Dockerfile.robot layers on, crashing
  apt-built nodes like mavros with symbol-lookup errors under tmux autolaunch.
- zed/Dockerfile.zed-l4t: bump ZED SDK 4.2 -> 5.2 and move the coupled ROS deps
  together (zed_msgs 5.2.1, point_cloud_transport(_plugins) 4.x, add backward_ros).

Document both gotchas in the docker-build-profiles skill, and correct the stale
unknown-robot -> unknown_robot in the robot_identity reference doc.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.7

Version-increment gate: bump above develop's 0.19.0-alpha.6 and record the l4t
deployment changes in the changelog.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* test(infra): collect co-located unit tests via the package list + integration tier

Unit tests are defined by tests/colcon_unit_test_packages.yaml: conftest.py resolves
each listed package to its <pkg>/test dir and collects the non-linter test_*.py files
under --import-mode=importlib (set in pytest.ini), marking each `unit` by path. ament
lint files are skipped (they run under colcon test). Removes two now-unnecessary files
under tests/robot/; the package test/ dirs are collected directly.

Also add an integration test tier: tests/integration/ + `integration` mark + a shared
robot_autonomy_stack fixture (robot-desktop container, no sim/GPU), slotted into
_MODULE_ORDER between build_packages and the sim tiers.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* docs(testing): describe unit tests as co-located and listed in the package YAML

Update the add-unit-tests and run-system-tests skills, AGENTS.md, and the unit-testing
docs: adding a unit test is "list the package in colcon_unit_test_packages.yaml", and the
source lives in the package's own test/ dir. Document the `integration` mark/tier.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.8

Version-increment gate: bump above develop (0.19.0-alpha.6); alpha.7 is taken by the
l4t-deployment-fix PR. Record the test-infra changes in the changelog.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* refactor(tests): split unit-test discovery + session state into tests/harness/

Begin modularizing conftest.py (959 lines) by concern. Extract two self-contained
pieces into a new tests/harness/ package:
- harness/session.py: session-scoped mutable state (results dir, current pytest item,
  last subprocess output, logger) with setter/getter accessors. Hooks write it; helpers
  read it, so helper modules no longer reach into conftest globals.
- harness/discovery.py: unit-test discovery driven by colcon_unit_test_packages.yaml
  (repo_path, load_colcon_unit_test_config, colcon_test_robot_command, unit_test_dirs,
  unit_test_files, _is_unit_item).

conftest.py imports from harness and its hooks delegate to the session accessors; it
re-exports AIRSTACK_ROOT / colcon_test_robot_command / load_colcon_unit_test_config /
logger so existing `from conftest import ...` in the system tests keeps working
unchanged. Behavior-preserving (host-validated): `-m unit` still 14 passed / 152
deselected, 166 collected, same order. Follow-on: the commands/containers/metrics/sim
helpers and collection ordering move out the same way.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* refactor(tests): extract commands/containers/metrics/sim helpers into tests/harness/

Continue modularizing conftest.py. Move the subprocess/ros2 command helpers
(harness/commands.py), docker container + compute-usage + image helpers
(harness/containers.py), MetricsRecorder + get_metrics/current_test_id
(harness/metrics.py), and the sim target configs + ros2 topic sampling
(harness/sim.py) out of conftest.py.

conftest.py drops from 836 to 360 lines and re-exports the harness helper API
(`from harness import *`) so `from conftest import <name>` in the system tests +
sensor_probes keeps working unchanged. Behavior-preserving: -m unit still 14 passed /
152 deselected, 166 collected, same order. Remaining in conftest: pytest hooks,
collection ordering, and the airstack_env / robot_autonomy_stack fixtures.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* refactor(tests): extract collection ordering into tests/harness/collection.py

Final step of the conftest.py modularization: move test ordering — _MODULE_ORDER, the
per-module phase chains, _module_key, and the parametrize-id rewrite — into
harness/collection.py. conftest's pytest_collection_modifyitems hook now delegates to
collection.modify_items(items).

conftest.py is now 246 lines (from 959): pytest hooks + the airstack_env /
robot_autonomy_stack fixtures. All helpers live in tests/harness/ by concern (session,
discovery, commands, containers, metrics, sim, collection). Behavior-preserving: -m unit
still 14 passed / 152 deselected, 166 collected, unit → build → integration → sim order
unchanged.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

Also sync docs/skills to the tests/harness/ layout (AGENTS.md, tests/README.md,
tests/integration/README.md, run-system-tests + add-unit-tests skills, unit_testing +
end_to_end_testing docs): helpers, MetricsRecorder, the workspace globs, and _MODULE_ORDER
now point at tests/harness/ instead of conftest.py (still re-exported via conftest).

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(robot): pin pytest to 7.4.* so apt launch_pytest stays compatible

The builder-stage pip block pulled pytest >=8 transitively into /usr/local (copied
into the runtime image), shadowing Jazzy's apt python3-pytest 7.4. pytest 8 removed
the `path` argument from pytest_pycollect_makemodule, which apt's launch_pytest plugin
still declares — so every pytest invocation in the robot container aborted at plugin
registration. This broke `colcon test` for ament_python packages (e.g.
lidar_point_cloud_filter in test_colcon_test_robot), while ament_cmake gtest packages
were unaffected.

Pin pytest to Jazzy's version so the container is internally consistent and
launch_testing / launch_pytest remain usable for future launch-based tests. The test
runner (tests/docker) is a separate interpreter and keeps its newer pytest.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* fix(isaac-sim): clear LD_LIBRARY_PATH for PX4 ubuntu.sh so ca-certificates configures

The global ENV LD_LIBRARY_PATH puts isaac-sim's bundled libs
(.../isaacsim.ros2.bridge/jazzy/lib) on the linker path. Its older libcrypto.so.3
shadows the system one, so when the updated ca-certificates (20240203 →
20260601~24.04.1) runs its postinst `openssl`, it fails with
`version 'OPENSSL_3.0.9' not found`, aborting the apt transaction and failing the
isaac-sim image build (PX4 Tools/setup/ubuntu.sh, exit 100).

Clear LD_LIBRARY_PATH for that RUN only so apt/openssl use the system libcrypto;
the global ENV still applies to every other layer. Environmental break (new
ca-certificates × isaac-sim's stale bundled openssl) — not a code regression.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

---------

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
Co-authored-by: Cursor <cursoragent@cursor.com>

* Add waypoint_flight system test judged by a standalone track checker (#378)

* Add waypoint_flight system test judged by standalone track checker

New end-to-end acceptance test for planner integration/swaps:
takeoff -> ordered waypoint route -> land, per (sim, num_robots, iter).

- tests/system/test_waypoint_flight.py (mark: waypoint_flight): after
  takeoff, sends the route to the local planner's NavigateTask action
  as a nav_msgs/Path and captures odometry throughout; reuses the
  flight-cycle workers from test_fixed_trajectory.py (chain guard,
  takeoff/land, odom CSV capture).
- tests/waypoint_checker.py: standalone stdlib-only judge — the
  odometry track must pass within --waypoint-tolerance of every
  waypoint IN ORDER, each within --waypoint-timeout of the previous
  arrival. Success is defined purely on the odometry track (not the
  action result), so swapping the global or local planner leaves the
  judgment unchanged; the checker also runs outside the harness on
  any ros2 `topic echo --csv` odometry dump.
- Waypoints are relative to the robot pose at dispatch (x forward
  along heading, z up), so routes are spawn/sim agnostic. Default:
  10 m square at takeoff altitude.
- New pytest options: --waypoints, --waypoint-tolerance,
  --waypoint-timeout; mark registered in pytest.ini; docs in
  tests/README.md and AGENTS.md; VERSION 0.19.0-alpha.9 + CHANGELOG.

Metrics recorded per robot: waypoint_success, waypoints_reached,
navigate_action_success, route_time_sim_s, worst_closest_approach_m.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Calibrate waypoint_flight to validated stock behavior in Isaac Sim

Validated end-to-end against Isaac Sim + the stock stack (4/4 phases
pass in 3m20s; corners cut 3.75/5.13 m, final goal error 0.63 m).
Fixes found by flying:

- Path header frame: an empty frame_id crashed droan_gl (uncaught
  tf2::InvalidArgumentException in its plan TF transform); the goal now
  carries the frame from the odometry snapshot (fallback "map").
- Dense plan dispatch: sparse poses get corner-skipped by the local
  planner's distance-walking look-ahead; the route is now interpolated
  at 1 m from the current pose (mirrors real global-planner output).
- Route/tolerance semantics: the stack's contract is "reach the goal
  precisely, follow the corridor loosely" (droan_gl cost =
  deviation - path_distance cuts corners ~4-7 m). Split tolerances:
  intermediate corridor 15 m, final goal 2.5 m (new --goal-tolerance;
  NavigateTask's 1.5 m + tracking lag). Default route is now an open
  30 m square — NavigateTask succeeds on distance to the FINAL pose,
  so closed loops succeed instantly without flying (documented).
- Settle capture: the action succeeds on the tracking point, which
  leads the drone by up to the look-ahead distance (~10 m); capture
  now continues until the drone is stationary (max 30 s) so the goal
  approach is recorded. New metric: final_goal_error_m.
- waypoint_checker: closest_approach now reports the true minimum over
  the remaining track instead of the tolerance-boundary crossing
  (arrival stays first-crossing, ordering semantics unchanged).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Raise default waypoint route +10m to clear scene clutter

Validated on both sim backends with the identical default config
(open 30 m square climbing to ~20 m AGL):
- Isaac Sim: corners 5.67/5.72 m, final goal 0.28 m, 4/4 phases
- ms-airsim (Blocks): corners 5.93/5.67 m, final goal 0.89 m, 4/4

At the old takeoff-altitude route the drone collided with a Blocks
obstacle (disparity was streaming, so DROAN had perception — the
corner-cut diagonals leave the forward stereo's coverage). This test
judges route-following, not obstacle avoidance, so the default route
flies above the clutter; documented in the option help and README.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Add waypoint_flight screenshots from validation runs

Captured mid-route during the validated flights: Isaac Sim viewport
with the drone on the square route, ms-airsim Blocks with the drone
clearing the obstacle field (collision count 0), and the Foxglove GCS
dashboard showing the planned path, expanded obstacle voxels, robot
task panel, and live stereo feed. Embedded in the waypoint section of
tests/README.md.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>

* Add feature-notebook workflow: per-feature design specs + test results feeding PRs (#381)

* Add feature-notebook workflow: local design specs + test results per feature

Every feature a coding agent implements now gets a numbered entry under
notebook/ (gitignored, local-only): a design_spec.md written before coding
(problem context from the session, proposed implementation with per-section
DESIGN/TODO / WIP / DONE status labels, lettered test plan) and a results/
tree with per-section raw artifacts plus a self-contained results_summary.md
(embedded tables + figures) that populates the feature's PR description.

- New skill .agents/skills/use-feature-notebook with SKILL.md and
  design_spec / results_summary templates
- AGENTS.md: skill registry row, notebook-first Agent Workflow Example,
  new "Feature Notebook" section
- .gitignore: /notebook/

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Bump version to 0.19.0-alpha.10

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Document the feature notebook workflow under Development docs

Adds docs/development/intermediate/feature_notebook.md (directory layout,
5-step workflow, status labels, local-only rule, notebook → PR flow), wires
it into the mkdocs nav under Development > Intermediate Tutorials >
Contributing, and lists it in the Development index.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>

* Remove stray files

* Robot deployment fixes: bag recording + adding warning for robot-identity failure (#377)

* make RECORD_BAGS actually reach the bag recorder

LOG_CONFIG selects which topic set in logging_bringup/config to record, default
log.yaml.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* warn when the robot identity fails to resolve

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.12

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fixed comments and documentation

* fix the bag recording status bridge direction

It was bridged gcs -> robot, the same direction as the command it answers, so
status never reached the GCS and the rqt Recording: label stayed blank.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fix the exclude flag so the main bag section records

ros2 bag record renamed --exclude to --exclude-regex, and the old name is now an
ambiguous prefix of four options, so argparse rejected the command and any section
using exclude: recorded nothing.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* restore the bags .gitignore files

#318 dropped robot/bags/.gitignore and gcs/bags/.gitignore while moving a dozen
others; nothing has covered recorded bags since.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Opus 5 <noreply@anthropic.com>

* ci: land OSMO ephemeral runners and system-test harness on develop (#382)

* ci(orchestrator): migrate ephemeral CI runners from OpenStack to NVIDIA OSMO

Replace the OpenStack-Nova spawn/reap backend with OSMO workflow submission. The GitHub side is unchanged (self-hosted/airstack-ephemeral labels, single-use JIT runner tokens, same-repo fork guard) and the one-job-per-worker destroy-after model is preserved; only the spawn target moved from creating a Nova VM to submitting an OSMO workflow.

orchestrator.py: submit/query/cancel/list via the osmo CLI, job_id -> workflow_id state, re-login-on-auth-failure, orphan sweep via osmo workflow list; drop floating-IP/boot-volume/placement/keypair/security-group logic.

runner.Dockerfile + runner-entrypoint.sh + runner-workflow.yaml.j2: prebaked privileged docker-in-docker + GPU GitHub runner image/task (replaces cloud-init.yaml.j2).

config.example.yaml, setup.sh, airstack-orchestrator.service, requirements.txt: OSMO service-account token auth, install the osmo CLI, drop openstacksdk. Docs (AGENTS.md, tests/README.md, orchestrator README) updated to OSMO.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci(orchestrator): pin AirLab OSMO JSON keys and runner image path

Resolve uuid/live name after submit (OSMO returns name-only + suffix),
default config to the Keycloak-backed airstack pool and Harbor runner
image, and add scripts to build/push airstack-ci-runner on OSMO DinD.

Co-authored-by: Cursor <cursoragent@cursor.com>

* docs(ci): document the OSMO-backed CI/CD pipeline

Fills in the empty ci_cd.md stub with an end-to-end guide to how CI runs
the full AirStack stack on ephemeral OSMO GPU pods: architecture and job
lifecycle diagrams, runner pod anatomy, the three trigger paths, what
each pytest mark catches, the metrics regression gate, the security
model, and layer-by-layer troubleshooting.

Adds the page to the mkdocs nav (it was previously unreachable) and
cross-links it from tests/README.md and the testing index.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): repair Docker builds on OSMO ephemeral runners

Every build_docker and build_packages test failed on the OSMO backend
because the inner dockerd kept its data-root on the pod's overlayfs
rootfs. Linux rejects a directory on overlayfs as an overlay upperdir,
so image pulls still succeeded -- containerd unpacks layers with plain
writes -- while every build step needing a real mount died with
"mount source: overlay ... err: invalid argument", surfacing as
unrelated-looking apt-get and WORKDIR failures.

runner-entrypoint.sh now picks a storage backend by attempting a real
overlay mount rather than trusting the filesystem type, preferring a
loopback ext4 data-root (real overlay2, sparse, dies with the pod) and
falling back to a pod-mounted filesystem, fuse-overlayfs, then vfs.
vfs is a last resort only: it copies the whole filesystem per layer and
would exhaust the storage request on the sim images.

Also bumps the GitHub Actions runner to 2.336.0, since 2.334.0 stops
being able to run jobs on 2026-08-10.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): seed PR Docker builds from a floating cache tag

Versioned cache_from entries always miss on PRs because VERSION is forced
up; add a stable cache_* tag published only by docker-build.yml so system
tests can reuse layers without writing the shared cache.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci(docker-build): retag unchanged images on VERSION bump

Skip full compose rebuilds when a service's content fingerprint matches
the previous versioned image label; registry-retag instead and only
rebuild services whose Docker inputs changed.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): parse quoted .env values before inline comments

docker_image_plan was feeding NUM_ROBOTS with a trailing comment into
compose config, which broke strconv.Atoi for deploy.replicas.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci(docker-build): build/push services sequentially

Publish successful images even when a sibling (e.g. isaac-sim) fails, and
still cosign whatever was retagged or pushed in the same run.

Co-authored-by: Cursor <cursoragent@cursor.com>

* chore: bump VERSION to 0.19.0-alpha.8 for retag validation

Seeded gcs/ms-airsim/robot images carry content-fingerprint labels; this
bump should registry-retag those digests without rebuilding.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): unblock isaac-sim PX4 apt and robot colcon pytest

Isaac's PX4 ubuntu.sh fails dpkg configure on the NVIDIA base; pre-fix
ca-certificates, drop software-properties-common, and skip NuttX/Gazebo
like ms-airsim. Pin pytest<8.1 and disable launch_testing for colcon
unit tests so ROS Jazzy's outdated pytest hook no longer aborts CI.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): pass colcon --pytest-args as separate tokens

A single quoted blob made pytest treat "-p no:launch_testing" as part of
the -m expression, which broke lidar_point_cloud_filter colcon tests.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): quote colcon pytest args through bash -ic

Nested single quotes around 'not linter' terminated the outer bash -ic
string early, so pytest saw 'not' as a path. Use shlex.quote for the
whole command and list-form pytest_args in the YAML.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): pass colcon pytest flags via PYTEST_ADDOPTS

colcon --pytest-args is a single nargs='*' option, so repeating it
dropped -p and pytest treated no:launch_testing as a file path.
Set PYTEST_ADDOPTS with docker exec -e instead.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): rename helper so pytest does not treat it as a hook

conftest functions named pytest_* are registered as hooks.
pytest_addopts_env caused PluginValidationError and exit code 3.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci: skip image-build for build_packages reruns

Pull and retag cache_* images instead of baking isaac/airsim on every
colcon/pytest iteration. /pytest --no-image-build does the same for
other marks. compose up --no-build when AIRSTACK_NO_IMAGE_BUILD=1.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): disable pytest plugin autoload for colcon tests

-p no:launch_testing is applied after setuptools entrypoints load, so
pytest 8.1+ still crashes on launch_testing's path= hook. Set
PYTEST_DISABLE_PLUGIN_AUTOLOAD so cache_* robot images (unpinned pytest)
can run lidar tests without a rebuild.

Co-authored-by: Cursor <cursoragent@cursor.com>

* fix(ci): skip lidar ament linters in package pytest config

PYTEST_ADDOPTS -m not linter never reached ament pytest, so copyright /
flake8 / pep257 still ran after the unit tests passed. Ignore those
modules in setup.cfg and collect_ignore.

Co-authored-by: Cursor <cursoragent@cursor.com>

* ci: default system tests to isaacsim only

PR-open and bare /pytest were sweeping both sims. Default --sim to
isaacsim; msairsim is opt-in via --sim msairsim.

Co-authored-by: Cursor <cursoragent@cursor.com>

---------

Co-authored-by: pvkumara <pkumara@andrew.cmu.edu>
Co-authored-by: Cursor <cursoragent@cursor.com>

* OptiTrack (1/3): robot-side NatNet client + PX4 external-vision fusion (#374)

* feat(perception): bring natnet_ros2 client up to the optitrack_emulation baseline

Take the natnet_ros2 package from #367 onto the reworked base: the C++ NatNet
client (natnet_ros2_node + client adapter + natnet_logic seam), the base
mavros_gp_origin and vision_pose_converter nodes, per-robot natnet_config profiles,
launch files, and the co-located C++/Python unit tests. natnet_ros2 is already
listed in tests/colcon_unit_test_packages.yaml, so the base's YAML-driven collection
picks up the updated unit tests directly — no proxy files.

Real-robot PX4 external-vision fusion (px4_param_setter, geoid-corrected origin,
EV-pose bounds) is layered on next.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* feat(natnet): real-robot PX4 external-vision fusion (mocap → EKF2)

Layer the Hummingbird real-robot fusion pipeline onto natnet_ros2 so an
OptiTrack-only drone (no GNSS/mag/baro) fuses mocap pose into PX4 EKF2:
- mavros_gp_origin_node: publishes a guarded synthetic GPS origin. On real HW,
  use_geoid_altitude feeds the egm96-5 geoid undulation (N ≈ 54 m at Lisbon) so
  mavros's ellipsoidal→AMSL conversion cancels and local z == OptiTrack z (fixes
  the ~36 m = 90 − 54 boot offset; see docs). Auto-skipped in sim.
- vision_pose_converter_node: rate-limited mocap → MAVROS vision_pose bridge.
- px4_params.yaml: the external-vision EKF2 param set.
- natnet_ros2.launch.py wires the bridges when a robot's vision_pose block is on.

px4_param_setter reworked into a **checker** (R3): auto_set=false by default — it
reads and *flags* FCU params that differ from the desired set instead of writing
them; on_mismatch=warn|halt (default warn). Set the params in QGroundControl; the
node is the pre-flight safety net. auto_set=true restores the legacy enforce path.

Excludes the duplicate vendored NatNet SDK (sensors/natnet_ros2) and deployment
override .envs.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* docs(natnet): PX4 external-vision setup guide + height-datum explainer

Move the PX4 external-vision setup guide into docs/ (was a repo-root markdown) and
wire it into the mkdocs nav under Perception. Adapt it to the reworked param
checker (auto_set default off; check-and-flag, not enforce), and add a "height
datum" section explaining the ~36 m local_z offset: AirStack's 90.0 ellipsoidal
world datum minus the egm96-5 geoid undulation (N ≈ 54 m at Lisbon) = 36 m; fixed by
publishing the geoid-corrected origin altitude so mavros's conversion cancels.
Documents why it's invisible in sim and why the shared 90.0 datum must not be
changed globally.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* feat(perception): point natnet launch include at the natnet_config schema

Refine the perception bringup comment on the LAUNCH_NATNET include so it points at
the per-robot natnet_config.yaml schema parsed by natnet_ros2.launch.py.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.14

* fix(natnet): make the NatNet client actually reachable + correct EV tuning

Three defects that together meant the OptiTrack client could never connect to
anything, in sim or on a real robot.

1. NATNET_SERVER_IP was unreachable config. natnet_config.yaml resolves it via
   $(env ...), but docker compose only injects variables named in a service's
   `environment:` block and no service declared it — not the compose files, not
   .env, not tests/system/test_optitrack_e2e.py. The client therefore always fell
   back to its hardcoded default (192.168.123.199), which is neither the in-sim
   emulator (172.31.0.200) nor any Motive host. Forwarded in
   robot-base-docker-compose.yaml, defaulting to the emulator so the sim path
   works unconfigured.

2. The tracked rigid body could never match. robot_1 pinned "Hummingbird" id
   1146 while the emulator streams "Drone" id 1, and the NatNet client filters
   incoming frames by NUMERIC id — a mismatch yields a connected client that
   silently never publishes. Body name/id now accept $(env ...) (expanded in
   _build_node_params, with the id still coerced to int) and default to the
   emulator's body; sites override via NATNET_BODY_NAME / NATNET_BODY_ID.

3. EV tuning was not the deployment-validated set. EKF2_EV_DELAY 8.0 -> 7.0 and
   EKF2_EVP_NOISE 0.01 -> 0.05. EKF2_EVP_NOISE is not marker precision: it also
   sets the innovation gate at EKF2_EVP_GATE (default 5) sigma, so 0.01 gave a
   5 cm gate that rejected legitimate mocap updates and refused to arm. 0.05 is
   a 25 cm gate, still far tighter than PX4's 0.1 default.

px4_params.yaml keeps the evidence inline, including two results that are
expensive to rediscover: raising EKF2_EV_DELAY to 50.0 measurably degrades
tracking (the negative best-fit time shift shows the estimate running ahead of
truth), and the drift-and-snap excursions were a 90 deg body-yaw offset in the
Motive rigid-body definition, not a gate problem — so the fix belongs in Motive,
never as yaw compensation in code.

Adds two unit tests covering body-field env expansion and the emulator-matching
defaults (natnet_ros2: 14 -> 16 passing).

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* add a real-robot OptiTrack deployment override

Mocap counterpart to l4t-px4-realrobot.env: same Jetson stack, plus the NatNet
server/body settings and LAUNCH_NATNET.

Carries the two things that are easy to get wrong and produce no error. The body id
must match Motive's streaming id, since the client filters frames numerically and a
mismatch just never publishes. And nothing writes the EKF2 external-vision parameters
to a real FCU — px4_param_setter only reads them back and warns — so they have to be
set once in QGroundControl.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* config bodies per robot profile; trim comments to the docs

The rigid body a robot tracks is now set only in its natnet_config.yaml profile,
keyed by ROBOT_NAME. NATNET_BODY_NAME / NATNET_BODY_ID are gone: a single global
env var cannot express per-robot values, so it blocked the multi-robot case the
profiles already handle. NATNET_SERVER_IP stays in the environment — one Motive
host serves every robot.

Comments across the package are cut back to what is not evident from the code.
The EKF2 tuning results that were buried in px4_params.yaml move into
docs/robot/px4_external_vision.md, which also had stale values (EV_DELAY 15.0,
EVP_NOISE 0.01) contradicting the config: that raising EV_DELAY measurably hurts
tracking, and that drift-and-snap was a Motive rigid-body yaw offset rather than
a gate problem.

Kept: the license header, and the note on why the SDK needs a reachability
pre-check before Connect().

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* put the mocap floor at the shared world datum

desired_floor_amsl 0.0 -> 36.0, the world datum (90 m ellipsoidal) expressed in
AMSL, so a mocap robot's reported global altitude agrees with sim and the GCS
instead of sitting at sea level. The published ellipsoidal origin works out to
~90 m, the datum itself.

local_position.z equals the OptiTrack height for any value of this parameter — it
only moves the global altitude. Reasoning lives in the external-vision doc, which
also now records that GeoPoint.altitude is ellipsoidal by contract, so AMSL must
not be sent here.

Not yet confirmed on hardware.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fail the build when the geoid dataset is missing

MAVROS constructs the egm96-5 geoid in its UAS core, before any plugin loads, and
throws std::invalid_argument if the dataset is absent — mavros_node terminates at
startup, so there is no MAVROS at all, GPS or mocap.

The image could ship without it. mavros' install_geographiclib_datasets.sh sends the
downloader's output to /dev/null and, on failure, prints "Error while installing" and
returns without a non-zero exit, so the RUN layer succeeded regardless. The tool it
calls, geographiclib-get-geoids, was also only a transitive dependency of ros-mavros
rather than something we pinned.

Now pins geographiclib-tools and asserts the file landed, so a failed download fails
the build. Verified against the shipped image: with the downloader broken the script
still exits 0, and the new test -f returns non-zero.

This is the dependency the OptiTrack external-vision path needs — mavros_gp_origin
resolves the geoid undulation with the same egm96-5 model — hence landing it here.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* abbreviated Dockerfile comment on geographic lib installation

* fix repo-root doc links in the external-vision guide

They resolved relative to docs/robot/, so mkdocs looked for
docs/robot/robot/ros_ws/... and warned on every one. Prefixed with ../../;
the file now builds warning-free.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* comment trim

* point the companion-link section at the PX4 docs

Section 3 documented MAVLink serial setup at length — MAV_n_CONFIG / SER_TEL2_BAUD
tables, wiring, USB-vs-TELEM2 comparison — all of which is standard PX4 setup that
PX4 documents better and keeps current. Replaced with links to the companion
computer, MAVLink peripherals, and serial configuration pages.

Kept the part PX4 does not cover: the Cube Orange USB CDC-ACM stall, which starves
EKF2 of vision updates and is why the companion link belongs on TELEM2. Four other
sections and the troubleshooting table point here for that symptom.

65 lines -> 19.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* frame section 4 around mavros_gp_origin, demote the 36 m note

Section 4 now leads with what mavros_gp_origin does — inject a synthetic global
position so PX4 will arm in modes that need one without GNSS — rather than
presenting the height datum as a peer topic.

The ~36 m offset becomes a note under it, scoped to real deployments and ending
with why sim never sees it (the geoid path is skipped under use_sim_time, and
sim's synthetic GPS is self-consistent with the spawn). Section 4b is gone; it
had no inbound references.

Dropped the "don't change the 90.0 globally" warning.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* reject an unknown connection_type instead of defaulting to unicast

validate_connection_type returned "unicast" for anything it did not recognise, so
"mutlicast" or "Unicast" produced a client that connected on the wrong transport
and then never received a frame — with only a warning to show for it.

It now throws std::invalid_argument naming the offending value, and the node turns
that into a fatal startup error rather than a warning it flies past. Case-sensitivity
is deliberate: accepting "Unicast" would mean the config silently disagrees with
itself.

Tests updated from fallback to throw, plus one asserting the message names the bad
value. 60 gtests pass.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* px4 external vision docs trim

* trim natnet node comments; note the latency figure is an estimate

Comment trims in natnet_ros2_node.cpp (no code change).

Records what cube_orange_latency_ms actually is: an estimate of the FCU hop, added
to a logged total and never fused. Only the transport half of EKF2_EV_DELAY is
measured, and that measurement starts at the NatNet server transmit, so Motive's
own capture pipeline is not in it either.

Also notes, for whoever retunes next, that the node stamps poses with its receive
time — so delay after that stamp does not belong in EKF2_EV_DELAY, which points
lower than 7.0 and matches the negative best-fit shift already recorded. Not chased
down; 7.0 flies. CameraMidExposureTimestamp would replace the estimate with a
measurement if it ever matters.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* trim the external-vision tuning notes

Replaces the two long tuning write-ups with a short troubleshooting tip (check the
Motive rigid-body definition first — x forward, z up) and cuts the latency section
back to what is measured versus estimated.

Fixed a dangling "see below" in the EKF2_EV_DELAY table row, which pointed at the
removed tuning result; the warning it carried is now stated inline.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* OptiTrack (2/3): NatNet server emulator + host integration tests (#375)

* feat(sim): add NatNet server emulator (protocol core) + register unit tests

The pure-Python NatNet server that emulates an OptiTrack Motive server so
natnet_ros2 can be driven without hardware. USD/Isaac-free — this is the protocol
+ server core (unicast server, data/model/server types, serializers, default
catalogs). The Isaac wrapper that maps a USD scene onto this server lands next.

Registers the emulator package's co-located unit tests via a `sim:` entry in
tests/colcon_unit_test_packages.yaml (base's simulation/**/<pkg>/test glob). The
root conftest now puts each unit-test package's import root on sys.path so
co-located tests import their package without a per-package conftest.py.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* test(natnet): host integration tests — emulator server → natnet_ros2

Drive the real natnet_ros2 client from the host NatNet server emulator and check
the drone pose reaches ROS at rate (single-body and multi-body profiles). No sim,
no GPU — uses the base's `robot_autonomy_stack` fixture + `integration` mark. The
Isaac-wrapper variant lands with the Isaac wrapper PR.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.15

* pack frame sections through one helper

* fix the labeled-marker struct format that raised on every pack

sMarker.pack used '<i5fhf', which expects five floats between ID and
params but is only given four (x, y, z, size), so every call raised
struct.error. Nothing hit it because the emulator streams rigid bodies
only, leaving nLabeledMarkers at 0 and the section never packed.

* ignore the editable-install egg-info in the emulator extension

* comment trim

* put helper-module dirs on sys.path for co-located unit tests

The emulator's tests import natnet_test_helpers from their own test/ dir,
which only resolved because test_natnet_integration.py inserts that path at
import time and pytest imports it during collection. Narrowing the run
(-m unit --ignore=integration) dropped that side effect and broke collection.

The dir is added only when it ships no conftest.py, so lidar_point_cloud_filter
keeps its parent-only path — putting its test/ dir on sys.path would shadow this
conftest as module `conftest` and break every `from conftest import`.

---------

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>

* OptiTrack (3/3): Isaac wrapper, mocap EV fusion in sim, and a Circle-trajectory e2e (#376)

* feat(sim): Isaac wrapper for the NatNet emulator (USD scene → server)

The Isaac integration layer that maps a live USD scene onto the NatNet server:
catalog/config/frames/manager/scene_setup/ui_extension/usd_bindings, the extension
manifest (config/), and the USD schema. Adds the natnet Pegasus launch scripts that
spawn the emulator alongside PX4 in Isaac Sim, the isaac unit tests (incl. a
float-tolerance loosen on the pose round-trip for float32/USD noise), and the
Isaac-wrapper host integration test. scipy + usd-core added for the emulator's
USD/pose-sampling tests.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* test(natnet): dedicated OptiTrack sim e2e (optitrack mark)

One dedicated Isaac bring-up (example_one_px4_pegasus_natnet_launch_script +
LAUNCH_NATNET=true) that asserts the full NatNet chain: emulator → natnet_ros2
pose_cov >= 5 Hz, then PX4 local_position alive (EKF2 fusing the vision). Its own
`optitrack` mark + _MODULE_ORDER slot — deliberately NOT a third parametrized sim,
so the generic liveliness/sensors/flight suites aren't re-run under NatNet.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* docs(natnet): emulator sim doc + optitrack-development skill

Add the NatNet emulator Isaac Sim documentation (docs/simulation/isaac_sim/
natnet_emulator.md) and the optitrack-development agent skill covering the emulator,
natnet_ros2, and the NatNet wire-protocol handshake.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

* chore: bump version to 0.19.0-alpha.16

* fix(sim): register the NatNet emulator via the Kit ext-folder

The Isaac launch scripts import `optitrack.natnet.emulator`, but Kit was only
pointed at the shared exts dir (`~/.local/share/ov/data/documents/Kit/shared/exts`),
where Dockerfile.isaac-ros installs pegasus.simulator at image build. The emulator
lives in the repo at simulation/isaac-sim/extensions/ and is never copied there, so
it was not a registered extension and the import depended on ambient sys.path.

Kit accepts repeated --ext-folder, so both standalone commands now pass the repo's
extensions dir as a second search root. Chosen over copying the extension into the
shared dir at build time because the repo tree is bind-mounted: emulator edits take
effect on relaunch instead of requiring an image rebuild.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* make the sim actually fuse the mocap stream

EKF2_EV_CTRL defaults to 0, and the isaac compose set no PX4 params at all, so PX4
discarded the vision entirely and flew on sim GPS. The emulator could stream
perfectly and change nothing.

PX4 SITL's rcS applies any PX4_PARAM_<NAME> env var at boot and Pegasus passes the
container env through, so no new mechanism is needed. Each entry defaults to PX4's
own default, read out of the firmware in this image — unset is an explicit no-op and
non-mocap sims are unaffected. They cannot be defined-but-empty: the rcS loop has no
empty-value guard.

Also hooks NATNET_BODY_ID in the single-drone launch script. The emulator hardcoded
streaming id 1 while the client reads the env var, so a real Motive id would desync
the two into a connected client that never publishes.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fly a circle on mocap fusion instead of asserting a topic exists

test_px4_fuses_vision claimed to prove EKF2 fused the external vision but only
waited for local_position/pose, which publishes off GPS regardless — it passed with
vision disabled.

The stack now comes up with GPS, baro and range aiding off, so mocap is the vehicle's
only position source, and the module flies the Circle trajectory. Sustained lateral
motion is where a wrong EV delay or a too-tight innovation gate shows up; a hover
would not reveal either. Cross-track error is scored by the same helpers the autonomy
benchmark uses, imported rather than reimplemented.

test_px4_fuses_vision is kept as the pre-flight gate — it now establishes only that an
estimate exists, and says so.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* enforce only the mocap circle flight on PR open

The pull_request branch passed no args, so opening a PR ran pytest's defaults: every
mark, both sims, all four trajectory types. Now it runs the one end-to-end flight that
covers the whole chain.

Every other suite is unchanged and still reachable on demand — /pytest comments,
workflow_dispatch inputs, and local airstack test.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* add an isaac natnet mocap override

Brings up the emulator plus PX4 on external-vision fusion in one command — the same
configuration test_optitrack_e2e.py uses, so the test environment is reproducible by
hand.

Sets PLAY_SIM_ON_START explicitly because the root .env ships it false: the scene then
loads paused, /clock never ticks, and every use_sim_time node sits frozen while the
stack looks healthy.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* install the natnet emulator as a real Kit extension

The natnet launch scripts died with ModuleNotFoundError: No module named 'optitrack'.
Pointing Kit's --ext-folder at the repo extensions dir was not enough — that only makes
Kit aware of an extension, it does not put the package on sys.path.

Handle it the same way pegasus.simulator already is: bake a copy into the Kit shared
exts dir and pip-install it editable, then bind-mount the repo copy over it so edits
stay live. The scripts now enable_extension() before importing, which registers the
extension and its omni.isaac.core / omni.usd dependencies.

The repo-extensions --ext-folder flag is dropped; the extension now lives in the dir the
image already searches.

Verified in a running container: extension starts, emulator serves on 172.31.0.200
:1510/:1511, and the robot sees /robot_1/perception/optitrack/drone/pose_cov at ~101 Hz
feeding vision_pose and PX4 local_position at ~32 Hz.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* set the streamed body in the script, not the environment

The emulator read NATNET_BODY_NAME / NATNET_BODY_ID from the environment to stay in
sync with the client. The client now takes its bodies from its per-robot profile in
natnet_config.yaml, so the env hook was asymmetric and, being global, could not
describe a multi-robot scene anyway.

Both are now constants in the launch scripts, with the pairing spelled out inline,
in the emulator sim doc, and in the optitrack-development skill — including that a
mismatched id fails silently: the client connects and never publishes.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* comment trim on isaac-sim docker compose

* point the isaac-sim env blocks at their documentation

* comment trim on editable installation of natnet emulator

* keep the full default test run on PR open

Narrowing the PR gate to `-m 'build_packages or optitrack'` also dropped the
unit tier — 155 tests, including the emulator's own suite, which colcon test
does not cover (it runs only the robot workspace packages).

The optitrack e2e needs no gate of its own: with no -m filter it is collected
like everything else, and it brings up its own mocap-EV stack via _E2E_ENV.
Only the heavy-mark classification stays, so /pytest -m optitrack still builds
sim images instead of taking the pull-only path.

* wait for a converged estimate before arming in the optitrack e2e

Gate on local_position/odom instead of /pose. odom goes live only once EKF2 has
converged and home is set, which is what PX4's arming preflight requires; /pose
fires earlier, and the takeoff dispatched in that window returned "failed to arm".
Both autonomy suites already gate on odom for this reason (test_px4_ready).

The gate alone is not sufficient under external vision: with GPS, baro and range
aiding off, PX4's heading and horizontal-position stability checks settle after
odom starts publishing — measured at ~26s past the gate. TakeoffTask does not
retry its own ARM, so retry here first.

* comment trim on optitrack e2e collection ordering

* comment trim on the PR-open test args

* rename the isaac natnet override to isaac-optitrack-simulation.env

* select PX4 SITL parameters with a named env_file

The isaac-sim service listed eleven PX4_PARAM_* entries, each defaulting to a
hardcoded copy of PX4's own default so that an unset value stayed a no-op — rcS
has no empty-value guard. Those copies can drift from firmware silently.

Replaced with env_file: ./px4-params/${PX4_PARAM_SET:-default}.env. default.env
is empty, so an unselected run injects nothing and PX4 keeps its firmware
defaults; external-vision.env holds the mocap set. An unknown name fails the
compose config rather than falling back.

Also corrects the natnet_emulator doc table, which described three robots, the
multi-drone script, and a SITL_PARAM_PROFILE variable that exists nowhere.

* comment trim in compose file

* trim verbose comments in the natnet sources

Shorten multi-line inline comments that explained rationale or compared the
chosen approach against alternatives. The longer explanations already live in
docs/simulation/isaac_sim/natnet_emulator.md, so the comments now state what
the code does and point there.

Limited to files this PR adds: the natnet launch scripts and the emulator's
isaac/ modules. The env files and the pre-existing launch script keep their
original comments.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* removed comment change

* drop the GPS origin change from the baseline pegasus launch script

example_one_px4_pegasus_launch_script.py is a pre-existing non-mocap script and
does not need to change for the NatNet emulator work, so restore it to develop.

The set_gps_origins call was also inert here: for a single drone spawned at the
world origin it computes (38.736832, -9.137977, 90.07), which is the Lisbon
default gps_utils already documents, and nothing in the Pegasus submodule reads
the PX4_HOME_LAT_<domain_id> vars it writes.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* assert the external-vision params actually reached the FCU

The rest of this module assumes PX4_PARAM_SET=external-vision took effect. If it
silently does not, EKF2_EV_CTRL stays 0 and EKF2_GPS_CTRL stays 7, the vehicle
flies the Circle on sim GPS, and every test still passes — the proof-by-elimination
in test_px4_fuses_vision collapses because the elimination never happened.

Read EKF2_EV_CTRL and EKF2_GPS_CTRL back off the FCU through the MAVROS param
plugin, so the check covers the whole chain: compose env_file -> container env ->
Pegasus -> PX4 rcS -> FCU. Runs before the flight tests so a param failure
short-circuits in seconds instead of after two 2400s timeouts.

Two params, not the full set: if these are right, PX4_PARAM_SET demonstrably
applied and the rest came with it. Matching is on the printed value line, not the
exit code — an unpulled param prints "Parameter not set." and still exits 0.

Verified against a live sim: passes on the real config, fails with distinct
messages for a wrong value and for a param that never appears.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* docs(natnet): publish the emulator page and correct the setup examples

Add the emulator doc to the nav as "MoCap Emulator" — it built and served but
was orphaned, so it was only reachable by knowing the URL, and the "See
docs/..." pointers in the code led somewhere unnavigable.

Fix the launch-script examples in the doc and the extension README. Both omitted
enable_extension(), which is the actual prerequisite: the package imports fine
because Dockerfile.isaac-ros pip-installs it, but the emulator's modules pull
omni.usd / omni.physx lazily, so Kit has to have the extension registered. The
doc also carried a sys.path.insert pointing at ../utils (where scene_prep lives)
that had nothing to do with the optitrack import. The README targeted
/World/drone1/base_link rather than the /body child the launch scripts stream.

Document that client registration does not survive a server restart: restart the
robot container after Stop/Start Server. Stopping and starting the simulation is
unaffected — frames are sampled on the physics step.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* feat(natnet): the extension owns the server, tied to the sim timeline

The Kit extension is the single owner of the NatNet server. It builds one from
the /World/NatNetInterface prim on Play and shuts it down on Stop, so the
server's lifetime matches the simulation and the panel reports its state rather
than controlling it.

Launch scripts author the interface prim before starting the timeline;
author_drone_natnet_interface writes the prim and returns the authored config.
Because the server is constructed on each Play, serverIp/ports/mode — bound into
the socket at construction — pick up whatever is authored at that point. Bodies,
up-axis and pose noise are re-read while running and need no rebuild.

The panel opens on the interface authored on the stage, so Save writes back what
is there; author_interface replaces the whole body set.

A client registers with the server instance it connects to, and natnet_ros2
handshakes only until its first success, so a client from an earlier run is
unknown to the server built by the next Play. Restart the robot container after
each Stop -> Play cycle; documented in natnet_emulator.md.

Not exercised against a live panel yet.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* docs trim

---------

Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>

* CI/CD Tuning PR - pytest collection bug fix (#384)

* docs(tests): align unit-test docs with the co-located layout

Unit test source moved into <package>/test/ and is collected from
colcon_unit_test_packages.yaml, but the surrounding documentation still described
the mirror-directory-and-proxy scheme that replaced. Six per-layer stubs under
tests/robot/ told authors to add tests in directories tests no longer live in, and
tests/sim/motive_emulator/README.md proposed a NatNet emulator that was built at
simulation/isaac-sim/extensions/optitrack.natnet.emulator/ instead. Remove them and
rewrite the two tree READMEs as signposts.

Correct the add-unit-tests and run-system-tests skills, which future agents read to
work in this area, on four points they had wrong:

- Running them. `pytest tests/` does not collect co-located unit tests — the
  injection in conftest.pytest_configure is skipped whenever a path is given on the
  command line. It reports "no tests collected" and exits 5, which reads as a
  failure but means nothing ran. `airstack test -m unit` and `cd tests && pytest -m
  unit` are the working forms; verified 155 passed vs exit 5.
- CI. No workflow runs unit tests. system-tests.yml invokes `pytest tests/`, and
  fires only on PR-open, /pytest, or workflow_dispatch.
- The mark. pytest_itemcollected applies @pytest.mark.unit by file location, so
  test sources should not declare it. The skill previously said "always decorate",
  which is where the redundant declarations came from.
- colcon. It runs only what a package's CMakeLists registers. natnet_ros2 has
  ament_add_gtest but no ament_add_pytest_test, so its Python tests run only under
  the root harness.

Also fixes a pytest_args example that would silently do nothing (`-m not linter`;
ament's pytest runner ignores -m via PYTEST_ADDOPTS, and the real value is []), and
the same stale layout claim in the testing docs and the emulator README.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* docs(tests): record how C++ and Python unit tests reach CI

C++ gtests run under colcon test, which CI executes inside the robot container via
the build_packages mark (test_build_packages.py::test_colcon_test_robot). Python unit
tests run under the root harness, which no workflow invokes.

Whether colcon test also picks up a package's Python tests depends on its build type:
lidar_point_cloud_filter is ament_python and exposes them via setup.cfg
(testpaths = test), so they run in both places; natnet_ros2 is ament_cmake and
registers only ament_add_gtest, so its Python tests run nowhere in CI.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fix(tests): collect co-located unit tests when the run is not narrowed

Unit-test source lives outside tests/, so pytest_configure appends it to the
collection args. That injection was gated on args_source != ARGS, which pytest sets
for any positional path — including `tests/`. The intent was that
`pytest tests/system/foo.py` should not drag in 155 unrelated tests, but the guard
could not tell narrowing from naming the whole suite, so CI's `pytest tests/`
collected 97 of 252 items and the Python unit tests ran nowhere.

Decide on the paths instead: a positional is broad when it names tests/ itself or an
ancestor, narrow otherwise. `pytest tests/` and `pytest .` inject; `pytest tests/system`,
a single file, and a node id do not. Node ids are split on `::` first, since only the
part before it addresses the filesystem.

`any` rather than `all` is deliberate — pytest_configure appends the co-located files
(narrow, absolute) to config.args, so `all` would flip the answer for anything
re-deriving it after that mutation. The decision is also stashed on config for the
contract test to read.

tests/meta/test_collection_contract.py pins the behaviour: a table over broad/narrow
invocations, a check that the command in system-tests.yml is classified broad (the
test that would have caught this), and a check that every discovered file produced
collected items. It lives under tests/ on purpose — co-located, it would stop being
collected at the same moment it stopped guarding anything.

Verified: `pytest tests/ -m unit` 0 -> 170 passed; `cd tests && pytest -m unit`
unchanged at 170; `pytest tests/system/test_liveliness.py` still collects 16.

Unit tests now run with every system-tests.yml invocation. That workflow's triggers
are unchanged and intentional — PR open, /pytest, workflow_dispatch — since the same
run drives the GPU system tests.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* docs(tests): explain why C++ and Python unit tests use different runners

The split was documented as a fact without its reason. A gtest is a binary compiled
against the package's headers and rclcpp, so it can only run where the ROS toolchain
is — colcon test inside the robot container, which build_packages reaches after
building with -DBUILD_TESTING=ON. Python unit tests stub ROS at the import boundary
and touch no ROS runtime, so they need neither a build nor a container, which is what
keeps the suite under a second.

State the invariant that follows: a Python test needing a live ROS node belongs in
tests/integration/ or tests/system/, not in a package test/ dir.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* test: run the collection contract tests with the fast tier

They are hermetic and they guard the collection of everything above them, so
running them after the GPU sim suites is backwards — a hung flight test would mean
they never execute. Rank them in _MODULE_ORDER right after the co-located unit
tests, ahead of system.test_build_docker.

Also drop the `from conftest import repo_path` in favour of harness.discovery,
which the module already imports from — one less thing between the test and the
function it needs.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

* fix(ci): make PR validation and metrics trustworthy

Run fast unit checks automatically, constrain host collection, and distinguish infrastructure failures from comparable simulation results.

---------

Co-authored-by: John <johnliuchs2022@gmail.com>
Co-authored-by: Claude Opus 5 <noreply@anthropic.com>
Co-authored-by: Pranav Kumara <pkumara@andrew.cmu.edu>

* docs(skills): require dates and timestamps in feature notebook entries

Add a 'date and timestamp everything' convention to the use-feature-notebook
skill: Date started / Last updated in design_spec.md, run timestamps on stored
test artifacts, and per-section run times in results_summary.md. Update both
templates accordingly and add a pitfall for undated documents.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Pre-RFC workflow cleanup: intent-based launch, readiness gates, launch-script dedup, truthful logs (#386)

* refactor(isaac): dedupe launch scripts into shared PegasusApp base

The six launch scripts were 80-90% copy-pasted boilerplate (extension
enabling, wait_for_stage, scene prep, spawn calls, run loop) that had
already drifted: livestream existed only in the *_one_* scripts (so the
isaac-sim-livestream service silently black-screened with multi scripts),
ISAAC_SIM_HEADLESS was honored only by the *_multi_* scripts, and
barebones_pegasus_launch.py (the documented template) crashed with a
NameError (os never imported).

pegasus_app.py now owns the skeleton once: create_simulation_app()
(livestream + headless env handling, uniform across all scripts),
extension enabling, world/env loading, scene prep, drone/sensor spawning
from config dicts, and the run loop. Scripts reduce to scenario
declarations plus hooks (pre_scene_prep/post_scene_prep/post_spawn).

Behavior preserved per script (spawn poses, prim/node names, sensor
offsets, NatNet bodies, GPS origins), with three deliberate fixes:
- ISAAC_SIM_HEADLESS and ISAAC_SIM_LIVESTREAM now work in every script
- barebones template runs again
- NATNET_BODY_NAME/NATNET_TARGET_NAME env overrides now work as the
  one-drone natnet script's docstring already claimed

example_multi_drone_scene_import keeps its historical ZED offset
[0.21, 0, 0.05] (drift vs the canonical [0.2, 0, -0.05] — now visible
and annotated instead of buried).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* feat(cli): intent flags on 'up', resolved-value preflight, and 'airstack ready'

airstack up learns intent flags that derive the coordinated env-var sets
users previously had to know by heart (they export leaf values only —
compose interpolation gives shell env precedence, so .env is untouched):

  --sim isaac|airsim   swap simulator profile + matching URDF
  --robots N           NUM_ROBOTS + auto-select one/multi Isaac script
                       (also natnet pair; warns on custom scripts)
  --headless           ISAAC_SIM_HEADLESS + MS_AIRSIM_HEADLESS + QT offscreen
  --play/--no-play     PLAY_SIM_ON_START
  --no-autolaunch      AUTOLAUNCH=false
  --wait               chain into 'airstack ready' after compose up
  --dry-run            print + validate the resolved config, start nothing

Every up prints the resolved launch config and dumps it to
.airstack/runs/<ts>/effective_config.env (gitignored; best-effort on
read-only checkouts).

Preflight now validates RESOLVED values (env > --env-file > .env),
fixing the historical guard bypass where 'up --env-file overrides/...'
was checked against .env only. New checks: NUM_ROBOTS>1 with the
single-drone Isaac script (previously a silent 3-containers-1-drone
failure) is a hard error; missing images are listed by name with an
image-pull hint before compose starts a multi-GB implicit build; missing
omni_pass.env / empty Pegasus submodule / docker<29 name-resolution are
surfaced on the host instead of dying invisibly inside tmux.
AIRSTACK_SKIP_PREFLIGHT=1 downgrades errors to warnings.

'airstack ready' (and 'up --wait') answers "can I press Takeoff yet?":
staged gates mirroring the system-test budgets — containers (120s) →
sim /clock (600s) → per-robot sentinel nodes (300s) → PX4 MAVROS
connected + local_position/odom streaming (300s, the EKF-converged
armable signal; connected alone fires ~25s early). --json for scripts;
per-gate failures name the container/tmux window to inspect.

tests/meta/test_launch_intent_contract.py pins the flag derivations,
guard behavior, and exit codes (runs under the unit mark).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* feat(docker): tee tmux pane output to container stdout

Every service runs its real workload inside tmux, so 'docker logs' /
'airstack logs' were empty by construction — colcon build failures,
Pegasus import errors, and scene downloads all landed in panes nobody
attaches to. tmux hooks in the shared .tmux.conf (mounted into robot,
gcs, isaac-sim, and ms-airsim containers) now pipe-pane every created
session/window/split to /proc/1/fd/1, making container logs truthful.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: fix launch-workflow drift against actual code behavior

Corrects statements the audit found wrong, and teaches the new flags:
- getting_started: sim comes up PAUSED by default (PLAY_SIM_ON_START=false
  in .env, docs claimed auto-play), operator UI is Foxglove not RViz
  (DEBUG_RVIZ=false by default), adds 'airstack ready' / --wait and
  --sim/--robots variants
- simulation index + isaac docker.md + key_concepts + docker_usage:
  ISAAC_SIM_SCENE does not exist — scene selection is
  ISAAC_SIM_SCRIPT_NAME (standalone) or ISAAC_SIM_GUI (USD path, non-
  standalone); defaults table now matches .env/compose (AUTOLAUNCH=true,
  PLAY_SIM_ON_START=false, ISAAC_SIM_USE_STANDALONE=true, 100 Hz physics)
- simulation index: NUM_ROBOTS=3 alone does NOT put 3 drones in Isaac —
  documents --robots (auto script switch) and the preflight guard
- docker_usage: the test service is robot-test, not autotest
- gcs user_interface: gcs service is not in the deploy profile (gcs-real is)
- ms-airsim: MAVROS connects on 14540+domain (24540+i is AirSim's own PX4
  channel), camera FOV default is 90 not 110, vehicles are robot_<i> not
  drone<i>
- AGENTS.md: airstack stop/build are not registered commands (down /
  image-build); documents the new up flags and ready
- .env: correct usage comment; PLAY_SIM_ON_START paused-by-default note

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* chore(release): bump VERSION to 0.19.0-alpha.18 and update CHANGELOG

Image inputs are unchanged (all edits are bind-mounted or host-side), so
docker-build should registry-retag rather than rebuild on merge.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs(sim): document PegasusApp launch-script authoring; re-teach stale skills

spawning_drones.md now documents the pegasus_app.PegasusApp base class as
the way to write a launch script: import-order contract, constructor
kwargs, the drone-config dict (incl. prim/node_name/sensor overrides),
hooks (pre_scene_prep/post_scene_prep/post_spawn), and which reference
subclass to study for scene-import and NatNet scenarios.
pegasus_scene_setup.md points at it and drops the false 'PLAY_SIM_ON_START
not supported in standalone mode' claim. docker_usage.md gains a 'Launch
flags and readiness' section (--sim/--robots/--headless/--play/--wait/
--dry-run, effective-config dumps, airstack ready).

The write-isaac-sim-scene skill was re-taught from scratch: it prescribed
copy-pasting a ~240-line skeleton whose API had drifted to non-runnable
(wrong add_zed_stereo_camera_subgraph signature, nonexistent
SIMULATION_ENVIRONMENTS keys, low-level Multirotor API no shipped script
uses). It now teaches scenario declaration on PegasusApp with an explicit
'do not copy-paste' rule. Other skills fixed where the old guidance became
wrong or footgun-inducing: integrate-module-into-layer ('airstack stop' is
not a command), test-in-simulation and configure-multi-robot (bare
NUM_ROBOTS=N up now fails preflight with the single-drone script — use
--robots), use-airstack-cli (new flags + ready in the reference),
optitrack-development (single-drone NatNet body names are env-overridable
now).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>

* Release 0.19.0 (#397)

Promote the 0.19 series (intent-flag launch workflow, airstack ready,
resolved-config preflight, OSMO ephemeral CI runners, OptiTrack
external-vision configurations, feature-notebook workflow) out of
pre-release: VERSION 0.19.0-alpha.18 -> 0.19.0; CHANGELOG [Unreleased]
promoted to [0.19.0] - 2026-08-22.

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: github-actions[bot] <41898282+github-actions[bot]@users.noreply.github.com>
Co-authored-by: Sebastian Scherer <basti@andrew.cmu.edu>
Co-authored-by: Cursor <cursoragent@cursor.com>
Co-authored-by: krrishj18 <krrishj@andrew.cmu.edu>
Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
Co-authored-by: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
Co-authored-by: John Liu <63010779+JohnYanxinLiu@users.noreply.github.com>
Co-authored-by: copilot-swe-agent[bot] <198982749+Copilot@users.noreply.github.com>
Co-authored-by: John <johnliuchs2022@gmail.com>
Co-authored-by: pvkumara <99618405+pvkumara@users.noreply.github.com>
Co-authored-by: pvkumara <pkumara@andrew.cmu.edu>
Co-authored-by: andrewjong <8121216+andrewjong@users.noreply.github.com>
Co-authored-by: krrishj18 <krrishj18@users.noreply.github.com>
```

+24706 / −3508 in 219 files:

- `.agents/skills/add-ros2-package/assets/package_template/setup.py`
- `.agents/skills/add-unit-tests/SKILL.md`
- `.agents/skills/bump-version-and-release/SKILL.md`
- `.agents/skills/configure-multi-robot/SKILL.md`
- `.agents/skills/docker-build-profiles/SKILL.md`
- `.agents/skills/integrate-module-into-layer/SKILL.md`
- `.agents/skills/optitrack-development/SKILL.md`
- `.agents/skills/run-system-tests/SKILL.md`
- `.agents/skills/test-in-simulation/SKILL.md`
- `.agents/skills/use-airstack-cli/SKILL.md`
- `.agents/skills/use-feature-notebook/SKILL.md`
- `.agents/skills/use-feature-notebook/assets/design_spec_template.md`
- `.agents/skills/use-feature-notebook/assets/results_summary_template.md`
- `.agents/skills/write-isaac-sim-scene/SKILL.md`
- `.airstack/modules/osmo.sh`
- `.airstack/modules/ready.sh`
- `.env`
- `.github/orchestrator/README.md`
- `.github/orchestrator/airstack-orchestrator.service`
- `.github/orchestrator/build-and-push.sh`
- `.github/orchestrator/build-runner-on-osmo.yaml`
- `.github/orchestrator/cloud-init.yaml.j2`
- `.github/orchestrator/config.example.yaml`
- `.github/orchestrator/orchestrator.py`
- `.github/orchestrator/requirements.txt`
- `.github/orchestrator/runner-entrypoint.sh`
- `.github/orchestrator/runner-workflow.yaml.j2`
- `.github/orchestrator/runner.Dockerfile`
- `.github/orchestrator/setup.sh`
- `.github/workflows/docker-build.yml`
- `.github/workflows/scripts/docker_image_plan.py`
- `.github/workflows/system-tests.yml`
- `.github/workflows/unit-tests.yml`
- `.gitignore`
- `AGENTS.md`
- `CHANGELOG.md`
- `airstack.sh`
- `common/.tmux.conf`
- `common/ros_packages/logging/bag_recorder_pid/bag_record_pid/bag_record_node.py`
- `common/ros_packages/logging/logging_bringup/launch/logging.launch.xml`
- … 179 more

## 385ca6854c — 2026-08-22 — Andrew Jong — KEYWORD fixed,fixes

**docs: deep standalone-snapshot audit — RFC scrub, motivation-first openings, code-truth corrections**

https://github.com/castacks/AirStack/commit/385ca6854cfb7989fafdabb0ad9e11708aafd687

```
Four-domain sweep of every rendered page (94 docs pages, stack READMEs,
nav'd package READMEs, tests/orchestrator READMEs, the generated module
catalog and its sources). ~100 RFC/discussion citations removed —
designs now described on their own terms; change-relative narration
relocated to the versioned Release Notes; pages/sections open with
motivation before mechanics.

Factual rot corrected against code, highlights: phantom GCS control
panel page rewritten for the real Foxglove surface; behavior-tree
framework docs deleted (packages removed in an earlier release — the
behavior layer is drone_safety_monitor); bag recording is NOT
auto-triggered at takeoff; tracking_point/trajectory topic types
corrected; DDS-router allowlist table regenerated from the real config;
nonexistent CLI commands/options (format, test --path/--filter),
robot_bringup, behavior_tree_example, ISAAC_SIM_GUI=false, PX4 Hz
defaults, scene paths, bridge.yaml phantom, Jetson install flow
(./configure.sh -> airstack setup), dead JWT-signed video embed.

Module pipeline: gen_docs_catalog.py strings de-RFC'd + install snippet
fixed (module compose overlay is auto-included); catalog regenerated
(0 RFC mentions); registry fixture snapshot synced; asm_* repos audited
in lockstep and asm_macvo re-pinned to 431d7faf (also fixes its test
stack's dispatcher recursion and stale CI-gap comments).

Gates: unit 397 passed / 7 skipped; mkdocs build --strict clean.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+773 / −814 in 105 files:

- `.claude/settings.json`
- `.github/orchestrator/README.md`
- `.github/orchestrator/cloud-init.yaml.j2`
- `README.md`
- `common/ros_packages/logging/bag_recorder_pid/README.md`
- `docs/README.md`
- `docs/about.md`
- `docs/development/advanced/ai_agent_guide.md`
- `docs/development/beginner/airstack-cli/docker_usage.md`
- `docs/development/beginner/airstack-cli/index.md`
- `docs/development/beginner/development_environment.md`
- `docs/development/beginner/key_concepts.md`
- `docs/development/development_environment.md`
- `docs/development/fleets.md`
- `docs/development/intermediate/docker-build-profiles.md`
- `docs/development/intermediate/documentation.md`
- `docs/development/intermediate/testing/ci_cd.md`
- `docs/development/intermediate/testing/end_to_end_testing.md`
- `docs/development/intermediate/testing/unit_testing.md`
- `docs/development/module_ci.md`
- `docs/development/modules.md`
- `docs/development/stacks.md`
- `docs/gcs/docker/index.md`
- `docs/gcs/foxglove.md`
- `docs/gcs/usage/user_interface.md`
- `docs/gcs/waypoints_and_geofences.md`
- `docs/getting_started/modular_airstack.md`
- `docs/modules/dfm2_disturbances.md`
- `docs/modules/index.md`
- `docs/modules/macvo.md`
- `docs/modules/optitrack.md`
- `docs/real_world/HITL/index.md`
- `docs/real_world/data_offloading/index.md`
- `docs/real_world/installation/index.md`
- `docs/robot/autonomy/behavior/behavior_executive.md`
- `docs/robot/autonomy/behavior/behavior_tree.md`
- `docs/robot/autonomy/behavior/index.md`
- `docs/robot/autonomy/dds_router.md`
- `docs/robot/autonomy/global/index.md`
- `docs/robot/autonomy/global/planning/index.md`
- … 65 more

## 277b6eb397 — 2026-08-22 — Andrew Jong — KEYWORD fix

**fix(tests): --stack accepts <name>[:<entry>] like the CLI; re-pin asm_macvo past its stale-import fix**

https://github.com/castacks/AirStack/commit/277b6eb3978c6468ccfb8adaa6ae43f8d35ffcae

```
The harness hardcoded AIRSTACK_STACK_ENTRY=stack, so a split stack
(lite_offload_global has only onboard/offboard entries) dispatched to a
nonexistent entry and stranded the launch after the dispatcher preamble
— the wiring sentinel gate caught it (mavros/trajectory_control never
appeared). --stack now parses the same <name>[:<entry>] syntax as
airstack up; airstack_env carries stack (entry stripped, so goldens and
doctor lookups keep keying on the folder) plus a new stack_entry.

asm_macvo re-pinned to 269ffc0b: macvo_node imported the trunk-deleted
sensor_interfaces srv it never used (camera params arrive via a plain
CameraInfo subscription) and crashed at startup — the full_macvo wiring
gate caught the missing node.

Wiring gates all green: full_default, full_droan_cpu, full_macvo,
lite_default (2 passed each), lite_offload_global:onboard (2 passed).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+21 / −10 in 2 files:

- `stacks/full_macvo/modules.repos`
- `tests/conftest.py`

## 002dd2366d — 2026-08-22 — Andrew Jong — KEYWORD failed,fix

**fix(simple-sim): source ROS Jazzy, not Humble — sim never started since the Jazzy migration**

https://github.com/castacks/AirStack/commit/002dd2366d341b3e0402bf8a454949c35589a9fd

```
The mounted bashrc still sourced /opt/ros/humble/setup.bash (gone since
the Jazzy migration), so the at-startup colcon build of the sim
workspace failed on ament_cmake and no sim node ever launched. Source
jazzy in the bashrc (guarding the not-yet-built install/setup.bash) and
explicitly in the compose startup command so the launch chain does not
depend on the interactive rc file.

Caught by the new simple_sim smoke mark on adoption: 4/4 passed, sim
publishes /clock ~25s after up.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+4 / −3 in 2 files:

- `simulation/simple-sim/docker/bashrc`
- `simulation/simple-sim/docker/docker-compose.yaml`

## 96ad668eee — 2026-08-22 — Andrew Jong — KEYWORD fixed

**chore(docker): dependency purge — robot image sheds unused apt/pip deps (-152MB)**

https://github.com/castacks/AirStack/commit/96ad668eee9d66765fc075e984ca6c1f86be01da

```
Audit ruling 14 (+7). Removed from Dockerfile.robot: libcgal-dev;
pips lxml, pkgconfig, pygments, toml, six, psutil, pymavlink, tqdm,
rich, pillow; ros-jazzy-foxglove-bridge (GCS owns the bridge) and its
8767-8787 port range from the robot compose service.

KEPT deliberately: libglm-dev/libglfw3-dev/assimp/opengl — droan_gl
uses them, and they are now DECLARED in its package.xml plus an
explicit named apt block in the runtime stage (previously they rode in
implicitly). matplotlib==3.8.4 and scipy kept (in-tree consumers).

Validated: image builds; 44/44 packages colcon-build in-container.
6.06GB -> 5.91GB. Module layers must declare their own runtime deps —
asm_macvo's missing rich/tqdm surfaced by this purge, fixed module-side.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+39 / −22 in 3 files:

- `robot/docker/Dockerfile.robot`
- `robot/docker/docker-compose.yaml`
- `robot/ros_ws/src/local/planners/droan_gl/package.xml`

## e5889c0640 — 2026-08-24 — Andrew Jong — KEYWORD error,fixed,wrong

**feat(sim): airstack up --scene <shortname> — simulator-agnostic scene selection**

https://github.com/castacks/AirStack/commit/e5889c06408ead7b14f1dfd15b66e6f8d00453da

```
One flag maps a scene shortname to whatever the active simulator understands,
from a new catalog (simulation/scenes.yaml, resolved host-side by
simulation/resolve_scene.py). Unknown or wrong-simulator scenes error with a
per-simulator availability table; without --scene the effective launch config
is byte-identical to before.

- Isaac: exports ISAAC_SIM_SCENE (+ ISAAC_SIM_STAGE_SCALE) — a Pegasus
  SIMULATION_ENVIRONMENTS key or a Nucleus/HTTP USD URL. The example launch
  scripts and fleet_spawn.py resolve their scene from env
  (pegasus_app.resolve_scene_from_env) instead of hardcoding env_url;
  --scene overrides a fleet's sim.scene with a logged note. Catalog includes
  the 15 Pegasus entries plus 6 guest-readable AirLab stages under
  omniverse://airlab-nucleus.andrew.cmu.edu:443/Public/AirStack/Stages/,
  each with stage_scale mirroring its layer's metersPerUnit.
- MS AirSim: exports MS_AIRSIM_SCENE (a fetch_scene.sh key); the entrypoint
  auto-fetch, previously Blocks-only, fetches any catalog scene, and an
  interactive `airstack up` asks before a multi-GB download.
- fetch_scene.sh catalog corrected to the UE4 binaries that actually ship in
  the AirSim v1.8.1 release: forest/soccerfield/building99 removed (never
  published or 0-byte), airsimnh fixed to AirSimNH.zip, africasavannah and
  msbuild2018 added; the entrypoint glob-resolves the UE launcher after
  extraction.
- Docs: new docs/simulation/scenes.md (catalog, env vars, public-stage URLs,
  add-a-scene guide), simulation index + MS AirSim quick start updated,
  release-notes entry under 0.20.0.

Validated: resolver contract + CLI dry-run matrix; Isaac headless run loaded
the ConstructionSite stage from Public Nucleus (682 colliders, drone spawned,
PX4 node created); MS AirSim blocks run reached PX4 "Ready for takeoff" with
the bridge publishing /clock; msbuild2018 exercised the full download →
extract → boot pipeline.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+663 / −27 in 16 files:

- `.env`
- `airstack.sh`
- `docs/release_notes/index.md`
- `docs/simulation/index.md`
- `docs/simulation/scenes.md`
- `mkdocs.yml`
- `simulation/isaac-sim/docker/docker-compose.yaml`
- `simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/launch_scripts/fleet_spawn.py`
- `simulation/isaac-sim/launch_scripts/pegasus_app.py`
- `simulation/ms-airsim/assets/scenes/fetch_scene.sh`
- `simulation/ms-airsim/docker/docker-compose.yaml`
- `simulation/ms-airsim/docker/entrypoint.sh`
- `simulation/resolve_scene.py`
- `simulation/scenes.yaml`

## 5c0665fec6 — 2026-08-24 — Andrew Jong — KEYWORD fix

**fix(cli): config nucleus blank input no longer clobbers an existing token**

https://github.com/castacks/AirStack/commit/5c0665fec67b1fcc06fdee4dbb0d9cdcb721ae78

```
The prompt says "leave it blank to skip", but the blank path
unconditionally copied the guest template over omni_pass.env, silently
wiping a saved Nucleus API token and breaking Isaac Sim asset auth.
Blank input now keeps an existing file untouched and only writes the
guest defaults when no file exists yet (first-time setup unchanged).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+6 / −1 in 1 files:

- `.airstack/modules/config.sh`

## f358273a70 — 2026-08-24 — Andrew Jong — KEYWORD fixes

**feat(cli): consolidate osmo/config/image commands into command groups**

https://github.com/castacks/AirStack/commit/f358273a70b90e97dc010bd60416454174ab80c4

```
airstack help shrinks from 17 flat rows to 3 groups, each with a full
'airstack help <group>' reference page:

- airstack osmo   setup|up|logs|ide|webrtc|foxglove|down
- airstack config all (default)|isaac-sim|nucleus|git-hooks
- airstack images list (default)|build|push|pull|delete|rm  (rm absorbs rmi)

Bare 'airstack config' still runs all tasks and bare 'airstack images'
still lists, so no-argument behavior is unchanged. The images dispatcher
takes the first NON-FLAG argument as the subcommand so CI's
'./airstack.sh --progress=quiet images pull ...' flag-first call order
keeps working.

Every old spelling (osmo:up, config:nucleus, image-build, rmi, ...) is
kept as a deprecated alias that warns and forwards, hidden from the help
listings via a new COMMAND_HIDDEN registry map.

Call sites and docs updated to the new spellings: CI workflows,
tests/system/test_build_docker.py, AGENTS.md, .airstack/README.md, the
CLI references, the OSMO tutorial, getting-started/ms-airsim/build docs,
and the agent skills. The extending-the-CLI guide now teaches the
dispatcher + COMMAND_HIDDEN pattern; the pytest flag --no-image-build
keeps its name. Also fixes a stale osmo/README.md anchor into the OSMO
tutorial.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+451 / −229 in 25 files:

- `.agents/skills/docker-build-profiles/SKILL.md`
- `.agents/skills/run-system-tests/SKILL.md`
- `.agents/skills/use-airstack-cli/SKILL.md`
- `.agents/skills/write-isaac-sim-scene/SKILL.md`
- `.airstack/README.md`
- `.airstack/modules/config.sh`
- `.airstack/modules/osmo.sh`
- `.github/workflows/module-system-tests.yml`
- `.github/workflows/system-tests.yml`
- `AGENTS.md`
- `airstack.sh`
- `docs/development/advanced/airstack-cli/extending.md`
- `docs/development/airstack-cli/index.md`
- `docs/development/beginner/airstack-cli/index.md`
- `docs/development/intermediate/docker-build-profiles.md`
- `docs/development/intermediate/testing/ci_cd.md`
- `docs/getting_started/index.md`
- `docs/getting_started/modular_airstack.md`
- `docs/simulation/ms-airsim/docker.md`
- `docs/simulation/ms-airsim/index.md`
- `docs/tutorials/airstack_on_osmo.md`
- `osmo/README.md`
- `robot/docker/docker-compose.yaml`
- `tests/conftest.py`
- `tests/system/test_build_docker.py`

## 5d367c8dc4 — 2026-08-24 — Andrew Jong — KEYWORD fails

**feat(sync): warn when airstack.yaml release: drifts off the .env VERSION line**

https://github.com/castacks/AirStack/commit/5d367c8dc4aaeba3e7e2f63057ac36d1d21781c2

```
release: is informational until registry release-set resolution lands
(RFC #379 §7), so nothing caught it rotting — it was authored as
0.19.0-alpha while .env VERSION was already on the 0.20.0-alpha line.
airstack sync now warns (never fails) when VERSION no longer sits on
the declared release line, and the stale value is bumped to match.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+17 / −3 in 3 files:

- `.airstack/modules/sync.sh`
- `airstack.sh`
- `airstack.yaml`

## 2f11a8cb56 — 2026-08-24 — Andrew Jong — KEYWORD failed,fails,fix,issue

**fix(bringup): loud AUTOLAUNCH FAILED banner instead of silent tmux prompt (#402)**

https://github.com/castacks/AirStack/commit/2f11a8cb5605f4c594cd72f7f8925d84b1d6d330

```
The autolaunch tmux chains (bws && sws && ros2 launch ...) died silently
when any package failed to build: the pane returned to a prompt, docker
logs showed nothing, and the symptom surfaced only as missing data far
downstream (e.g. 'Foxglove has a connection issue'). The GCS variant
(bws && sws; ros2 launch) even launched unsourced after a failed build.

Both bind-mounted .bashrc files gain an autolaunch() helper that runs
build → source → launch and prints an unmissable red banner (plus a
plain-text repeat for ANSI-stripping log processors) when the build
fails or the launch exits nonzero. All robot variants (desktop, voxl,
l4t) and the GCS compose commands now use it; the zed camera service
keeps its own chain (separate bashrc).

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+61 / −5 in 6 files:

- `.env`
- `docs/release_notes/index.md`
- `gcs/docker/.bashrc`
- `gcs/docker/gcs-base-docker-compose.yaml`
- `robot/docker/.bashrc`
- `robot/docker/docker-compose.yaml`

## 7ead8d8ad1 — 2026-08-25 — Andrew Jong — KEYWORD fix

**fix(docs): search dropdown rendered behind nav tabs and version text**

https://github.com/castacks/AirStack/commit/7ead8d8ad13ccb38dfcb1d0efd456df541f1c4a4

```
.md-tabs sits inside .md-header, so the custom rules
'.md-tabs { position: relative; z-index: 3 }' (added to layer tabs above
the landing splash) and '.md-version * { z-index: 5 }' compete in the
header's stacking context with Material's search dropdown, which is only
'.md-search__output { z-index: 1 }' — the dropdown painted behind both.
Lift the whole search subtree with '.md-search { z-index: 6 }'
(.md-search is already position:relative upstream).

VERSION 0.20.0-alpha.15 -> 0.20.0-alpha.16 for the increment gate;
release-notes line added.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+13 / −1 in 3 files:

- `.env`
- `docs/release_notes/index.md`
- `docs/stylesheets/extra.css`

## 66e62efdbe — 2026-08-25 — Andrew Jong — KEYWORD broken,fix,fixed,fixes,wrong

**docs: Diátaxis documentation overhaul (#404)**

https://github.com/castacks/AirStack/commit/66e62efdbeb46ffbb1201a94296dd58ce0218959

```
* docs: fix 18 verified accuracy defects across the docs tree

Point fixes from the Diátaxis docs audit (notebook 006), each verified
against the code before editing:

- CLI reference: remove phantom --recreate, add real --scene flag
- robot/docker: ROBOT_LAUNCH_PACKAGE/FILE -> real LAUNCH_PACKAGE var
- Replace nonexistent airstack_msgs types (TrajectorySegment,
  TrajectoryOverride) with TrajectoryXYZVYaw in doc standards, AGENTS.md
- Fix dead local/c_controls path -> local/controls (AGENTS.md, agent guide)
- pegasus_scene_setup: .env snippet now matches actual .env defaults
- scenes.md: fetch_scene.sh example uses a real key (blocks)
- Testing docs: add missing wiring/waypoint_flight marks; drop false
  'autonomy mark unregistered' troubleshooting row
- Getting Started: Move Robot step updated to the auto-seeded Foxglove
  layout flow (+ host port 8766 note); requirements canonicalized
  (RTX 3070 min / 4080 rec, 100GB free disk, Ubuntu 22.04/24.04) and
  aligned in docs/README.md and about.md
- OSMO positioning reconciled: recommended REMOTE path; local Linux+GPU
  remains the golden path
- tasks.md + interface conventions spec reconciled to the eight
  task_msgs actions (spec bumped to v1.0.1 per its changelog rules)
- perception index outputs corrected to spec (odometry_conversion/odometry)
- exploration README: fix random-walk copy-paste; vscode_debug:
  per-container .devcontainer layout; docker_usage: retire pre-harness
  'Automated Testing' section, fix subnet, drop dead Isaac streaming refs
- Remove deprecated Ascent extension from simulation Getting-Started list

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: remove fossil pages, add redirects, deprecate git-hooks

Deletes the superseded/orphaned stratum found by the audit:
- docs/tutorials/index.md (zero inbound links; duplicate of
  tutorials_reference.md)
- docs/development/development_environment.md and
  docs/development/airstack-cli/index.md (stale orphaned twins of the
  in-nav beginner/ versions)
- docs/development/intermediate/testing/testing_frameworks.md
  (pre-harness taxonomy, broken example; inbound links repointed)
- docs/simulation/isaac_sim/scene_setup.md and
  ascent_sitl_extension.md (Ascent-era pipeline; nav entry removed)
  plus six images referenced only by them

frame_conventions.md rewritten from live facts: the world->map static
identity TF in autonomy_bringup robot.launch.xml and the canonical TF
table in the interface conventions spec. (scene_setup's map_FLU
90-degree-rotation story matched nothing in the code.)

mkdocs-redirects entries added for all six removed URLs. git-hooks
READMEs rewritten as deprecation notices: the docker-versioning hook
writes a commit hash into .env VERSION, which the semver
check-version-increment gate rejects; note left that
'airstack config git-hooks' still installs it (CLI removal is a
follow-up code change).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: surface orphaned knowledge into the site nav

Adds seven documents that existed only as off-site READMEs to
mkdocs.yml (served via the same-dir plugin, like the stack READMEs):

- config/vehicles/README.md — the vehicle.yaml schema (fleets.md now
  links it relatively instead of via GitHub URL)
- config/local/README.md — the CALIBRATION_DIR unit-calibration contract
- common/module_schema/README.md — the authoritative module.yaml
  field/validation reference, under the Modules section
- robot/ros_ws/src/sensors/lidar_point_cloud_filter/README.md — the
  only full-quality robot package README that was not in nav
- common/ros_packages/gui/rviz/rviz_tasks_panel/README.md and
  3d_waypoint_rviz2_plugin/README.md — operator manuals for the task
  and waypoint panels
- osmo/README.md — the OSMO lab-admin half of the OSMO story,
  complementing the student tutorial

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: restructure nav into Diátaxis quadrant tabs

mkdocs.yml nav reorganized into Tutorials / How-to Guides / Reference /
Concepts tabs (plus Home, Release Notes, About), with product areas as
second-level groups inside each tab. No files move — nav labels only —
so every page keeps its URL. This also restores the repo's own 3-level
nav-depth rule (the old Robot tree reached 5 levels).

The mislabeled Beginner/Intermediate/Advanced 'Tutorials' buckets are
gone: the Development section contained zero tutorials (all how-to,
reference, and explanation content, now filed by type).

Two hub pages rewritten for the new shape:
- getting_started/tutorials_reference.md — quadrant-honest 'What Next'
  page (by-goal table + persona paths) replacing the level-labeled
  catalog that called reference/explanation pages tutorials
- development/index.md — How-to Guides landing (guide groups + the
  handful of daily commands, deferring tables to the CLI reference)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: de-duplicate drift-prone content to canonical homes

One canonical home per fact; clones replaced with links (net -131 lines):

- Topic/interface tables -> interface_conventions.md spec:
  robot/index.md keeps 3 labeled examples; integration_checklist.md's
  ~110-line drifted reference block (incl. nonexistent interface/cmd_vel)
  replaced with deep links into spec sections (anchors verified against
  the rendered toc)
- airstack up flags -> CLI reference: docker_usage, key_concepts, and
  the Getting Started launch tail keep 2-3 examples + a link
- pytest marks -> tests/README.md: testing/index.md table replaced with
  a summary + link; ci_cd.md keeps its table for the CI-cost narrative
  but now cites tests/README.md as authoritative
- scene_prep.py helpers -> spawning_drones.md (richer copy); pegasus
  page links it via a stable {#scene-prep-helpers} anchor
- modular_airstack.md: duplicated setup section replaced with its own
  prerequisite pointer; redundant -f docker-compose.modules.yaml flag
  removed (airstack.sh auto-includes it, verified cmd_up/cmd_down)
- MS-AirSim pair merged: index.md owns overview/quickstart/settings/
  topics/user troubleshooting; docker.md owns container internals.
  Ground truth from entrypoint.sh: BOTH pages had the tmux ordering
  backwards (bridges are windows 1..N, PX4 N+1..2N, named
  robot_<i>_bridge / robot_<i>_px4); port table corrected to
  4560+i / 24540+i / 24580+i per settings.json.j2, and the wrong
  MAVLink-port troubleshooting bullet fixed

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: split hybrid pages by quadrant and audience

Four splits from the audit's Phase-3 list (originals keep their URLs;
new pages added to nav):

- system_architecture.md (643 -> ~375 lines): keeps the explanation
  core (node types, task cascade, layer table, communication patterns,
  multi-robot); per-layer topic lists (a drifted 4th copy of spec data)
  replaced with layer-index + spec links; the data-flow diagram now
  matches the real action cascade (results return to the action client,
  not a Behavior node); invented Performance Characteristics deleted;
  integration guidelines deferred to the checklist
- ci_cd.md -> ci_cd.md (maintainer explanation: architecture, cache,
  security, pod anatomy) + NEW using_ci.md (developer how-to:
  triggering, /pytest syntax, marks/cost, reading results,
  user-facing troubleshooting); 18 absolute blob/main links converted
  to relative
- foxglove.md -> operator page + NEW extending_foxglove.md (the 6-step
  marker-type recipe and visualizer-source guidance)
- isaac_sim/docker.md (459 -> 193 lines, pure container reference) +
  NEW container_workflows.md (launch modes, credentials, access,
  dev workflow, image management, troubleshooting); verbatim compose
  quoting trimmed to excerpts; two placeholder sections folded away

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: rewrite wrong/filler content and fill layer-index stubs

Rewrites of pages whose content was fictional, generic filler, or
contradicted the code (net -600 lines of invented material):

- global/planning/index.md: never-built Global-Manager/PlanRequest
  protocol (live TODO markers included) replaced with the shipped
  architecture: task-executor action servers -> global_plan handoff,
  verified against tasks.md, spec sections, and both planner sources
  (toggle attributed only to exploration — random_walk declares the
  remap but never serves it)
- interface/index.md: hotlinked placeholder diagram replaced with a
  verified mermaid of the real command/state flow; State section
  corrected — canonical odometry + map->base_link TF come from the
  odometry_conversion node, not RobotInterface; both TODOs gone
- configuration/index.md (369 -> 96 lines): fabricated config YAML,
  invented validation script, and generic Linux/Jetson sections
  deleted; keeps verified stack selection, env vars, identity pointer,
  and a new where-config-actually-lives table
- rosbags.md (210 -> 86): generic ros2-bag tutorial cut to a link;
  fake bag-info output removed; BAG_STORAGE_PATH documented
- logging/data_offloading.md (323 -> 23): gutted to a pointer at the
  real storage-tools workflow with one honest rsync one-liner using
  real bag paths (the old /opt/airstack paths exist nowhere)
- HITL/index.md rewritten for the current stack: gcs-real hitl
  profile, l4t launch, domain-ID alignment and fastdds.xml, honest
  unverified-caveats for cross-machine DDS and sim-SITL FCU paths,
  Foxglove-based verification (RViz-era steps gone)
- Six layer-index stubs filled to a uniform template (role, child
  links, stack-launch note, spec-linked interchanges): autonomy front
  door, local, global, global/world_model, static_transforms,
  behavior (first paragraph no longer contradicts the page)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: fill the P0/P1 gaps — new reference and onboarding docs

New documents for surfaces that had none, every claim verified in code:

- Environment Variables reference (docs/robot/configuration/
  environment_variables.md): the complete .env schema — every
  uncommented var with default and verified consumer, plus the
  variables exported by airstack up flags and the notable optional vars
- airstack_msgs README (nav-linked under Reference -> Interfaces): all
  10 messages + 4 services with field tables; unused types marked
  'defined; no trunk consumer' honestly; found PlanToWaypoint.srv is
  not in rosidl_generate_interfaces (never built) — documented as such
- trajectory_library README (replaces the 6-line in-nav TODO stub):
  classes, the YAML config schema from the parser (degrees conversion,
  $(param) substitution), real config excerpts; corrects the
  assumption that droan_gl loads the YAML (it only links the classes)
- Supported-platform matrix (docs/real_world/supported_platforms.md):
  honest statuses — CI-tested only for desktop x86 sim; Orin
  field-used per install guide; VOXL 'profile exists, docs in progress'
- Deploying to Hardware rewritten from its TODO skeleton into a real
  8-step tutorial (identity, FCU serial, stack choice, props-off bench
  test, HITL rehearsal, first-flight checks) with per-step checks and
  hardware-verify notes where the repo can't confirm specifics; nav
  entry promoted to the Tutorials tab
- Operating the GCS (rewrites the 23-line user_interface.md stub):
  panel walkthrough verified from the seeded layout JSON and the
  robot-commands extension bundle (7 task tabs; explicitly notes the
  Foxglove panel lacks the RViz panel's airborne gating)
- robot/docker/index.md base-image rows updated to current compose
  values (ubuntu24.04 / dustynv jazzy r36.4.0)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: adopt Diátaxis in the authoring standards; bump to 0.20.0-alpha.15

Updates the doc-authoring standards so the new structure stays clean:

- documentation.md: the three-type taxonomy (Tutorials/Guides/Reference)
  replaced with the four Diátaxis quadrants plus a decision tree for
  choosing the kind of a new page; package READMEs stay exempt
  (intentionally all-in-one)
- contributing.md: Documentation section now names the quadrant rule
  and links the decision tree; docs commands unified on airstack docs
- write-mkdocs-documentation skill: the beginner/intermediate/advanced
  organization pattern (the root cause of the old mislabeled nav)
  replaced with the quadrant-tab layout; nav example updated
- update-documentation skill + AGENTS.md: mkdocs nav examples updated
  to the new Reference -> Autonomy Packages shape; AGENTS.md points at
  the decision tree

Release flow per bump-version-and-release: VERSION 0.20.0-alpha.14 ->
0.20.0-alpha.15 (regex-validated), with a Documentation Overhaul entry
added to the 0.20.0 (Unreleased) release-notes section summarizing the
restructure, the 18 accuracy fixes, removals/redirects, rewrites,
splits, newly surfaced references, and the interface-spec v1.0.1 bump.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: six new how-to guides, Autonomy section, Concepts tab reorder

New guides (every command/topic verified in code, each offering the
in-tree package vs 'airstack module create --in-tree' scaffolding
choice per the module workflow):

- Adding a State Estimator (perception): conform to spec §2, swap via
  the interface_odometry_in_topic launch arg in a stack entry (the
  odometry_conversion node owns the canonical topic and map->base_link
  TF, launched unconditionally); asm_macvo as the module precedent
- Adding a Planner: local (DROAN pattern, §5 surface; verified swap
  example from full_droan_cpu) and global (task-executor pattern,
  random_walk); flight verification via -m waypoint_flight since
  -m autonomy never exercises tasks/navigate
- Creating a Multi-Agent Coordination Algorithm: gossip/PeerProfile bus
  (peer_registry over raw /gossip/peers, QoS verified), honest
  platform-gaps section from the coordination README
- Creating a Custom Stack Topology: airstack stack new -> entry-file
  wiring (single-locus) -> modules.repos pins -> wiring regen ->
  doctor/wiring-mark validation; splits deferred to lite_offload_global
- Adding a Vehicle Type, Unit, or Platform: vehicle.yaml (quad_default
  example, honest sim pass-through note), calibration overlay units,
  compute-platform compose/build chain (l4t precedent, CI-untested)
- Getting the Most out of Your Coding Agent: design-spec-first notebook
  workflow, skill-directed prompting, lettered results artifacts,
  capture-discovered-knowledge loop

Nav: new How-to -> Autonomy group (also absorbs Integration Checklist
and Coordination Payloads); Create a Custom Stack under Modules &
Stacks; vehicle guide under Robot & Field; coding-agent guide under
Contributing. Concepts tab moved directly after Tutorials (conceptual
grounding before task recipes).

UE->Isaac export tutorial refreshed: new walkthrough video
(cMjO7Sb7Zmo), instruction to export Z-up with scale in meters (unlike
the video), and a warning that UE Decals (paint markings, dirt,
puddles) do not export — bake them into textures instead.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs: five beginner tutorials, controller how-to, world-model+planner guide

New tutorials completing the Tutorials tab learning path (Get AirStack
Flying -> GCS mission -> parameter loop -> modular walkthrough -> first
module -> first fleet -> own scene -> hardware), each a single golden
path with per-step checks, every command verified (several executed):

- Fly a Mission from the GCS: takeoff, waypoints, geofence, Navigate,
  land, save (panel names/defaults verified against the extension
  bundle; honest no-obstacles note for the default scene)
- Change a Parameter: takeoff_velocity edit -> relaunch -> observe;
  verified that config YAML is symlink-installed from the bind-mounted
  source so NO rebuild is needed, and the code path that makes the
  panel's velocity=0 fall back to the config value
- Write Your First Module: ran the actual scaffold for ground truth;
  handles the scaffold stub's double-namespace gotcha (robot.launch.xml
  already pushes the ROBOT_NAME namespace)
- Your First Fleet: two-robot fleet YAML validated with
  resolve_fleet.py; documents the real per-robot tab / Robot-field
  targeting mechanics from render_layout.py
- Build and Fly Your Own Scene: no-UE golden path (GUI stage ->
  scenes.yaml entry -> --scene flight -> baked contained scene.usd);
  the catalog-resolution check was actually executed

How-to guides:
- NEW Adding a Controller: verified chain trajectory_controller
  tracking_point -> pid_controller -> interface/cmd_roll_pitch_
  yawrate_thrust; Path A (swap the feedback controller) vs Path B
  (take over the full spec-§5 surface, obligations enumerated);
  flight-test marks chosen for what each actually exercises
- Adding a Planner renamed/expanded to Adding a World Model and
  Planner: local world-model+planner matched pairs (disparity pipeline
  worked example; full_droan_cpu swaps the pair together) vs the
  spec'd §3 global map interchange; new Path C

Nav: Tutorials tab reordered to the learning path; Autonomy group gains
Add a Controller and the renamed guide. What-Next hub lists the
sequence. Release notes updated.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* docs(tutorials): use airstack connect instead of docker exec one-liners

Human-facing tutorials now teach the human workflow: hop into the
container with 'airstack connect robot-desktop' (attaches to the
container's tmux session; new shell window with Ctrl-b c, detach with
Ctrl-b d) and run bws/sws/ros2 commands interactively, instead of
docker exec bash -c one-liners. Applies to first_module.md and
change_a_parameter.md (5 spots). Host-side 'docker logs' checks stay,
noted as the tmux-mirror view. The non-interactive docker exec pattern
remains, deliberately, in the agent-facing docs
(working_with_coding_agents.md, AGENTS.md).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* Rename Tutorials to Beginner Tutorials

---------

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+3555 / −3074 in 90 files:

- `.agents/skills/update-documentation/SKILL.md`
- `.agents/skills/write-mkdocs-documentation/SKILL.md`
- `.env`
- `AGENTS.md`
- `common/ros_packages/msgs/airstack_msgs/README.md`
- `docs/README.md`
- `docs/about.md`
- `docs/development/adding_a_vehicle.md`
- `docs/development/advanced/ai_agent_guide.md`
- `docs/development/airstack-cli/index.md`
- `docs/development/beginner/airstack-cli/docker_usage.md`
- `docs/development/beginner/airstack-cli/index.md`
- `docs/development/beginner/key_concepts.md`
- `docs/development/beginner/vscode/vscode_debug.md`
- `docs/development/creating_a_stack.md`
- `docs/development/development_environment.md`
- `docs/development/fleets.md`
- `docs/development/index.md`
- `docs/development/intermediate/contributing.md`
- `docs/development/intermediate/documentation.md`
- `docs/development/intermediate/frame_conventions.md`
- `docs/development/intermediate/testing/ci_cd.md`
- `docs/development/intermediate/testing/end_to_end_testing.md`
- `docs/development/intermediate/testing/index.md`
- `docs/development/intermediate/testing/testing_frameworks.md`
- `docs/development/intermediate/testing/unit_testing.md`
- `docs/development/intermediate/testing/using_ci.md`
- `docs/development/working_with_coding_agents.md`
- `docs/gcs/extending_foxglove.md`
- `docs/gcs/foxglove.md`
- `docs/gcs/usage/user_interface.md`
- `docs/getting_started/build_your_own_scene.md`
- `docs/getting_started/change_a_parameter.md`
- `docs/getting_started/first_fleet.md`
- `docs/getting_started/first_module.md`
- `docs/getting_started/fly_a_mission.md`
- `docs/getting_started/index.md`
- `docs/getting_started/modular_airstack.md`
- `docs/getting_started/tutorials_reference.md`
- `docs/real_world/HITL/index.md`
- … 50 more

## 383d344e2e — 2026-08-25 — Andrew Jong — KEYWORD break,fix

**fix(docs): prevent landing sections from overflowing on narrow screens**

https://github.com/castacks/AirStack/commit/383d344e2e80af0cf31878a14bb577a600ece246

```
Grid/flex items default to min-width:auto, so the wide <pre> quickstart and
compose snippets could force the pillar-section columns (and the page) past
the viewport on phones. Add min-width guards on the section grid children
and terminal/snippet cards, clamp the landing column with overflow-x:clip,
and shrink the CI-matrix chip font at the mobile breakpoint so long mark
names don't break mid-word.

Verified with true CDP mobile emulation at 390px (cssContentSize width
== viewport; plain --window-size screenshots are unreliable below ~500px
because headless Chrome clamps the window width).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+15 / −0 in 1 files:

- `docs/stylesheets/extra.css`

## 45e2a652a4 — 2026-08-25 — Andrew Jong — KEYWORD break,fix,fixed

**fix(docs): hero no longer clips CTAs on short viewports**

https://github.com/castacks/AirStack/commit/45e2a652a4187accc785d58af4a803e639a8b54f

```
The splash container used a fixed height (100vh - 98px) with
overflow:hidden, clipping the quickstart terminal, CTA buttons, and
footnote on short laptop screens (e.g. 13" MacBooks). The hero now grows
with its content (min-height instead of height), a max-height:900px
profile tightens hero spacing so everything still fits above the fold,
and the mobile breakpoint gets the same min-height treatment. Flex
min-width guards keep the terminal card from forcing horizontal overflow
on phones, and the h1 line break collapses to a proper space on mobile.

Verified headless at 1440x790, 1920x1080, and 390x844.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+61 / −8 in 2 files:

- `docs/overrides/home.html`
- `docs/stylesheets/extra.css`

## a238c5a9ab — 2026-08-25 — Andrew Jong — KEYWORD fixes

**docs: prefer airstack CLI flags over env vars in launch instructions**

https://github.com/castacks/AirStack/commit/a238c5a9ab5fddc2a0bf5eed752dfffbfb58b19a

```
Sweep the docs site and agent docs (AGENTS.md, skills, package template)
for `VAR=value airstack up` instructions and replace them with the
launch-intent flags where one exists: --no-autolaunch, --robots N,
--headless, --no-play, --sim isaac|airsim. Env vars are kept only where
no flag equivalent exists (ISAAC_SIM_SCRIPT_NAME, RECORD_BAGS,
MS_AIRSIM_BINARY_PATH, COMPOSE_PROFILES=desktop_split, ...); the
docker_usage.md overrides section now says so explicitly.

Also fixes the configure-multi-robot checklist command
`NUM_ROBOTS=3 airstack up`, which preflight would reject with the
single-drone default Isaac script (now `airstack up --sim isaac --robots 3`).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+44 / −47 in 15 files:

- `.agents/skills/add-ros2-package/SKILL.md`
- `.agents/skills/add-ros2-package/assets/package_template/README.md`
- `.agents/skills/configure-multi-robot/SKILL.md`
- `.agents/skills/update-documentation/SKILL.md`
- `AGENTS.md`
- `docs/development/advanced/ai_agent_guide.md`
- `docs/development/beginner/airstack-cli/docker_usage.md`
- `docs/development/beginner/development_environment.md`
- `docs/development/beginner/key_concepts.md`
- `docs/development/development_environment.md`
- `docs/gcs/docker/index.md`
- `docs/robot/autonomy_modes.md`
- `docs/robot/index.md`
- `docs/simulation/isaac_sim/spawning_drones.md`
- `docs/simulation/ms-airsim/docker.md`

## d3e746a77d — 2026-08-25 — Andrew Jong — KEYWORD fixing

**feat(sim): Isaac follow-cam, spawn relocation, and scene light-boost env knobs**

https://github.com/castacks/AirStack/commit/d3e746a77d0727d20752a1975f2e4b61dcc9fa69

```
- ISAAC_SIM_FOLLOW_CAM / _OFFSET: smoothed viewport chase camera tracking a
  drone via live Pegasus VehicleManager state (PhysX/fabric never writes
  poses back to USD). Frames the spawn point pre-Play, fixing the black
  default viewport on cm-authored stages. On by default; 'off' disables.
- ISAAC_SIM_FOLLOW_CAM_LIGHT: optional headlight riding the follow camera
  for interiors the dome light cannot reach.
- ISAAC_SIM_SPAWN_XY: recenter the spawn row away from a cluttered scene
  origin (row_spawn_configs center_xy; consumed by example_multi).
- ISAAC_SIM_LIGHT_BOOST: scene_prep.boost_scene_lights multiplies the
  scene's own lights via exposure += log2(factor), de-instancing
  light-bearing subtrees first (NVIDIA ceiling lights are instanceable).
- ISAAC_SIM_DOME_LIGHT: dome light intensity[,exposure] override.

All knobs plumbed through the isaac-sim compose service.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

+296 / −7 in 4 files:

- `simulation/isaac-sim/docker/docker-compose.yaml`
- `simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/launch_scripts/pegasus_app.py`
- `simulation/isaac-sim/utils/scene_prep.py`

## 02f7660545 — 2026-08-28 — Andrew Jong — KEYWORD crash,failed,failure,fix,regression

**fix(ci): un-red every PR — metrics-job deps and unit-test workflow environment (#407)**

https://github.com/castacks/AirStack/commit/02f7660545cc442a28415edad7674151e908b49e

```
Every PR has been red since ~2026-08-20 for reasons unrelated to the code
under test:

- system-tests.yml / Metrics Report installed only `tabulate`, but
  tests/parse_metrics.py imports the tests/harness package, whose import
  chain needs pyyaml — the job crashed with ModuleNotFoundError on every
  run, and the crash's exit 1 was indistinguishable from the deliberate
  exit-1-on-regression, so it was misreported as "Metric regression
  detected". Install tests/requirements.txt (same set every other
  harness-touching job uses) and require report.md to exist before
  claiming a regression (parse_metrics.py writes the report before its
  regression exit, and writes a failure report + exit 2 for handled
  errors — exit 1 without report.md can only be a crash).

- unit-tests.yml checked out without submodules and without omni_pass.env,
  so the tests/meta contract suite failed on hosted runners: every
  `airstack up --dry-run --sim isaac` contract hard-errored in preflight
  (missing Nucleus credentials, empty PegasusSimulator submodule) and the
  docs-catalog contract missed the submodule-resident vdb_mapping_ros2
  README. Check out submodules recursively and provision the same guest
  omni_pass.env stub module-system-tests.yml already creates. No
  airstack.sh change: preflight stays strict; CI provisions the
  environment it claims to test.

Validated locally in CI-equivalent environments: the yaml crash reproduces
in a tabulate-only venv and disappears with requirements.txt; the full
`pytest tests/ -m unit` goes from ~20 failures to 402 passed / 0 failed
after submodule init + stub.

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+44 / −3 in 4 files:

- `.env`
- `.github/workflows/system-tests.yml`
- `.github/workflows/unit-tests.yml`
- `docs/release_notes/index.md`

## 6c0a4e700f — 2026-08-28 — Andrew Jong — KEYWORD error

**feat(stacks): full_mighty reference stack + mighty module catalog entry (#405)**

https://github.com/castacks/AirStack/commit/6c0a4e700f1fd0a14c0d3ee0a48a50306eec20b7

```
Upstreams the MIGHTY local-planner integration from the paper branch:

- stacks/full_mighty: full_default with the local-planner include swapped
  to the asm_mighty module's mighty_module.launch.xml (MIGHTY Hermite-
  spline planner + acl-mapping voxel world model + NavigateTask bridge);
  modules.repos pins asm_mighty v0.1.1. The module-swap demonstration
  for the modular architecture. wiring.md pending the first validated
  wiring-snapshot run (bootstrap rule).
- Marketplace: mighty module + full_mighty stack registered in the
  fixture index and docs/modules regenerated (registry PR to
  airstack-modules-index carries the live entries); mkdocs nav +
  reference-stack enumerations updated.
- VERSION 0.20.0-alpha.16 -> 0.20.0-alpha.17 + release notes entry.

Validated at the pinned module version on Isaac Sim: 44/44 vendored
gtests, synthetic smoke, empty-world NavigateTask route (goal error
0.14 m), 7/7 pillar-field traversals, 5/5 judged obstacle-route flights
(min clearances 1.59-1.65 m vs the 1.0 m gate). Motivating DROAN
comparison (figures + numbers) lives in the asm_mighty README.

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+335 / −3 in 15 files:

- `.env`
- `docs/development/stacks.md`
- `docs/modules/index.md`
- `docs/modules/mighty.md`
- `docs/release_notes/index.md`
- `docs/robot/autonomy/adding_a_controller.md`
- `docs/robot/autonomy/adding_a_world_model_and_planner.md`
- `docs/robot/autonomy_modes.md`
- `mkdocs.yml`
- `stacks/full_mighty/README.md`
- `stacks/full_mighty/docker-compose.yaml`
- `stacks/full_mighty/launch/stack.launch.xml`
- `stacks/full_mighty/modules.repos`
- `tests/meta/fixtures/modules_index/modules/mighty.yaml`
- `tests/meta/fixtures/modules_index/stacks/full_mighty.yaml`

## 8aed8fc1a0 — 2026-08-29 — Andrew Jong — KEYWORD fixes,hotfix

**hotfix(docs): README-page 404s, link previews, edit-this-page button (0.20.7) (#419)**

https://github.com/castacks/AirStack/commit/8aed8fc1a007e2b8e3116e9cd014619523583a14

```
* hotfix(docs): anchor exclude_docs README pattern to repo root

The exclude_docs entry meant to hide the GitHub-facing repo README used
the bare pattern 'README.md'. exclude_docs patterns are gitignore-style,
so an unanchored name matches at every depth — silently excluding every
README.md the same-dir plugin serves as a docs page. On the published
0.20 site this 404'd the System Test Suite (/tests/), the OSMO Lab
Admin Guide (/osmo/), and all autonomy package reference pages, while
non-README sources (e.g. tests/ci-cd-orchestrator.md) kept working.

Anchor the pattern as '/README.md' so only the repo root is excluded.
Verified with a local mkdocs build: tests/, osmo/, and package README
pages are generated again and the root URL still serves the redirect.

VERSION 0.20.6 -> 0.20.7 with a dated release-notes section per the
hotfix rule from #418.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* hotfix(docs): Open Graph metadata for link previews

Sharing docs.theairlab.org in Slack/WhatsApp/iMessage previewed as
"Redirecting": preview scrapers fetch raw HTML without running JS or
following meta-refresh, and every layer of the site lacked metadata —
the root redirect stub (mike set-default), the version-root stub
(/0.20/, from the mkdocs-redirects index.md map), and the content
pages themselves (Material emits og: tags only via its social plugin).

Three fixes, one per layer:
- root: deploy the redirect via `mike set-default -T` with a custom
  template (.github/workflows/templates/root-redirect.html) carrying a
  real title, description, and OG/Twitter Card tags ({{href}} keeps the
  instant redirect)
- version root: a post-build hook
  (docs/hooks/social_meta_redirect_stub.py) retitles the generated
  redirect stub and injects the same meta block
- content pages: docs/overrides/main.html adds per-page og:title /
  og:description / og:image + twitter tags in block extrahead
  (home.html already extends main.html, so the whole site inherits),
  and mkdocs.yml gains site_description

og:image is the 1600x900 splash poster via the moving /main/ alias,
which serves real asset files, so the absolute URL survives releases.

Verified with a local mkdocs build (stub injected, home hero intact,
per-page titles on README pages) and by rendering the mike template
with Jinja2 exactly as mike 2.2 does.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* feat(docs): "Edit this page" pencil on every docs page

Enable Material's content.action.edit feature with edit_uri so each
page links to GitHub's editor for its source file; contributors
without write access get GitHub's fork-and-propose flow automatically.

docs_dir is the repo root, so page paths are already repo-relative and
edit_uri is just the branch prefix. It defaults to edit/main/ and is
overridable via the DOCS_EDIT_URI env var (mkdocs !ENV tag), which the
develop docs deploy sets to edit/develop/ so unstable docs edit the
branch they were built from.

The landing page keeps no edit button by design: its home.html hero
template has no content header. Verified with a local build — README
pages link to e.g. edit/main/tests/README.md, regular pages to their
own .md source.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+205 / −5 in 8 files:

- `.env`
- `.github/workflows/deploy_docs_from_develop.yaml`
- `.github/workflows/deploy_docs_from_main.yaml`
- `.github/workflows/templates/root-redirect.html`
- `docs/hooks/social_meta_redirect_stub.py`
- `docs/overrides/main.html`
- `docs/release_notes/index.md`
- `mkdocs.yml`

## 57027c6748 — 2026-08-29 — Andrew Jong — KEYWORD hotfix,patch

**hotfix(docs): dated release-notes sections for 0.20.1–0.20.5; require them for all hotfixes (#418)**

https://github.com/castacks/AirStack/commit/57027c67486dd7f02d356517a269360d1477570b

```
* docs(release-notes): dated sections for 0.20.1–0.20.5; require them for all hotfixes

Add a dated release-notes section for this hotfix (0.20.5, public
registry pulls) and backfill 0.20.1–0.20.4, which landed without
sections. Codify the rule in the bump-version-and-release skill and the
AGENTS.md skill table: every hotfix on main — docs-only and CI-only
included — adds its own dated '## X.Y.Z — YYYY-MM-DD' patch-notes
section, since the MAJOR.MINOR docs build renders all of a line's patch
sections together.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* hotfix(docs): dated release-notes sections for 0.20.1–0.20.5; require them for all hotfixes

Follow-up to #417, whose release-notes/skill commit was pushed after
the PR had already merged. Adds the dated patch-notes sections for
hotfixes 0.20.1–0.20.5 (none had one) and codifies the rule in the
bump-version-and-release skill and the AGENTS.md skill table: every
hotfix on main adds its own dated '## X.Y.Z — YYYY-MM-DD' section.
VERSION 0.20.5 → 0.20.6 (gate).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+58 / −3 in 4 files:

- `.agents/skills/bump-version-and-release/SKILL.md`
- `.env`
- `AGENTS.md`
- `docs/release_notes/index.md`

## e0a09f27a8 — 2026-08-29 — Andrew Jong — KEYWORD hotfix

**hotfix(docs): Harbor registry is public — drop docker login from pull instructions (#417)**

https://github.com/castacks/AirStack/commit/e0a09f27a8260374407b0d4b623958b3647af50b

```
The airstack project on airlab-docker.andrew.cmu.edu is now public
(verified: anonymous token grant, tag list, manifest and blob fetch all
succeed with no credentials), so pulling images no longer requires an
AirLab account. Remove the docker login step from all pull instructions
(getting started, CLI docker usage, Isaac Sim / MS-AirSim image
management) and note that pushing still requires login.
VERSION 0.20.4 → 0.20.5 (gate).

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+10 / −19 in 5 files:

- `.env`
- `docs/development/beginner/airstack-cli/docker_usage.md`
- `docs/getting_started/index.md`
- `docs/simulation/isaac_sim/container_workflows.md`
- `docs/simulation/ms-airsim/docker.md`

## 946ca01647 — 2026-08-29 — Andrew Jong — KEYWORD hotfix

**hotfix(docs): pin the site root redirect to the MAJOR.MINOR slug (#416)**

https://github.com/castacks/AirStack/commit/946ca0164788f65004e06942521b4f2d82e9678c

```
docs.theairlab.org now redirects to the current release slug
(/0.20/docs/) instead of the moving /main/ alias, so the URLs readers
land on and copy stay valid across future releases. /main/ remains as
a moving alias for deep links. VERSION 0.20.3 → 0.20.4 (gate).

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+15 / −9 in 6 files:

- `.agents/skills/bump-version-and-release/SKILL.md`
- `.env`
- `.github/workflows/deploy_docs_from_main.yaml`
- `.github/workflows/deploy_docs_from_release.yaml`
- `.github/workflows/scripts/docs_reorder_versions.py`
- `.openhands/microagents/repo.md`

## 2adc4a30b5 — 2026-08-29 — Andrew Jong — KEYWORD hotfix

**hotfix(docs): rename the stable docs alias 'latest' → 'main'; serialize gh-pages pushes (#415)**

https://github.com/castacks/AirStack/commit/2adc4a30b534081ee0cfea928c157ed277d0fd33

```
The site default and stable-docs alias is now /main/ (restores pre-0.20
bookmark URLs) instead of /latest/. Both deploy workflows point the
'main' alias at the current MAJOR.MINOR slug and set-default main; repo
links and skill/microagent references updated.

Also adds a shared concurrency group to the three docs deploy workflows:
concurrent deploys raced on the gh-pages push (observed: develop deploy
rejected with 'fetch first' while the main deploy pushed).

VERSION 0.20.2 → 0.20.3 (CI/docs-only; bump required by the PR gate).

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+29 / −11 in 8 files:

- `.agents/skills/bump-version-and-release/SKILL.md`
- `.env`
- `.github/workflows/deploy_docs_from_develop.yaml`
- `.github/workflows/deploy_docs_from_main.yaml`
- `.github/workflows/deploy_docs_from_release.yaml`
- `.github/workflows/scripts/docs_reorder_versions.py`
- `.openhands/microagents/repo.md`
- `README.md`

## 4c27432276 — 2026-08-29 — Andrew Jong — KEYWORD break,hotfix,patch

**hotfix(docs): MAJOR.MINOR docs slugs; retire the duplicate 'main' docs version (#414)**

https://github.com/castacks/AirStack/commit/4c274322765f22c75fa0efe4b49a84ec7009e519

```
The version selector showed the same release twice: '0.20.0' (release
deploy) and '0.20.0 (stable)' (the main-branch deploy at /main/). Docs
versions now use MAJOR.MINOR slugs (/0.20/) with the selector title
carrying the full patch version:

- deploy_docs_from_main.yaml: deploys main's docs onto the X.Y slug from
  .env with --title X.Y.Z + latest alias, and sets the site default to
  'latest' (was: separate 'main' version + set-default main). A docs
  hotfix republishes the slug in place and retitles the entry, so URLs
  never break within a minor line.
- deploy_docs_from_release.yaml: same slug scheme (tag 0.21.0 → /0.21/),
  plus set-default latest.
- release_notes_current_version.py hook: match sections by MAJOR.MINOR,
  so a /0.20/ build shows the 0.20.0 section plus any 0.20.x hotfix
  sections.
- docs_reorder_versions.py: pin only 'develop' to the top (no 'main'
  entry to pin anymore).
- sync-develop-from-main.yaml: read NEW_VERSION from .env for the bump
  commit message — GITHUB_ENV exports only apply to later steps, so the
  message said 'Bump VERSION to  ' (empty).
- Skill + microagent references updated. VERSION 0.20.1 → 0.20.2 (CI/
  docs-only change; bump required by the PR gate).

One-time gh-pages cleanup after this merges: delete the 'main' and
'0.20.0' versions, migrate '0.19.0' → '0.19'.

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+67 / −40 in 9 files:

- `.agents/skills/bump-version-and-release/SKILL.md`
- `.env`
- `.github/workflows/deploy_docs_from_develop.yaml`
- `.github/workflows/deploy_docs_from_main.yaml`
- `.github/workflows/deploy_docs_from_release.yaml`
- `.github/workflows/scripts/docs_reorder_versions.py`
- `.github/workflows/sync-develop-from-main.yaml`
- `.openhands/microagents/repo.md`
- `docs/hooks/release_notes_current_version.py`

## 7ec95bb75e — 2026-08-29 — Andrew Jong — KEYWORD fix,hotfix

**hotfix(ci): post-release main→develop sync must not skip on identical trees (#413)**

https://github.com/castacks/AirStack/commit/7ec95bb75e9bcab02825b934493085c7b0386d60

```
Backport of the sync-develop-from-main fix from develop (#412) so it is
live for the 0.21.0 release: push-triggered workflows run from main's
copy of the file. Skip now requires main to already be an ancestor of
develop — a tree-diff skip fired right after every release (merge result
content-identical to develop), so the ancestry-advancing merge commit
was never pushed and develop's VERSION never rolled forward.

VERSION 0.20.0 → 0.20.1 (CI-only change; bump required by the PR gate).

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+9 / −4 in 2 files:

- `.env`
- `.github/workflows/sync-develop-from-main.yaml`

## e4102b6165 — 2026-08-29 — Andrew Jong — KEYWORD fix,fixes

**Bump version to 0.21.0-dev.0; fix post-release main→develop sync skip (#412)**

https://github.com/castacks/AirStack/commit/e4102b6165caf2236ca568507706ec198226065c

```
First version on the new -dev.N pre-release line (replacing -alpha.N as
of release 0.20.0). Also fixes sync-develop-from-main to skip only when
main is already an ancestor of develop: the old tree-diff skip fired
right after every release (merge result content-identical to develop),
so the workflow never pushed the ancestry-advancing merge commit or
rolled develop's VERSION forward — both had to be done by hand.

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+16 / −6 in 4 files:

- `.env`
- `.github/workflows/sync-develop-from-main.yaml`
- `airstack.yaml`
- `docs/release_notes/index.md`

## ff7c4c50e4 — 2026-08-29 — Andrew Jong — KEYWORD fails,issue

**feat(ci): registry→trunk catalog sync automation + drift alarm (#408)**

https://github.com/castacks/AirStack/commit/ff7c4c50e463f25e47c41415c6536610ca3cb028

```
* feat(ci): registry→trunk catalog sync automation + drift alarm

Module registration is two merges (registry PR + trunk fixture/catalog
sync); the docs deploy regenerates the published catalog from the LIVE
registry, so a missed half silently dropped the module from the site
(happened with mighty, 2026-08-29). Two mechanical guards:

- deploy_docs_from_develop: drift alarm step — gen_docs_catalog --check
  against a fresh registry clone (empty modules-dir, the committed-page
  variant); on mismatch emits a warning annotation, a job summary, and
  files one docs-catalog-drift issue. Never fails the deploy.
- sync-modules-index (new, daily + dispatch): mirrors registry entries
  into tests/meta/fixtures/modules_index/, regenerates docs/modules/,
  bumps VERSION for the increment gate, and opens/refreshes the
  bot/sync-modules-index PR (scripts/registry_sync.py, locally testable).

AGENTS.md + create-module/extract-module skills updated to point at the
automation. VERSION 0.20.0-alpha.21.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

* sync-modules-index: prefer REGISTRY_SYNC_TOKEN so bot PRs trigger CI

Falls back to the workflow token when the secret is absent (close/reopen
the bot PR to run checks in that mode). Registry-side dispatch workflow
uses the same secret name.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+290 / −4 in 8 files:

- `.agents/skills/create-module/SKILL.md`
- `.agents/skills/extract-module/SKILL.md`
- `.env`
- `.github/workflows/deploy_docs_from_develop.yaml`
- `.github/workflows/scripts/registry_sync.py`
- `.github/workflows/sync-modules-index.yml`
- `AGENTS.md`
- `docs/release_notes/index.md`

## 270910ac97 — 2026-08-29 — pvkumara — KEYWORD fail,failed,fix

**Ci/trustworthy system tests (#403)**

https://github.com/castacks/AirStack/commit/270910ac970032442d7bda4ba30b53302b660d00

```
* Make system-test outcomes trustworthy and actionable

Separate infrastructure failures, test assertions, and advisory metric changes so CI blocks only on real test or integrity failures while preserving comparable performance evidence.

* Classify readiness failures as infrastructure

Preserve explicit infrastructure intent through pytest call reports so simulator startup crashes cannot be misreported as algorithm assertions.

* Allow focused manual flight campaigns

Expose trajectory and takeoff sweeps in workflow dispatch so CI validation can run minimal algorithm samples before expanding to expensive matrices.

* Finish leftover merge markers from the develop rebase.

Keep develop's images CLI and Using CI layout while preserving this PR's advisory metrics, diagnostics artifacts, and infrastructure mark.

* Fix unit-test merge fallout and bump VERSION.

Keep --config-only hermetic for fleet contracts, still fail AUTONOMY_ROLE as a config check, and document the new help tokens so CI can pass the increment gate.

* Strip post-extraction residue and metadata plumbing; refresh CI reference docs

Scope cut agreed with Andrew before merge:

- Drop tests/system/test_optitrack_e2e.py: it sets LAUNCH_NATNET (a removed
  no-op shim) and targets robot/ros_ws/src/perception/natnet_ros2, which was
  extracted from trunk (70423c4c). It could never pass on develop and PR CI
  never executes the optitrack mark, so it would rot silently. The same test
  lives where it belongs: asm_optitrack/tests/system/, exercised by the
  module CI that is now served by the multi-repo orchestrator (#400).
  The mark stays registered in tests/pytest.ini for module-CI runs.
- Drop tests/report-requirements.txt: develop's metrics job already installs
  tests/requirements.txt (#407); a second requirements file for the same
  tree is a drift hazard. The workflow contract now pins the #407 behavior.
- Drop harness/image_prep.py and the image-preparation.json workflow
  instrumentation: metadata plumbing whose value doesn't carry its weight —
  the associative-array bash was the most fragile code in the PR. The
  image-prep step returns to develop's plain version.
- diagnostics: drop LAUNCH_NATNET from SAFE_ENV_KEYS (dead); keep
  PX4_PARAM_SET (live compose env-file selector).
- Un-clobber develop content the branch predated: run-system-tests SKILL
  description (waypoint_flight, wiring marks), BSD-3-Clause-Clear license,
  `airstack images build` spelling.
- Reference docs refreshed to where CI stands now: AGENTS.md test-suite
  paragraph (schema-v2 run_meta, advisory metric deltas, diagnostics
  bundle, fingerprint-only comparisons) and --config-only intent flag;
  module_ci.md + module-system-tests.yml header (orchestrator polls a
  repos: list — asm_optitrack included today); Release Notes entry for the
  trustworthy-outcomes policy.

Full unit suite on this branch: 419 passed, 0 failed.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>

---------

Co-authored-by: Pranav Kumara <pkumara@andrew.cmu.edu>
Co-authored-by: Andrew Jong <ajong@andrew.cmu.edu>
Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
```

+1027 / −182 in 33 files:

- `.agents/skills/run-system-tests/SKILL.md`
- `.env`
- `.github/workflows/module-system-tests.yml`
- `.github/workflows/system-tests.yml`
- `.github/workflows/unit-tests.yml`
- `AGENTS.md`
- `airstack.sh`
- `docs/development/intermediate/testing/ci_cd.md`
- `docs/development/intermediate/testing/using_ci.md`
- `docs/development/module_ci.md`
- `docs/release_notes/index.md`
- `tests/README.md`
- `tests/conftest.py`
- `tests/harness/__init__.py`
- `tests/harness/baseline.py`
- `tests/harness/commands.py`
- `tests/harness/diagnostics.py`
- `tests/harness/run_meta.py`
- `tests/harness/session.py`
- `tests/harness/sim.py`
- `tests/harness/test_ids.py`
- `tests/meta/test_campaign_reporting_contract.py`
- `tests/meta/test_collection_contract.py`
- `tests/meta/test_diagnostics_contract.py`
- `tests/meta/test_docs_catalog_contract.py`
- `tests/meta/test_fleet_contract.py`
- `tests/meta/test_launch_intent_contract.py`
- `tests/meta/test_workflow_contract.py`
- `tests/parse_metrics.py`
- `tests/pytest.ini`
- `tests/run_summary.py`
- `tests/system/test_liveliness.py`
- `tests/system/test_sensors.py`

## a6dad8caf5 — 2026-09-04 — Andrew Jong — KEYWORD failed,hotfix

**hotfix(docs): exclude AirSim scene binaries from docs builds (0.20.8) (#421)**

https://github.com/castacks/AirStack/commit/a6dad8caf54722e5eba3481367e914ce213e6135

```
* hotfix(docs): exclude AirSim scene binaries from docs builds (0.20.8)

The same-dir plugin publishes the whole repo tree as site content, and the
downloaded Microsoft AirSim UE4 scenes (simulation/ms-airsim/assets/scenes/)
and environments (simulation/ms-airsim/environments/) were not in
exclude_docs, so every mkdocs build/serve on a machine with scenes fetched
copied ~15 GB of Unreal .debug binaries and zips into the site directory.
No pages or links reference either directory.

Bumps VERSION 0.20.7 -> 0.20.8 and records the change in the release notes.

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>

* test(docs): tolerate mkdocs !ENV tag in docs-catalog contract loader

PR #419 added 'edit_uri: !ENV [DOCS_EDIT_URI, "edit/main/"]' to mkdocs.yml.
The contract test parses mkdocs.yml with a SafeLoader that only knew the
!!python/name tag, so all four docs-catalog contract tests have failed on
main and develop since 2026-08-29. Collapse !ENV to its default value.

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>

---------

Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
```

+31 / −2 in 4 files:

- `.env`
- `docs/release_notes/index.md`
- `mkdocs.yml`
- `tests/meta/test_docs_catalog_contract.py`
