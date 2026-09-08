# Merged PRs into develop, main of castacks/AirStack, 2026-04-28..2026-09-08

52 PRs; 926 workflow runs in window; 58 RED→FIX sequences (a failing automated check on a PR commit followed by a later push to the same PR).

## PR #348 — Johnliu/px4 cpu optimization (2026-04-30, → main, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/348

17 commits, +195/−10, 0 review comments, 12 issue comments, **3 RED→FIX**

- `d1ea859411` 2026-04-27 added option for physics step frequency  ⟶ no runs
- `1324a9c568` 2026-04-28 reverted example launch script  ⟶ no runs
- `85cfbad7ab` 2026-04-29 patches PX4 simulation startup script and fixes robot DDS version  ⟶ no runs
- `06c9478f3a` 2026-04-29 set default physics Hz for PX4 to be 100Hz which is the minimum.  ⟶ no runs
- `d7a2358509` 2026-04-29 reverted simulation changes  ⟶ no runs
- `5f8f32236d` 2026-04-29 updated docs  ⟶ no runs
- `e0432879a9` 2026-04-29 Merge branch 'main' into johnliu/px4_cpu_optimization  ⟶ SystemTests=failure, CheckVERSIONIncrement=success
- `d3096ab454` 2026-04-29 Merge branch 'main' into johnliu/px4_cpu_optimization  ⟶ SystemTests=failure, CheckVERSIONIncrement=success
- `9eaa2fb638` 2026-04-29 Merge branch 'main' into johnliu/px4_cpu_optimization  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success
- `208b32cbdd` 2026-04-29 Merge branch 'main' into johnliu/px4_cpu_optimization  ⟶ CheckVERSIONIncrement=success, SystemTests=cancelled
- `9b53769b22` 2026-04-30 Better error logging for ci/cd orchestrator  ⟶ no runs
- `9719f67e4b` 2026-04-30 Add check system resources before spawning server; if resources not available, report back and try again later  ⟶ no runs
- `54befc772e` 2026-04-27 added option for physics step frequency  ⟶ CheckVERSIONIncrement=success, SystemTests=failure
- `71fac5816e` 2026-04-27 added option for physics step frequency  ⟶ CheckVERSIONIncrement=success, SystemTests=success
- `2f50a26b1f` 2026-04-30 removed physics frequency from .env and set working PX4 values in docker-compose defaults.  ⟶ SystemTests=success, CheckVERSIONIncrement=success, CheckVERSIONIncrement=success, SystemTests=success
- `b6ca4a11ae` 2026-04-30 removed unnecessary benchmarking from AirStack launch scripts.  ⟶ CheckVERSIONIncrement=success, CheckVERSIONIncrement=success
- `7abbb6a130` 2026-04-30 Merge branch 'main' into johnliu/px4_cpu_optimization  ⟶ CheckVERSIONIncrement=success

  **RED→FIX** after `e0432879a9` (System Tests): https://github.com/castacks/AirStack/actions/runs/25124667802
  followed by: `d3096ab454` Merge branch 'main' into johnliu/px4_cpu_optimization; `9eaa2fb638` Merge branch 'main' into johnliu/px4_cpu_optimization; `208b32cbdd` Merge branch 'main' into johnliu/px4_cpu_optimization

  **RED→FIX** after `d3096ab454` (System Tests): https://github.com/castacks/AirStack/actions/runs/25130574097
  followed by: `9eaa2fb638` Merge branch 'main' into johnliu/px4_cpu_optimization; `208b32cbdd` Merge branch 'main' into johnliu/px4_cpu_optimization; `9b53769b22` Better error logging for ci/cd orchestrator

  **RED→FIX** after `54befc772e` (System Tests): https://github.com/castacks/AirStack/actions/runs/25182301651
  followed by: `71fac5816e` added option for physics step frequency; `2f50a26b1f` removed physics frequency from .env and set working PX4 values in docker-compose defaults.; `b6ca4a11ae` removed unnecessary benchmarking from AirStack launch scripts.

  Conversation:
  - @andrewjong: /pytest -m 'build_packages or liveliness or takeoff_hover_land' --sim isaacsim  (https://github.com/castacks/AirStack/pull/348#issuecomment-4356396395)
  - @andrewjong: This is cool, the tests show realtime_factor is roughly 0.75 with John's update, which is x2 as good as the previous 0.38 real time factor we were empirically experiencing before.  (https://github.com/castacks/AirStack/pull/348#issuecomment-4356773023)

## PR #351 — Johnliu/rtx lidar update (2026-05-08, → main, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/351

26 commits, +1580/−489, 53 review comments, 0 issue comments, **1 RED→FIX**

- `95467faf60` 2026-04-06 Update PegasusSim lidar to new rtx lidar and optional min_sensor_range parameter to vdb model to avoid self-detection.  ⟶ no runs
- `0bc8306973` 2026-04-06 removed deprecated ouster lidar. Completely integrated new rtx lidar  ⟶ no runs
- `59885ebb88` 2026-04-07 renaming frame id back to ouster  ⟶ no runs
- `9b5da1d305` 2026-05-04 Added node to filter near and invalid lidar points  ⟶ no runs
- `e7b15d5206` 2026-05-04 merge with main  ⟶ no runs
- `d570271170` 2026-05-04 reconciled topic names for lidar point cloud  ⟶ no runs
- `28a58970b3` 2026-05-05 fixed example scripts to use rtx lidar api  ⟶ no runs
- `cb8b616530` 2026-05-05 fixed tmux closing and rclpy path issue  ⟶ no runs
- `28cfaa4e66` 2026-05-05 uses add_rtx in multi px4 script  ⟶ Copilotcodereview=success, CheckVERSIONIncrement=failure, SystemTests=cancelled
- `b5ed13e721` 2026-05-05 bumping version index  ⟶ CheckVERSIONIncrement=success
- `b568c0719d` 2026-05-06 docs added  ⟶ CheckVERSIONIncrement=success
- `1424fbf7d6` 2026-05-06 unit testing and documentation updates  ⟶ Copilotcodereview=success, CheckVERSIONIncrement=success
- `9732ca1dd1` 2026-05-06 cleaning code from copilot suggestions  ⟶ Copilotcodereview=success, CheckVERSIONIncrement=success
- `260ea33c0c` 2026-05-06 Merge branch 'main' into johnliu/rtx_lidar_update  ⟶ AddressingcommentonPR#351=success, CheckVERSIONIncrement=success
- `00082932c6` 2026-05-06 docs(tests): fix pytest marker example for running liveliness and sensors  ⟶ AddressingcommentonPR#351=success
- `1cd7040b7b` 2026-05-06 docs(tests): fix marker semantics in test_sensors module docstring  ⟶ no runs
- `6a8a04afa7` 2026-05-07 addressing github copilot concerns  ⟶ AddressingcommentonPR#351=success, Copilotcodereview=success, CheckVERSIONIncrement=success
- `64e5901cb6` 2026-05-07 docs(bridge): remove stale camera topics comment  ⟶ no runs
- `f1ec85cb4c` 2026-05-08 addressing copilot concerns  ⟶ AddressingcommentonPR#351=success, Copilotcodereview=success, CheckVERSIONIncrement=success
- `a73079c5d8` 2026-05-08 removing debug print statement from reading point cloud  ⟶ CheckVERSIONIncrement=success
- `07a5e7849b` 2026-05-08 fix(isaac-sim): align drone1 lidar prim path with spawned prim  ⟶ no runs
- `c6f0c3fde8` 2026-05-08 more succint comment in sim bashrc  ⟶ no runs
- `792c143cb1` 2026-05-08 resolving discrepant comments in ros bridge yaml  ⟶ no runs
- `a1659fbd86` 2026-05-08 removed bug allocated new copy of point cloud array  ⟶ Copilotcodereview=success, CheckVERSIONIncrement=success
- `11640e0826` 2026-05-08 logs lidaar test with boolean instead of hz  ⟶ CheckVERSIONIncrement=success
- `8a026ad6dc` 2026-05-08 and --> or for marks  ⟶ CheckVERSIONIncrement=success

  **RED→FIX** after `28cfaa4e66` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/25396154695
  followed by: `b5ed13e721` bumping version index; `b568c0719d` docs added; `1424fbf7d6` unit testing and documentation updates

  Review comments:
  - @Copilot `simulation/isaac-sim/launch_scripts/example_two_px4_pegasus_launch_script.py`: `asyncio` is referenced in the scene export block (`asyncio.get_event_loop()...`) but not imported anywhere in this script. If `SAVE_SCENE_TO` is enabled this will crash with `NameError`; add an `impo  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886202)
  - @Copilot `simulation/isaac-sim/docker/docker-compose.yaml`: `isaac-sim-gui` now launches `bash /isaac-sim/AirStack/simulation/isaac-sim/docker/tmux_gui_with_shell.sh`, but that script does not exist in the repository. Either add the script to `simulation/isaac  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886240)
  - @Copilot `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`: This docstring still says the script spawns an "Ouster lidar", but the implementation now uses `add_rtx_lidar_subgraph`. Update the bullet to avoid misleading users about which sensor pipeline is bein  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886275)
  - @Copilot `robot/ros_ws/src/sensors/lidar_point_cloud_filter/package.xml`: `sensor_msgs_py` is imported by the node (`from sensor_msgs_py import point_cloud2`) but the package.xml does not declare a dependency on the `sensor_msgs_py` ROS package. Add it as a `<depend>`/`<exe  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886305)
  - @Copilot `robot/ros_ws/src/sensors/lidar_point_cloud_filter/lidar_point_cloud_filter/lidar_point_cloud_filter_node.py`: `read_points(..., skip_nans=True)` filters NaNs but not +/-Inf. As a result, infinite points can pass through and be published, despite callers/docs expecting invalid points to be removed. Consider fi  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886338)
  - @Copilot `robot/ros_ws/src/sensors/lidar_point_cloud_filter/lidar_point_cloud_filter/lidar_point_cloud_filter_node.py`: The current implementation materializes the point cloud multiple times (`list(read_points(...))` → list comprehension → numpy array → list again). For typical LiDAR clouds this can add significant lat  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886358)
  - @Copilot `simulation/isaac-sim/config/sim_to_robot_bridge.yaml`: The new lidar filter defaults to subscribing to `/.../sensors/ouster/point_cloud_raw`, and the example Isaac scripts set `lidar_topic_name="point_cloud_raw"`, but the sim→robot bridge forwards only `/  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886386)
  - @Copilot `simulation/isaac-sim/launch_scripts/two_drone_scene_import.py`: This launch script switches to `add_rtx_lidar_subgraph` but does not set `lidar_topic_name`, while the example scripts explicitly publish `point_cloud_raw` to feed the new `lidar_point_cloud_filter` d  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886418)
  - @Copilot `simulation/isaac-sim/launch_scripts/two_drone_scene_import.py`: Same as the first drone: `lidar_topic_name` is omitted here, but other launch scripts publish `point_cloud_raw` to integrate with the new point cloud filter. Setting it explicitly avoids topic-name dr  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886432)
  - @Copilot `docs/robot/autonomy/sensors/index.md`: This doc claims the filter removes "NaN or infinite" points, but the current implementation only uses `skip_nans=True` (NaNs) and does not explicitly filter +/-Inf. Either update the implementation to  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886466)
  - @Copilot `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`: `asyncio` is no longer imported in this file (it was removed from the stdlib import block), but it’s still referenced later in the scene export section (`asyncio.get_event_loop().run_until_complete(..  (https://github.com/castacks/AirStack/pull/351#discussion_r3190886487)
  - @Copilot `tests/sensor_probes.py`: `check_lidar_filtered_cloud_sanity()` returns only a 2-tuple when there are fewer robot containers than `env["num_robots"]`, but callers (e.g. `test_sensors.py`) unpack three values `(ok, msg, rates)`  (https://github.com/castacks/AirStack/pull/351#discussion_r3197447100)
  - @Copilot `simulation/isaac-sim/launch_scripts/example_two_px4_pegasus_launch_script.py`: This script uses `asyncio.get_event_loop()` when `SAVE_SCENE_TO` is set, but `asyncio` is never imported. Setting `SAVE_SCENE_TO` will raise `NameError: name 'asyncio' is not defined`. Add the missing  (https://github.com/castacks/AirStack/pull/351#discussion_r3197447141)
  - @Copilot `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`: The optional `SAVE_SCENE_TO` export path later in this script uses `asyncio.get_event_loop()`, but `asyncio` is no longer imported in the updated import block. If `SAVE_SCENE_TO` is enabled, the scrip  (https://github.com/castacks/AirStack/pull/351#discussion_r3197447153)
  - @Copilot `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`: The module docstring still claims this example spawns an "Ouster lidar", but the implementation was switched to `add_rtx_lidar_subgraph` (RTX OmniLidar) with `lidar_config="ouster_os1"`. Update the do  (https://github.com/castacks/AirStack/pull/351#discussion_r3197447184)
  - @Copilot `robot/ros_ws/src/sensors/lidar_point_cloud_filter/package.xml`: `lidar_point_cloud_filter_node.py` imports `sensor_msgs_py.point_cloud2`, but this package is not declared as a dependency in `package.xml`. Relying on it transitively can break builds in slimmer envi  (https://github.com/castacks/AirStack/pull/351#discussion_r3197447208)
  - @Copilot `tests/test_sensors.py`: The docstring suggests running `-m "liveliness and sensors"` to execute both suites, but pytest’s marker expression semantics mean `and` selects tests that have *both* marks (likely none). To run both  (https://github.com/castacks/AirStack/pull/351#discussion_r3197781876)
  - @Copilot `tests/test_sensors.py`: In `test_sensor_streams_stable`, `rates = {**rates_sim, **rates_rsd, **rates_lidar}` overwrites entries because sim-side and robot-side stereo/depth use the same topic names. This drops one side’s sam  (https://github.com/castacks/AirStack/pull/351#discussion_r3197781929)
  - @Copilot `tests/sensor_probes.py`: `sim_side_topics()` appends `(“/clock”, n)` for every robot domain, but downstream `parallel_sample_hz()` keys results by topic string, so duplicate `/clock` entries will overwrite each other and you   (https://github.com/castacks/AirStack/pull/351#discussion_r3197781957)
  - @Copilot `tests/sensor_probes.py`: `check_lidar_filtered_cloud_sanity()` sometimes returns only `(ok, msg)` (when there are fewer robot containers than expected), but callers unpack `(ok, msg, rates)`. This will raise a `ValueError` an  (https://github.com/castacks/AirStack/pull/351#discussion_r3197781976)
  - @Copilot `tests/README.md`: This section states `-m "liveliness and sensors"` runs both suites, but `and` in pytest’s marker expressions selects tests that have both marks (so it will typically run nothing). To run both, use `-m  (https://github.com/castacks/AirStack/pull/351#discussion_r3197781994)
  - @Copilot `simulation/isaac-sim/config/sim_to_robot_bridge.yaml`: The `topics:` section comment says “Camera topics” but camera (and LiDAR) topics are no longer configured below, while the header comment still lists them as bridged. Either add the intended topic map  (https://github.com/castacks/AirStack/pull/351#discussion_r3197782011)
  - @Copilot `simulation/isaac-sim/docker/.bashrc`: `ISAAC_SIM_PYTHONPATH` is constructed as `<filtered>:/isaac-sim/...`. If the filtered prefix is empty (e.g., `PYTHONPATH` unset or fully filtered), this produces a leading `:` which adds the current w  (https://github.com/castacks/AirStack/pull/351#discussion_r3197782038)
  - @Copilot `.agents/skills/run-system-tests/SKILL.md`: This uses `-m "liveliness and sensors"` as an example of running both suites, but pytest’s marker expression `and` selects tests that have both marks (so it will usually run zero tests). Use `-m "live  (https://github.com/castacks/AirStack/pull/351#discussion_r3197782060)
  - @Copilot `tests/sensor_probes.py`: For `isaacsim`, the intended batching is “at most two `ros2 topic hz` clients at once”, but `image_pairs` / `depth_pairs` include topics for *all* robots. With `--num-robots 3` this still launches 6 H  (https://github.com/castacks/AirStack/pull/351#discussion_r3197782090)
  - @Copilot `tests/sensor_probes.py`: `check_robot_stereo_hz()`’s docstring says Isaac probes are batched so “at most two `ros2 topic hz` clients run at once”, but `image_pairs` / `depth_pairs` include topics for all robots. With multiple  (https://github.com/castacks/AirStack/pull/351#discussion_r3197782107)
  - @andrewjong `tests/README.md`: @copilot apply changes based on [this feedback](https://github.com/castacks/AirStack/pull/351#discussion_r3197781994)  (https://github.com/castacks/AirStack/pull/351#discussion_r3197899642)
  - @andrewjong `tests/test_sensors.py`: @copilot apply changes based on [this feedback](https://github.com/castacks/AirStack/pull/351#discussion_r3197781876)  (https://github.com/castacks/AirStack/pull/351#discussion_r3197902234)
  - @Copilot `tests/README.md`: Applied in 0008293. I updated `tests/README.md` to use the correct marker semantics: selecting both suites now references `-m "liveliness or sensors"` instead of `-m "liveliness and sensors"`.  (https://github.com/castacks/AirStack/pull/351#discussion_r3197907377)
  - @Copilot `tests/test_sensors.py`: Applied in 1cd7040. Updated `tests/test_sensors.py` docstring to use correct pytest marker semantics for selecting both suites: `-m "liveliness or sensors"`.  (https://github.com/castacks/AirStack/pull/351#discussion_r3197915195)

## PR #350 — Krrish/coord pr (2026-05-11, → main, @krrishj18)

https://github.com/castacks/AirStack/pull/350

81 commits, +9065/−457, 17 review comments, 7 issue comments, **2 RED→FIX**

- `a06bb25e14` 2026-02-13 Fixed multi-drone global plan  ⟶ no runs
- `3b993b7d88` 2026-02-17 added sep files for fire and retro  ⟶ no runs
- `b68bd9b1fa` 2026-02-18 added robot2 relative pos; diff rviz files; bridge for rayfronts topics  ⟶ no runs
- `5d32861dfc` 2026-02-19 added sharing of semantic rays  ⟶ no runs
- `e68e269542` 2026-02-19 changed rviz for both drones  ⟶ no runs
- `f5a5c9ef34` 2026-02-24 added target sharing  ⟶ no runs
- `efb2e55db4` 2026-02-24 changed drone start pos  ⟶ no runs
- `435837c47e` 2026-03-16 gossip layer w/o relay  ⟶ no runs
- `45dd5e8211` 2026-03-16 added global coords under /{ROBOT_NAME}/interface/mavros/global_position/raw/fix(not my topic, it was already publishing  ⟶ no runs
- `8729411a3f` 2026-03-20 gossip, threedrone,peerprofile  ⟶ no runs
- `9d66ab8224` 2026-03-21 multi drone vis in foxglove, odom doesn't work in foxglove yet  ⟶ no runs
- `cabf9ae0b7` 2026-03-21 multi drone vis in foxglove works with odom  ⟶ no runs
- `ffd1ff6ebb` 2026-03-22 global plan added  ⟶ no runs
- `9fdf0455f3` 2026-03-22 added image, vdb markers(not transformed yet)  ⟶ no runs
- `22322124d3` 2026-03-22 fixed state estimation flickering and vdb transform  ⟶ no runs
- `66f044a9bc` 2026-03-22 added custom foxglove buttons for commands  ⟶ no runs
- `7e344f10e5` 2026-03-23 added modular payloads to peerprofile, foxglove reads the payloads and vizualizes it,currently works for rayfronts  ⟶ no runs
- `ae062e5191` 2026-03-25 fixing the rotation of payload  ⟶ no runs
- `ce32a224e3` 2026-03-25 syncing devices  ⟶ no runs
- `61befb9431` 2026-03-26 fixed gossip + translate  ⟶ no runs
- `ed4abfe5e3` 2026-03-27 added skill for foxglove/coordination  ⟶ no runs
- `ddee8f4543` 2026-03-27 removed VDB ENV  ⟶ no runs
- `088259fdec` 2026-03-27 rebase with main  ⟶ no runs
- `6c9fefa301` 2026-03-27 updated docs  ⟶ no runs
- `27355fac5e` 2026-04-03 fixed launch files so they have play start on sim. scene_prep utils: added non-world prims to save in flattened manner  ⟶ no runs
- `946441e88b` 2026-04-07 created raven_nav package  ⟶ no runs
- `402fe3beef` 2026-04-14 Merge remote-tracking branch 'origin/main' into multi-raven  ⟶ no runs
- `3329d8805c` 2026-04-14 moved coordination to common  ⟶ no runs
- `79a584da65` 2026-04-14 fixed gcs<->robot dds  ⟶ no runs
- `b1f2b06648` 2026-04-14 added hitl functionality  ⟶ no runs
- `4a5019d731` 2026-04-14 fixes to dds  ⟶ no runs
- `c101593ecc` 2026-04-14 put dds hitl under gcs  ⟶ no runs
- `89a572c431` 2026-04-14 fixes to robot hitl  ⟶ no runs
- `bc67c9ae08` 2026-04-14 syncing both computers  ⟶ no runs
- `6a9db210aa` 2026-04-14  mimiced robot-l4t for dataflow  ⟶ no runs
- `e78f29b3ca` 2026-04-14 fixed path to ddsrouter_yaml  ⟶ no runs
- `8f06837e4d` 2026-04-14 fixed dds server  ⟶ no runs
- `c654bc3176` 2026-04-14 fixed two_drone_fire  ⟶ no runs
- `66cf6dc747` 2026-04-15 rayfronts is now a ros package  ⟶ no runs
- `f41d6234e8` 2026-04-15 Merge remote-tracking branch 'origin/main' into multi-raven  ⟶ no runs
- `715938382a` 2026-04-17 added feedback, it's sending success too early though  ⟶ no runs
- `6b53e57a10` 2026-04-23 fixed raven behavior  ⟶ no runs
- `1dcfd6c9cf` 2026-04-24 foxglove panel with working executors  ⟶ no runs
- `bdb416980d` 2026-04-27 random walk fixed  ⟶ no runs
- `243292e768` 2026-04-27 fixed random walk bringup. Added saves and viz for multiple waypoints and polygons  ⟶ no runs
- `5ee9c77d6d` 2026-04-28 fixed bounds for exploration task, combined waypoint/polygon editor into task panel  ⟶ no runs
- `f9fe44c319` 2026-04-28 made waypoint/polygon gui larger  ⟶ no runs
- `b0c822c914` 2026-04-28 added 2d map to foxglove  ⟶ no runs
- `62eb1bddeb` 2026-04-29 WIP: pre-merge snapshot  ⟶ no runs
- `df0de5c9ff` 2026-04-29 Merge origin/main into multi-raven (preserve branch features)  ⟶ no runs
- `7b190b92a2` 2026-04-29 added changes from main  ⟶ no runs
- `5aab17b53b` 2026-04-29 WIP: pre-branch-split snapshot  ⟶ no runs
- `4d91ddf2a0` 2026-04-30 PR for foxglove+multi-robot  ⟶ no runs
- `d75f6566ac` 2026-04-30 Merge remote-tracking branch 'origin/main' into krrish/coord-pr  ⟶ no runs
- `432e423bd7` 2026-04-30 merged with main  ⟶ no runs
- `ed3cacfc9a` 2026-04-30 PR cleanup: revert unrelated changes and drop extra files  ⟶ no runs
- `e4a56d56bb` 2026-04-30 Trim noisy inline comments in PR-added Python files  ⟶ CheckVERSIONIncrement=success, SystemTests=failure
- `957f5089e2` 2026-05-01 Pin vdb_mapping_ros2 to public main (was at unpushed 68fe8dde)  ⟶ CheckVERSIONIncrement=success
- `824d911a3d` 2026-05-01 fixed launch script  ⟶ CheckVERSIONIncrement=success
- `83b72ba8de` 2026-05-01 merged with main  ⟶ CheckVERSIONIncrement=success
- `5bc2803659` 2026-05-05 Merge remote-tracking branch 'origin/main' into krrish/coord-pr  ⟶ no runs
- `38ed80ff2d` 2026-05-05 fixed foxglove bugs, added dynamic fg layout, updated docs  ⟶ Copilotcodereview=success, CheckVERSIONIncrement=success
- `efad619056` 2026-05-08 fixed bugs found by copilot. Removed rviz by adding a node  ⟶ Copilotcodereview=success, CheckVERSIONIncrement=success
- `eb4ed2df0a` 2026-05-08 fixed comment  ⟶ CheckVERSIONIncrement=success
- `85c8addfa6` 2026-05-08 fixed path in skill  ⟶ CheckVERSIONIncrement=success
- `078c182c7f` 2026-05-08 fixes from copilot  ⟶ no runs
- `0b9d79168e` 2026-05-08 Merge remote-tracking branch 'origin/krrish/coord-pr' into krrish/coord-pr  ⟶ no runs
- `a2a1147047` 2026-05-08 Merge remote-tracking branch 'origin/main' into krrish/coord-pr  ⟶ no runs
- `bfdab681ef` 2026-05-08 Fix Pegasus submodule pointer after merge  ⟶ no runs
- `b07cf268cf` 2026-05-08 fix(coordination): align gossip with steady clock + manifest hygiene  ⟶ no runs
- `beba768362` 2026-05-08 fix(gcs+autonomy): drop dead BT panel, lint payload imports, name-map override  ⟶ no runs
- `fba3078e19` 2026-05-08 fix(foxglove): clean panel-id stacking, atomic render, drop dead .foxe  ⟶ no runs
- `21e18206ca` 2026-05-08 bug fixes  ⟶ no runs
- `009d339b35` 2026-05-08 bug fixes  ⟶ CheckVERSIONIncrement=failure
- `d594e1c261` 2026-05-08 version  ⟶ CheckVERSIONIncrement=success
- `a5bb2f62fb` 2026-05-08 reverted env  ⟶ CheckVERSIONIncrement=success
- `fef20d5274` 2026-05-08 updated gitignore and docs  ⟶ CheckVERSIONIncrement=success
- `f92ceb9487` 2026-05-08 updated foxglove viz + consistent spellings across repo  ⟶ CheckVERSIONIncrement=success
- `e7da5ab49a` 2026-05-11 Move layout file to /root/ so it's immediately accessible, also fix template path  ⟶ CheckVERSIONIncrement=success
- `6c25275cc1` 2026-05-11 Change so that file name reflects NUM_ROBOTS  ⟶ CheckVERSIONIncrement=success
- `3f49b90ffb` 2026-05-11 Add a DEBUG_RVIZ flag to launch robot rviz if needed  ⟶ CheckVERSIONIncrement=success

  **RED→FIX** after `e4a56d56bb` (System Tests): https://github.com/castacks/AirStack/actions/runs/25195327326
  followed by: `957f5089e2` Pin vdb_mapping_ros2 to public main (was at unpushed 68fe8dde); `824d911a3d` fixed launch script; `83b72ba8de` merged with main

  **RED→FIX** after `009d339b35` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/25577534420
  followed by: `d594e1c261` version; `a5bb2f62fb` reverted env; `fef20d5274` updated gitignore and docs

  Review comments:
  - @Copilot `robot/ros_ws/src/global/planners/random_walk/src/random_walk_logic.cpp`: check_if_collided() calls point_in_search_bounds() without holding the planner mutex, but search_bounds_xy_ can be mutated concurrently by set_search_bounds()/clear_search_bounds(). This creates a dat  (https://github.com/castacks/AirStack/pull/350#discussion_r3193069369)
  - @Copilot `robot/ros_ws/src/global/planners/random_walk/config/random_walk_config.yaml`: random_walk_config.yaml adds parameters sub_robot_tf_topic and srv_random_walk_toggle_topic, but RandomWalkNode::readParameters() never declares/reads them and the node code doesn't reference them. Th  (https://github.com/castacks/AirStack/pull/350#discussion_r3193069379)
  - @Copilot `gcs/ros_ws/src/gcs_visualizer/package.xml`: gcs_visualizer Python code imports coordination_msgs (PeerProfile) and coordination_bringup (peer_profile/frame_utils), but those packages are not declared as dependencies here. This can break build o  (https://github.com/castacks/AirStack/pull/350#discussion_r3193069385)
  - @Copilot `common/ros_packages/robot_descriptions/launch/robot_state_publisher.launch.py`: The fallback URDF lookup uses FindPackageShare('autonomy_bringup')/urdf, but autonomy_bringup has no urdf/ directory in this repo. With the current default urdf_file_path='robot.urdf.xacro', this bran  (https://github.com/castacks/AirStack/pull/350#discussion_r3193069396)
  - @Copilot `common/ros_packages/coordination/coordination_msgs/msg/PeerProfile.msg`: PeerProfile.msg claims gps_fix.header.stamp carries the NavSatFix timestamp, but gossip_node overwrites gps_fix.header.stamp at publish time for dedup/ordering. This mismatch can confuse downstream co  (https://github.com/castacks/AirStack/pull/350#discussion_r3193069408)
  - @Copilot `simulation/isaac-sim/utils/scene_prep.py`: Collector option flat_collection=True conflicts with the inline comment (“preserve source folder hierarchy”). If you intend to preserve hierarchy, this should likely remain false; if you intend a flat  (https://github.com/castacks/AirStack/pull/350#discussion_r3193069419)
  - @Copilot `.env`: This change updates the default developer configuration (NUM_ROBOTS=2, Isaac script selection) and also introduces a stray typo in the section divider comment. Consider keeping .env defaults conservat  (https://github.com/castacks/AirStack/pull/350#discussion_r3193069427)
  - @Copilot `simulation/isaac-sim/utils/scene_prep.py`: The OmniGraph SET_VALUES entries are using a ((attr, node_path), value) tuple form, but the Isaac Sim/OmniGraph examples in this repo use a single attribute path string like "<Node>.inputs:<attr>". As  (https://github.com/castacks/AirStack/pull/350#discussion_r3210757019)
  - @Copilot `common/ros_packages/coordination/coordination_bringup/coordination_bringup/gossip_node.py`: These timers are created with ROSClock(), but the node’s docstring/log message claims the publish tick is wall-clock (unaffected by sim time). When use_sim_time=true (set in robot.launch.xml for sim),  (https://github.com/castacks/AirStack/pull/350#discussion_r3210757069)
  - @Copilot `common/ros_packages/coordination/coordination_bringup/setup.py`: peer_registry_monitor is installed as scripts/peer_registry_monitor.py but there is no console_scripts entry (and no extensionless wrapper like scripts/gossip_node). This makes the documented command   (https://github.com/castacks/AirStack/pull/350#discussion_r3210757119)
  - @Copilot `gcs/ros_ws/src/gcs_visualizer/gcs_visualizer/gcs_utils.py`: This module has several unused imports (e.g. copy, gps_to_enu as _gps_to_enu_abs, heading_to_quat, rotate_vector, transform_marker_array). With ament_flake8 enabled for gcs_visualizer, these will fail  (https://github.com/castacks/AirStack/pull/350#discussion_r3210757151)
  - @Copilot `.agents/skills/visualize-in-foxglove/SKILL.md`: This skill refers to a non-existent GCS node/file (`robot_marker_node.py` / `robot_marker_node`). In this PR the implementation appears to be `foxglove_visualizer_node.py` under gcs_visualizer. Update  (https://github.com/castacks/AirStack/pull/350#discussion_r3210757187)
  - @Copilot `.agents/skills/attach-gossip-payload/SKILL.md`: This path to gossip_payloads.yaml is incorrect for this repo layout; the file added in this PR is under common/ros_packages/coordination/coordination_bringup/config/gossip_payloads.yaml (not robot/ros  (https://github.com/castacks/AirStack/pull/350#discussion_r3210757215)
  - @Copilot `.agents/skills/attach-gossip-payload/SKILL.md`: This section again points to robot/ros_ws/src/coordination/... for gossip_payloads.yaml, but the config lives under common/ros_packages/coordination/coordination_bringup/config/gossip_payloads.yaml in  (https://github.com/castacks/AirStack/pull/350#discussion_r3210757231)
  - @Copilot `gcs/foxglove_extensions/render_layout.py`: The defaults set both --input and --output to the same airstack_default.json path. Since gcs startup runs this script on every container start, overwriting the template in-place can cause panel IDs to  (https://github.com/castacks/AirStack/pull/350#discussion_r3210757250)
  - @Copilot `common/ros_packages/desktop_bringup/launch/gcs.launch.xml`: Typo in comment: "standbox" → "sandbox".  (https://github.com/castacks/AirStack/pull/350#discussion_r3210757272)
  - @Copilot `common/ros_packages/gui/rviz/rviz_tasks_panel/src/tasks_panel.cpp`: SemanticSearchTask.action now documents background_queries as required for meaningful softmax normalization, but the RViz Tasks panel registers background_queries with an empty default. Consider provi  (https://github.com/castacks/AirStack/pull/350#discussion_r3210757307)

  Conversation:
  - @krrishj18: /pytest -m "liveliness or takeoff_hover_land"  (https://github.com/castacks/AirStack/pull/350#issuecomment-4361480565)
  - @krrishj18: /pytest  (https://github.com/castacks/AirStack/pull/350#issuecomment-4361965967)
  - @andrewjong: /pytest -m "build_docker or build_packages or liveliness or takeoff_hover_land"  (https://github.com/castacks/AirStack/pull/350#issuecomment-4364348531)

## PR #354 — Scene prep bug fix (2026-05-20, → main, @krrishj18)

https://github.com/castacks/AirStack/pull/354

11 commits, +290/−76, 9 review comments, 0 issue comments, **3 RED→FIX**

- `4e2b011268` 2026-05-15 fixes to scene_prep_utils.py  ⟶ CheckVERSIONIncrement=failure, SystemTests=cancelled
- `6f3c8f6bb4` 2026-05-18 edited docs  ⟶ CheckVERSIONIncrement=failure
- `ed095fc0e4` 2026-05-18 clean launch script  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=failure, CheckVERSIONIncrement=failure
- `30287f6bd5` 2026-05-18 updated version  ⟶ Copilotcodereview=success, CheckVERSIONIncrement=success
- `1cf3d16b0d` 2026-05-18 fixed comments inconsistency and typos  ⟶ Copilotcodereview=success, CheckVERSIONIncrement=success
- `825b24ae87` 2026-05-18 formatting fix  ⟶ CheckVERSIONIncrement=success
- `ab2daec373` 2026-05-19 bug in gossip if payload is empty  ⟶ CheckVERSIONIncrement=success
- `8668e3f806` 2026-05-19 fixed omni_pass.env file creation bug from CICD guest default profile  ⟶ no runs
- `053bce0cf2` 2026-05-19 fixed depth topic naming in foxglove gcs  ⟶ CheckVERSIONIncrement=success
- `a02ad93c66` 2026-05-19 changed gps topic  ⟶ CheckVERSIONIncrement=success
- `bf9642e04a` 2026-05-19 removed redundant exntentions  ⟶ CheckVERSIONIncrement=success

  **RED→FIX** after `4e2b011268` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/25935841573
  followed by: `6f3c8f6bb4` edited docs; `ed095fc0e4` clean launch script; `30287f6bd5` updated version

  **RED→FIX** after `6f3c8f6bb4` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/26049766164
  followed by: `ed095fc0e4` clean launch script; `30287f6bd5` updated version; `1cf3d16b0d` fixed comments inconsistency and typos

  **RED→FIX** after `ed095fc0e4` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/26050135474; https://github.com/castacks/AirStack/actions/runs/26049860270
  followed by: `30287f6bd5` updated version; `1cf3d16b0d` fixed comments inconsistency and typos; `825b24ae87` formatting fix

  Review comments:
  - @Copilot `simulation/isaac-sim/utils/scene_prep.py`: `og.Controller.Keys.SET_VALUES` entries are using a tuple-of-(attr,node) form (e.g. `(("inputs:domain_id", nodes["context"]), ...)`), but every other OmniGraph usage in this repo uses the string path   (https://github.com/castacks/AirStack/pull/354#discussion_r3261037055)
  - @Copilot `simulation/isaac-sim/utils/scene_prep.py`: The `dedupe_physics_scenes` docstring says it will “delete the rest”, but the implementation only deactivates duplicates via `SetActive(False)` and never attempts to remove prim specs. Please update t  (https://github.com/castacks/AirStack/pull/354#discussion_r3261037130)
  - @Copilot `simulation/isaac-sim/utils/scene_prep.py`: The `flat_collection=True` argument appears to mean “flatten asset references into one directory”; the inline comment now says “preserve source folder hierarchy”, which is the opposite and contradicts  (https://github.com/castacks/AirStack/pull/354#discussion_r3261037163)
  - @Copilot `docs/simulation/isaac_sim/spawning_drones.md`: Spelling: “Incase” should be “In case”.  (https://github.com/castacks/AirStack/pull/354#discussion_r3261037206)
  - @Copilot `docs/simulation/isaac_sim/spawning_drones.md`: This paragraph implies that calling `save_scene_as_contained_usd(source_usd_url=ENV_URL, ...)` will include scene-prep modifications (scale/colliders/etc). However, `save_scene_as_contained_usd` only   (https://github.com/castacks/AirStack/pull/354#discussion_r3261037246)
  - @Copilot `simulation/isaac-sim/utils/scene_prep.py`: `og.Controller.Keys.SET_VALUES` uses a tuple-of-tuples form like `(("inputs:domain_id", nodes["context"]), ...)`, while the rest of this file (and other OmniGraph examples in the repo) use the standar  (https://github.com/castacks/AirStack/pull/354#discussion_r3261208831)
  - @Copilot `docs/simulation/isaac_sim/spawning_drones.md`: The docs import `scene_prep` as `from utils.scene_prep import ...`, but `simulation/isaac-sim/utils/` is not a Python package in the launch scripts (the examples add that directory to `sys.path` and t  (https://github.com/castacks/AirStack/pull/354#discussion_r3261208893)
  - @Copilot `simulation/isaac-sim/launch_scripts/example_multi_drone_scene_import.py`: Minor formatting: the closing `]` of `DRONE_CONFIGS` is indented deeper than the opening `[` which makes the list harder to scan and differs from typical formatting in the rest of the script. Consider  (https://github.com/castacks/AirStack/pull/354#discussion_r3261208928)
  - @JohnYanxinLiu `simulation/isaac-sim/launch_scripts/example_multi_drone_scene_import.py`: Do we not want to keep these examples? Especially since it's an example file.  (https://github.com/castacks/AirStack/pull/354#discussion_r3269142728)

## PR #352 — feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO (2026-05-22, → develop, @smash0190)

https://github.com/castacks/AirStack/pull/352

27 commits, +2383/−21, 14 review comments, 1 issue comments, **3 RED→FIX**

- `03cd145e96` 2026-05-14 feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO  ⟶ no runs
- `64a2cebf2f` 2026-05-14 fix(osmo): harden CLI + workspace image against stale-state, port-forward race, and cursor-server install hangs  ⟶ no runs
- `769c4a2e82` 2026-05-14 fix(osmo): correct osmo:logs CLI invocation; install Foxglove extensions locally on osmo:foxglove  ⟶ no runs
- `2d9b1611b2` 2026-05-14 fix(osmo): pin Kit livestream UDP media port to 49099 so osmo:webrtc actually shows pixels  ⟶ no runs
- `7c95b4e1be` 2026-05-14 fix(osmo): render Kit GUI in WebRTC stream; document SSH agent forward for in-pod git push  ⟶ no runs
- `02fcc1969c` 2026-05-14 fix(osmo): make osmo:setup idempotent + paste-safe; document Nucleus auth-debug path  ⟶ no runs
- `c7f89a867e` 2026-05-14 fix(osmo): use Nucleus API-token auth, with double-dollar to survive compose parser  ⟶ no runs
- `6f3a8e56bd` 2026-05-15 docs(osmo): make OSMO the recommended dev path, single clone-the-repo flow  ⟶ no runs
- `b61b18b4e4` 2026-05-15 fix(osmo): make osmo:logs actually stream + survive pod host-key churn  ⟶ no runs
- `98b00ad991` 2026-05-15 fix(osmo): auto-pin --branch to local checkout + clean error UX when workflow dies  ⟶ no runs
- `17ca30d1b7` 2026-05-15 perf(osmo): bump inner dockerd concurrency to saturate 10 GbE pulls  ⟶ no runs
- `838ec7dba4` 2026-05-15 docs(osmo): require buildx --platform linux/amd64 for workspace image  ⟶ no runs
- `2dbfdf486e` 2026-05-15 perf(osmo): move dockerd data-root to /osmo/run for native overlay2  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=failure
- `1a26c1938e` 2026-05-19 updated version  ⟶ CheckVERSIONIncrement=success
- `e77337bce8` 2026-05-19 added virtual display for GL context  ⟶ Copilotcodereview=success, CheckVERSIONIncrement=success
- `6981d6a588` 2026-05-19 added virtual display for droan_gl  ⟶ CheckVERSIONIncrement=success
- `8698ce2473` 2026-05-20 Merge branch 'develop' into feat/osmo-integration  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success
- `0e4fbf2a64` 2026-05-22 droan_gl patch  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=failure
- `37968c32c6` 2026-05-22 run Xvfb in its own tmux session  ⟶ CheckVERSIONIncrement=failure, EnforceBranchTargets=success
- `64e1c44d9a` 2026-05-22 updated dockerfile + version  ⟶ RunningCopilotCodeReview=success, EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `02c3b3437e` 2026-05-22 typo in docs  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `e1af102cc3` 2026-05-22 typo in comments  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success
- `02f7a98d9c` 2026-05-22 typo in comments  ⟶ no runs
- `b3d69de525` 2026-05-22 typo in comments  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `e1304399be` 2026-05-22 typo in osmo logs, renamed airstack-isaac-sim to just isaac-sim  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `d760a5b836` 2026-05-22 typo in container name for isaac-sim-livestream  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `1526625539` 2026-05-22 airstack-dev version overwrite removed  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

  **RED→FIX** after `2dbfdf486e` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/25923948777
  followed by: `1a26c1938e` updated version; `e77337bce8` added virtual display for GL context; `6981d6a588` added virtual display for droan_gl

  **RED→FIX** after `0e4fbf2a64` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/26305158098
  followed by: `37968c32c6` run Xvfb in its own tmux session; `64e1c44d9a` updated dockerfile + version; `02c3b3437e` typo in docs

  **RED→FIX** after `37968c32c6` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/26305668103
  followed by: `64e1c44d9a` updated dockerfile + version; `02c3b3437e` typo in docs; `e1af102cc3` typo in comments

  Review comments:
  - @Copilot `osmo/workflows/airstack-dev.yaml`: These raw WebRTC port-forward instructions omit the pinned UDP media port 49099 that the livestream service publishes. Users following this YAML will forward signaling but not SRTP media, resulting in  (https://github.com/castacks/AirStack/pull/352#discussion_r3269679704)
  - @Copilot `osmo/README.md`: The NetworkPolicy requirement does not include UDP 49099, but the new livestream path pins SRTP media to 49099/udp. A pool configured from this table could still block the media stream even though TCP  (https://github.com/castacks/AirStack/pull/352#discussion_r3269679749)
  - @Copilot `osmo/README.md`: These validation commands still forward the old UDP ranges and omit the pinned 49099/udp media port used by the `isaac-sim-livestream` service. Following this section will leave the WebRTC media path   (https://github.com/castacks/AirStack/pull/352#discussion_r3269679769)
  - @Copilot `docs/tutorials/index.md`: This description says the OSMO flow needs no local AirStack clone, but the tutorial's prerequisites and setup steps require cloning the repo so the `airstack osmo:*` wrappers and workflow YAML are ava  (https://github.com/castacks/AirStack/pull/352#discussion_r3269679805)
  - @Copilot `docs/tutorials/airstack_on_osmo.md`: `UseKeychain` is a macOS-specific OpenSSH option; Linux OpenSSH clients generally fail with “Bad configuration option: usekeychain” rather than ignoring it. Because this tutorial targets Linux too, ad  (https://github.com/castacks/AirStack/pull/352#discussion_r3269679822)
  - @Copilot `osmo/workspace/entrypoint.sh`: The suggested Isaac Sim log container name does not match the service started by the default OSMO livestream profile, which sets `container_name: isaac-sim-livestream`. Users copying this command will  (https://github.com/castacks/AirStack/pull/352#discussion_r3269679847)
  - @Copilot `docs/tutorials/airstack_on_osmo.md`: The expected Compose milestone uses the project-prefixed service name, but the livestream service declares `container_name: isaac-sim-livestream`, so Docker Compose will report/start that fixed name i  (https://github.com/castacks/AirStack/pull/352#discussion_r3269679873)
  - @Copilot `.env`: The version is bumped to `0.18.0-alpha.11`, but `CHANGELOG.md` has no corresponding entry for this release. Add the OSMO changes to the changelog under the same version so the release metadata stays c  (https://github.com/castacks/AirStack/pull/352#discussion_r3269679892)
  - @Copilot `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`: `ISAAC_SIM_LIVESTREAM_UDP_PORT` is parsed with `int(...)` without validation; a non-numeric value will raise `ValueError` and crash the launch script before Isaac Sim starts. Consider guarding with a   (https://github.com/castacks/AirStack/pull/352#discussion_r3291651238)
  - @Copilot `osmo/workflows/airstack-dev.yaml`: The workflow hard-codes `VERSION: "droan_gl_test"`, which will override `.env` and likely cause inner `docker compose` pulls to fail unless images are published with that exact tag. For a user-facing   (https://github.com/castacks/AirStack/pull/352#discussion_r3291651260)
  - @Copilot `osmo/workflows/airstack-dev.yaml`: Top-of-file port-forward instructions still reference the old WebRTC port ranges (47995–48012, 49000–49007) and do not include the pinned UDP media port 49099. Since the stack now pins media to 49099   (https://github.com/castacks/AirStack/pull/352#discussion_r3291651269)
  - @Copilot `.airstack/modules/osmo.sh`: `osmo:ide` relies on `nc -z` to detect when the local port-forward is listening, but `nc` is not guaranteed to be installed on all target hosts (especially Windows setups). Consider checking for `nc`   (https://github.com/castacks/AirStack/pull/352#discussion_r3291651282)
  - @Copilot `gcs/foxglove_extensions/install.py`: `json.load(open(pkg_path))` leaves the file handle unclosed. Prefer `with open(pkg_path, "r", encoding="utf-8") as f:` to avoid descriptor leaks and make the read encoding explicit.  (https://github.com/castacks/AirStack/pull/352#discussion_r3291651288)
  - @Copilot `robot/docker/docker-compose.yaml`: After starting Xvfb and waiting up to 10s for `/tmp/.X11-unix/X99`, the script proceeds even if the socket never appears (e.g., Xvfb failed to start). Consider checking for the socket after the loop a  (https://github.com/castacks/AirStack/pull/352#discussion_r3291651296)

  Conversation:
  - @krrishj18: /pytest -m "liveliness or takeoff_hover_land"  (https://github.com/castacks/AirStack/pull/352#issuecomment-4491181090)

## PR #363 — fix(isaac-sim): pegasus drone retains PX4 state across Stop/Play (2026-05-28, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/363

2 commits, +2/−2, 0 review comments, 0 issue comments

- `2068677702` 2026-05-28 Update submodule to point to pegasus fix fixing start/stop behavior  ⟶ no runs
- `3970998c93` 2026-05-28 Bump VERSION to 0.19.0-alpha.2  ⟶ RunningCopilotCodeReview=success, EnforceBranchTargets=success, CheckVERSIONIncrement=success, SystemTests=cancelled

## PR #359 — Johnliu/optitrack autonomy (2026-05-29, → develop, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/359

30 commits, +4599/−159, 13 review comments, 2 issue comments, **4 RED→FIX**

- `dfb798e0cf` 2026-05-12 incremented version tag  ⟶ no runs
- `cf78c9eea1` 2026-05-12 docker image builds on l4t with generalizability features for other ros and linux versions  ⟶ no runs
- `b2531de74e` 2026-05-12 documentation and claude skills for developing a new profile.  ⟶ no runs
- `5821f2d1c6` 2026-05-13 initial natnet implementation  ⟶ no runs
- `be71915d5c` 2026-05-14 deployment to jetson with ros2 jazzy now fixed  ⟶ no runs
- `6312dbdfed` 2026-05-14 unit testing dependency fix  ⟶ no runs
- `1c6f423509` 2026-05-14 added optitrack perception to launch  ⟶ no runs
- `5826ab7c51` 2026-05-14 put tag version back in  ⟶ no runs
- `75b7a44e4f` 2026-05-15 added instructions for Agents to run tests  ⟶ no runs
- `74ce0f4ce4` 2026-05-15 attempt at completely custom Optitrack Parser (Not working)  ⟶ no runs
- `95287d8585` 2026-05-15 fully implemented NatNetSDK natnet ros2 wrapper natively in AirStack. Hand test in mocap room successful  ⟶ no runs
- `c6ed2f86b1` 2026-05-19 unit test restructuring  ⟶ no runs
- `77b32b7966` 2026-05-20 reorganized natnet logic for unit-testability  ⟶ no runs
- `138a303b4d` 2026-05-20 unit testing restructuring to have unit tests in src and proxies in test. Unit tests workflows created  ⟶ UnitTests=failure
- `bd6a4e266b` 2026-05-20 reupdated documentation for current state of testing  ⟶ no runs
- `5377171fd9` 2026-05-20 change unit tests to occur with system tests so that environment is builtgit status  ⟶ no runs
- `ef95968bfd` 2026-05-20 generalizes natnet parameters and disables natnet automatically for launch  ⟶ no runs
- `26119bc1fb` 2026-05-21 natnet client adaptor now references correct error code from NatNet SDK 4.4.0.0  ⟶ no runs
- `b777ed76a4` 2026-05-21 Merge branch 'main' into johnliu/optitrack_autonomy  ⟶ no runs
- `f27911e4a9` 2026-05-21 increment version tag  ⟶ no runs
- `08040b72e7` 2026-05-21 bug fixes to natnet launching from env file  ⟶ no runs
- `6debb778d3` 2026-05-21 fixed failing systems test due to depends issue and specifying unit tests via yaml  ⟶ no runs
- `37119e4981` 2026-05-21 incremented version tag from develop  ⟶ AddressingcommentonPR#359=success, Copilotcodereview=success, EnforceBranchTargets=success, CheckVERSIONIncrement=success, SystemTests=cancelled, CheckVERSIONIncrement=success, EnforceBranchTargets=failure, SystemTests=cancelled
- `6a2d393ca4` 2026-05-26 Use NatNet callback context instead of thread-local dispatch  ⟶ no runs
- `190edafbc6` 2026-05-28 addressing Krrish' documentation comments  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=failure
- `709a765baf` 2026-05-28 incrementing version tag after osmo PR merge  ⟶ no runs
- `9030d4cb22` 2026-05-28 documentation corrections  ⟶ no runs
- `8d3ee46a71` 2026-05-28 Merge branch 'develop' into johnliu/optitrack_autonomy  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `cb2f42ed8e` 2026-05-29 Merge branch 'develop' into johnliu/optitrack_autonomy  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=failure
- `385a085fcc` 2026-05-29 Bump VERSION Update .env  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success

  **RED→FIX** after `138a303b4d` (Unit Tests): https://github.com/castacks/AirStack/actions/runs/26189385268
  followed by: `bd6a4e266b` reupdated documentation for current state of testing; `5377171fd9` change unit tests to occur with system tests so that environment is builtgit status; `ef95968bfd` generalizes natnet parameters and disables natnet automatically for launch

  **RED→FIX** after `37119e4981` (Enforce Branch Targets): https://github.com/castacks/AirStack/actions/runs/26232439963
  followed by: `6a2d393ca4` Use NatNet callback context instead of thread-local dispatch; `190edafbc6` addressing Krrish' documentation comments; `709a765baf` incrementing version tag after osmo PR merge

  **RED→FIX** after `190edafbc6` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/26588042044
  followed by: `709a765baf` incrementing version tag after osmo PR merge; `9030d4cb22` documentation corrections; `8d3ee46a71` Merge branch 'develop' into johnliu/optitrack_autonomy

  **RED→FIX** after `cb2f42ed8e` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/26610216209
  followed by: `385a085fcc` Bump VERSION Update .env

  Review comments:
  - @Copilot `robot/ros_ws/src/perception/natnet_ros2/src/natnet_client_adapter.cpp`: The SDK callback uses a `thread_local` pointer for dispatch. NatNet frame callbacks are typically invoked on an SDK/network receive thread, so the `thread_local` value set in the main thread will be n  (https://github.com/castacks/AirStack/pull/359#discussion_r3282097018)
  - @Copilot `robot/ros_ws/src/perception/natnet_ros2/README.md`: The documented PoseWithCovarianceStamped topic path omits the `{body_name}` segment (shown as `/.../optitrack/pose_cov`). The implementation publishes per-rigid-body topics at `/{ROBOT_NAME}/perceptio  (https://github.com/castacks/AirStack/pull/359#discussion_r3282097112)
  - @Copilot `robot/ros_ws/src/perception/natnet_ros2/README.md`: The configuration example is not in ROS 2 parameter-file format (it shows a `natnet:` root key), but the shipped `config/natnet_config.yaml` uses the standard `/**: ros__parameters:` structure and is   (https://github.com/castacks/AirStack/pull/359#discussion_r3282097163)
  - @krrishj18 `docs/development/intermediate/docker-build-profiles.md`: When I view the docs i can't find a link to this file unless i search for it. Either link it from an existing page under development of have it as an option in the right side menu  (https://github.com/castacks/AirStack/pull/359#discussion_r3312671311)
  - @krrishj18 `airstack.sh`: Do we want natnet to be installed by default?   (https://github.com/castacks/AirStack/pull/359#discussion_r3312711642)
  - @krrishj18 `robot/ros_ws/src/perception/natnet_ros2/README.md`: Inline with my other comment, we should decide if it's enabled by default and accordingly edit this sentence  (https://github.com/castacks/AirStack/pull/359#discussion_r3312732890)
  - @krrishj18 `robot/ros_ws/src/perception/natnet_ros2/README.md`: I think it'll be helpful if we have a link to slite on setting up/calibrating the optitrack system at the RIC.   (https://github.com/castacks/AirStack/pull/359#discussion_r3312762945)
  - @krrishj18 `mkdocs.yml`: Not something you did but i just looked and this state_estimation.md file doesn't exist. Can you just delete this line as part of docs clean up? Thanks   (https://github.com/castacks/AirStack/pull/359#discussion_r3312767766)
  - @JohnYanxinLiu `airstack.sh`: Thanks! We want NatNet to not be installed by default. Should be opt-in for users. Fixed.  (https://github.com/castacks/AirStack/pull/359#discussion_r3313696369)
  - @JohnYanxinLiu `robot/ros_ws/src/perception/natnet_ros2/README.md`: Resolved with previous comment setting skipping SDK installation to true by default.  (https://github.com/castacks/AirStack/pull/359#discussion_r3313701982)
  - @JohnYanxinLiu `robot/ros_ws/src/perception/natnet_ros2/README.md`: I don't think the link to the slite or slides should be added to AirStack docs in that case.  I think AirStack should be strictly kept general for everyone. We can keep CMU/RIC-specific things interna  (https://github.com/castacks/AirStack/pull/359#discussion_r3313753200)
  - @JohnYanxinLiu `docs/development/intermediate/docker-build-profiles.md`: Moved, added to yaml, adjusted location of the doc.  (https://github.com/castacks/AirStack/pull/359#discussion_r3314628196)
  - @JohnYanxinLiu `mkdocs.yml`: cleaned it up  (https://github.com/castacks/AirStack/pull/359#discussion_r3319642314)

  Conversation:
  - @JohnYanxinLiu: /pytest  (https://github.com/castacks/AirStack/pull/359#issuecomment-4566450400)
  - @JohnYanxinLiu: /pytest  (https://github.com/castacks/AirStack/pull/359#issuecomment-4566493248)

## PR #368 — Fix/camera init (2026-07-07, → develop, @krrishj18)

https://github.com/castacks/AirStack/pull/368

3 commits, +4/−2, 0 review comments, 0 issue comments

- `a43467b7b9` 2026-07-03 bump PegasusSimulator: stereo camera_info init fix  ⟶ no runs
- `2dc708d3ec` 2026-07-03 version incr  ⟶ SystemTests=cancelled, EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `d5273bd745` 2026-07-07 merged hotfix back into main of pegasus extension  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

## PR #365 — Add fixed-trajectory system tests with cross-track error metrics (2026-07-10, → develop, @pvkumara)

https://github.com/castacks/AirStack/pull/365

76 commits, +1813/−524, 26 review comments, 0 issue comments

- `480f7ae6ad` 2026-04-27 Add fixed-trajectory evaluation tests  ⟶ no runs
- `b616dbf41c` 2026-04-30 Remove module docstring from test_fixed_trajectory.py  ⟶ no runs
- `d06effbaf0` 2026-04-27 Aj/GitHub ci cd (#347)  ⟶ no runs
- `ee99e88b3d` 2026-04-27 Add fix for boot volume size blocking orchestrator  ⟶ no runs
- `78f50e70ae` 2026-04-28 Add floating IPs to CI/CD  ⟶ no runs
- `d22aa322bd` 2026-04-28 Bump gh runner_version to latest  ⟶ no runs
- `a74b591e11` 2026-04-28 Update cicd defaults  ⟶ no runs
- `0a2c7c8def` 2026-04-28 Rename integration-tests.yml to system-tests.yml  ⟶ no runs
- `862e12b5bd` 2026-04-28 Add debugging tips and add to mkdocs  ⟶ no runs
- `6b70d16922` 2026-04-28 Use venv instead of pip3 to fix error: externally-managed-environment  ⟶ no runs
- `504644c5e3` 2026-04-28 Explicitly fail autonomy test if images not yet built  ⟶ no runs
- `8d200d2e64` 2026-04-28 Enable using docker cache from docker registry to speed up docker image build tests for ci/cd  ⟶ no runs
- `cbe56f9b51` 2026-04-28 Fix bug  ⟶ no runs
- `5d494e372e` 2026-04-28 Update docs and change docker image build/push to also run on self-hosted runner  ⟶ no runs
- `144a7fd858` 2026-04-28 Enable trigger docker build workflow on via manual dispatch  ⟶ no runs
- `4a5f184cff` 2026-04-28 Increase instance volume size so that space doesn't run out when building docker images  ⟶ no runs
- `7512e5b1ad` 2026-04-28 Update to always try build all images  ⟶ no runs
- `f27e432132` 2026-04-28 Create dummy file for docker compose push to pass  ⟶ no runs
- `be6815890b` 2026-04-28 Add omni_pass.env with guest access to AirLab nucleus  ⟶ no runs
- `5cd5f7e0b6` 2026-04-28 Update ci/cd tests to make sure image is present before running tests  ⟶ no runs
- `70e1584f44` 2026-04-29 Make sure images for profiles get built  ⟶ no runs
- `073aa69f63` 2026-04-29 Update system tests to not build images if pull available  ⟶ no runs
- `c70e7fdd15` 2026-04-29 Make build/pull quiet  ⟶ no runs
- `3aa93265a1` 2026-04-29 Pin empy version to fix ROS2 jazzy version bug  ⟶ no runs
- `3bbd970a84` 2026-04-29 Switch image to desktop so that tests run successfully  ⟶ no runs
- `bca80ef5f8` 2026-04-29 Add docker image signing to workflow  ⟶ no runs
- `53b12869af` 2026-04-29 Change pytest mark 'autonomy' to 'takeoff_hover_land'  ⟶ no runs
- `2e0130be50` 2026-04-29 update comments on workflow  ⟶ no runs
- `6fa3843118` 2026-04-29 Recurisve checkout of airstack  ⟶ no runs
- `622e510270` 2026-04-29 Log more to GitHub  ⟶ no runs
- `2f6b43fcb1` 2026-04-30 Better error logging for ci/cd orchestrator  ⟶ no runs
- `2bb8de0545` 2026-04-30 Add check system resources before spawning server; if resources not available, report back and try again later  ⟶ no runs
- `7197a15f5c` 2026-04-30 Make it so that pytest no longer triggers from pushes on PR; make it so we can manually trigger pytest by commenting /py  ⟶ no runs
- `8e12672c09` 2026-04-30 Update PR template  ⟶ no runs
- `92231d72b1` 2026-04-30 Update AGENTS.md  ⟶ no runs
- `d4d41d62b0` 2026-04-30 Fix finding baseline metrics  ⟶ no runs
- `129586944a` 2026-04-30 Update workflow to comment instead of react  ⟶ no runs
- `df244c20f6` 2026-04-30 Fix bug  ⟶ no runs
- `3765172fd1` 2026-04-30 Try fix another bug  ⟶ no runs
- `80fc6d07b7` 2026-04-30 Update omni_pass_TEMPLATE.env to use 'guest'; update default on system tests to include build_packages  ⟶ no runs
- `be72b372a1` 2026-04-30 Auto prepend 'build_packages' mark to ensure code is built before tests  ⟶ no runs
- `7f06a532c2` 2026-04-30 Lower default stress-iterations to 1 and single takeoff-velocity to 0.5  ⟶ no runs
- `ac4c4e1cec` 2026-04-30 Johnliu/px4 cpu optimization (#348)  ⟶ no runs
- `1d437c75db` 2026-05-01 Add new skills  ⟶ no runs
- `fc438e1a21` 2026-05-05 Revise pull request template for clarity and detail  ⟶ no runs
- `d3a0d45059` 2026-05-08 Johnliu/rtx lidar update (#351)  ⟶ no runs
- `63cfd4466d` 2026-05-11 Krrish/coord pr (#350)  ⟶ no runs
- `910fe427db` 2026-05-20 Scene prep bug fix (#354)  ⟶ no runs
- `47958ba18b` 2026-05-20 Add workflows to (1) enforce correct branch merge convention (2) update develop from main  ⟶ no runs
- `648e7d7ebe` 2026-05-20 Update docs on branches  ⟶ no runs
- `a682ded79d` 2026-05-20 Update workflow to handle develop version increment  ⟶ no runs
- `2ccade776b` 2026-05-20 Release 0.18.0  ⟶ no runs
- `179de2aa2b` 2026-05-20 Bump VERSION to  after sync from main  ⟶ no runs
- `35ac5165bf` 2026-05-22 feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO (#352)  ⟶ no runs
- `ce2ae2d198` 2026-04-27 Add fixed-trajectory evaluation tests  ⟶ no runs
- `cb9c96cc1d` 2026-06-03 Spherical lookahead bug that fixed the circle test and caused the circle test to pass  ⟶ no runs
- `59eef1bfa3` 2026-06-05 Added in code that consolidated all the results code so the user can easily see their results in one file without having  ⟶ no runs
- `c6ba9d7e9c` 2026-06-05 Results for 10 tries headless summary statistics  ⟶ no runs
- `df16cb9205` 2026-06-09 Fixed the logging files so now it only outputs one summary file and it doesn't inundate the user with a ton of log files  ⟶ no runs
- `6e147ef42a` 2026-06-09 deleted cleanup_old_results.sh which was a local tool for cleaning up everything  ⟶ no runs
- `e4df6c6841` 2026-06-09 Added preliminary docs to explain changes made  ⟶ no runs
- `6182b93a4b` 2026-06-09 Changed .env to say 0.19.0-alpha.4  ⟶ no runs
- `f288c37f88` 2026-06-29 Resolved all the merge conflicts that are in this file  ⟶ no runs
- `78f5772e8f` 2026-06-30 Merge remote-tracking branch 'origin/develop' into pkumaraTrajectoryTesting  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `8209379201` 2026-07-02 Revert sphere_radius to 1.0; velocity_sphere_radius_multiplier=1.0 makes the fixed value inert  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success
- `4d40505016` 2026-07-02 Remove internal branch reference from baseline; note AirStation hardware  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `70b091d3f4` 2026-07-02 Remove parameter tuning bullet from docs after reverting sphere_radius  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `b232871d17` 2026-07-02 Move system-test prerequisites to index.md and reference it from fixed-trajectory doc  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `d46ccf216f` 2026-07-02 Remove path tracker bug fixes section from docs (covered in PR description)  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `f5d1b1e98c` 2026-07-02 Trim duplicated stack bring-up from manual usage; link to Getting Started  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `ed188f7862` 2026-07-02 Reframe fixed-trajectory doc as end-to-end testing guide  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `92b38257a2` 2026-07-06 removed stale test_sensors file  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success
- `9a02947da0` 2026-07-07 incremented version tag  ⟶ no runs
- `3cb079ab80` 2026-07-09 Fixed the summary.txt file after it broke after a ton of commits were completed.  ⟶ no runs
- `21d6220124` 2026-07-10 resyncing Pegasus module to fixed camera initialization fix  ⟶ no runs
- `7a9b35cabb` 2026-07-10 Merge branch 'develop' into pkumaraTrajectoryTesting  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

  Review comments:
  - @JohnYanxinLiu `.agents/skills/run-system-tests/SKILL.md`: Was there a reason logs were taken out? I feel they would be useful. Feel free to push back tho, I could see it going either way.  (https://github.com/castacks/AirStack/pull/365#discussion_r3502751809)
  - @pvkumara `.agents/skills/run-system-tests/SKILL.md`: Basically, i created a single summary.txt that takes the key metrics from every run and puts all the info in one clean file. when we ran the logs, there were just way too many that were formatted weir  (https://github.com/castacks/AirStack/pull/365#discussion_r3502758341)
  - @JohnYanxinLiu `docs/development/intermediate/testing/end_to_end_testing.md`: Do you think we could actually start up a new file called end_to_end_testing.md instead of very specifically "fixed_trajectory_testing." This would match the rest of the structure and better set the p  (https://github.com/castacks/AirStack/pull/365#discussion_r3502770690)
  - @pvkumara `docs/development/intermediate/testing/end_to_end_testing.md`: Yeah that works for me John. We could do that for sure   (https://github.com/castacks/AirStack/pull/365#discussion_r3502774723)
  - @JohnYanxinLiu `docs/development/intermediate/testing/fixed_trajectory_testing.md`: I don't think we really need to reference internal branches. These docs are meant to become somewhat public. We may archive old branches. I think it's fine to just leave the suggested numbers. Also ma  (https://github.com/castacks/AirStack/pull/365#discussion_r3502806943)
  - @JohnYanxinLiu `docs/development/intermediate/testing/end_to_end_testing.md`: And then we can have a specific fixed_trajectory testing subsection as well, but a lot of these things can build into stronger e2e tests later on I feel like.  (https://github.com/castacks/AirStack/pull/365#discussion_r3502812545)
  - @JohnYanxinLiu `docs/development/intermediate/testing/end_to_end_testing.md`: Could these prereqs be moved to index.md, and then this doc just references index.md?  (https://github.com/castacks/AirStack/pull/365#discussion_r3502820069)
  - @JohnYanxinLiu `docs/development/intermediate/testing/fixed_trajectory_testing.md`: I feel like this should go into the PR template instead of into the direct documentation.  (https://github.com/castacks/AirStack/pull/365#discussion_r3502825748)
  - @JohnYanxinLiu `docs/development/intermediate/testing/fixed_trajectory_testing.md`: Isn't most of this stuff in the beginner docs for AirStack? Can we remove this fluff? Or is there something about this section that the other docs don't have?  (https://github.com/castacks/AirStack/pull/365#discussion_r3502834712)
  - @JohnYanxinLiu `docs/development/intermediate/testing/index.md`: I guess once the fixed_trajectory.md file is renamed, this should be switched to e2e (end_to_end) benchmarking.  (https://github.com/castacks/AirStack/pull/365#discussion_r3502840727)
  - @JohnYanxinLiu `robot/ros_ws/src/local/local_bringup/launch/local.launch.xml`: Just curious, what was the reason for this?  (https://github.com/castacks/AirStack/pull/365#discussion_r3502982512)
  - @JohnYanxinLiu `robot/ros_ws/src/local/local_bringup/launch/local_droan_cpu.launch.xml`: Same question as in local.launch.xml. What is the purpose of this?  (https://github.com/castacks/AirStack/pull/365#discussion_r3502983797)
  - @JohnYanxinLiu `tests/pytest.ini`: Following the e2e testing precedent above, it might be nice to begin putting things together. I feel takeoff_hover_land could be combined with your new autonomy testing to become a general e2e testing  (https://github.com/castacks/AirStack/pull/365#discussion_r3502995122)
  - @JohnYanxinLiu `tests/README.md`: If we're combining other things into an e2e categorization, make sure to update this documentation  (https://github.com/castacks/AirStack/pull/365#discussion_r3502997228)
  - @krrishj18 `tests/conftest.py`: Why are we removing this? Can't we leave it in as optional logs?   (https://github.com/castacks/AirStack/pull/365#discussion_r3507614299)
  - @pvkumara `robot/ros_ws/src/local/local_bringup/launch/local.launch.xml`: yeah I think this is an artifact of me trying to fix why the drone was stalling during simulation, so I changed the radius to give the drone more look ahead so the pure pursuit path tracker could work  (https://github.com/castacks/AirStack/pull/365#discussion_r3514285551)
  - @pvkumara `robot/ros_ws/src/local/local_bringup/launch/local_droan_cpu.launch.xml`: This is the same issue that was in the local.launch.xml, I will change it back to 1.0 to make it consistent.  (https://github.com/castacks/AirStack/pull/365#discussion_r3514291027)
  - @pvkumara `docs/development/intermediate/testing/fixed_trajectory_testing.md`: I removed the mention of my internal branch and I made sure to say that it was on a AirStation where the code was validated.  (https://github.com/castacks/AirStack/pull/365#discussion_r3514381586)
  - @pvkumara `docs/development/intermediate/testing/end_to_end_testing.md`: Yeah, this is actually a much better setup. I put the prereqs into index.md and then I added a reference in the .md file for the end to end testing referencing the index.md prereq section.  (https://github.com/castacks/AirStack/pull/365#discussion_r3514495622)
  - @pvkumara `docs/development/intermediate/testing/fixed_trajectory_testing.md`: yeah lol, I kept that there for myself and forgot to remove it. I removed that section from the file now.  (https://github.com/castacks/AirStack/pull/365#discussion_r3514526171)
  - @pvkumara `docs/development/intermediate/testing/fixed_trajectory_testing.md`: I removed the initial boiler plate stuff that does the airstack up bring-up, the takeoff action block, the land action block, and the airstack down stuff and only left the fixed trajectory task dispat  (https://github.com/castacks/AirStack/pull/365#discussion_r3514580553)
  - @pvkumara `docs/development/intermediate/testing/end_to_end_testing.md`: Thanks for the e2e testing suggestion — I reworked the docs around it in this PR. Here's what I did and what I'm leaving as follow-up.  (https://github.com/castacks/AirStack/pull/365#discussion_r3514844042)
  - @pvkumara `docs/development/intermediate/testing/index.md`: yep switched that with my new end to end testing file changes.  (https://github.com/castacks/AirStack/pull/365#discussion_r3514848118)
  - @pvkumara `tests/pytest.ini`: yeah I made a doc combining how everything is going to be lined out in this PR, but I think we should do the entire upheaval in another PR down the line to actually confirm all this so this PR doesn't  (https://github.com/castacks/AirStack/pull/365#discussion_r3514885789)
  - @pvkumara `tests/README.md`: yeah once the next PR happens and we're good, I'm going to update the README documentation.   (https://github.com/castacks/AirStack/pull/365#discussion_r3514889061)
  - @pvkumara `tests/conftest.py`: The logs are still all in the terminal, so you can technically still see all of them. The issue with the logs was that they spit out a bunch of different unstructured information for every test, so if  (https://github.com/castacks/AirStack/pull/365#discussion_r3514896660)

## PR #370 — l4t infra fixes: aarch64 build args + robot-name resolution (2026-07-21, → develop, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/370

1 commits, +47/−30, 0 review comments, 0 issue comments

- `2d71bf29a7` 2026-07-17 Generic robot infra: aarch64 build args + robot-name resolution fixes  ⟶ SystemTests=cancelled, EnforceBranchTargets=success, CheckVERSIONIncrement=success, SystemTests=cancelled, EnforceBranchTargets=success, CheckVERSIONIncrement=success

## PR #371 — l4t deployment fixes: make the Jetson profile build + boot on real hardware (2026-07-22, → develop, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/371

4 commits, +89/−14, 0 review comments, 2 issue comments

- `a151edd8a1` 2026-07-21 feat(l4t): make robot-l4t deployment knobs overridable + document name resolution  ⟶ no runs
- `db50f988b3` 2026-07-21 feat(l4t): add site-agnostic l4t-px4-realrobot override template  ⟶ no runs
- `aa1e9acfb0` 2026-07-21 fix(l4t): entrypoint passthrough + ZED SDK 5.2; document build gotchas  ⟶ no runs
- `61a94b929c` 2026-07-21 chore: bump version to 0.19.0-alpha.7  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

  Conversation:
  - @JohnYanxinLiu: Successfully built and ran on real jetson hardware.  (https://github.com/castacks/AirStack/pull/371#issuecomment-5040222329)

## PR #372 — Test infra rework: YAML-driven unit-test collection + integration tier (2026-07-31, → develop, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/372

13 commits, +1145/−916, 0 review comments, 0 issue comments

- `a151edd8a1` 2026-07-21 feat(l4t): make robot-l4t deployment knobs overridable + document name resolution  ⟶ no runs
- `db50f988b3` 2026-07-21 feat(l4t): add site-agnostic l4t-px4-realrobot override template  ⟶ no runs
- `aa1e9acfb0` 2026-07-21 fix(l4t): entrypoint passthrough + ZED SDK 5.2; document build gotchas  ⟶ no runs
- `61a94b929c` 2026-07-21 chore: bump version to 0.19.0-alpha.7  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `77d385189c` 2026-07-22 test(infra): collect co-located unit tests via the package list + integration tier  ⟶ no runs
- `26f7d97e45` 2026-07-22 docs(testing): describe unit tests as co-located and listed in the package YAML  ⟶ no runs
- `21e28f7380` 2026-07-22 chore: bump version to 0.19.0-alpha.8  ⟶ no runs
- `ef3492fced` 2026-07-22 refactor(tests): split unit-test discovery + session state into tests/harness/  ⟶ no runs
- `24919d5db6` 2026-07-22 refactor(tests): extract commands/containers/metrics/sim helpers into tests/harness/  ⟶ no runs
- `35be702d07` 2026-07-22 refactor(tests): extract collection ordering into tests/harness/collection.py  ⟶ no runs
- `b732196dac` 2026-07-22 Merge branch 'develop' into johnliu/test-infra-rework  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success
- `0b131d62fe` 2026-07-22 fix(robot): pin pytest to 7.4.* so apt launch_pytest stays compatible  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `cc833a300b` 2026-07-23 fix(isaac-sim): clear LD_LIBRARY_PATH for PX4 ubuntu.sh so ca-certificates configures  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

## PR #378 — Add waypoint_flight system test judged by a standalone track checker (2026-08-04, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/378

4 commits, +687/−2, 0 review comments, 4 issue comments, **1 RED→FIX**

- `586849af62` 2026-08-04 Add waypoint_flight system test judged by standalone track checker  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, SystemTests=failure
- `e41e30dcf3` 2026-08-04 Calibrate waypoint_flight to validated stock behavior in Isaac Sim  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `50be34dd86` 2026-08-04 Raise default waypoint route +10m to clear scene clutter  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success
- `53e0a11503` 2026-08-04 Add waypoint_flight screenshots from validation runs  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

  **RED→FIX** after `586849af62` (System Tests): https://github.com/castacks/AirStack/actions/runs/30941508053
  followed by: `e41e30dcf3` Calibrate waypoint_flight to validated stock behavior in Isaac Sim; `50be34dd86` Raise default waypoint route +10m to clear scene clutter; `53e0a11503` Add waypoint_flight screenshots from validation runs

  Conversation:
  - @andrewjong: ## Validation: full flight in Isaac Sim (stock stack) ✅  (https://github.com/castacks/AirStack/pull/378#issuecomment-5183843738)
  - @andrewjong: ## Validation part 2: ms-airsim (Blocks) + Isaac re-confirmation ✅  (https://github.com/castacks/AirStack/pull/378#issuecomment-5184120829)
  - @andrewjong: ## Screenshots from the validation flights 📸  (https://github.com/castacks/AirStack/pull/378#issuecomment-5184930481)
  - @pvkumara: /pytest  (https://github.com/castacks/AirStack/pull/378#issuecomment-5289192783)

## PR #381 — Add feature-notebook workflow: per-feature design specs + test results feeding PRs (2026-08-05, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/381

3 commits, +282/−8, 0 review comments, 0 issue comments, **1 RED→FIX**

- `689f2d40ad` 2026-08-05 Add feature-notebook workflow: local design specs + test results per feature  ⟶ no runs
- `d1364bef53` 2026-08-05 Bump version to 0.19.0-alpha.10  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success, SystemTests=failure
- `8da9eaf5a0` 2026-08-05 Document the feature notebook workflow under Development docs  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

  **RED→FIX** after `d1364bef53` (System Tests): https://github.com/castacks/AirStack/actions/runs/30962701610
  followed by: `8da9eaf5a0` Document the feature notebook workflow under Development docs

## PR #377 — Robot deployment fixes: bag recording + adding warning for robot-identity failure (2026-08-05, → develop, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/377

8 commits, +120/−31, 0 review comments, 0 issue comments

- `38be4ba09b` 2026-08-04 make RECORD_BAGS actually reach the bag recorder  ⟶ no runs
- `ff39dedbac` 2026-08-04 warn when the robot identity fails to resolve  ⟶ no runs
- `fd4b1319e8` 2026-08-04 chore: bump version to 0.19.0-alpha.12  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `05715469a0` 2026-08-04 fixed comments and documentation  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success
- `cec4be2362` 2026-08-05 fix the bag recording status bridge direction  ⟶ no runs
- `a9ab6774a7` 2026-08-05 fix the exclude flag so the main bag section records  ⟶ no runs
- `d89765b3bd` 2026-08-05 restore the bags .gitignore files  ⟶ no runs
- `0ed3bb2c13` 2026-08-05 Merge branch 'develop' into johnliu/robot-deployment-fixes  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

## PR #382 — ci: land OSMO ephemeral runners and system-test harness on develop (2026-08-14, → develop, @pvkumara)

https://github.com/castacks/AirStack/pull/382

19 commits, +2708/−862, 0 review comments, 1 issue comments, **5 RED→FIX**

- `ee1c269a3c` 2026-07-27 ci(orchestrator): migrate ephemeral CI runners from OpenStack to NVIDIA OSMO  ⟶ no runs
- `88d8c59a5f` 2026-07-29 ci(orchestrator): pin AirLab OSMO JSON keys and runner image path  ⟶ SystemTests=cancelled, SystemTests=failure, SystemTests=cancelled
- `165d01138a` 2026-08-07 docs(ci): document the OSMO-backed CI/CD pipeline  ⟶ no runs
- `f56810dca5` 2026-08-07 fix(ci): repair Docker builds on OSMO ephemeral runners  ⟶ SystemTests=failure
- `4583165626` 2026-08-07 fix(ci): seed PR Docker builds from a floating cache tag  ⟶ no runs
- `5550b76ec7` 2026-08-07 ci(docker-build): retag unchanged images on VERSION bump  ⟶ AutoBuildonDockerImageTagChange=failure
- `b093327b69` 2026-08-07 fix(ci): parse quoted .env values before inline comments  ⟶ AutoBuildonDockerImageTagChange=failure
- `56a60c7cc6` 2026-08-08 ci(docker-build): build/push services sequentially  ⟶ AutoBuildonDockerImageTagChange=failure
- `624dec798e` 2026-08-08 chore: bump VERSION to 0.19.0-alpha.8 for retag validation  ⟶ AutoBuildonDockerImageTagChange=success
- `d18efe9762` 2026-08-11 fix(ci): unblock isaac-sim PX4 apt and robot colcon pytest  ⟶ no runs
- `81f79105fd` 2026-08-11 fix(ci): pass colcon --pytest-args as separate tokens  ⟶ no runs
- `959380d34d` 2026-08-12 fix(ci): quote colcon pytest args through bash -ic  ⟶ no runs
- `9667722f95` 2026-08-12 fix(ci): pass colcon pytest flags via PYTEST_ADDOPTS  ⟶ no runs
- `8acd7a5152` 2026-08-12 fix(ci): rename helper so pytest does not treat it as a hook  ⟶ no runs
- `241d86a0aa` 2026-08-12 ci: skip image-build for build_packages reruns  ⟶ no runs
- `865eb0de95` 2026-08-12 fix(ci): disable pytest plugin autoload for colcon tests  ⟶ no runs
- `40f526f21d` 2026-08-13 fix(ci): skip lidar ament linters in package pytest config  ⟶ no runs
- `f937b50fb1` 2026-08-14 ci: default system tests to isaacsim only  ⟶ SystemTests=cancelled
- `95a8d87802` 2026-08-14 Merge origin/develop into ci/osmo-orchestrator  ⟶ CheckVERSIONIncrement=success, SystemTests=failure, EnforceBranchTargets=success

  **RED→FIX** after `88d8c59a5f` (System Tests): https://github.com/castacks/AirStack/actions/runs/30657350644
  followed by: `165d01138a` docs(ci): document the OSMO-backed CI/CD pipeline; `f56810dca5` fix(ci): repair Docker builds on OSMO ephemeral runners; `4583165626` fix(ci): seed PR Docker builds from a floating cache tag

  **RED→FIX** after `f56810dca5` (System Tests): https://github.com/castacks/AirStack/actions/runs/31210711111
  followed by: `4583165626` fix(ci): seed PR Docker builds from a floating cache tag; `5550b76ec7` ci(docker-build): retag unchanged images on VERSION bump; `b093327b69` fix(ci): parse quoted .env values before inline comments

  **RED→FIX** after `5550b76ec7` (Auto Build on Docker Image Tag Change): https://github.com/castacks/AirStack/actions/runs/31219923715
  followed by: `b093327b69` fix(ci): parse quoted .env values before inline comments; `56a60c7cc6` ci(docker-build): build/push services sequentially; `624dec798e` chore: bump VERSION to 0.19.0-alpha.8 for retag validation

  **RED→FIX** after `b093327b69` (Auto Build on Docker Image Tag Change): https://github.com/castacks/AirStack/actions/runs/31220240599
  followed by: `56a60c7cc6` ci(docker-build): build/push services sequentially; `624dec798e` chore: bump VERSION to 0.19.0-alpha.8 for retag validation; `d18efe9762` fix(ci): unblock isaac-sim PX4 apt and robot colcon pytest

  **RED→FIX** after `56a60c7cc6` (Auto Build on Docker Image Tag Change): https://github.com/castacks/AirStack/actions/runs/31237536714
  followed by: `624dec798e` chore: bump VERSION to 0.19.0-alpha.8 for retag validation; `d18efe9762` fix(ci): unblock isaac-sim PX4 apt and robot colcon pytest; `81f79105fd` fix(ci): pass colcon --pytest-args as separate tokens

## PR #374 — OptiTrack (1/3): robot-side NatNet client + PX4 external-vision fusion (2026-08-15, → develop, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/374

19 commits, +1956/−333, 0 review comments, 0 issue comments

- `81f1e80c73` 2026-07-23 feat(perception): bring natnet_ros2 client up to the optitrack_emulation baseline  ⟶ no runs
- `6f3d94e9c7` 2026-07-23 feat(natnet): real-robot PX4 external-vision fusion (mocap → EKF2)  ⟶ no runs
- `632b715bcb` 2026-07-23 docs(natnet): PX4 external-vision setup guide + height-datum explainer  ⟶ no runs
- `38102087e3` 2026-07-23 feat(perception): point natnet launch include at the natnet_config schema  ⟶ no runs
- `b5f0efe8f1` 2026-07-23 chore: bump version to 0.19.0-alpha.14  ⟶ no runs
- `56814b3424` 2026-07-31 fix(natnet): make the NatNet client actually reachable + correct EV tuning  ⟶ no runs
- `926f5bb3d5` 2026-08-13 add a real-robot OptiTrack deployment override  ⟶ no runs
- `7dcdd17f7e` 2026-08-13 config bodies per robot profile; trim comments to the docs  ⟶ no runs
- `7b5bacd7e0` 2026-08-13 put the mocap floor at the shared world datum  ⟶ no runs
- `2a5d89bdb1` 2026-08-13 fail the build when the geoid dataset is missing  ⟶ no runs
- `76d3f8e1b3` 2026-08-14 abbreviated Dockerfile comment on geographic lib installation  ⟶ no runs
- `164e37b53b` 2026-08-14 fix repo-root doc links in the external-vision guide  ⟶ no runs
- `536bf21775` 2026-08-14 comment trim  ⟶ no runs
- `337bcb71ca` 2026-08-14 point the companion-link section at the PX4 docs  ⟶ no runs
- `2e365fed5c` 2026-08-14 frame section 4 around mavros_gp_origin, demote the 36 m note  ⟶ no runs
- `de383c7870` 2026-08-14 reject an unknown connection_type instead of defaulting to unicast  ⟶ no runs
- `50eb6a5e71` 2026-08-14 px4 external vision docs trim  ⟶ no runs
- `b3a03a05f3` 2026-08-14 trim natnet node comments; note the latency figure is an estimate  ⟶ no runs
- `8fbb1c0434` 2026-08-14 trim the external-vision tuning notes  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

## PR #375 — OptiTrack (2/3): NatNet server emulator + host integration tests (2026-08-16, → develop, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/375

8 commits, +2653/−1, 0 review comments, 0 issue comments

- `83b70568e1` 2026-07-23 feat(sim): add NatNet server emulator (protocol core) + register unit tests  ⟶ no runs
- `2c6af01ef4` 2026-07-23 test(natnet): host integration tests — emulator server → natnet_ros2  ⟶ no runs
- `10657581f8` 2026-07-23 chore: bump version to 0.19.0-alpha.15  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `630f40d2a0` 2026-08-15 pack frame sections through one helper  ⟶ no runs
- `ddd28996cf` 2026-08-15 fix the labeled-marker struct format that raised on every pack  ⟶ no runs
- `3b25facfd2` 2026-08-15 ignore the editable-install egg-info in the emulator extension  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `f72be26028` 2026-08-15 comment trim  ⟶ no runs
- `fc7043480e` 2026-08-15 put helper-module dirs on sys.path for co-located unit tests  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success

## PR #376 — OptiTrack (3/3): Isaac wrapper, mocap EV fusion in sim, and a Circle-trajectory e2e (2026-08-17, → develop, @JohnYanxinLiu)

https://github.com/castacks/AirStack/pull/376

28 commits, +4689/−11, 6 review comments, 3 issue comments

- `c6063db04f` 2026-07-23 feat(sim): Isaac wrapper for the NatNet emulator (USD scene → server)  ⟶ no runs
- `21e6000705` 2026-07-23 test(natnet): dedicated OptiTrack sim e2e (optitrack mark)  ⟶ no runs
- `323e203b6e` 2026-07-23 docs(natnet): emulator sim doc + optitrack-development skill  ⟶ no runs
- `f1f8290f6a` 2026-07-23 chore: bump version to 0.19.0-alpha.16  ⟶ no runs
- `6a808bf8c2` 2026-07-31 fix(sim): register the NatNet emulator via the Kit ext-folder  ⟶ no runs
- `fe3fc72e8a` 2026-08-12 make the sim actually fuse the mocap stream  ⟶ no runs
- `5e9bf19a28` 2026-08-12 fly a circle on mocap fusion instead of asserting a topic exists  ⟶ no runs
- `6f647f51c1` 2026-08-12 enforce only the mocap circle flight on PR open  ⟶ no runs
- `29c64c8004` 2026-08-13 add an isaac natnet mocap override  ⟶ no runs
- `80d5f2f51e` 2026-08-13 install the natnet emulator as a real Kit extension  ⟶ no runs
- `59935486ff` 2026-08-13 set the streamed body in the script, not the environment  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `1e6f79feba` 2026-08-16 comment trim on isaac-sim docker compose  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `4bad9607e0` 2026-08-16 point the isaac-sim env blocks at their documentation  ⟶ no runs
- `92da44f18c` 2026-08-16 comment trim on editable installation of natnet emulator  ⟶ no runs
- `89e1e09cd7` 2026-08-16 keep the full default test run on PR open  ⟶ no runs
- `8bb38ba059` 2026-08-16 wait for a converged estimate before arming in the optitrack e2e  ⟶ no runs
- `d984111823` 2026-08-16 comment trim on optitrack e2e collection ordering  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success
- `8af9e980f0` 2026-08-16 comment trim on the PR-open test args  ⟶ no runs
- `c9e9681723` 2026-08-16 rename the isaac natnet override to isaac-optitrack-simulation.env  ⟶ no runs
- `c0157af3bf` 2026-08-16 select PX4 SITL parameters with a named env_file  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `9e1fed3564` 2026-08-16 comment trim in compose file  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `5094a7eee5` 2026-08-16 trim verbose comments in the natnet sources  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `01f0867144` 2026-08-16 removed comment change  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `0fcb102141` 2026-08-16 drop the GPS origin change from the baseline pegasus launch script  ⟶ no runs
- `aad472a8e3` 2026-08-16 assert the external-vision params actually reached the FCU  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success
- `f3b335945d` 2026-08-17 docs(natnet): publish the emulator page and correct the setup examples  ⟶ no runs
- `02da092c8e` 2026-08-17 feat(natnet): the extension owns the server, tied to the sim timeline  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `d0a0863434` 2026-08-17 docs trim  ⟶ RunningCopilotCodeReview=success, EnforceBranchTargets=success, CheckVERSIONIncrement=success

  Review comments:
  - @Copilot `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/config.py`: BodyBinding.from_dict rejects an explicitly empty target_prim (""), but the rest of the config model/docs treat an empty target as valid (e.g., UI can add a body before it’s pointed). This also breaks  (https://github.com/castacks/AirStack/pull/376#discussion_r3793967216)
  - @Copilot `docs/simulation/isaac_sim/natnet_emulator.md`: The doc says frames are encoded in “NatNet 4.1 wire format”, but the emulator config and tests advertise/use NatNet 4.4 (e.g., default natnet_version is 4.4.0.0). This is likely to confuse users when   (https://github.com/castacks/AirStack/pull/376#discussion_r3793967235)
  - @Copilot `docs/simulation/isaac_sim/natnet_emulator.md`: The documented default publish rate is 120 Hz, but the code defaults to 100 Hz (NatNetInterfaceConfig.DEFAULT_PUBLISH_RATE=100.0 and schema default natnet:publishRate=100). The docs should match the a  (https://github.com/castacks/AirStack/pull/376#discussion_r3793967245)
  - @Copilot `docs/simulation/isaac_sim/natnet_emulator.md`: The troubleshooting section still references SITL_PARAM_PROFILE=px4-vision, but earlier in this doc (and in the new sim override) the EV configuration is described via PX4_PARAM_SET=external-vision. M  (https://github.com/castacks/AirStack/pull/376#discussion_r3793967259)
  - @Copilot `simulation/isaac-sim/extensions/optitrack.natnet.emulator/optitrack/natnet/emulator/isaac/frames.py`: The module docstring claims this path is “Pure Python + ctypes”, but the implementation imports NumPy and SciPy (Rotation). Either make those imports optional/lazy, or update the docstring to reflect   (https://github.com/castacks/AirStack/pull/376#discussion_r3793967279)
  - @Copilot `tests/system/test_optitrack_e2e.py`: Trailing whitespace in the module docstring (line ends with an extra space). This can cause lint noise and makes diffs harder to read.  (https://github.com/castacks/AirStack/pull/376#discussion_r3793967302)

  Conversation:
  - @pvkumara: /pytest  (https://github.com/castacks/AirStack/pull/376#issuecomment-5319725381)

## PR #384 — CI/CD Tuning PR - pytest collection bug fix (2026-08-18, → develop, @pvkumara)

https://github.com/castacks/AirStack/pull/384

6 commits, +1377/−303, 0 review comments, 1 issue comments, **1 RED→FIX**

- `a1b3763191` 2026-08-17 docs(tests): align unit-test docs with the co-located layout  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, SystemTests=failure
- `30f0a79d20` 2026-08-17 docs(tests): record how C++ and Python unit tests reach CI  ⟶ no runs
- `a615a2466e` 2026-08-17 fix(tests): collect co-located unit tests when the run is not narrowed  ⟶ no runs
- `6a77c39c44` 2026-08-17 docs(tests): explain why C++ and Python unit tests use different runners  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `25c266fa14` 2026-08-18 test: run the collection contract tests with the fast tier  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success
- `4a78ea28cf` 2026-08-18 fix(ci): make PR validation and metrics trustworthy  ⟶ EnforceBranchTargets=success, UnitTests=success, SystemTests=failure, CheckVERSIONIncrement=success

  **RED→FIX** after `a1b3763191` (System Tests): https://github.com/castacks/AirStack/actions/runs/32073548050
  followed by: `30f0a79d20` docs(tests): record how C++ and Python unit tests reach CI; `a615a2466e` fix(tests): collect co-located unit tests when the run is not narrowed; `6a77c39c44` docs(tests): explain why C++ and Python unit tests use different runners

## PR #386 — Pre-RFC workflow cleanup: intent-based launch, readiness gates, launch-script dedup, truthful logs (2026-08-20, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/386

6 commits, +1617/−1753, 0 review comments, 4 issue comments

- `87a22f4b54` 2026-08-20 refactor(isaac): dedupe launch scripts into shared PegasusApp base  ⟶ no runs
- `af9bb1919b` 2026-08-20 feat(cli): intent flags on 'up', resolved-value preflight, and 'airstack ready'  ⟶ no runs
- `090356bf0e` 2026-08-20 feat(docker): tee tmux pane output to container stdout  ⟶ no runs
- `a5ff57237a` 2026-08-20 docs: fix launch-workflow drift against actual code behavior  ⟶ no runs
- `90b9955c97` 2026-08-20 chore(release): bump VERSION to 0.19.0-alpha.18 and update CHANGELOG  ⟶ no runs
- `b1a3e4a22e` 2026-08-20 docs(sim): document PegasusApp launch-script authoring; re-teach stale skills  ⟶ CheckVERSIONIncrement=success, UnitTests=failure, SystemTests=failure, EnforceBranchTargets=success

  Conversation:
  - @andrewjong: /pytest -m liveliness --sim msairsim  (https://github.com/castacks/AirStack/pull/386#issuecomment-5359165306)

## PR #397 — Release 0.19.0 — version + changelog promotion (2026-08-22, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/397

1 commits, +3/−1, 0 review comments, 1 issue comments

- `16ca2cc7f5` 2026-08-22 Release 0.19.0  ⟶ SystemTests=failure, EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=failure

## PR #398 — Release 0.19.0 (2026-08-22, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/398

21 commits, +24706/−3508, 0 review comments, 1 issue comments, **2 RED→FIX**

- `cce3977cae` 2026-05-20 Bump VERSION to  after sync from main  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `25d52da575` 2026-05-22 feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO (#352)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `dff3dc6f76` 2026-05-28 fix(isaac-sim): pegasus drone retains PX4 state across Stop/Play (#363)  ⟶ AutoBuildonDockerImageTagChange=cancelled
- `8b927e465c` 2026-05-29 Johnliu/optitrack autonomy (#359)  ⟶ AutoBuildonDockerImageTagChange=cancelled, Build/PublishDevelopDocs=success
- `6279bea8be` 2026-07-07 Fix/camera init (#368)  ⟶ AutoBuildonDockerImageTagChange=cancelled
- `fa990f4dc0` 2026-07-10 Add fixed-trajectory system tests with cross-track error metrics (#365)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `c476db32a4` 2026-07-21 General robot deployment infra: aarch64 build args + robot-name resolution fixes (#370)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `1a25d60b43` 2026-07-22 l4t deployment fixes: make the Jetson profile build + boot on real hardware (#371)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `47f8c798d5` 2026-07-31 Test infra rework: YAML-driven unit-test collection + integration tier (#372)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `55d9b887d9` 2026-08-04 Add waypoint_flight system test judged by a standalone track checker (#378)  ⟶ AutoBuildonDockerImageTagChange=failure, Build/PublishDevelopDocs=success
- `ae2e942029` 2026-08-05 Add feature-notebook workflow: per-feature design specs + test results feeding PRs (#381)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `9788b14a86` 2026-08-05 Remove stray files  ⟶ no runs
- `234587aa05` 2026-08-05 Robot deployment fixes: bag recording + adding warning for robot-identity failure (#377)  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `5cf595523e` 2026-08-14 ci: land OSMO ephemeral runners and system-test harness on develop (#382)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `0ae96fe8c4` 2026-08-15 OptiTrack (1/3): robot-side NatNet client + PX4 external-vision fusion (#374)  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `19402c2223` 2026-08-16 OptiTrack (2/3): NatNet server emulator + host integration tests (#375)  ⟶ AutoBuildonDockerImageTagChange=cancelled, Build/PublishDevelopDocs=success
- `1c41f8c029` 2026-08-17 OptiTrack (3/3): Isaac wrapper, mocap EV fusion in sim, and a Circle-trajectory e2e (#376)  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `f4697265e4` 2026-08-18 CI/CD Tuning PR - pytest collection bug fix (#384)  ⟶ SystemTests=failure, SystemTests=cancelled, AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `9e2e0e3991` 2026-08-20 docs(skills): require dates and timestamps in feature notebook entries  ⟶ no runs
- `262f12679a` 2026-08-20 Pre-RFC workflow cleanup: intent-based launch, readiness gates, launch-script dedup, truthful logs (#386)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `9104249b35` 2026-08-22 Release 0.19.0 (#397)  ⟶ SystemTests=failure, EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=failure, AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success

  **RED→FIX** after `55d9b887d9` (Auto Build on Docker Image Tag Change): https://github.com/castacks/AirStack/actions/runs/30953320508
  followed by: `ae2e942029` Add feature-notebook workflow: per-feature design specs + test results feeding PRs (#381); `9788b14a86` Remove stray files; `234587aa05` Robot deployment fixes: bag recording + adding warning for robot-identity failure (#377)

  **RED→FIX** after `f4697265e4` (System Tests): https://github.com/castacks/AirStack/actions/runs/32194437120
  followed by: `9e2e0e3991` docs(skills): require dates and timestamps in feature notebook entries; `262f12679a` Pre-RFC workflow cleanup: intent-based launch, readiness gates, launch-script dedup, truthful logs (#386); `9104249b35` Release 0.19.0 (#397)

## PR #388 — Modular AirStack 1/9: wiring observability, module manifest, reusable module CI (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/388

7 commits, +10478/−2, 0 review comments, 2 issue comments, **1 RED→FIX**

- `b6e4dba963` 2026-08-20 P1(rfc-379): module.yaml manifest schema, validator, fixture module, contract tests  ⟶ no runs
- `05db6c5f9e` 2026-08-20 fix(cli): PID-suffix effective-config run dirs to avoid same-second collisions  ⟶ no runs
- `5c6c4a4191` 2026-08-20 P0(rfc-379): observed wiring-snapshot tool, wiring mark, drift check  ⟶ no runs
- `11f51c538b` 2026-08-20 P3(rfc-379): reusable module-system-tests workflow + module CI docs  ⟶ no runs
- `b2364b96ca` 2026-08-20 P0(rfc-379): first observed wiring golden + snapshot robustness fixes  ⟶ no runs
- `aebd6e49eb` 2026-08-22 Bump version to 0.20.0-alpha.1  ⟶ EnforceBranchTargets=success, UnitTests=failure, CheckVERSIONIncrement=success, SystemTests=failure
- `84b15b9a65` 2026-08-22 Merge develop (Release 0.19.0) — keep the stack's 0.20.0-alpha.1  ⟶ SystemTests=failure, EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=failure

  **RED→FIX** after `aebd6e49eb` (System Tests, Unit Tests): https://github.com/castacks/AirStack/actions/runs/32561435375; https://github.com/castacks/AirStack/actions/runs/32561435334
  followed by: `84b15b9a65` Merge develop (Release 0.19.0) — keep the stack's 0.20.0-alpha.1

## PR #389 — Modular AirStack 2/9: module CLI, workspace overlay, Docker layer composition (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/389

6 commits, +2917/−1, 0 review comments, 1 issue comments

- `326c27a745` 2026-08-20 P2(rfc-379): airstack module CLI, vcs2l sync, workspace overlay  ⟶ no runs
- `73a53ee82b` 2026-08-20 P2(rfc-379): auto-include generated module compose override in airstack up  ⟶ no runs
- `6d40d9aa3e` 2026-08-20 P2(rfc-379): update sync hint and docs for automatic module-compose include  ⟶ no runs
- `4a76935c04` 2026-08-20 P4(rfc-379): docker module-layer composition, layer plan, modules.lock  ⟶ no runs
- `7933f405e8` 2026-08-20 P2(rfc-379): key Isaac overlay placements off targets, not module type  ⟶ no runs
- `592f6cf9d5` 2026-08-22 Bump version to 0.20.0-alpha.2  ⟶ CheckVERSIONIncrement=success, SystemTests=failure, SystemTests=cancelled, CheckVERSIONIncrement=success

## PR #390 — Modular AirStack 3/9: reference stacks (wrap form), --stack dispatch, single-locus lint (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/390

4 commits, +24341/−300, 0 review comments, 1 issue comments

- `19c4adc4e2` 2026-08-20 P5-E1(rfc-379): reference stack folders (wrap form), --stack dispatch, single-locus lint  ⟶ no runs
- `5efffa3d1b` 2026-08-20 P5-E1(rfc-379): observed wiring.md for all three reference stacks  ⟶ no runs
- `e361ef18ac` 2026-08-20 P3(rfc-379): build module Docker layers in the reusable workflow  ⟶ no runs
- `f1c1352b5b` 2026-08-22 Bump version to 0.20.0-alpha.3  ⟶ SystemTests=failure, CheckVERSIONIncrement=success

## PR #391 — Modular AirStack 4/9: asm_optitrack extraction, local-layer flatten (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/391

7 commits, +781/−11871, 0 review comments, 0 issue comments

- `70423c4cb1` 2026-08-20 M2(rfc-379): remove OptiTrack/natnet from trunk — extracted to castacks/asm_optitrack  ⟶ no runs
- `70a6fc665f` 2026-08-20 P5-E2(rfc-379): flatten the local layer into stack files  ⟶ no runs
- `1477df3938` 2026-08-20 P3(rfc-379): hook_env input for module host_setup hooks  ⟶ no runs
- `76af860239` 2026-08-20 P2(rfc-379): warn on stale colcon caches for extracted module packages  ⟶ no runs
- `e58caad8a2` 2026-08-20 P5(rfc-379): stack entries must never include the dispatcher (contract)  ⟶ no runs
- `b0541c15a6` 2026-08-20 P2(rfc-379): sync self-heals stale partial module checkouts  ⟶ no runs
- `7980fb0bbd` 2026-08-22 Bump version to 0.20.0-alpha.4  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success

## PR #392 — Modular AirStack 5/9: asm_macvo extraction (−65% image), full flatten, split stacks, doctor (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/392

8 commits, +11706/−2464, 0 review comments, 0 issue comments

- `aa578de902` 2026-08-20 fix(tests): widen landing phase-timeout margin — evidence-based  ⟶ no runs
- `b951fd30ab` 2026-08-20 M3(rfc-379): remove MACVO from trunk — extracted to castacks/asm_macvo  ⟶ no runs
- `343fc5c4a4` 2026-08-21 M3(rfc-379): regen wiring baselines post-MACVO removal; exclude sim render-pipeline nodes  ⟶ no runs
- `115c991acf` 2026-08-21 M3(rfc-379): bless full_macvo wiring baseline on the composed module image  ⟶ no runs
- `6f436d8798` 2026-08-21 P5-E3a(rfc-379): flatten perception/sensors/global/behavior into stacks; prefix generic launch args  ⟶ no runs
- `9cc7b3383a` 2026-08-21 P5-E3b(rfc-379): split stack + bridge.yaml, doctor, interface conventions spec, stack CLI  ⟶ no runs
- `2a178e6812` 2026-08-21 P5-E3(rfc-379): bless lite_default wiring baseline  ⟶ no runs
- `4044aa15ef` 2026-08-22 Bump version to 0.20.0-alpha.5  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success

## PR #393 — Modular AirStack 6/9: vehicles, fleets, airstack.yaml, generic fleet spawner (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/393

8 commits, +2796/−30, 0 review comments, 0 issue comments

- `38f228aa03` 2026-08-21 P6(rfc-380): airstack.yaml, vehicles, fleets, fleet resolver, generic Isaac spawner  ⟶ no runs
- `efb7c86860` 2026-08-21 P6(rfc-380): fix fleet-service discovery in ready.sh; clamp harness campaigns to fleet size  ⟶ no runs
- `ff35e368bc` 2026-08-21 P6(rfc-380): mount .airstack/generated into robot containers; auto-generate bridge router configs  ⟶ no runs
- `a9f56b0a10` 2026-08-21 P6(rfc-380): doctor --live handles heterogeneous fleets  ⟶ no runs
- `643d0c3475` 2026-08-21 P6(rfc-380): single-robot fleets use the historical single-drone prim names  ⟶ no runs
- `b408ee7193` 2026-08-21 fix(tests): wait for the state-estimate watchdog before commanding takeoff  ⟶ no runs
- `b8999066a8` 2026-08-21 P6(rfc-380): airstack down sees generated module/fleet compose services  ⟶ no runs
- `17de16f0cc` 2026-08-22 Bump version to 0.20.0-alpha.6  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success

## PR #394 — Modular AirStack 7/9: module catalog + AUTONOMY_ROLE removal (breaking) (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/394

7 commits, +2062/−11716, 0 review comments, 0 issue comments

- `3852ae2d31` 2026-08-21 P7(rfc-379): marketplace catalog, module-docs fetch, new-developer walkthrough  ⟶ no runs
- `2b0274058a` 2026-08-21 chore: delete the stillborn ensemble_planner skeleton  ⟶ no runs
- `1ccbb20244` 2026-08-21 chore(launch): legibility sweep — delete dead launch files, purge dead blocks, header discipline  ⟶ no runs
- `46c499bd19` 2026-08-21 Remove the legacy AUTONOMY_ROLE dispatch — stacks are the only launch path  ⟶ no runs
- `c9e3cf6eab` 2026-08-21 P2(rfc-379): module remove survives root-owned container artifacts  ⟶ no runs
- `9184110beb` 2026-08-21 docs(stacks): why stacks don't launch standalone — the dispatcher philosophy  ⟶ no runs
- `4f4a22410b` 2026-08-22 Bump version to 0.20.0-alpha.7  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success

## PR #395 — Modular AirStack 8/9: full-stack audit — deletions, dep purge, simple-sim, BSD-3-Clause-Clear (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/395

15 commits, +6967/−83526, 0 review comments, 2 issue comments, **1 RED→FIX**

- `0a13f04c9e` 2026-08-22 audit(cli): truthful help/README, one fleet pipeline, shared lib, hardened commands  ⟶ no runs
- `8242ae85cb` 2026-08-22 audit(robot): index-collision fix, live plugin param, honest baselines, orphan purge  ⟶ no runs
- `0b7b33d85b` 2026-08-22 audit(gcs/sim): fleet-aware GCS, distinct GPS homes, sshd fix, dead-file purge  ⟶ no runs
- `dbead2a748` 2026-08-22 audit(docs): link integrity 81->4, truthful CHANGELOG, root README, beginner funnel  ⟶ no runs
- `6ea3e810fd` 2026-08-22 chore(audit): execute owner-ruled deletions — dead interfaces, GUIs, sensors trio, TAK, standalone examples  ⟶ no runs
- `4d97b89f19` 2026-08-22 feat(sim): adopt simple-sim as a first-class --sim target; doc isaac-sim-gui; demote prebuilt to tag-only; drop WinTAK C  ⟶ no runs
- `96ad668eee` 2026-08-22 chore(docker): dependency purge — robot image sheds unused apt/pip deps (-152MB)  ⟶ no runs
- `4cacc81d78` 2026-08-22 chore(modules): re-pin asm_macvo to 06f3a8c8 — module now declares rich/tqdm  ⟶ no runs
- `8f269766fe` 2026-08-22 chore(license): relicense trunk to BSD-3-Clause-Clear; real maintainers + descriptions everywhere  ⟶ no runs
- `002dd2366d` 2026-08-22 fix(simple-sim): source ROS Jazzy, not Humble — sim never started since the Jazzy migration  ⟶ no runs
- `277b6eb397` 2026-08-22 fix(tests): --stack accepts <name>[:<entry>] like the CLI; re-pin asm_macvo past its stale-import fix  ⟶ no runs
- `c9bc2b43f2` 2026-08-22 chore(audit): delete rqt_behavior_tree — zero referrers after the GUI-set removal  ⟶ no runs
- `298cb72eaa` 2026-08-22 Bump version to 0.20.0-alpha.8  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success
- `0a161dce87` 2026-08-22 Merge develop (Release 0.19.0) — keep this branch's audit CHANGELOG and 0.20.0-alpha.8  ⟶ CheckVERSIONIncrement=success, SystemTests=failure
- `7d2782100b` 2026-08-24 Merge develop (stack levels 1-7) — keep 0.20.0-alpha.8  ⟶ SystemTests=failure, EnforceBranchTargets=success, CheckVERSIONIncrement=failure, UnitTests=failure

  **RED→FIX** after `0a161dce87` (System Tests): https://github.com/castacks/AirStack/actions/runs/32591310686
  followed by: `7d2782100b` Merge develop (stack levels 1-7) — keep 0.20.0-alpha.8

## PR #396 — Modular AirStack 9/9: standalone-snapshot docs, versioned Release Notes replace CHANGELOG (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/396

8 commits, +1252/−1315, 0 review comments, 1 issue comments, **3 RED→FIX**

- `c97e30831a` 2026-08-22 docs: versioned Release Notes replace CHANGELOG.md; docs-are-snapshots policy in the writing skill  ⟶ no runs
- `385ca6854c` 2026-08-22 docs: deep standalone-snapshot audit — RFC scrub, motivation-first openings, code-truth corrections  ⟶ no runs
- `d4802e4082` 2026-08-22 chore(audit): remove flagged leftovers — vestigial rqt trajectory selector, orphaned images, empty stub page  ⟶ no runs
- `86da1f9076` 2026-08-22 Bump version to 0.20.0-alpha.9  ⟶ CheckVERSIONIncrement=success, SystemTests=failure
- `5efa388877` 2026-08-22 Release 0.19.0  ⟶ no runs
- `a871838ee5` 2026-08-22 docs(release-notes): record 0.19.0 — the versioned page now carries the full release history  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=failure
- `06981aa5f7` 2026-08-22 Merge rfc/s8-audit (develop release sync) — CHANGELOG stays deleted, VERSION stays 0.20.0-alpha.9  ⟶ CheckVERSIONIncrement=failure, SystemTests=cancelled
- `03ef065647` 2026-08-22 fix(.env): restore the audited env comments and the 0.20.0-alpha.9 bump  ⟶ CheckVERSIONIncrement=success, SystemTests=cancelled

  **RED→FIX** after `86da1f9076` (System Tests): https://github.com/castacks/AirStack/actions/runs/32561448070
  followed by: `5efa388877` Release 0.19.0; `a871838ee5` docs(release-notes): record 0.19.0 — the versioned page now carries the full release history; `06981aa5f7` Merge rfc/s8-audit (develop release sync) — CHANGELOG stays deleted, VERSION stays 0.20.0-alpha.9

  **RED→FIX** after `a871838ee5` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/32590640607
  followed by: `06981aa5f7` Merge rfc/s8-audit (develop release sync) — CHANGELOG stays deleted, VERSION stays 0.20.0-alpha.9; `03ef065647` fix(.env): restore the audited env comments and the 0.20.0-alpha.9 bump

  **RED→FIX** after `06981aa5f7` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/32591326340
  followed by: `03ef065647` fix(.env): restore the audited env comments and the 0.20.0-alpha.9 bump

## PR #401 — feat(gcs): auto-load NUM_ROBOTS-matched Foxglove layout — no manual import (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/401

1 commits, +146/−24, 0 review comments, 1 issue comments

- `b3056ae9e0` 2026-08-24 feat(gcs): auto-load NUM_ROBOTS-matched Foxglove layout — no manual import  ⟶ UnitTests=failure, EnforceBranchTargets=success, CheckVERSIONIncrement=success, SystemTests=failure

## PR #402 — fix(bringup): loud AUTOLAUNCH FAILED banner instead of silent tmux prompt (2026-08-24, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/402

1 commits, +61/−5, 0 review comments, 1 issue comments

- `e9c79a8ccc` 2026-08-24 fix(bringup): loud AUTOLAUNCH FAILED banner instead of silent tmux prompt  ⟶ CheckVERSIONIncrement=success, UnitTests=failure, SystemTests=failure, EnforceBranchTargets=success

## PR #404 — docs: Diátaxis documentation overhaul (2026-08-25, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/404

13 commits, +3555/−3074, 0 review comments, 0 issue comments

- `6d0bf18d0e` 2026-08-25 docs: fix 18 verified accuracy defects across the docs tree  ⟶ no runs
- `31f2ffc2f5` 2026-08-25 docs: remove fossil pages, add redirects, deprecate git-hooks  ⟶ no runs
- `20a6b1f6b4` 2026-08-25 docs: surface orphaned knowledge into the site nav  ⟶ no runs
- `6197dd8c7b` 2026-08-25 docs: restructure nav into Diátaxis quadrant tabs  ⟶ no runs
- `d2dbe79c32` 2026-08-25 docs: de-duplicate drift-prone content to canonical homes  ⟶ no runs
- `25ccbd84d4` 2026-08-25 docs: split hybrid pages by quadrant and audience  ⟶ no runs
- `1b988c99f2` 2026-08-25 docs: rewrite wrong/filler content and fill layer-index stubs  ⟶ no runs
- `4c3ccf3d63` 2026-08-25 docs: fill the P0/P1 gaps — new reference and onboarding docs  ⟶ no runs
- `65e3930d69` 2026-08-25 docs: adopt Diátaxis in the authoring standards; bump to 0.20.0-alpha.15  ⟶ no runs
- `c98a3d1e85` 2026-08-25 docs: six new how-to guides, Autonomy section, Concepts tab reorder  ⟶ no runs
- `d73d44e754` 2026-08-25 docs: five beginner tutorials, controller how-to, world-model+planner guide  ⟶ no runs
- `ba9432557a` 2026-08-25 docs(tutorials): use airstack connect instead of docker exec one-liners  ⟶ no runs
- `3f0e4aee14` 2026-08-25 Rename Tutorials to Beginner Tutorials  ⟶ EnforceBranchTargets=success, SystemTests=cancelled, CheckVERSIONIncrement=success, UnitTests=failure

## PR #405 — feat(stacks): full_mighty reference stack + mighty module marketplace entry (2026-08-29, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/405

1 commits, +335/−3, 0 review comments, 1 issue comments

- `b32775ad1f` 2026-08-28 feat(stacks): full_mighty reference stack + mighty module catalog entry  ⟶ UnitTests=failure, SystemTests=failure, EnforceBranchTargets=success, CheckVERSIONIncrement=success

## PR #407 — fix(ci): un-red every PR — metrics-job deps and unit-test workflow environment (2026-08-29, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/407

1 commits, +44/−3, 0 review comments, 1 issue comments

- `a8ce342d99` 2026-08-29 fix(ci): un-red every PR — metrics-job deps and unit-test workflow environment  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

## PR #400 — Orchestrator: multi-repo polling for asm_* module CI (2026-08-29, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/400

2 commits, +64/−29, 0 review comments, 2 issue comments, **1 RED→FIX**

- `e3d866b663` 2026-08-24 feat(orchestrator): poll a repos: list — one instance covers trunk + module repos  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, SystemTests=failure, UnitTests=failure
- `5e8f223245` 2026-08-29 Merge develop (CI fixes #407) into orchestrator-multi-repo; VERSION -> 0.20.0-alpha.19  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

  **RED→FIX** after `e3d866b663` (System Tests, Unit Tests): https://github.com/castacks/AirStack/actions/runs/32775855167; https://github.com/castacks/AirStack/actions/runs/32775855089
  followed by: `5e8f223245` Merge develop (CI fixes #407) into orchestrator-multi-repo; VERSION -> 0.20.0-alpha.19

## PR #403 — Ci/trustworthy system tests (2026-08-29, → develop, @pvkumara)

https://github.com/castacks/AirStack/pull/403

8 commits, +1027/−182, 0 review comments, 4 issue comments, **3 RED→FIX**

- `7d2b4dae9d` 2026-08-21 Make system-test outcomes trustworthy and actionable  ⟶ SystemTests=failure, SystemTests=success
- `be9ed0d6a2` 2026-08-21 Classify readiness failures as infrastructure  ⟶ SystemTests=cancelled, SystemTests=success, SystemTests=failure
- `4640b56aa0` 2026-08-21 Allow focused manual flight campaigns  ⟶ SystemTests=success
- `acd21dd269` 2026-08-29 Merge commit to take care of all the merge conflicts for the PR 403  ⟶ no runs
- `2222efdd35` 2026-08-29 Finish leftover merge markers from the develop rebase.  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=failure, UnitTests=failure, SystemTests=success
- `c4d2ca21c1` 2026-08-29 Fix unit-test merge fallout and bump VERSION.  ⟶ UnitTests=success, CheckVERSIONIncrement=success, SystemTests=success, EnforceBranchTargets=success
- `83a537c0c7` 2026-08-29 Merge develop into ci/trustworthy-system-tests; VERSION -> 0.20.0-alpha.20  ⟶ no runs
- `7d5a566f6c` 2026-08-29 Strip post-extraction residue and metadata plumbing; refresh CI reference docs  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

  **RED→FIX** after `7d2b4dae9d` (System Tests): https://github.com/castacks/AirStack/actions/runs/32528385863
  followed by: `be9ed0d6a2` Classify readiness failures as infrastructure; `4640b56aa0` Allow focused manual flight campaigns; `acd21dd269` Merge commit to take care of all the merge conflicts for the PR 403

  **RED→FIX** after `be9ed0d6a2` (System Tests): https://github.com/castacks/AirStack/actions/runs/32529755897
  followed by: `4640b56aa0` Allow focused manual flight campaigns; `acd21dd269` Merge commit to take care of all the merge conflicts for the PR 403; `2222efdd35` Finish leftover merge markers from the develop rebase.

  **RED→FIX** after `2222efdd35` (Check VERSION Increment, Unit Tests): https://github.com/castacks/AirStack/actions/runs/33234614147; https://github.com/castacks/AirStack/actions/runs/33234614177
  followed by: `c4d2ca21c1` Fix unit-test merge fallout and bump VERSION.; `83a537c0c7` Merge develop into ci/trustworthy-system-tests; VERSION -> 0.20.0-alpha.20; `7d5a566f6c` Strip post-extraction residue and metadata plumbing; refresh CI reference docs

  Conversation:
  - @andrewjong: From Claude Fable:  (https://github.com/castacks/AirStack/pull/403#issuecomment-5460890340)

## PR #408 — feat(ci): registry→trunk catalog sync automation + drift alarm (2026-08-29, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/408

2 commits, +290/−4, 0 review comments, 1 issue comments

- `95bae1c1aa` 2026-08-29 feat(ci): registry→trunk catalog sync automation + drift alarm  ⟶ EnforceBranchTargets=success, UnitTests=success, CheckVERSIONIncrement=success, SystemTests=cancelled
- `9b7339b180` 2026-08-29 sync-modules-index: prefer REGISTRY_SYNC_TOKEN so bot PRs trigger CI  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

## PR #409 — Release 0.20.0 (2026-08-29, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/409

1 commits, +178/−176, 0 review comments, 1 issue comments

- `29616d7f57` 2026-08-29 Release 0.20.0: switch pre-release terminology to -dev.N, single revamped README  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

## PR #411 — Re-establish main↔develop ancestry for release 0.20.0 (2026-08-29, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/411

3 commits, +0/−0, 0 review comments, 1 issue comments, **2 RED→FIX**

- `f14ad6d6da` 2026-08-22 Release 0.19.0 (#398)  ⟶ SystemTests=skipped, AutoBuildonDockerImageTagChange=success, SystemTests=skipped, Build/PublishLatestReleaseDocs=failure, GraphUpdate:pipin/.agents/skills/add-ros2-package/assets/package_template,/.github/orchestrator,/robot/ros_ws/src/sensors/lidar_point_cloud_filter,/simulation/isaac-sim/extensions/optitrack.natnet.emulator,/tests#1536328705=success, AutoBuildonDockerImageTagChange=cancelled, Syncmain→develop=success, Build/PublishMainDocs=success
- `3b5416cae4` 2026-08-29 ci: register sync-modules-index + updated develop-docs deploy on main  ⟶ Syncmodulecatalogfromregistry=success, SystemTests=failure, SystemTests=skipped, Syncmain→develop=failure
- `aa2784c26e` 2026-08-29 Merge main into develop (ours): re-establish common ancestry for release 0.20.0  ⟶ SystemTests=success, CheckVERSIONIncrement=failure, EnforceBranchTargets=success, UnitTests=success

  **RED→FIX** after `f14ad6d6da` (Build/Publish Latest Release Docs): https://github.com/castacks/AirStack/actions/runs/32590442072
  followed by: `3b5416cae4` ci: register sync-modules-index + updated develop-docs deploy on main; `aa2784c26e` Merge main into develop (ours): re-establish common ancestry for release 0.20.0

  **RED→FIX** after `3b5416cae4` (Sync main → develop, System Tests): https://github.com/castacks/AirStack/actions/runs/33251381975; https://github.com/castacks/AirStack/actions/runs/33240957587
  followed by: `aa2784c26e` Merge main into develop (ours): re-establish common ancestry for release 0.20.0

## PR #410 — Release 0.20.0 (2026-08-29, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/410

129 commits, +33189/−76955, 0 review comments, 1 issue comments, **18 RED→FIX**

- `cce3977cae` 2026-05-20 Bump VERSION to  after sync from main  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `25d52da575` 2026-05-22 feat(osmo): VS Code/Cursor dev workflow on NVIDIA OSMO (#352)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `dff3dc6f76` 2026-05-28 fix(isaac-sim): pegasus drone retains PX4 state across Stop/Play (#363)  ⟶ AutoBuildonDockerImageTagChange=cancelled
- `8b927e465c` 2026-05-29 Johnliu/optitrack autonomy (#359)  ⟶ AutoBuildonDockerImageTagChange=cancelled, Build/PublishDevelopDocs=success
- `6279bea8be` 2026-07-07 Fix/camera init (#368)  ⟶ AutoBuildonDockerImageTagChange=cancelled
- `fa990f4dc0` 2026-07-10 Add fixed-trajectory system tests with cross-track error metrics (#365)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `c476db32a4` 2026-07-21 General robot deployment infra: aarch64 build args + robot-name resolution fixes (#370)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `1a25d60b43` 2026-07-22 l4t deployment fixes: make the Jetson profile build + boot on real hardware (#371)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `47f8c798d5` 2026-07-31 Test infra rework: YAML-driven unit-test collection + integration tier (#372)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `55d9b887d9` 2026-08-04 Add waypoint_flight system test judged by a standalone track checker (#378)  ⟶ AutoBuildonDockerImageTagChange=failure, Build/PublishDevelopDocs=success
- `ae2e942029` 2026-08-05 Add feature-notebook workflow: per-feature design specs + test results feeding PRs (#381)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `9788b14a86` 2026-08-05 Remove stray files  ⟶ no runs
- `234587aa05` 2026-08-05 Robot deployment fixes: bag recording + adding warning for robot-identity failure (#377)  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `5cf595523e` 2026-08-14 ci: land OSMO ephemeral runners and system-test harness on develop (#382)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `0ae96fe8c4` 2026-08-15 OptiTrack (1/3): robot-side NatNet client + PX4 external-vision fusion (#374)  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `19402c2223` 2026-08-16 OptiTrack (2/3): NatNet server emulator + host integration tests (#375)  ⟶ AutoBuildonDockerImageTagChange=cancelled, Build/PublishDevelopDocs=success
- `1c41f8c029` 2026-08-17 OptiTrack (3/3): Isaac wrapper, mocap EV fusion in sim, and a Circle-trajectory e2e (#376)  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `f4697265e4` 2026-08-18 CI/CD Tuning PR - pytest collection bug fix (#384)  ⟶ SystemTests=failure, SystemTests=cancelled, AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `9e2e0e3991` 2026-08-20 docs(skills): require dates and timestamps in feature notebook entries  ⟶ no runs
- `262f12679a` 2026-08-20 Pre-RFC workflow cleanup: intent-based launch, readiness gates, launch-script dedup, truthful logs (#386)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `b6e4dba963` 2026-08-20 P1(rfc-379): module.yaml manifest schema, validator, fixture module, contract tests  ⟶ no runs
- `05db6c5f9e` 2026-08-20 fix(cli): PID-suffix effective-config run dirs to avoid same-second collisions  ⟶ no runs
- `5c6c4a4191` 2026-08-20 P0(rfc-379): observed wiring-snapshot tool, wiring mark, drift check  ⟶ no runs
- `11f51c538b` 2026-08-20 P3(rfc-379): reusable module-system-tests workflow + module CI docs  ⟶ no runs
- `b2364b96ca` 2026-08-20 P0(rfc-379): first observed wiring golden + snapshot robustness fixes  ⟶ no runs
- `aa578de902` 2026-08-20 fix(tests): widen landing phase-timeout margin — evidence-based  ⟶ no runs
- `38f228aa03` 2026-08-21 P6(rfc-380): airstack.yaml, vehicles, fleets, fleet resolver, generic Isaac spawner  ⟶ no runs
- `70423c4cb1` 2026-08-20 M2(rfc-379): remove OptiTrack/natnet from trunk — extracted to castacks/asm_optitrack  ⟶ no runs
- `b951fd30ab` 2026-08-20 M3(rfc-379): remove MACVO from trunk — extracted to castacks/asm_macvo  ⟶ no runs
- `efb7c86860` 2026-08-21 P6(rfc-380): fix fleet-service discovery in ready.sh; clamp harness campaigns to fleet size  ⟶ no runs
- `326c27a745` 2026-08-20 P2(rfc-379): airstack module CLI, vcs2l sync, workspace overlay  ⟶ no runs
- `70a6fc665f` 2026-08-20 P5-E2(rfc-379): flatten the local layer into stack files  ⟶ no runs
- `343fc5c4a4` 2026-08-21 M3(rfc-379): regen wiring baselines post-MACVO removal; exclude sim render-pipeline nodes  ⟶ no runs
- `ff35e368bc` 2026-08-21 P6(rfc-380): mount .airstack/generated into robot containers; auto-generate bridge router configs  ⟶ no runs
- `73a53ee82b` 2026-08-20 P2(rfc-379): auto-include generated module compose override in airstack up  ⟶ no runs
- `1477df3938` 2026-08-20 P3(rfc-379): hook_env input for module host_setup hooks  ⟶ no runs
- `115c991acf` 2026-08-21 M3(rfc-379): bless full_macvo wiring baseline on the composed module image  ⟶ no runs
- `a9f56b0a10` 2026-08-21 P6(rfc-380): doctor --live handles heterogeneous fleets  ⟶ no runs
- `6d40d9aa3e` 2026-08-20 P2(rfc-379): update sync hint and docs for automatic module-compose include  ⟶ no runs
- `19c4adc4e2` 2026-08-20 P5-E1(rfc-379): reference stack folders (wrap form), --stack dispatch, single-locus lint  ⟶ no runs
- `76af860239` 2026-08-20 P2(rfc-379): warn on stale colcon caches for extracted module packages  ⟶ no runs
- `6f436d8798` 2026-08-21 P5-E3a(rfc-379): flatten perception/sensors/global/behavior into stacks; prefix generic launch args  ⟶ no runs
- `643d0c3475` 2026-08-21 P6(rfc-380): single-robot fleets use the historical single-drone prim names  ⟶ no runs
- `4a76935c04` 2026-08-20 P4(rfc-379): docker module-layer composition, layer plan, modules.lock  ⟶ no runs
- `5efffa3d1b` 2026-08-20 P5-E1(rfc-379): observed wiring.md for all three reference stacks  ⟶ no runs
- `e58caad8a2` 2026-08-20 P5(rfc-379): stack entries must never include the dispatcher (contract)  ⟶ no runs
- `9cc7b3383a` 2026-08-21 P5-E3b(rfc-379): split stack + bridge.yaml, doctor, interface conventions spec, stack CLI  ⟶ no runs
- `b408ee7193` 2026-08-21 fix(tests): wait for the state-estimate watchdog before commanding takeoff  ⟶ no runs
- `7933f405e8` 2026-08-20 P2(rfc-379): key Isaac overlay placements off targets, not module type  ⟶ no runs
- `e361ef18ac` 2026-08-20 P3(rfc-379): build module Docker layers in the reusable workflow  ⟶ no runs
- `b0541c15a6` 2026-08-20 P2(rfc-379): sync self-heals stale partial module checkouts  ⟶ no runs
- `2a178e6812` 2026-08-21 P5-E3(rfc-379): bless lite_default wiring baseline  ⟶ no runs
- `b8999066a8` 2026-08-21 P6(rfc-380): airstack down sees generated module/fleet compose services  ⟶ no runs
- `aebd6e49eb` 2026-08-22 Bump version to 0.20.0-alpha.1  ⟶ EnforceBranchTargets=success, UnitTests=failure, CheckVERSIONIncrement=success, SystemTests=failure
- `592f6cf9d5` 2026-08-22 Bump version to 0.20.0-alpha.2  ⟶ CheckVERSIONIncrement=success, SystemTests=failure, SystemTests=cancelled, CheckVERSIONIncrement=success
- `f1c1352b5b` 2026-08-22 Bump version to 0.20.0-alpha.3  ⟶ SystemTests=failure, CheckVERSIONIncrement=success
- `7980fb0bbd` 2026-08-22 Bump version to 0.20.0-alpha.4  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success
- `4044aa15ef` 2026-08-22 Bump version to 0.20.0-alpha.5  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success
- `17de16f0cc` 2026-08-22 Bump version to 0.20.0-alpha.6  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success
- `3852ae2d31` 2026-08-21 P7(rfc-379): marketplace catalog, module-docs fetch, new-developer walkthrough  ⟶ no runs
- `2b0274058a` 2026-08-21 chore: delete the stillborn ensemble_planner skeleton  ⟶ no runs
- `1ccbb20244` 2026-08-21 chore(launch): legibility sweep — delete dead launch files, purge dead blocks, header discipline  ⟶ no runs
- `46c499bd19` 2026-08-21 Remove the legacy AUTONOMY_ROLE dispatch — stacks are the only launch path  ⟶ no runs
- `c9e3cf6eab` 2026-08-21 P2(rfc-379): module remove survives root-owned container artifacts  ⟶ no runs
- `0a13f04c9e` 2026-08-22 audit(cli): truthful help/README, one fleet pipeline, shared lib, hardened commands  ⟶ no runs
- `9184110beb` 2026-08-21 docs(stacks): why stacks don't launch standalone — the dispatcher philosophy  ⟶ no runs
- `8242ae85cb` 2026-08-22 audit(robot): index-collision fix, live plugin param, honest baselines, orphan purge  ⟶ no runs
- `4f4a22410b` 2026-08-22 Bump version to 0.20.0-alpha.7  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success
- `0b7b33d85b` 2026-08-22 audit(gcs/sim): fleet-aware GCS, distinct GPS homes, sshd fix, dead-file purge  ⟶ no runs
- `dbead2a748` 2026-08-22 audit(docs): link integrity 81->4, truthful CHANGELOG, root README, beginner funnel  ⟶ no runs
- `6ea3e810fd` 2026-08-22 chore(audit): execute owner-ruled deletions — dead interfaces, GUIs, sensors trio, TAK, standalone examples  ⟶ no runs
- `4d97b89f19` 2026-08-22 feat(sim): adopt simple-sim as a first-class --sim target; doc isaac-sim-gui; demote prebuilt to tag-only; drop WinTAK C  ⟶ no runs
- `96ad668eee` 2026-08-22 chore(docker): dependency purge — robot image sheds unused apt/pip deps (-152MB)  ⟶ no runs
- `4cacc81d78` 2026-08-22 chore(modules): re-pin asm_macvo to 06f3a8c8 — module now declares rich/tqdm  ⟶ no runs
- `8f269766fe` 2026-08-22 chore(license): relicense trunk to BSD-3-Clause-Clear; real maintainers + descriptions everywhere  ⟶ no runs
- `002dd2366d` 2026-08-22 fix(simple-sim): source ROS Jazzy, not Humble — sim never started since the Jazzy migration  ⟶ no runs
- `277b6eb397` 2026-08-22 fix(tests): --stack accepts <name>[:<entry>] like the CLI; re-pin asm_macvo past its stale-import fix  ⟶ no runs
- `c9bc2b43f2` 2026-08-22 chore(audit): delete rqt_behavior_tree — zero referrers after the GUI-set removal  ⟶ no runs
- `c97e30831a` 2026-08-22 docs: versioned Release Notes replace CHANGELOG.md; docs-are-snapshots policy in the writing skill  ⟶ no runs
- `385ca6854c` 2026-08-22 docs: deep standalone-snapshot audit — RFC scrub, motivation-first openings, code-truth corrections  ⟶ no runs
- `d4802e4082` 2026-08-22 chore(audit): remove flagged leftovers — vestigial rqt trajectory selector, orphaned images, empty stub page  ⟶ no runs
- `86da1f9076` 2026-08-22 Bump version to 0.20.0-alpha.9  ⟶ CheckVERSIONIncrement=success, SystemTests=failure
- `298cb72eaa` 2026-08-22 Bump version to 0.20.0-alpha.8  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=success
- `5efa388877` 2026-08-22 Release 0.19.0  ⟶ no runs
- `9104249b35` 2026-08-22 Release 0.19.0 (#397)  ⟶ SystemTests=failure, EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=failure, AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `84b15b9a65` 2026-08-22 Merge develop (Release 0.19.0) — keep the stack's 0.20.0-alpha.1  ⟶ SystemTests=failure, EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=failure
- `a871838ee5` 2026-08-22 docs(release-notes): record 0.19.0 — the versioned page now carries the full release history  ⟶ SystemTests=cancelled, CheckVERSIONIncrement=failure
- `0a161dce87` 2026-08-22 Merge develop (Release 0.19.0) — keep this branch's audit CHANGELOG and 0.20.0-alpha.8  ⟶ CheckVERSIONIncrement=success, SystemTests=failure
- `06981aa5f7` 2026-08-22 Merge rfc/s8-audit (develop release sync) — CHANGELOG stays deleted, VERSION stays 0.20.0-alpha.9  ⟶ CheckVERSIONIncrement=failure, SystemTests=cancelled
- `03ef065647` 2026-08-22 fix(.env): restore the audited env comments and the 0.20.0-alpha.9 bump  ⟶ CheckVERSIONIncrement=success, SystemTests=cancelled
- `968d6238cb` 2026-08-24 Merge pull request #388 from castacks/rfc/s1-observability-manifest  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `bf0b12a310` 2026-08-24 Merge pull request #389 from castacks/rfc/s2-module-cli-layers  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `7759539fcf` 2026-08-24 Merge pull request #390 from castacks/rfc/s3-stacks-wrap  ⟶ Build/PublishDevelopDocs=failure, AutoBuildonDockerImageTagChange=cancelled
- `3e4e726ed5` 2026-08-24 Merge pull request #391 from castacks/rfc/s4-optitrack-flatten-local  ⟶ Build/PublishDevelopDocs=failure, AutoBuildonDockerImageTagChange=cancelled
- `54efefe18e` 2026-08-24 Merge pull request #392 from castacks/rfc/s5-macvo-flatten-split  ⟶ Build/PublishDevelopDocs=failure, AutoBuildonDockerImageTagChange=cancelled
- `fe23f14785` 2026-08-24 Merge pull request #393 from castacks/rfc/s6-fleets  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=cancelled
- `ce348c2e5a` 2026-08-24 Merge pull request #394 from castacks/rfc/s7-catalog-role-removal  ⟶ Build/PublishDevelopDocs=failure, AutoBuildonDockerImageTagChange=cancelled
- `7d2782100b` 2026-08-24 Merge develop (stack levels 1-7) — keep 0.20.0-alpha.8  ⟶ SystemTests=failure, EnforceBranchTargets=success, CheckVERSIONIncrement=failure, UnitTests=failure
- `7d474a31db` 2026-08-24 Merge pull request #395 from castacks/rfc/s8-audit  ⟶ Build/PublishDevelopDocs=failure, AutoBuildonDockerImageTagChange=cancelled
- `86ea6ff062` 2026-08-24 Merge pull request #396 from castacks/rfc/s9-docs-release-notes  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `8e5e8179da` 2026-08-24 feat(gcs): auto-load NUM_ROBOTS-matched Foxglove layout — no manual import (#401)  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `2f11a8cb56` 2026-08-24 fix(bringup): loud AUTOLAUNCH FAILED banner instead of silent tmux prompt (#402)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `5d367c8dc4` 2026-08-24 feat(sync): warn when airstack.yaml release: drifts off the .env VERSION line  ⟶ no runs
- `bd6d546d16` 2026-08-25 feat(docs): modernize mkdocs-material theme with official CMU palette  ⟶ no runs
- `f358273a70` 2026-08-25 feat(cli): consolidate osmo/config/image commands into command groups  ⟶ no runs
- `5c0665fec6` 2026-08-25 fix(cli): config nucleus blank input no longer clobbers an existing token  ⟶ no runs
- `e5889c0640` 2026-08-25 feat(sim): airstack up --scene <shortname> — simulator-agnostic scene selection  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `d3e746a77d` 2026-08-25 feat(sim): Isaac follow-cam, spawn relocation, and scene light-boost env knobs  ⟶ no runs
- `bfb17a98d9` 2026-08-25 feat(docs): redesign landing page around demonstrated pillars + live demo videos  ⟶ no runs
- `a238c5a9ab` 2026-08-25 docs: prefer airstack CLI flags over env vars in launch instructions  ⟶ no runs
- `d036a9ccb1` 2026-08-25 feat(docs): release notes page shows only the docs version being viewed  ⟶ no runs
- `e5c6629460` 2026-08-25 feat(sim): sim starts playing by default (PLAY_SIM_ON_START=true)  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `45e2a652a4` 2026-08-25 fix(docs): hero no longer clips CTAs on short viewports  ⟶ no runs
- `fce0f4ce56` 2026-08-25 feat(docs): version selector shows release numbers, develop listed first  ⟶ Build/PublishDevelopDocs=success
- `383d344e2e` 2026-08-25 fix(docs): prevent landing sections from overflowing on narrow screens  ⟶ no runs
- `02a12d56a3` 2026-08-25 feat(docs): shorten version selector titles to (stable)/(unstable)  ⟶ Build/PublishDevelopDocs=success
- `66e62efdbe` 2026-08-25 docs: Diátaxis documentation overhaul (#404)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `7ead8d8ad1` 2026-08-25 fix(docs): search dropdown rendered behind nav tabs and version text  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `e3baec7f3d` 2026-08-26 docs: sync How-to Guides landing table with the current nav groups  ⟶ Build/PublishDevelopDocs=success
- `d4a04df6a3` 2026-08-26 docs(skills): require branch + commit hash alongside timestamps in feature notebook  ⟶ no runs
- `6c0a4e700f` 2026-08-29 feat(stacks): full_mighty reference stack + mighty module catalog entry (#405)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `02f7660545` 2026-08-29 fix(ci): un-red every PR — metrics-job deps and unit-test workflow environment (#407)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `c5fc15c40e` 2026-08-29 feat(orchestrator): poll a repos: list — one instance covers trunk + module repos (#400)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success
- `270910ac97` 2026-08-29 Ci/trustworthy system tests (#403)  ⟶ Build/PublishDevelopDocs=success, AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `24a80f789c` 2026-08-29 docs(agents): module registration is TWO merges — registry PR must land or the deployed catalog drops the module  ⟶ Build/PublishDevelopDocs=success
- `ff7c4c50e4` 2026-08-29 feat(ci): registry→trunk catalog sync automation + drift alarm (#408)  ⟶ Syncmodulecatalogfromregistry=success, AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `c33f3f7f9b` 2026-08-29 Release 0.20.0: switch pre-release terminology to -dev.N, single revamped README (#409)  ⟶ AutoBuildonDockerImageTagChange=success, Build/PublishDevelopDocs=success
- `aa2784c26e` 2026-08-29 Merge main into develop (ours): re-establish common ancestry for release 0.20.0  ⟶ SystemTests=success, CheckVERSIONIncrement=failure, EnforceBranchTargets=success, UnitTests=success
- `cbdcd6d78f` 2026-08-29 Merge pull request #411 from castacks/sync/main-ancestry  ⟶ SystemTests=success, EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success

  **RED→FIX** after `55d9b887d9` (Auto Build on Docker Image Tag Change): https://github.com/castacks/AirStack/actions/runs/30953320508
  followed by: `ae2e942029` Add feature-notebook workflow: per-feature design specs + test results feeding PRs (#381); `9788b14a86` Remove stray files; `234587aa05` Robot deployment fixes: bag recording + adding warning for robot-identity failure (#377)

  **RED→FIX** after `f4697265e4` (System Tests): https://github.com/castacks/AirStack/actions/runs/32194437120
  followed by: `9e2e0e3991` docs(skills): require dates and timestamps in feature notebook entries; `262f12679a` Pre-RFC workflow cleanup: intent-based launch, readiness gates, launch-script dedup, truthful logs (#386); `b6e4dba963` P1(rfc-379): module.yaml manifest schema, validator, fixture module, contract tests

  **RED→FIX** after `aebd6e49eb` (System Tests, Unit Tests): https://github.com/castacks/AirStack/actions/runs/32561435375; https://github.com/castacks/AirStack/actions/runs/32561435334
  followed by: `592f6cf9d5` Bump version to 0.20.0-alpha.2; `f1c1352b5b` Bump version to 0.20.0-alpha.3; `7980fb0bbd` Bump version to 0.20.0-alpha.4

  **RED→FIX** after `592f6cf9d5` (System Tests): https://github.com/castacks/AirStack/actions/runs/32774182354
  followed by: `f1c1352b5b` Bump version to 0.20.0-alpha.3; `7980fb0bbd` Bump version to 0.20.0-alpha.4; `4044aa15ef` Bump version to 0.20.0-alpha.5

  **RED→FIX** after `f1c1352b5b` (System Tests): https://github.com/castacks/AirStack/actions/runs/32561437474
  followed by: `7980fb0bbd` Bump version to 0.20.0-alpha.4; `4044aa15ef` Bump version to 0.20.0-alpha.5; `17de16f0cc` Bump version to 0.20.0-alpha.6

  **RED→FIX** after `86da1f9076` (System Tests): https://github.com/castacks/AirStack/actions/runs/32561448070
  followed by: `298cb72eaa` Bump version to 0.20.0-alpha.8; `5efa388877` Release 0.19.0; `9104249b35` Release 0.19.0 (#397)

  **RED→FIX** after `9104249b35` (System Tests, Unit Tests): https://github.com/castacks/AirStack/actions/runs/32590357840; https://github.com/castacks/AirStack/actions/runs/32590357789
  followed by: `84b15b9a65` Merge develop (Release 0.19.0) — keep the stack's 0.20.0-alpha.1; `a871838ee5` docs(release-notes): record 0.19.0 — the versioned page now carries the full release history; `0a161dce87` Merge develop (Release 0.19.0) — keep this branch's audit CHANGELOG and 0.20.0-alpha.8

  **RED→FIX** after `84b15b9a65` (System Tests, Unit Tests): https://github.com/castacks/AirStack/actions/runs/32590622212; https://github.com/castacks/AirStack/actions/runs/32590622234
  followed by: `a871838ee5` docs(release-notes): record 0.19.0 — the versioned page now carries the full release history; `0a161dce87` Merge develop (Release 0.19.0) — keep this branch's audit CHANGELOG and 0.20.0-alpha.8; `06981aa5f7` Merge rfc/s8-audit (develop release sync) — CHANGELOG stays deleted, VERSION stays 0.20.0-alpha.9

  **RED→FIX** after `a871838ee5` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/32590640607
  followed by: `0a161dce87` Merge develop (Release 0.19.0) — keep this branch's audit CHANGELOG and 0.20.0-alpha.8; `06981aa5f7` Merge rfc/s8-audit (develop release sync) — CHANGELOG stays deleted, VERSION stays 0.20.0-alpha.9; `03ef065647` fix(.env): restore the audited env comments and the 0.20.0-alpha.9 bump

  **RED→FIX** after `0a161dce87` (System Tests): https://github.com/castacks/AirStack/actions/runs/32591310686
  followed by: `06981aa5f7` Merge rfc/s8-audit (develop release sync) — CHANGELOG stays deleted, VERSION stays 0.20.0-alpha.9; `03ef065647` fix(.env): restore the audited env comments and the 0.20.0-alpha.9 bump; `968d6238cb` Merge pull request #388 from castacks/rfc/s1-observability-manifest

  **RED→FIX** after `06981aa5f7` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/32591326340
  followed by: `03ef065647` fix(.env): restore the audited env comments and the 0.20.0-alpha.9 bump; `968d6238cb` Merge pull request #388 from castacks/rfc/s1-observability-manifest; `bf0b12a310` Merge pull request #389 from castacks/rfc/s2-module-cli-layers

  **RED→FIX** after `7759539fcf` (Build/Publish Develop Docs): https://github.com/castacks/AirStack/actions/runs/32774319252
  followed by: `3e4e726ed5` Merge pull request #391 from castacks/rfc/s4-optitrack-flatten-local; `54efefe18e` Merge pull request #392 from castacks/rfc/s5-macvo-flatten-split; `fe23f14785` Merge pull request #393 from castacks/rfc/s6-fleets

  **RED→FIX** after `3e4e726ed5` (Build/Publish Develop Docs): https://github.com/castacks/AirStack/actions/runs/32774347861
  followed by: `54efefe18e` Merge pull request #392 from castacks/rfc/s5-macvo-flatten-split; `fe23f14785` Merge pull request #393 from castacks/rfc/s6-fleets; `ce348c2e5a` Merge pull request #394 from castacks/rfc/s7-catalog-role-removal

  **RED→FIX** after `54efefe18e` (Build/Publish Develop Docs): https://github.com/castacks/AirStack/actions/runs/32774373885
  followed by: `fe23f14785` Merge pull request #393 from castacks/rfc/s6-fleets; `ce348c2e5a` Merge pull request #394 from castacks/rfc/s7-catalog-role-removal; `7d2782100b` Merge develop (stack levels 1-7) — keep 0.20.0-alpha.8

  **RED→FIX** after `ce348c2e5a` (Build/Publish Develop Docs): https://github.com/castacks/AirStack/actions/runs/32774425624
  followed by: `7d2782100b` Merge develop (stack levels 1-7) — keep 0.20.0-alpha.8; `7d474a31db` Merge pull request #395 from castacks/rfc/s8-audit; `86ea6ff062` Merge pull request #396 from castacks/rfc/s9-docs-release-notes

  **RED→FIX** after `7d2782100b` (Check VERSION Increment, System Tests, Unit Tests): https://github.com/castacks/AirStack/actions/runs/32774678383; https://github.com/castacks/AirStack/actions/runs/32774678371; https://github.com/castacks/AirStack/actions/runs/32774678379
  followed by: `7d474a31db` Merge pull request #395 from castacks/rfc/s8-audit; `86ea6ff062` Merge pull request #396 from castacks/rfc/s9-docs-release-notes; `8e5e8179da` feat(gcs): auto-load NUM_ROBOTS-matched Foxglove layout — no manual import (#401)

  **RED→FIX** after `7d474a31db` (Build/Publish Develop Docs): https://github.com/castacks/AirStack/actions/runs/32774594266
  followed by: `86ea6ff062` Merge pull request #396 from castacks/rfc/s9-docs-release-notes; `8e5e8179da` feat(gcs): auto-load NUM_ROBOTS-matched Foxglove layout — no manual import (#401); `2f11a8cb56` fix(bringup): loud AUTOLAUNCH FAILED banner instead of silent tmux prompt (#402)

  **RED→FIX** after `aa2784c26e` (Check VERSION Increment): https://github.com/castacks/AirStack/actions/runs/33277338922
  followed by: `cbdcd6d78f` Merge pull request #411 from castacks/sync/main-ancestry

## PR #412 — Bump version to 0.21.0-dev.0; fix post-release sync skip (2026-08-29, → develop, @andrewjong)

https://github.com/castacks/AirStack/pull/412

1 commits, +16/−6, 0 review comments, 1 issue comments

- `8d39c91db0` 2026-08-29 Bump version to 0.21.0-dev.0; fix post-release main→develop sync skip  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

## PR #413 — hotfix(ci): make the fixed post-release sync live on main (2026-08-29, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/413

1 commits, +9/−4, 0 review comments, 1 issue comments

- `03d6fb6212` 2026-08-29 hotfix(ci): post-release main→develop sync must not skip on identical trees  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

## PR #414 — hotfix(docs): MAJOR.MINOR docs slugs; retire the duplicate 'main' docs version (2026-08-29, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/414

1 commits, +67/−40, 0 review comments, 1 issue comments

- `09f204e5b5` 2026-08-29 hotfix(docs): MAJOR.MINOR docs slugs; retire the duplicate 'main' docs version  ⟶ SystemTests=success, EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success

## PR #415 — hotfix(docs): rename stable docs alias 'latest' → 'main'; serialize gh-pages pushes (2026-08-29, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/415

1 commits, +29/−11, 0 review comments, 1 issue comments

- `9e93f61cc0` 2026-08-29 hotfix(docs): rename the stable docs alias 'latest' → 'main'; serialize gh-pages pushes  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

## PR #416 — hotfix(docs): pin the site root redirect to the MAJOR.MINOR slug (2026-08-29, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/416

1 commits, +15/−9, 0 review comments, 1 issue comments

- `70f412c424` 2026-08-29 hotfix(docs): pin the site root redirect to the MAJOR.MINOR slug  ⟶ SystemTests=success, EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success

## PR #417 — hotfix(docs): Harbor registry is public — drop docker login from pull instructions (2026-08-29, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/417

1 commits, +10/−19, 0 review comments, 1 issue comments

- `cb652ff437` 2026-08-29 hotfix(docs): Harbor registry is public — drop docker login from pull instructions  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

## PR #418 — hotfix(docs): dated release-notes sections for 0.20.1–0.20.5; require them for all hotfixes (2026-08-29, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/418

2 commits, +58/−3, 0 review comments, 1 issue comments

- `cd91c2fd30` 2026-08-29 docs(release-notes): dated sections for 0.20.1–0.20.5; require them for all hotfixes  ⟶ no runs
- `bff960c46e` 2026-08-29 hotfix(docs): dated release-notes sections for 0.20.1–0.20.5; require them for all hotfixes  ⟶ EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=success

## PR #419 — hotfix(docs): README-page 404s, link previews, edit-this-page button (0.20.7) (2026-08-30, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/419

3 commits, +205/−5, 0 review comments, 1 issue comments, **2 RED→FIX**

- `9ef135a795` 2026-08-30 hotfix(docs): anchor exclude_docs README pattern to repo root  ⟶ SystemTests=cancelled, EnforceBranchTargets=failure, UnitTests=success, CheckVERSIONIncrement=success
- `d341ea10a9` 2026-08-30 hotfix(docs): Open Graph metadata for link previews  ⟶ EnforceBranchTargets=failure, CheckVERSIONIncrement=success, UnitTests=success, SystemTests=cancelled
- `877a5b509f` 2026-08-30 feat(docs): "Edit this page" pencil on every docs page  ⟶ EnforceBranchTargets=failure, CheckVERSIONIncrement=success, UnitTests=failure, SystemTests=success

  **RED→FIX** after `9ef135a795` (Enforce Branch Targets): https://github.com/castacks/AirStack/actions/runs/33293964101
  followed by: `d341ea10a9` hotfix(docs): Open Graph metadata for link previews; `877a5b509f` feat(docs): "Edit this page" pencil on every docs page

  **RED→FIX** after `d341ea10a9` (Enforce Branch Targets): https://github.com/castacks/AirStack/actions/runs/33294416640
  followed by: `877a5b509f` feat(docs): "Edit this page" pencil on every docs page

## PR #421 — hotfix(docs): exclude AirSim scene binaries from docs builds (0.20.8) (2026-09-05, → main, @andrewjong)

https://github.com/castacks/AirStack/pull/421

2 commits, +31/−2, 0 review comments, 1 issue comments, **1 RED→FIX**

- `795b502dfd` 2026-09-05 hotfix(docs): exclude AirSim scene binaries from docs builds (0.20.8)  ⟶ CheckVERSIONIncrement=success, EnforceBranchTargets=success, UnitTests=failure, SystemTests=cancelled, SystemTests=success, EnforceBranchTargets=failure, CheckVERSIONIncrement=success, UnitTests=failure
- `3abcad1a2e` 2026-09-05 test(docs): tolerate mkdocs !ENV tag in docs-catalog contract loader  ⟶ SystemTests=success, EnforceBranchTargets=success, CheckVERSIONIncrement=success, UnitTests=success

  **RED→FIX** after `795b502dfd` (Enforce Branch Targets, Unit Tests): https://github.com/castacks/AirStack/actions/runs/33933054857; https://github.com/castacks/AirStack/actions/runs/33932954085; https://github.com/castacks/AirStack/actions/runs/33932954098
  followed by: `3abcad1a2e` test(docs): tolerate mkdocs !ENV tag in docs-catalog contract loader
