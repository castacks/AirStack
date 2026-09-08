# Candidate commits

Every commit in the window whose message matches the fix-keyword regex (plus all others, --all-messages). Read the diff before classifying; the message alone is `low` confidence.

## e41e0de6fd — 2025-08-12 — Junbin Yuan — no keyword

**migrate exploration planner from subt**

https://github.com/castacks/AirStack/commit/e41e0de6fd7477de395f1721c4154b49b29cea5d

+6823 / −0 in 24 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/.vscode/c_cpp_properties.json`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/.vscode/settings.json`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/CMakeModules/FindOpenVDB.cmake`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/CMakeModules/OpenVDBUtils.cmake`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/README.md`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/config/exploration_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/exploration_logic.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/exploration_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/utils/collision_checker.h`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/utils/rrt_planner.h`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/utils/utils.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/utils/viewpoint.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/viewpoint_sampling.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/exploration_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/random_walk_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/package.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/random_walk.png`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/collision_checker.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_logic.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/rrt_planner.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/utils.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/viewpoint_sampling.cpp`

## dbbe2a33d5 — 2025-08-28 — Junbin Yuan — no keyword

**config for running in ros2 gazebo env**

https://github.com/castacks/AirStack/commit/dbbe2a33d596e6ccab224abc46d75bd2cba8c55e

+320 / −3 in 8 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/config/exploration_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_autonomy_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_behavior_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_domain_bridge.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_global_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_local_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_static_transforms.launch.xml`

## 7efcea73cb — 2025-08-31 — Junbin Yuan — no keyword

**scripts for manual takeoff without gui**

https://github.com/castacks/AirStack/commit/7efcea73cbeda8d138f26d067ffe1b192dcde78a

+24 / −0 in 2 files:

- `robot/ros_ws/manual_exploration.sh`
- `robot/ros_ws/manual_takeoff.sh`

## 72d5334fd3 — 2025-08-31 — Junbin Yuan — no keyword

**gazebo configuration finished**

https://github.com/castacks/AirStack/commit/72d5334fd3fc959602d11dca0708d26669aa81c1

+805 / −50 in 6 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gazebo_vis.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_autonomy_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_domain_bridge.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_interface_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_node.cpp`

## 7d754556c5 — 2025-09-03 — Junbin Yuan — no keyword

**manual launch of robot**

https://github.com/castacks/AirStack/commit/7d754556c56b10f837611a8f412d77a860ed5bc1

+1 / −1 in 1 files:

- `robot/docker/docker-compose.yaml`

## 44f2040414 — 2025-09-03 — Junbin Yuan — no keyword

**some config change for exploration run**

https://github.com/castacks/AirStack/commit/44f2040414f312ea7093163e4ec2f4e7aae267ff

+275 / −14 in 5 files:

- `robot/ros_ws/src/autonomy/2_perception/perception_bringup/launch/perception.launch.xml`
- `robot/ros_ws/src/autonomy/3_local/a_world_models/disparity_graph_cost_map/src/disparity_graph_cost_map.cpp`
- `robot/ros_ws/src/autonomy/3_local/b_planners/droan_local_planner/config/droan.yaml`
- `robot/ros_ws/src/autonomy/3_local/b_planners/trajectory_library/config/acceleration_magnitudes.yaml`
- `robot/ros_ws/src/autonomy/4_global/global_bringup/launch/global.launch.xml`

## 37b3e0e4d5 — 2025-09-03 — Junbin Yuan — no keyword

**adding a mode for receiving target path instead of extracting frontier**

https://github.com/castacks/AirStack/commit/37b3e0e4d587e056656d7fe0709895b41a66b692

+451 / −16 in 7 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/exploration_logic.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/exploration_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_logic.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_node_run.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/waypoint_routing_node.cpp`

## ed4fdc4e46 — 2025-09-06 — Junbin Yuan — KEYWORD error

**blosc config for build error**

https://github.com/castacks/AirStack/commit/ed4fdc4e464374a5909f02fa9398e3383b3cdeef

+394 / −0 in 1 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/CMakeModules/FindBlosc.cmake`

## 393142b843 — 2025-09-10 — Junbin Yuan — no keyword

**tiny modification of waypoing routing**

https://github.com/castacks/AirStack/commit/393142b843ec7f85f574daa7ddcee1fc3f8f1301

+3 / −0 in 2 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/config/exploration_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/waypoint_routing_node.cpp`

## 2ed7fecd57 — 2025-09-10 — Junbin Yuan — KEYWORD fix

**another fix**

https://github.com/castacks/AirStack/commit/2ed7fecd576e1eed57f1be321554db9ac3791a3f

+1 / −1 in 1 files:

- `robot/docker/docker-compose.yaml`

## 54a1f7e44b — 2025-09-10 — Junbin Yuan — no keyword

**manually launch robot**

https://github.com/castacks/AirStack/commit/54a1f7e44b8fad5a987f085da5331db7fb428336

+1 / −1 in 1 files:

- `robot/docker/docker-compose.yaml`

## e579063384 — 2025-09-10 — Junbin Yuan — KEYWORD fixed

**fixed docker compoer env**

https://github.com/castacks/AirStack/commit/e579063384cc1e52c07cec9cb885ff005a02b584

+17 / −12 in 1 files:

- `robot/docker/docker-compose.yaml`

## 593be4ac51 — 2025-09-10 — Junbin Yuan — KEYWORD fix

**fix cmakelist after merge**

https://github.com/castacks/AirStack/commit/593be4ac51500706ae78dbc61d25a5202232c4aa

+3 / −3 in 1 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/CMakeLists.txt`

## 0ea9a854e4 — 2025-09-10 — Junbin Yuan — KEYWORD fix

**fix cmakelist after merge**

https://github.com/castacks/AirStack/commit/0ea9a854e41fbe2fb1a22206814229062a470ff5

+3 / −3 in 1 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/CMakeLists.txt`

## a7a53ae718 — 2025-09-10 — Junbin Yuan — KEYWORD repair

**repair docker compose**

https://github.com/castacks/AirStack/commit/a7a53ae718ad91d8613df673b15d94835a9e833e

+1 / −6 in 1 files:

- `robot/docker/docker-compose.yaml`

## 2d5142109e — 2025-09-10 — caomuqing — no keyword

**add inspection_planner**

https://github.com/castacks/AirStack/commit/2d5142109ef4a1887895a954e2111da3ba147e75

+2958 / −22 in 14 files:

- `robot/docker/docker-compose.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/CMakeModules/FindBlosc.cmake`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/README.md`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/config/inspection_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/include/inspection_planner.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/launch/inspection_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/package.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/panoptic_3d_locator.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/panoptic_semantics_splitter.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/tsp_planner.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/vdb_pointcloud_analyzer.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/src/inspection_planner.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/src/inspection_planner_node.cpp`

## be77510f33 — 2025-09-13 — caomuqing — no keyword

**publishing inspection pose markers**

https://github.com/castacks/AirStack/commit/be77510f33427fc96d2e69096373db0585d2772d

+192 / −69 in 2 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/include/inspection_planner.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/src/inspection_planner.cpp`

## be6731472a — 2025-09-14 — caomuqing — no keyword

**add filter prediction semantics**

https://github.com/castacks/AirStack/commit/be6731472a97b6d72b2826a9965d1016208f2a6d

+4033 / −55 in 13 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/config/inspection_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/launch/filter_predicted_semantics.launch.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/launch/inspection_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/package.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/filter_predicted_semantics.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/red_beam_clusterer.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/red_beam_clusterer_simple.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/red_beam_line_detector.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/red_beam_line_detector_fast.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/red_beam_line_detector_working.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/red_beam_voxelizer_fixed.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/src/inspection_planner.cpp`

## 8cee231cc3 — 2025-09-15 — caomuqing — no keyword

**update filter prediction semantics**

https://github.com/castacks/AirStack/commit/8cee231cc3ab8a6b4e6aec3801d5108f9631b634

+119 / −69 in 1 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/scripts/filter_predicted_semantics.py`

## b533c76f9f — 2025-09-26 — Junbin Yuan — no keyword

**add doc line in mkdocs**

https://github.com/castacks/AirStack/commit/b533c76f9f3093665790f92e94951b1eee8e30ec

+1 / −0 in 1 files:

- `mkdocs.yml`

## e69727d828 — 2025-09-26 — Junbin Yuan — no keyword

**Update README.md**

https://github.com/castacks/AirStack/commit/e69727d82813cdf47b435398c78e38d41646f793

+14 / −11 in 1 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/README.md`

## b3fc0558e1 — 2025-09-26 — Junbin Yuan — no keyword

**RRT shortening, difference interpolation and extend step size**

https://github.com/castacks/AirStack/commit/b3fc0558e1892d024b0e0dadf4404d312caa5c7b

+455 / −646 in 15 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/config/exploration_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/exploration_logic.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/exploration_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/utils/collision_checker.h`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/utils/rrt_planner.h`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/utils/utils.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/utils/viewpoint.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/exploration_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/robot_launch_gazebo/gz_global_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/collision_checker.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_logic.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/rrt_planner.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/utils.cpp`

## 4399ab4c14 — 2025-10-08 — Junbin Yuan — no keyword

**shimizu demo**

https://github.com/castacks/AirStack/commit/4399ab4c146e054808b615905711fae866a2ffff

+1424 / −23 in 20 files:

- `robot/ros_ws/src/autonomy/0_interface/mavros_interface/scripts/position_setpoint_pub.py`
- `robot/ros_ws/src/autonomy/3_local/a_world_models/disparity_graph_cost_map/src/disparity_graph_cost_map.cpp`
- `robot/ros_ws/src/autonomy/3_local/b_planners/droan_local_planner/config/droan.yaml`
- `robot/ros_ws/src/autonomy/3_local/b_planners/trajectory_library/config/acceleration_magnitudes.yaml`
- `robot/ros_ws/src/autonomy/3_local/b_planners/trajectory_library/src/trajectory_library.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/config/exploration_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/exploration_logic.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/exploration_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/include/viewpoint_sampling.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/shimizu_robot/shimizu_autonomy.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/shimizu_robot/shimizu_ego.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/shimizu_robot/shimizu_global.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/shimizu_robot/shimizu_local.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/shimizu_robot/shimizu_static_transforms.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/shimizu_robot/shimizu_vis.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/launch/shimizu_robot_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/collision_checker.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_logic.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/exploration_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/exploration/src/viewpoint_sampling.cpp`

## 1dd5bd7105 — 2025-11-17 — Junbin Yuan — no keyword

**gitmodules for vdbedt**

https://github.com/castacks/AirStack/commit/1dd5bd7105bc421e53d75d25ded3e6a13895134c

+3 / −0 in 1 files:

- `.gitmodules`

## 52869a5a2b — 2025-11-17 — Junbin Yuan — no keyword

**add vdb_edt and an integrated planner**

https://github.com/castacks/AirStack/commit/52869a5a2be4444bea16206b4af17a763725c1f2

+10480 / −0 in 49 files:

- `robot/ros_ws/src/autonomy/4_global/a_world_models/vdb_edt_ros2`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeModules/FindBlosc.cmake`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeModules/FindOpenVDB.cmake`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeModules/OpenVDBUtils.cmake`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/README.md`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/exploration_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/integrated_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/astar_vdb.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/exploration_logic.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/integrated_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/utils/collision_checker.h`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/utils/rrt_planner.h`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/utils/utils.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/utils/viewpoint.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/viewpoint_sampling.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/vis_tools.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/ego_planner_launch_1.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/ego_planner_launch_2.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/integrated_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/random_walk_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gazebo_vis.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_autonomy_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_behavior_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_domain_bridge.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_global_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_interface_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_local_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_static_transforms.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_traj_controller_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_exploration.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_autonomy.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_ego.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_global.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_local.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_static_transforms.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_vis.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/package.xml`
- … 9 more

## c6f542d2b8 — 2025-11-19 — Junbin Yuan — no keyword

**Add trajectory output, clean code**

https://github.com/castacks/AirStack/commit/c6f542d2b82a166db3cb3053aca4bf327e3f69e1

+390 / −5314 in 18 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/astar_vdb.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/exploration_logic.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/integrated_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/utils/collision_checker.h`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/utils/rrt_planner.h`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/utils/utils.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/utils/viewpoint.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/viewpoint_sampling.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/package.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/astar_vdb.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/collision_checker.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/exploration_logic.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/rrt_planner.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/utils.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/viewpoint_sampling.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/waypoint_routing_node.cpp`

## 6ce66123f0 — 2025-11-20 — Junbin Yuan — no keyword

**add a path tracker that subscribe to an entire trajectory**

https://github.com/castacks/AirStack/commit/6ce66123f05f5a6362dc99477a1ade8712286c8d

+492 / −0 in 3 files:

- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/package.xml`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker.cpp`

## 56a83354a8 — 2025-11-20 — Junbin Yuan — no keyword

**testing traj controller**

https://github.com/castacks/AirStack/commit/56a83354a8df6670a9a0365ea0702c882ab747e1

+165 / −109 in 8 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/ego_planner_launch_1.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/random_walk_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/{shimizu_exploration.launch.xml => shimizu_exploration_debug.launch.xml}`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/{ego_planner_launch_2.xml => shimizu_local_debug.launch.xml}`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_autonomy.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_local.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_local_old.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node.cpp`

## da248b1c6b — 2025-11-26 — Junbin Yuan — no keyword

**sync vdb-edt change**

https://github.com/castacks/AirStack/commit/da248b1c6bb114b80de0fb3a9d79c0902bb5317a

+1 / −1 in 1 files:

- `robot/ros_ws/src/autonomy/4_global/a_world_models/vdb_edt_ros2`

## 4def27ac86 — 2025-11-26 — Junbin Yuan — no keyword

**add tracking controller for gazebo**

https://github.com/castacks/AirStack/commit/4def27ac860b9474cdca2bca223f3ac3f4d82dab

+1179 / −264 in 19 files:

- `robot/ros_ws/manual_exploration.sh`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker.cpp`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_cmd_vel.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/integrated_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/astar_vdb.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/integrated_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/takeoff_hack_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/{robot_launch_gazebo.xml => gazebo_robot_launch.xml}`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/integrated_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gazebo_vis.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_autonomy_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_local_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_local_launch_old.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_autonomy.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/astar_vdb.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/takeoff_hack_node.cpp`

## 50b44d15b4 — 2026-01-03 — Junbin Yuan — no keyword

**pre model ai test commit**

https://github.com/castacks/AirStack/commit/50b44d15b4fffb41aead3b5cf65a05cf3f1632b7

+2646 / −103 in 24 files:

- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_cmd_vel.cpp`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_cmd_vel_stamped.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/fov_gazebo_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/integrated_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/model_ai_extrinsics.urdf`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/fov_aware_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/integrated_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/takeoff_hack_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/fov_gazebo_robot_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/hack_takeoff_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/integrated_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/model_ai.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gazebo_vis.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_local_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_local.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/astar_vdb.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/fov_aware_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/model_ai_tfpub.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/takeoff_hack_node.cpp`

## 3431e263f0 — 2026-01-19 — Junbin Yuan — no keyword

**model ai demo**

https://github.com/castacks/AirStack/commit/3431e263f079b1ebfe4855ffa5a92e6cdf0ef44a

+2042 / −422 in 21 files:

- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_cmd_vel.cpp`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_px4.cpp`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/vdb_edt_ros2`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/fov_gazebo_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/{fov_aware_node.hpp => cpa_node.hpp}`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/integrated_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/fov_gazebo_robot_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/model_ai.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/px4.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gazebo_vis.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gz_local_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/astar_vdb.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/{fov_aware_node.cpp => cpa_node.cpp}`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/model_ai_framefix.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/model_ai_px4_test.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/model_ai_tfpub.cpp`

## dd521a9094 — 2026-01-27 — Junbin Yuan — no keyword

**mod for shimizu sim demo**

https://github.com/castacks/AirStack/commit/dd521a9094466f0c3a08f37631c0b270e0b5e90a

+286 / −80 in 11 files:

- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_cmd_vel.cpp`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_cmd_vel_stamped.cpp`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/vdb_edt_ros2`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/integrated_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/astar_vdb.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/integrated_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_autonomy.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/astar_vdb.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/cpa_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node_run.cpp`

## 06e83c961a — 2026-01-28 — Junbin Yuan — no keyword

**Update submodule URL to HTTPS and track new commits on frontier_map**

https://github.com/castacks/AirStack/commit/06e83c961ad8e23ecae956995336e81968eb135e

+61 / −25 in 5 files:

- `.gitmodules`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/vdb_edt_ros2`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/integrated_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/astar_vdb.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node.cpp`

## f16d4b956a — 2026-01-29 — Junbin Yuan — no keyword

**update rviz for shimizu**

https://github.com/castacks/AirStack/commit/f16d4b956a53ea1292fc425e0178bb360b5a8da4

+154 / −317 in 1 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_vis.rviz`

## 19f88a55c3 — 2026-01-29 — Junbin Yuan — no keyword

**add a lidar config for mid360 in isaac sim**

https://github.com/castacks/AirStack/commit/19f88a55c3cc5242158b97206b2a9e57030db0bf

+60 / −0 in 1 files:

- `livox_mimic.yaml`

## 31f969e66b — 2026-01-30 — Junbin Yuan — no keyword

**config for shimizu running**

https://github.com/castacks/AirStack/commit/31f969e66b80f299b5b40648d6727b9509bbdafc

+127 / −21 in 6 files:

- `livox_mimic.yaml => livox_mimic.json`
- `robot/ros_ws/src/autonomy/4_global/b_planners/inspection_planning/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_autonomy.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_sim.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot/shimizu_vis.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/shimizu_robot_launch.xml`

## 2799a9c15e — 2026-03-07 — Junbin Yuan — no keyword

**modal ai and research related development**

https://github.com/castacks/AirStack/commit/2799a9c15edc030b6a58bdaa571b86c349ce488d

+3455 / −543 in 27 files:

- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/package.xml`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_cmd_vel.cpp`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_cmd_vel_stamped.cpp`
- `robot/ros_ws/src/autonomy/3_local/c_controls/path_tracker/src/pid_path_tracker_px4.cpp`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/vdb_edt_ros2`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/fov_gazebo_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/cpa_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/goal_planner_node.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/include/pa_kino_astar.hpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/{fov_gazebo_robot_launch.xml => cpa_gazebo_robot_launch.xml}`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/mocap_modal_ai.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/model_ai.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/p2p_gazebo_robot_launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/fov_gazebo.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/robot_launch_gazebo/gazebo_vis.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/package.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/scripts/sample_path_mocap_data_collect.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/astar_vdb.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/cpa_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/goal_planner_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/model_ai_tfpub.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/pa_kino_astar.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/takeoff_hack_node.cpp`

## f1f655934d — 2026-04-09 — YuanJunbin — KEYWORD fix

**frame fix for superodom lidar**

https://github.com/castacks/AirStack/commit/f1f655934d3d12e40b65a7ab78c33c4b69c8d183

+134 / −40 in 5 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai_lidar.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/lidar_modal_ai.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/modalai_tfpub_lidar.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/model_ai_framefix_lidar.cpp`

## 2a36101182 — 2026-04-09 — Junbin Yuan — no keyword

**lidar config**

https://github.com/castacks/AirStack/commit/2a361011826942d97f781b1c6a2bcf0c0fd1f78a

+633 / −0 in 4 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai_lidar.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/lidar_modal_ai.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/modalai_tfpub_lidar.cpp`

## a50c19e9ed — 2026-06-08 — YuanJunbin — no keyword

**livox config**

https://github.com/castacks/AirStack/commit/a50c19e9edd092f2606432be3e046a090a10da10

+1415 / −122 in 21 files:

- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modalai_livox_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/model_ai_extrinsics.urdf`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai_config.yaml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/config/modelai_lidar.rviz`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/lidar_modal_ai.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/livox_exploration.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/launch/mocap_modal_ai_straight_line.launch.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/package.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/scripts/straight_line_planner_node.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/astar_vdb.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/integrated_node.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/model_ai_framefix_lidar.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/integrated_planner/src/model_ai_tfpub.cpp`
- `robot/ros_ws/src/autonomy/4_global/b_planners/modalai_livox_tf_bridge/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/b_planners/modalai_livox_tf_bridge/bridge_script.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/modalai_livox_tf_bridge/bridge_script_cpp_style.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/modalai_livox_tf_bridge/local_position_to_odom.py`
- `robot/ros_ws/src/autonomy/4_global/b_planners/modalai_livox_tf_bridge/package.xml`
- `robot/ros_ws/src/autonomy/4_global/b_planners/modalai_livox_tf_bridge/src/modalai_tfpub_lidar.cpp`
