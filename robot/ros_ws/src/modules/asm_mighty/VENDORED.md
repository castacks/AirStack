# Vendored upstream sources

This module vendors (copies, with local patches) the following upstream
sources. Patches are marked with `asm_mighty` / `asm_mighty port` comments.

| Package dir | Upstream | Pin | License | Trimmed |
|---|---|---|---|---|
| `mighty/` | https://github.com/mit-acl/mighty | `16394021` (main, 2026-08-27) | BSD-3-Clause | dropped: `meshes/ worlds/ urdf/ imgs/ benchmarking/ data/ docker/ docs/ src/sim include/sim` (Gazebo Classic sim assets); Gazebo deps removed; `BUILD_SIMULATION` defaults OFF; `fake_sim` Gazebo mirror stripped |
| `dynus_interfaces/` | https://github.com/kotakondo/dynus_interfaces | `628a682c` | (upstream) | verbatim |
| `decomp_util/` | https://github.com/kotakondo/DecompROS2 (`DecompUtil/`) | `a2bc42ed` | per upstream LICENSE | cmake_minimum bumped to 3.14 |
| `decomp_ros_msgs/` | https://github.com/kotakondo/DecompROS2 (`decomp_ros_msgs/`) | `a2bc42ed` | per upstream LICENSE | verbatim |
| `decomp_ros_utils/` | NEW shim | — | Apache-2.0 (header from DecompROS2) | installs `decomp_rviz_plugins/data_ros_utils.hpp` (header-only conversions) without the rviz plugin package, so the planner has no rviz dependency |
| `fla_interfaces/` | https://gitlab.com/mit-acl/lab/acl-mapping | `e87600aa` | BSD-3-Clause | verbatim |
| `fla_utils/` | https://gitlab.com/mit-acl/lab/acl-mapping | `e87600aa` | BSD-3-Clause | headers-only (param_utils.h, process_status.h); executables + laser_geometry/cv_bridge/joy deps removed |
| `global_mapper/` | https://gitlab.com/mit-acl/lab/acl-mapping | `e87600aa` | BSD-3-Clause | dropped stale `install/` dir |
| `global_mapper_ros/` | https://gitlab.com/mit-acl/lab/acl-mapping | `e87600aa` | BSD-3-Clause | see port patches |

Not vendored (deliberately): `uav_simulator`, `gazebo_ros_pkgs` fork,
`livox_laser_simulation_ros2`, `realsense_gazebo_plugin` (Gazebo Classic sim,
absent on Jazzy), `mpc` + casadi (ground-robot only), `livox_ros_driver2` /
`Livox-SDK2` (hardware lidar driver — AirStack uses its own sensor drivers),
`decomp_rviz_plugins` / `decomp_test_node` (rviz display plugins; replaced by
the `decomp_ros_utils` shim).

Vendoring rationale: the Jazzy port needs source patches across four upstream
repos; a self-contained module repo keeps `airstack module add` a single
clone with no fork-network indirection. Upstream sync = re-copy + re-apply
the `asm_mighty` marked patches.
