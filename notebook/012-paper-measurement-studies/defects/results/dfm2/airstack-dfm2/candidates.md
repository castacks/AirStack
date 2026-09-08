# Candidate commits

Every commit in the window whose message matches the fix-keyword regex (plus all others, --all-messages). Read the diff before classifying; the message alone is `low` confidence.

## f887778024 — 2026-01-27 — Andrew Jong — no keyword

**Bring over changes from airstack-dfm2-old**

https://github.com/castacks/airstack-dfm2/commit/f8877780247df2773f95140caf932ac09d063769

+2245 / −55 in 16 files:

- `common/ros_packages/3d-waypoint-rviz2-plugin/.gitignore`
- `common/ros_packages/3d-waypoint-rviz2-plugin/CMakeLists.txt`
- `common/ros_packages/3d-waypoint-rviz2-plugin/README.md`
- `common/ros_packages/3d-waypoint-rviz2-plugin/include/waypoint_rviz2_plugin/waypoint_tool.hpp`
- `common/ros_packages/3d-waypoint-rviz2-plugin/include/waypoint_rviz2_plugin/waypoint_widget.hpp`
- `common/ros_packages/3d-waypoint-rviz2-plugin/launch/rviz2.launch.py`
- `common/ros_packages/3d-waypoint-rviz2-plugin/media/axis.dae`
- `common/ros_packages/3d-waypoint-rviz2-plugin/package.xml`
- `common/ros_packages/3d-waypoint-rviz2-plugin/plugin_description.xml`
- `common/ros_packages/3d-waypoint-rviz2-plugin/src/waypoint_tool.cpp`
- `common/ros_packages/3d-waypoint-rviz2-plugin/src/waypoint_widget.cpp`
- `common/ros_packages/3d-waypoint-rviz2-plugin/ui/waypoint_plugin.ui`
- `robot/ros_ws/src/autonomy/3_local/b_planners/takeoff_landing_planner/config/takeoff_landing_planner.yaml`
- `robot/ros_ws/src/autonomy/5_behavior/behavior_executive/src/behavior_executive.cpp`
- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`
- `simulation/isaac-sim/docker/user_TEMPLATE.config.json`

## 4c9e56bfdb — 2026-01-28 — Andrew Jong — no keyword

**Update rviz**

https://github.com/castacks/airstack-dfm2/commit/4c9e56bfdba08ef0c26636ee8ad4475aac033d08

+15 / −10 in 1 files:

- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`

## 09fc1f2680 — 2026-02-09 — Andrew Jong — no keyword

**Fast forward pegasus simulator**

https://github.com/castacks/airstack-dfm2/commit/09fc1f2680b02bced5d827c72c58479dd1a90c2b

+1 / −1 in 1 files:

- `simulation/isaac-sim/extensions/PegasusSimulator`

## f37a9faf75 — 2026-02-09 — Andrew Jong — no keyword

**Change project name to airstack-dfm2**

https://github.com/castacks/airstack-dfm2/commit/f37a9faf75a5c11fad176e2e7680b3fbee95c490

+1 / −1 in 1 files:

- `.env`

## 784b5c08ec — 2026-02-10 — N47IN — no keyword

**Change version to 0.0.1**

https://github.com/castacks/airstack-dfm2/commit/784b5c08ec6ecd91b41aa5c0d254820eb611de6a

+1 / −1 in 1 files:

- `.env`

## 1f3383e0fd — 2026-02-10 — N47IN — no keyword

**Add DFM2**

https://github.com/castacks/airstack-dfm2/commit/1f3383e0fd141e6fb3d7af777c05a396bd4bef41

+32776 / −0 in 176 files:

- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/CMakeLists.txt`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/README.md`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/.gitignore`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/.gitmodules`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/CONTRIBUTING.md`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/LICENSE`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/README.md`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/assets/abstract_fig.jpg`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/assets/example1.jpg`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/assets/example2.jpg`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/assets/example3.jpg`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/assets/logo.gif`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/assets/method_teaser.gif`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/compile.sh`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/docker/desktop.Dockerfile`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/docker/jetson.Dockerfile`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/environment.yml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/README.md`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/patches/concept_graphs.patch`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/patches/hovsg.patch`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/replica_conceptfusion.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/replica_concpgr.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/replica_naclip.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/replica_naradio.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/replica_trident.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/scannet_conceptfusion.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/scannet_concpgr.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/scannet_naclip.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/scannet_naradio.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/scannet_trident.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/tartanair_conceptfusion.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/tartanair_naclip.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/tartanair_naradio.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/semseg_configs/tartanair_trident.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/srchvol_configs/base.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/srchvol_configs/rayfronts_0.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/srchvol_configs/rayfronts_10.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/srchvol_configs/rayfronts_20.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/srchvol_configs/sempose_0.yaml`
- `robot/ros_ws/src/autonomy/4_global/a_world_models/resilience/RayFronts/experiments/srchvol_configs/spherical_semfronts_10.yaml`
- … 136 more

## 624598fde4 — 2026-02-10 — N47IN — no keyword

**Changes to make dfm2/rayfronts work**

https://github.com/castacks/airstack-dfm2/commit/624598fde4b9fe7972f8338f6c524249ac15ff2d

+49 / −3 in 2 files:

- `robot/docker/.bashrc`
- `robot/docker/Dockerfile.robot`

## 6c0fb1ea0b — 2026-02-10 — Andrew Jong — no keyword

**Switch to standalone mode by default, we'll be using this mostly for dfm2**

https://github.com/castacks/airstack-dfm2/commit/6c0fb1ea0b34b1b58a898bf3cce88f3dfd54f424

+2 / −2 in 1 files:

- `.env`

## f5de521461 — 2026-02-10 — Andrew Jong — no keyword

**Update pegasus extension auto enable**

https://github.com/castacks/airstack-dfm2/commit/f5de5214618fdfeced1b6895ae7f02fe70ae5279

+1 / −2 in 1 files:

- `simulation/isaac-sim/docker/user_TEMPLATE.config.json`

## 84acd9ef59 — 2026-02-11 — Andrew Jong — no keyword

**Add force vector field magnitude visualization with arrows**

https://github.com/castacks/airstack-dfm2/commit/84acd9ef5913ab15ca5840ad2c4810d2db5f4189

+223 / −6 in 3 files:

- `simulation/isaac-sim/launch_scripts/example_px4_pegasus_with_fan_force_field.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/base.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`

## 57e6fb7a56 — 2026-02-11 — Andrew Jong — no keyword

**Try tune parameter values to get some drift**

https://github.com/castacks/airstack-dfm2/commit/57e6fb7a569f284366071e5fd6541052b1435a05

+35 / −27 in 2 files:

- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`
- `simulation/isaac-sim/launch_scripts/example_px4_pegasus_with_fan_force_field.py`

## bb78dec8fb — 2026-02-11 — Andrew Jong — no keyword

**Add source prim, e.g. fan to visualize**

https://github.com/castacks/airstack-dfm2/commit/bb78dec8fb3f56fe62f95a3c433e8b3f84251bc0

+218 / −67 in 7 files:

- `simulation/isaac-sim/assets/objects/box_fan.usd`
- `simulation/isaac-sim/assets/objects/box_fan_spinning.usd`
- `simulation/isaac-sim/assets/objects/box_fan_spinning_2.usd`
- `simulation/isaac-sim/launch_scripts/example_px4_pegasus_with_fan_force_field.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/README.md`
- `simulation/isaac-sim/scene_library/physics/force_fields/base.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`

## 284b92b1d4 — 2026-02-11 — Andrew Jong — no keyword

**Remove shadow from sphere visualization**

https://github.com/castacks/airstack-dfm2/commit/284b92b1d4529834c80db179082c2b12d1835ee5

+4 / −0 in 1 files:

- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`

## 1178b827c2 — 2026-02-11 — Andrew Jong — no keyword

**Wind sim (#1)**

https://github.com/castacks/airstack-dfm2/commit/1178b827c2d1c1dba2721346c9a62f66e7bcd999

```
* Add fan force field (doesn't work yet)

* Add fan force field (doesn't work yet)

* Working force field

* Multiple force fields

* working force fields

* Rename wind to spherical force field

* Refactor force fields into library package

* Turn on visualization

* Make original script use the new scene library API

---------

Co-authored-by: krrishj18 <krrishjain1801@gmail.com>
Co-authored-by: krrishj18 <krrishj18@users.noreply.github.com>
```

+1348 / −1 in 8 files:

- `.env`
- `simulation/isaac-sim/launch_scripts/example_px4_pegasus_with_fan_force_field.py`
- `simulation/isaac-sim/scene_library/physics/__init__.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/README.md`
- `simulation/isaac-sim/scene_library/physics/force_fields/__init__.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/base.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/utils.py`

## 81e767a5f2 — 2026-02-11 — Andrew Jong — no keyword

**Update exts**

https://github.com/castacks/airstack-dfm2/commit/81e767a5f210ff5db8da20b2e4c6b38e9ad8c338

+0 / −3 in 1 files:

- `simulation/isaac-sim/launch_scripts/barebones_pegasus_launch.py`

## 1f4669e93d — 2026-02-11 — Andrew Jong — no keyword

**Remove old files**

https://github.com/castacks/airstack-dfm2/commit/1f4669e93dc28e92bb336f83a2f4ef1b52dd5fdf

+10 / −1129 in 10 files:

- `simulation/isaac-sim/launch_scripts/README.md`
- `simulation/isaac-sim/launch_scripts/barebones_pegasus_launch.py`
- `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`
- `simulation/isaac-sim/launch_scripts/fan_field.py`
- `simulation/isaac-sim/launch_scripts/launch_sim.py`
- `simulation/isaac-sim/launch_scripts/pegasus_scene.py`
- `simulation/isaac-sim/launch_scripts/plane_with_cubes.py`
- `simulation/isaac-sim/launch_scripts/random_primitive_obstacles.py`
- `simulation/isaac-sim/launch_scripts/scene_customizer.py`
- `simulation/isaac-sim/launch_scripts/test_scene.py`

## 143be32fd7 — 2026-02-11 — Andrew Jong — no keyword

**Add structure for scene library**

https://github.com/castacks/airstack-dfm2/commit/143be32fd71af52c40780ca1f751c0cafd3eb23e

+31 / −0 in 5 files:

- `simulation/isaac-sim/scene_library/__init__.py`
- `simulation/isaac-sim/scene_library/environments/__init__.py`
- `simulation/isaac-sim/scene_library/objects/__init__.py`
- `simulation/isaac-sim/scene_library/physics/__init__.py`
- `simulation/isaac-sim/scene_library/utils/__init__.py`

## 045eb2727f — 2026-02-12 — Andrew Jong — no keyword

**Tune strength parameter, 0.2 for small fan is better**

https://github.com/castacks/airstack-dfm2/commit/045eb2727f7d02fe42f5c18460e478cfae80e5c5

+1 / −1 in 1 files:

- `simulation/isaac-sim/launch_scripts/warehouse_single_pegasus_with_force_fields.py`

## 96b99d0e3a — 2026-02-12 — Andrew Jong — no keyword

**Tune params for demo**

https://github.com/castacks/airstack-dfm2/commit/96b99d0e3ad63104bd53a1e61ebbeeced57926f0

+10 / −10 in 2 files:

- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`
- `simulation/isaac-sim/launch_scripts/warehouse_single_pegasus_with_force_fields.py`

## 47883795fd — 2026-02-12 — Andrew Jong — no keyword

**Make visualization prims 'guides' for easy toggle**

https://github.com/castacks/airstack-dfm2/commit/47883795fd270c3dfdeba8d709b90950a5a21577

+24 / −15 in 3 files:

- `simulation/isaac-sim/scene_library/physics/force_fields/base.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/conical.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`

## db45864821 — 2026-02-12 — Andrew Jong — no keyword

**Double up on fans in aisle, reduce strength**

https://github.com/castacks/airstack-dfm2/commit/db458648212e5d736fdc6d0a8335f353ab08d734

+10 / −6 in 1 files:

- `simulation/isaac-sim/launch_scripts/warehouse_single_pegasus_with_force_fields.py`

## b39ed2581c — 2026-02-12 — Andrew Jong — no keyword

**Reduce expansion radius to 0.5 to match small drone**

https://github.com/castacks/airstack-dfm2/commit/b39ed2581c6b40e126cfcf72390c8a1e28d59af7

+1 / −1 in 1 files:

- `robot/ros_ws/src/autonomy/3_local/local_bringup/launch/local.launch.xml`

## e303fac3a0 — 2026-02-12 — Andrew Jong — no keyword

**Update warehouse params**

https://github.com/castacks/airstack-dfm2/commit/e303fac3a0d0da705f2f28fb133aa782150d199d

+15 / −7 in 2 files:

- `.env`
- `simulation/isaac-sim/launch_scripts/warehouse_single_pegasus_with_force_fields.py`

## 071e3ded9a — 2026-02-12 — Andrew Jong — no keyword

**Add floor fan**

https://github.com/castacks/airstack-dfm2/commit/071e3ded9a02ff472811d322be29e330e793cdb5

+0 / −0 in 15 files:

- `simulation/isaac-sim/assets/objects/floor_fan.usdc`
- `simulation/isaac-sim/assets/objects/floor_fan_spinning.usd`
- `simulation/isaac-sim/assets/objects/textures/Blade.001_baseColor.png`
- `simulation/isaac-sim/assets/objects/textures/Blade.001_metallicRoughness_metal_scale0.jpg`
- `simulation/isaac-sim/assets/objects/textures/Blade.001_metallicRoughness_rough.jpg`
- `simulation/isaac-sim/assets/objects/textures/Grille_baseColor.jpg`
- `simulation/isaac-sim/assets/objects/textures/Grille_metallicRoughness_metal_scale3.jpg`
- `simulation/isaac-sim/assets/objects/textures/Grille_metallicRoughness_rough_scale4.jpg`
- `simulation/isaac-sim/assets/objects/textures/Grille_normal_norm.jpg`
- `simulation/isaac-sim/assets/objects/textures/base_baseColor.jpg`
- `simulation/isaac-sim/assets/objects/textures/base_emissive.jpg`
- `simulation/isaac-sim/assets/objects/textures/base_metallicRoughness_metal_scale1.jpg`
- `simulation/isaac-sim/assets/objects/textures/base_metallicRoughness_rough_scale2.jpg`
- `simulation/isaac-sim/assets/objects/textures/base_normal_norm.jpg`
- `simulation/isaac-sim/assets/objects/textures/color_0C0C0C.exr`

## d70ef98f6d — 2026-02-12 — Andrew Jong — no keyword

**Enable translation offset for source prim**

https://github.com/castacks/airstack-dfm2/commit/d70ef98f6d99f10ece22cef0c97485e8c3a102c2

+12 / −7 in 3 files:

- `simulation/isaac-sim/scene_library/physics/force_fields/base.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/conical.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`

## 88669bfbd0 — 2026-02-12 — Andrew Jong — no keyword

**Start warehouse env**

https://github.com/castacks/airstack-dfm2/commit/88669bfbd0764730dcbd6afef2b888720c1feb72

+361 / −10 in 2 files:

- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`
- `simulation/isaac-sim/launch_scripts/warehouse_single_pegasus_with_force_fields.py`

## a6e09fd1e0 — 2026-02-12 — Andrew Jong — no keyword

**Add bookmark for dontfoolmetwice on nucleus**

https://github.com/castacks/airstack-dfm2/commit/a6e09fd1e062c6b371506197feec87c5e059acba

+3 / −1 in 1 files:

- `simulation/isaac-sim/docker/omniverse.toml`

## fa089a78e3 — 2026-02-12 — Andrew Jong — KEYWORD fix

**Fix fan blades not staying with fan**

https://github.com/castacks/airstack-dfm2/commit/fa089a78e32a96aead38b28a83f6e6f8cc370e9c

+0 / −0 in 2 files:

- `simulation/isaac-sim/assets/objects/box_fan.usd`
- `simulation/isaac-sim/assets/objects/box_fan_spinning.usd`

## 341e9e1724 — 2026-02-12 — Andrew Jong — KEYWORD fix

**Fix orientation to inherit from parent**

https://github.com/castacks/airstack-dfm2/commit/341e9e172419f9840a34091b01af08d57ca9dee3

+26 / −21 in 4 files:

- `simulation/isaac-sim/launch_scripts/example_px4_pegasus_with_fan_force_field.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/README.md`
- `simulation/isaac-sim/scene_library/physics/force_fields/__init__.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`

## f99ca6a3e9 — 2026-02-12 — Andrew Jong — no keyword

**Improve rotation parameter for conical**

https://github.com/castacks/airstack-dfm2/commit/f99ca6a3e9b57327c40dd11a48301de83f18074e

+72 / −13 in 6 files:

- `simulation/isaac-sim/assets/objects/box_fan.usd`
- `simulation/isaac-sim/assets/objects/box_fan_spinning.usd`
- `simulation/isaac-sim/launch_scripts/example_px4_pegasus_with_fan_force_field.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/base.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/conical.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`

## d9ccd6600d — 2026-02-12 — Andrew Jong — KEYWORD fix

**Fix box fan orientation properties**

https://github.com/castacks/airstack-dfm2/commit/d9ccd6600d138108b771a9624427951a173c055f

+0 / −0 in 3 files:

- `simulation/isaac-sim/assets/objects/box_fan.usd`
- `simulation/isaac-sim/assets/objects/box_fan_spinning.usd`
- `simulation/isaac-sim/assets/objects/box_fan_spinning_2.usd`

## a2708e5fc0 — 2026-02-12 — Andrew Jong — KEYWORD fix

**Fix arrows and conical force field**

https://github.com/castacks/airstack-dfm2/commit/a2708e5fc04c77df9015c6f1b142af9c3ec53ffb

+36 / −14 in 2 files:

- `simulation/isaac-sim/scene_library/physics/force_fields/base.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/conical.py`

## 5d85f8bdcd — 2026-02-12 — Andrew Jong — no keyword

**Refactor prims under parent xform for source and visualizations**

https://github.com/castacks/airstack-dfm2/commit/5d85f8bdcd65e734b61cc48ec0e9c76a9295e8cf

+90 / −54 in 3 files:

- `simulation/isaac-sim/scene_library/physics/force_fields/base.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/conical.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`

## 3545d61ec1 — 2026-02-12 — Andrew Jong — no keyword

**Add conicalforcefield**

https://github.com/castacks/airstack-dfm2/commit/3545d61ec1ea5e0c553a3fe61a4a11ed53844f29

+576 / −10 in 7 files:

- `simulation/isaac-sim/launch_scripts/example_px4_pegasus_with_fan_force_field.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/ConicalForceField_Parameters.md`
- `simulation/isaac-sim/scene_library/physics/force_fields/README.md`
- `simulation/isaac-sim/scene_library/physics/force_fields/__init__.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/base.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/conical.py`
- `simulation/isaac-sim/scene_library/physics/force_fields/spherical.py`

## e541ba78d1 — 2026-02-13 — krrishj18 — no keyword

**added spot and omni directional strobe lights**

https://github.com/castacks/airstack-dfm2/commit/e541ba78d1096ac62af0507a98e64f117fabf9ae

+739 / −17 in 7 files:

- `.env`
- `robot/ros_ws/src/autonomy/3_local/local_bringup/launch/local.launch.xml`
- `simulation/isaac-sim/launch_scripts/example_px4_pegasus_with_strobe_lights.py`
- `simulation/isaac-sim/scene_library/objects/__init__.py`
- `simulation/isaac-sim/scene_library/objects/strobe_light/StrobeLight.py`
- `simulation/isaac-sim/scene_library/objects/strobe_light/__init__.py`
- `simulation/isaac-sim/scene_library/objects/strobe_light/base.py`

## bc865d7658 — 2026-02-16 — krrishj18 — no keyword

**added lensFlare launch file**

https://github.com/castacks/airstack-dfm2/commit/bc865d7658fbb96360139b3462e5f00700dbd910

+359 / −0 in 1 files:

- `simulation/isaac-sim/launch_scripts/warehouse_single_pegasus_with_force_fields_lensflare.py`

## 5a0f6acb38 — 2026-02-17 — krrishj18 — no keyword

**added flare settings to py**

https://github.com/castacks/airstack-dfm2/commit/5a0f6acb389272be7dbb5023a0d0769752b24648

+76 / −11 in 3 files:

- `.env`
- `robot/docker/Dockerfile.robot`
- `simulation/isaac-sim/launch_scripts/example_px4_pegasus_with_strobe_lights.py`

## 3af8f8e079 — 2026-02-19 — krrishj18 — no keyword

**added ice plane function**

https://github.com/castacks/airstack-dfm2/commit/3af8f8e0797207ac9d0c8232012ec1362101d04a

+108 / −0 in 1 files:

- `simulation/isaac-sim/scene_library/physics/ground_plane.py`
