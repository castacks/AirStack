# Candidate commits

Every commit in the window whose message matches the fix-keyword regex (plus all others, --all-messages). Read the diff before classifying; the message alone is `low` confidence.

## e828ef059e — 2026-03-31 — Seungchan (airstation-01) — no keyword

**raven-airstack setup**

https://github.com/castacks/AirStack/commit/e828ef059e9e42459c1c14ad7a808486f7a88491

+15 / −2 in 1 files:

- `docs/README.md`

## b9af9df84b — 2026-03-31 — Seungchan (airstation-01) — no keyword

**raven-specific readme**

https://github.com/castacks/AirStack/commit/b9af9df84b7e26f14c3d11d7b23d16341d4f9cc7

+3 / −94 in 1 files:

- `docs/README.md`

## 4241433b28 — 2026-03-31 — Seungchan (airstation-01) — no keyword

**change to .env file**

https://github.com/castacks/AirStack/commit/4241433b28173243f542d335fa949b6747df5793

+2 / −2 in 1 files:

- `.env`

## 5667c6b390 — 2026-04-03 — krrishj18 — no keyword

**updated download script**

https://github.com/castacks/AirStack/commit/5667c6b3903d74e20566606594b68bdc11275c77

+10 / −13 in 1 files:

- `scenes/download_scenes.sh`

## 94b8d42c4b — 2026-04-03 — krrishj18 — no keyword

**added download scenes script**

https://github.com/castacks/AirStack/commit/94b8d42c4bd549b8cdfa3829a1dc018b2d21fd0e

+20 / −0 in 1 files:

- `scenes/download_scenes.sh`

## a24530bb33 — 2026-04-03 — krrishj18 — no keyword

**added scene placeholders**

https://github.com/castacks/AirStack/commit/a24530bb332f9e4f972a9d70c49a5e8ebf03e930

+12 / −0 in 8 files:

- `scenes/AbandonedFactory/.gitignore`
- `scenes/AbandonedFactory/.gitkeep`
- `scenes/ConstructionSite/.gitignore`
- `scenes/ConstructionSite/.gitkeep`
- `scenes/FireAcademy/.gitignore`
- `scenes/FireAcademy/.gitkeep`
- `scenes/RetroNeighborhood/.gitignore`
- `scenes/RetroNeighborhood/.gitkeep`

## 3f5f2a4ecf — 2026-04-03 — krrishj18 — no keyword

**added launch files**

https://github.com/castacks/AirStack/commit/3f5f2a4ecf7d3f7a86921799f1266defcc2cb75e

+604 / −0 in 4 files:

- `simulation/isaac-sim/launch_scripts/AbandonedFactory_Launch.py`
- `simulation/isaac-sim/launch_scripts/ConstructionSite_Launch.py`
- `simulation/isaac-sim/launch_scripts/FireAcademy_Launch.py`
- `simulation/isaac-sim/launch_scripts/RetroNeighborhood_Launch.py`

## 9a8cfd7c32 — 2026-04-05 — Seungchan (airstation-01) — no keyword

**update download_scenes.sh with new google drive link**

https://github.com/castacks/AirStack/commit/9a8cfd7c32a4a1c1fa82845467e85a1232e26c1b

+1 / −1 in 1 files:

- `scenes/download_scenes.sh`

## 1f48484c72 — 2026-04-26 — Seungchan (airstation-01) — KEYWORD fix,typo

**fix typo in .env; add omni_pass.env and api token warning**

https://github.com/castacks/AirStack/commit/1f48484c7270b3de884ba7b36a08d15a60f2048f

+7 / −1 in 2 files:

- `.env`
- `docs/README.md`

## f7ca3698ed — 2026-04-27 — Seungchan (airstation-01) — no keyword

**lidar min range 0.75->3.5**

https://github.com/castacks/AirStack/commit/f7ca3698ed37c46e286812592b51a4204e15fe57

+14 / −12 in 5 files:

- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`
- `simulation/isaac-sim/launch_scripts/AbandonedFactory_Launch.py`
- `simulation/isaac-sim/launch_scripts/ConstructionSite_Launch.py`
- `simulation/isaac-sim/launch_scripts/FireAcademy_Launch.py`
- `simulation/isaac-sim/launch_scripts/RetroNeighborhood_Launch.py`

## 947a189acc — 2026-04-27 — Seungchan (airstation-01) — no keyword

**change default rviz setting**

https://github.com/castacks/AirStack/commit/947a189acc87b0816c07d01201e1e46750a31bc5

+63 / −79 in 1 files:

- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`

## d074a58a79 — 2026-04-29 — Seungchan (airstation-01) — no keyword

**rviz includes rayfronts voxels rgb, frontier clouds, and frontier centroids**

https://github.com/castacks/AirStack/commit/d074a58a7907e66610d763e60c3b16ec3a6a4f77

+125 / −19 in 1 files:

- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`

## 0f674fa2f0 — 2026-04-30 — Seungchan (airstation-01) — no keyword

**remove gcs rviz & gui for launch**

https://github.com/castacks/AirStack/commit/0f674fa2f0658fce4687fff672d5561dd63a70c0

+16 / −4 in 2 files:

- `docker-compose.yaml`
- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`

## 32532c0efb — 2026-04-30 — Seungchan (airstation-01) — no keyword

**add mode_text vis**

https://github.com/castacks/AirStack/commit/32532c0efbfca22dd685b4e656874caf89dbaef8

+26 / −12 in 1 files:

- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`

## ec3c9b958d — 2026-05-02 — Seungchan (airstation-01) — no keyword

**modify the trajectory_vis and droan/traj_debug thickness for rviz**

https://github.com/castacks/AirStack/commit/ec3c9b958d6a5760e5ee7dfd1d11c54a928b1d0f

+7 / −7 in 2 files:

- `robot/ros_ws/src/autonomy/3_local/b_planners/droan_gl/src/expand.cpp`
- `robot/ros_ws/src/autonomy/3_local/c_controls/trajectory_controller/src/trajectory_controller.cpp`

## d7fb468bfc — 2026-05-05 — Seungchan (airstation-01) — no keyword

**trajectory thickness 1.0**

https://github.com/castacks/AirStack/commit/d7fb468bfc5472fc3b1f016aff68a6a99bbc45d0

+1 / −1 in 1 files:

- `robot/ros_ws/src/autonomy/3_local/c_controls/trajectory_controller/src/trajectory_controller.cpp`

## 7fb479be06 — 2026-05-05 — Seungchan (airstation-01) — no keyword

**trajectory thickness 0.75**

https://github.com/castacks/AirStack/commit/7fb479be06848e7cbad869d5c87a690a56b9f728

+1 / −1 in 1 files:

- `robot/ros_ws/src/autonomy/3_local/c_controls/trajectory_controller/src/trajectory_controller.cpp`

## 43e794aa7f — 2026-05-12 — Seungchan (airstation-01) — no keyword

**rviz show annotation box for default**

https://github.com/castacks/AirStack/commit/43e794aa7f882d242b9d7cecdc2d96728f71b123

+54 / −6 in 1 files:

- `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`

## 6b9c0d5092 — 2026-05-12 — Seungchan (airstation-01) — KEYWORD fix

**minor fix on .env**

https://github.com/castacks/AirStack/commit/6b9c0d50921ed071e88ef1315c99e631b43fd260

+8 / −0 in 1 files:

- `.env`

## 9d7c3231b0 — 2026-05-12 — Seungchan (airstation-01) — no keyword

**user input arguments for launch_raven**

https://github.com/castacks/AirStack/commit/9d7c3231b0c0ad8364df5bdac44fffc8437e695c

+37 / −35 in 6 files:

- `.env`
- `simulation/isaac-sim/docker/docker-compose.yaml`
- `simulation/isaac-sim/launch_scripts/AbandonedFactory_Launch.py`
- `simulation/isaac-sim/launch_scripts/ConstructionSite_Launch.py`
- `simulation/isaac-sim/launch_scripts/FireAcademy_Launch.py`
- `simulation/isaac-sim/launch_scripts/RetroNeighborhood_Launch.py`

## 88764bc651 — 2026-05-12 — Seungchan (airstation-01) — no keyword

**parameterize drone x,y,z, qx,qy,qz,qz inputs**

https://github.com/castacks/AirStack/commit/88764bc651fa55daf21fcfb50122a77457a7d567

+34 / −12 in 4 files:

- `simulation/isaac-sim/launch_scripts/AbandonedFactory_Launch.py`
- `simulation/isaac-sim/launch_scripts/ConstructionSite_Launch.py`
- `simulation/isaac-sim/launch_scripts/FireAcademy_Launch.py`
- `simulation/isaac-sim/launch_scripts/RetroNeighborhood_Launch.py`

## bc4eb5be3b — 2026-05-18 — Seungchan (airstation-01) — no keyword

**added scene_prep add collider script**

https://github.com/castacks/AirStack/commit/bc4eb5be3bd4f65b3dde542eadead151eb83fdcc

+705 / −0 in 5 files:

- `simulation/isaac-sim/launch_scripts/DowntownWest_Launch.py`
- `simulation/isaac-sim/launch_scripts/ModernCityDowntown_Launch.py`
- `simulation/isaac-sim/launch_scripts/Shipyard_Launch.py`
- `simulation/isaac-sim/utils/__init__.py`
- `simulation/isaac-sim/utils/scene_prep.py`

## c093d6ea05 — 2026-05-18 — Seungchan (airstation-01) — no keyword

**adding launch scripts for five scenes**

https://github.com/castacks/AirStack/commit/c093d6ea054f39d969a641d9f873c547393e5c7f

+775 / −0 in 5 files:

- `simulation/isaac-sim/launch_scripts/AbandonedCity_Launch.py`
- `simulation/isaac-sim/launch_scripts/DowntownWest_Launch.py`
- `simulation/isaac-sim/launch_scripts/MilitaryBase_Launch.py`
- `simulation/isaac-sim/launch_scripts/ModernCityDowntown_Launch.py`
- `simulation/isaac-sim/launch_scripts/Shipyard_Launch.py`

## e7ef77c03c — 2026-05-19 — Seungchan (airstation-01) — no keyword

**add colliders for original 4 envs**

https://github.com/castacks/AirStack/commit/e7ef77c03c570397443737ab0deb032a4f1ae5af

+84 / −0 in 4 files:

- `simulation/isaac-sim/launch_scripts/AbandonedFactory_Launch.py`
- `simulation/isaac-sim/launch_scripts/ConstructionSite_Launch.py`
- `simulation/isaac-sim/launch_scripts/FireAcademy_Launch.py`
- `simulation/isaac-sim/launch_scripts/RetroNeighborhood_Launch.py`

## 278acbffaf — 2026-05-22 — Seungchan (airstation-01) — no keyword

**modify construction site launch**

https://github.com/castacks/AirStack/commit/278acbffaf748cd6e0102b3a25cfea544e031c83

+9 / −23 in 1 files:

- `simulation/isaac-sim/launch_scripts/ConstructionSite_Launch.py`
