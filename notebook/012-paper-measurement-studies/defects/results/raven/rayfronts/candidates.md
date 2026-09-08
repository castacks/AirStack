# Candidate commits

Every commit in the window whose message matches the fix-keyword regex (plus all others, --all-messages). Read the diff before classifying; the message alone is `low` confidence.

## 288694dffb — 2025-08-03 — Seungchan (airstation-01) — no keyword

**copied working branch of airstation01 for planner-develop branch of forked repo**

https://github.com/seungchan-kim/RayFronts/commit/288694dffb24eca834a2afff32b57fcb8adaa341

+2125 / −22 in 11 files:

- `docker/desktop.Dockerfile`
- `input_prompt.py`
- `mapping_server_rosnode.py`
- `rayfronts/configs/dataset/ros2isaacsim.yaml`
- `rayfronts/configs/default.yaml`
- `rayfronts/configs/vis/base_vis.yaml`
- `rayfronts/datasets/ros.py`
- `rayfronts/mapping_server_rosnode.py`
- `rayfronts/ros_utils.py`
- `run_docker.sh`
- `run_mapping_server.sh`

## 82915fde96 — 2025-08-06 — Seungchan (airstation-01) — no keyword

**modularize behaviors; behavior-manager-tree**

https://github.com/seungchan-kim/RayFronts/commit/82915fde96a9b4bff9d6ce49a34ecef0c6642c75

+617 / −428 in 6 files:

- `rayfronts/behavior_manager.py`
- `rayfronts/behaviors/frontier_behavior.py`
- `rayfronts/behaviors/ray_behavior.py`
- `rayfronts/behaviors/voxel_behavior.py`
- `rayfronts/mapping_server_rosnode.py`
- `rayfronts/mode_text_visualizer.py`

## 5a9199a650 — 2025-08-07 — Seungchan (airstation-01) — no keyword

**voxel-based behavior planning**

https://github.com/seungchan-kim/RayFronts/commit/5a9199a650be2f3c95976808966758879b1b13d8

+72 / −12 in 3 files:

- `rayfronts/behavior_manager.py`
- `rayfronts/behaviors/voxel_behavior.py`
- `rayfronts/mapping_server_rosnode.py`

## 2bf21a2b60 — 2025-08-07 — Seungchan (airstation-01) — no keyword

**voxel-behavior visualization publisher**

https://github.com/seungchan-kim/RayFronts/commit/2bf21a2b605da8f6c977e6122141fe23bcc23c67

+59 / −93 in 2 files:

- `rayfronts/behavior_manager.py`
- `rayfronts/behaviors/voxel_behavior.py`

## 59b8e8808a — 2025-08-07 — Seungchan (airstation-01) — KEYWORD fix

**ray-behavior fix on heading all same directions**

https://github.com/seungchan-kim/RayFronts/commit/59b8e8808a9be20addcaa4948387990110adcc40

+12 / −9 in 1 files:

- `rayfronts/behaviors/ray_behavior.py`

## fb1b9ffe46 — 2025-08-13 — Seungchan (airstation-01) — no keyword

**voxel-behavior refinement**

https://github.com/seungchan-kim/RayFronts/commit/fb1b9ffe46a5d31481b297cbd5b41ab3f35b248e

+127 / −47 in 2 files:

- `rayfronts/behaviors/voxel_behavior.py`
- `rayfronts/mapping_server_rosnode.py`

## f18f92e5ce — 2025-08-17 — Seungchan (airstation-01) — no keyword

**added neighborhood annotate origin script**

https://github.com/seungchan-kim/RayFronts/commit/f18f92e5ceffe6f738391b29dd6e9ed3e11cbc03

+66 / −0 in 1 files:

- `rayfronts/annotations/neighborhood_annotate_origin.py`

## d1f73821da — 2025-08-18 — Seungchan (airstation-01) — no keyword

**let robot chooses top 5 best frontier randomly**

https://github.com/seungchan-kim/RayFronts/commit/d1f73821da677043ee1adfb7fa4e4bc9a3d91552

+10 / −4 in 1 files:

- `rayfronts/behaviors/frontier_behavior.py`

## c4c4516e10 — 2025-08-19 — Seungchan (airstation-01) — no keyword

**annotating script for constructionsite/militarybase**

https://github.com/seungchan-kim/RayFronts/commit/c4c4516e104aa00ec69eeb7426dbbdd122217ea2

+153 / −0 in 2 files:

- `rayfronts/annotations/constructionsite_annotate_origin.py`
- `rayfronts/annotations/militarybase_annotate_origin.py`

## 7c57142760 — 2025-08-22 — Seungchan (airstation-01) — no keyword

**adding lvlm-guided behavior**

https://github.com/seungchan-kim/RayFronts/commit/7c57142760bcc18dbfbba5c2cd37bc559d640421

+165 / −19 in 3 files:

- `rayfronts/behavior_manager.py`
- `rayfronts/behaviors/lvlm_behavior.py`
- `rayfronts/mapping_server_rosnode.py`

## 36f030e1cc — 2025-08-24 — Seungchan (airstation-01) — no keyword

**transformed annotations by starting_pos**

https://github.com/seungchan-kim/RayFronts/commit/36f030e1cc7190608844507d2f53503ada24c81c

+15094 / −0 in 37 files:

- `rayfronts/annotations/raw_annotations/{abandonedcity.json => AbandonedCity.json}`
- `rayfronts/annotations/raw_annotations/{abandonedfactory.json => AbandonedFactory.json}`
- `rayfronts/annotations/raw_annotations/{constructionsite.json => ConstructionSite.json}`
- `rayfronts/annotations/raw_annotations/{downtownwest.json => DowntownWest.json}`
- `rayfronts/annotations/raw_annotations/{fireacademy.json => FireAcademy.json}`
- `rayfronts/annotations/raw_annotations/{militarybase.json => MilitaryBase.json}`
- `rayfronts/annotations/raw_annotations/{moderncitydowntown.json => ModernCityDowntown.json}`
- `rayfronts/annotations/raw_annotations/{neighborhood.json => Neighborhood.json}`
- `rayfronts/annotations/raw_annotations/{snowyvillage.json => SnowyVillage.json}`
- `rayfronts/annotations/transform_annotation_coordinates.py`
- `rayfronts/annotations/transformed_annotations/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/AbandonedFactory_t_x-5_y-15_z0.5_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/AbandonedFactory_t_x-5_y35_z0.5_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/AbandonedFactory_t_x0_y0_z0.5_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/ConstructionSite_t_x48_y-39_z0.2_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/ConstructionSite_t_x60_y-3_z0.2_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/DowntownWest_t_x-120_y0_z0_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/DowntownWest_t_x0_y0_z0_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/DowntownWest_t_x2_y-60_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/FireAcademy_t_x-15_y0_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/FireAcademy_t_x0_y0_z0_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/FireAcademy_t_x30_y30_z1_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/MilitaryBase_t_x1070_y300_z0_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/MilitaryBase_t_x1100_y200_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/MilitaryBase_t_x1114_y28_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/ModernCityDowntown_t_x-17_y-69_z0.2_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/ModernCityDowntown_t_x-4_y20_z0.2_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/ModernCityDowntown_t_x36_y-80_z0.2_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/Neighborhood_t_x-20_y-80_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/Neighborhood_t_x0_y0_z0_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/Neighborhood_t_x160_y-19_z0_o_x0_y0_z180.json`
- `rayfronts/annotations/transformed_annotations/SnowyVillage_t_x-145_y20_z-2.5_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/SnowyVillage_t_x-152_y-80_z-2_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/SnowyVillage_t_x-200_y-80_z-2_o_x0_y0_z0.json`

## 462c443f40 — 2025-08-24 — Seungchan (airstation-01) — no keyword

**raw annotations file (coordinate transform not applied)**

https://github.com/seungchan-kim/RayFronts/commit/462c443f400fa553937ad7bdab5a31d9eeeda15f

+5013 / −0 in 9 files:

- `rayfronts/annotations/raw_annotations/abandonedcity.json`
- `rayfronts/annotations/raw_annotations/abandonedfactory.json`
- `rayfronts/annotations/raw_annotations/constructionsite.json`
- `rayfronts/annotations/raw_annotations/downtownwest.json`
- `rayfronts/annotations/raw_annotations/fireacademy.json`
- `rayfronts/annotations/raw_annotations/militarybase.json`
- `rayfronts/annotations/raw_annotations/moderncitydowntown.json`
- `rayfronts/annotations/raw_annotations/neighborhood.json`
- `rayfronts/annotations/raw_annotations/snowyvillage.json`

## b1be17508b — 2025-08-24 — Seungchan (airstation-01) — no keyword

**abandoned factory annotations file**

https://github.com/seungchan-kim/RayFronts/commit/b1be17508bae18ee9ee7d54cf74ce40c086d525d

+45 / −0 in 1 files:

- `rayfronts/annotations/abandonedfactory_annotate_origin.py`

## 0f4455cf82 — 2025-08-24 — Seungchan (airstation-01) — no keyword

**fireacademy annotation file**

https://github.com/seungchan-kim/RayFronts/commit/0f4455cf82006507ecc562af4403a80c9aad74c3

+61 / −0 in 1 files:

- `rayfronts/annotations/fireacademy_annotate_origin.py`

## 1342214637 — 2025-08-24 — Seungchan (airstation-01) — no keyword

**moderncity downtown annotation file**

https://github.com/seungchan-kim/RayFronts/commit/134221463775fff58dc9bcba7e589b99b6c70a57

+33 / −0 in 1 files:

- `rayfronts/annotations/moderncitydowntown_annotate_origin.py`

## 5ceb006a37 — 2025-08-24 — Seungchan (airstation-01) — no keyword

**snowy village annotation file**

https://github.com/seungchan-kim/RayFronts/commit/5ceb006a3754db222cd35938520208bfa7efad0c

+58 / −1 in 2 files:

- `rayfronts/annotations/downtownwest_annotate_origin.py`
- `rayfronts/annotations/snowyvillage_annotate_origin.py`

## 01af64c4c8 — 2025-08-24 — Seungchan (airstation-01) — no keyword

**downtown west annotation file**

https://github.com/seungchan-kim/RayFronts/commit/01af64c4c8b7a681340ace379d12351f6ced3873

+57 / −0 in 1 files:

- `rayfronts/annotations/downtownwest_annotate_origin.py`

## 2cada6ea50 — 2025-08-24 — Seungchan (airstation-01) — no keyword

**abandonedcity annotation file**

https://github.com/seungchan-kim/RayFronts/commit/2cada6ea5068cf511e4598cfdf0e3b816b41fbdc

+71 / −0 in 1 files:

- `rayfronts/annotations/abandonedcity_annotate_origin.py`

## 2933ed9f47 — 2025-08-25 — Seungchan (airstation-01) — no keyword

**multi-objects query for voxel and ray behaviors**

https://github.com/seungchan-kim/RayFronts/commit/2933ed9f4740c7ab6ce64fee68a3c231c406c75e

+72 / −40 in 6 files:

- `rayfronts/behavior_manager.py`
- `rayfronts/behaviors/frontier_behavior.py`
- `rayfronts/behaviors/ray_behavior.py`
- `rayfronts/behaviors/voxel_behavior.py`
- `rayfronts/mapping_server_rosnode.py`
- `rayfronts/mode_text_visualizer.py`

## 5d058f158f — 2025-08-25 — Seungchan (airstation-01) — KEYWORD fixing

**fixing ray_behavior to select one direction at a time**

https://github.com/seungchan-kim/RayFronts/commit/5d058f158f795422a2ce4e2682a8907faa19c74e

+85 / −35 in 1 files:

- `rayfronts/behaviors/ray_behavior.py`

## 96b873146a — 2025-08-25 — Seungchan (airstation-01) — KEYWORD fix

**voxel behavior fix on surface point intersection of cuboid and robot pose ray**

https://github.com/seungchan-kim/RayFronts/commit/96b873146a800fda2e6c85662731d271237c6395

+24 / −10 in 1 files:

- `rayfronts/behaviors/voxel_behavior.py`

## 8858896292 — 2025-08-26 — Seungchan (airstation-01) — no keyword

**added success check and success metric real-time in mission_checker, mode_text_visualizer**

https://github.com/seungchan-kim/RayFronts/commit/8858896292ea2e023fec489941d27f83a858a188

+12 / −4 in 1 files:

- `rayfronts/mode_text_visualizer.py`

## 05c51775a4 — 2025-08-27 — Seungchan (airstation-01) — no keyword

**mission checker update**

https://github.com/seungchan-kim/RayFronts/commit/05c51775a4e5ccae3dfb8015f2ee44c8fb8843ac

+196 / −0 in 1 files:

- `mission_checker.py`

## d993234341 — 2025-08-28 — Seungchan (airstation-01) — no keyword

**oracle lengths**

https://github.com/seungchan-kim/RayFronts/commit/d993234341ed336846808c3bf318edc9b5c3e158

+669 / −0 in 108 files:

- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_building.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_bus stop, yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_car, bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_yellow motorhome, car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_building.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_bus stop, yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_car, bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_yellow motorhome, car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_building.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_bus stop, yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_car, bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_yellow motorhome, car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_biotoilet.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_blue tarp, orange tarp, yellow towercrane.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_blue tarp.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_cabling winches.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_construction lift, asphalt roller.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_orange towercrane, forklift.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_orange towercrane.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x48_y-39_z0.2_o_x0_y0_z90_biotoilet.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x48_y-39_z0.2_o_x0_y0_z90_blue tarp, orange tarp, yellow towercrane.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x48_y-39_z0.2_o_x0_y0_z90_blue tarp.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x48_y-39_z0.2_o_x0_y0_z90_cabling winches.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x48_y-39_z0.2_o_x0_y0_z90_construction lift, asphalt roller.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x48_y-39_z0.2_o_x0_y0_z90_orange towercrane, forklift.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x48_y-39_z0.2_o_x0_y0_z90_orange towercrane.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x60_y-3_z0.2_o_x0_y0_z-90_biotoilet.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x60_y-3_z0.2_o_x0_y0_z-90_blue tarp, orange tarp, yellow towercrane.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x60_y-3_z0.2_o_x0_y0_z-90_blue tarp.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x60_y-3_z0.2_o_x0_y0_z-90_cabling winches.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x60_y-3_z0.2_o_x0_y0_z-90_construction lift, asphalt roller.json`
- … 68 more

## c1b5065bf8 — 2025-08-28 — Seungchan (airstation-01) — KEYWORD fixed

**fixed ATV to all-terrain vehicle**

https://github.com/seungchan-kim/RayFronts/commit/c1b5065bf86e77c51c7812ebf093b70f1fe64744

+21 / −21 in 5 files:

- `rayfronts/annotations/raw_annotations/MilitaryBase.json`
- `rayfronts/annotations/raw_annotations/Neighborhood.json`
- `rayfronts/annotations/transformed_annotations/MilitaryBase_t_x1070_y300_z0_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/MilitaryBase_t_x1100_y200_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/MilitaryBase_t_x1114_y28_z0_o_x0_y0_z90.json`

## 413d2c0f35 — 2025-08-28 — Seungchan (airstation-01) — KEYWORD fixes

**small fixes on annotation files**

https://github.com/seungchan-kim/RayFronts/commit/413d2c0f3525909d1b8f2699b83885fa416c0801

+69 / −69 in 4 files:

- `rayfronts/annotations/transformed_annotations/{FireAcademy_t_x-15_y0_z0_o_x0_y0_z90.json => FireAcademy_t_x-15_y0_z0_o_x0_y0_z-90.json}`
- `rayfronts/annotations/transformed_annotations/Neighborhood_t_x-20_y-80_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/Neighborhood_t_x0_y0_z0_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/Neighborhood_t_x160_y-19_z0_o_x0_y0_z180.json`

## f0204284a8 — 2025-08-30 — Seungchan (airstation-01) — no keyword

**change all-terrain_vechicle -> ATV**

https://github.com/seungchan-kim/RayFronts/commit/f0204284a893c5aead300a59f99ac2585afa36fe

+12 / −12 in 3 files:

- `rayfronts/annotations/transformed_annotations/MilitaryBase_t_x1070_y300_z0_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/MilitaryBase_t_x1100_y200_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/MilitaryBase_t_x1114_y28_z0_o_x0_y0_z90.json`

## b5e999a0ca — 2025-08-30 — Seungchan (airstation-01) — KEYWORD fix

**construction-site transformed annotation and oracle path fix**

https://github.com/seungchan-kim/RayFronts/commit/b5e999a0ca5417eb3b836d11a7217e687b993cfd

+64 / −64 in 15 files:

- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y-8.5_z0.2_o_x0_y0_z0_biotoilet.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y-8.5_z0.2_o_x0_y0_z0_blue tarp, orange tarp, yellow towercrane.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y-8.5_z0.2_o_x0_y0_z0_blue tarp.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y-8.5_z0.2_o_x0_y0_z0_cabling winches.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y-8.5_z0.2_o_x0_y0_z0_construction lift, asphalt roller.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y-8.5_z0.2_o_x0_y0_z0_orange towercrane, forklift.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y-8.5_z0.2_o_x0_y0_z0_orange towercrane.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_biotoilet.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_blue tarp, orange tarp, yellow towercrane.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_blue tarp.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_cabling winches.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_construction lift, asphalt roller.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_orange towercrane, forklift.json`
- `rayfronts/annotations/oracle_paths/ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0_orange towercrane.json`
- `rayfronts/annotations/transformed_annotations/{ConstructionSite_t_x-27_y8.5_z0.2_o_x0_y0_z0.json => ConstructionSite_t_x-27_y-8.5_z0.2_o_x0_y0_z0.json}`

## a057f17868 — 2025-09-01 — Seungchan (airstation-01) — KEYWORD fixed

**fixed vlfm planner**

https://github.com/seungchan-kim/RayFronts/commit/a057f1786829f77d6ba3b0e061b658b844e2759d

+8 / −4 in 1 files:

- `rayfronts/mapping_server_vlfm.py`

## 5b5912230d — 2025-09-01 — Seungchan (airstation-01) — no keyword

**adding vlfm implementations**

https://github.com/seungchan-kim/RayFronts/commit/5b5912230dab47b7429ecb36e88adb95b65c9360

+746 / −12 in 6 files:

- `rayfronts/behavior_manager.py`
- `rayfronts/mapping/semantic_ray_frontiers_map.py`
- `rayfronts/mapping_server_rosnode.py`
- `rayfronts/mapping_server_vlfm.py`
- `rayfronts/mode_text_visualizer.py`
- `run_vlfm.sh`

## 3c909f85fe — 2025-09-01 — Seungchan (airstation-01) — KEYWORD fix

**filter out rays behind; LVLM behavior fix**

https://github.com/seungchan-kim/RayFronts/commit/3c909f85feff37154ad01333d57711b69773ec1b

+41 / −18 in 3 files:

- `rayfronts/behaviors/lvlm_behavior.py`
- `rayfronts/behaviors/ray_behavior.py`
- `rayfronts/behaviors/voxel_behavior.py`

## eca66ecb81 — 2025-09-05 — Seungchan (airstation-01) — no keyword

**downtown west new annotation files**

https://github.com/seungchan-kim/RayFronts/commit/eca66ecb811ee9be9e3efd3098d671e18cf97374

+10 / −10 in 1 files:

- `rayfronts/annotations/raw_annotations/DowntownWest.json`

## c36fa0e231 — 2025-09-05 — Seungchan (airstation-01) — no keyword

**adding oracle paths to abfac, mb, mcd, sv**

https://github.com/seungchan-kim/RayFronts/commit/c36fa0e23102d2f43e0d308d9cf4f84e9c980dee

+351 / −0 in 69 files:

- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y-15_z0.5_o_x0_y0_z90_building.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y-15_z0.5_o_x0_y0_z90_pipe, building.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y-15_z0.5_o_x0_y0_z90_pipe.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y-15_z0.5_o_x0_y0_z90_water tower, pipe.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y-15_z0.5_o_x0_y0_z90_water tower.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y-15_z0.5_o_x0_y0_z90_white silo, building.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y-15_z0.5_o_x0_y0_z90_white silo.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y35_z0.5_o_x0_y0_z-90_building.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y35_z0.5_o_x0_y0_z-90_pipe, building.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y35_z0.5_o_x0_y0_z-90_pipe.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y35_z0.5_o_x0_y0_z-90_water tower, pipe.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y35_z0.5_o_x0_y0_z-90_water tower.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y35_z0.5_o_x0_y0_z-90_white silo, building.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x-5_y35_z0.5_o_x0_y0_z-90_white silo.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x0_y0_z0.5_o_x0_y0_z0_building.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x0_y0_z0.5_o_x0_y0_z0_pipe, building.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x0_y0_z0.5_o_x0_y0_z0_pipe.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x0_y0_z0.5_o_x0_y0_z0_water tower, pipe.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x0_y0_z0.5_o_x0_y0_z0_water tower.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x0_y0_z0.5_o_x0_y0_z0_white silo, building.json`
- `rayfronts/annotations/oracle_paths/AbandonedFactory_t_x0_y0_z0.5_o_x0_y0_z0_white silo.json`
- `rayfronts/annotations/oracle_paths/MilitaryBase_t_x1070_y300_z0_o_x0_y0_z-90_ATV.json`
- `rayfronts/annotations/oracle_paths/MilitaryBase_t_x1100_y200_z0_o_x0_y0_z90_ATV.json`
- `rayfronts/annotations/oracle_paths/MilitaryBase_t_x1114_y28_z0_o_x0_y0_z90_ATV.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-17_y-69_z0.2_o_x0_y0_z0_bankomat, yellow truck.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-17_y-69_z0.2_o_x0_y0_z0_bankomat.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-17_y-69_z0.2_o_x0_y0_z0_cafe table, obelisk.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-17_y-69_z0.2_o_x0_y0_z0_cafe table.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-17_y-69_z0.2_o_x0_y0_z0_obelisk, yellow truck.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-17_y-69_z0.2_o_x0_y0_z0_obelisk.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-17_y-69_z0.2_o_x0_y0_z0_yellow truck.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-4_y20_z0.2_o_x0_y0_z-90_bankomat, yellow truck.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-4_y20_z0.2_o_x0_y0_z-90_bankomat.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-4_y20_z0.2_o_x0_y0_z-90_cafe table, obelisk.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-4_y20_z0.2_o_x0_y0_z-90_cafe table.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-4_y20_z0.2_o_x0_y0_z-90_obelisk, yellow truck.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-4_y20_z0.2_o_x0_y0_z-90_obelisk.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x-4_y20_z0.2_o_x0_y0_z-90_yellow truck.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x36_y-80_z0.2_o_x0_y0_z90_bankomat, yellow truck.json`
- `rayfronts/annotations/oracle_paths/ModernCityDowntown_t_x36_y-80_z0.2_o_x0_y0_z90_bankomat.json`
- … 29 more

## 4f2d2ecaf7 — 2025-09-05 — Seungchan (airstation-01) — KEYWORD fix

**snowy village annotations fix**

https://github.com/seungchan-kim/RayFronts/commit/4f2d2ecaf7776cb1d708c3f7070994ca1973dffd

+64 / −4 in 5 files:

- `rayfronts/annotations/raw_annotations/MilitaryBase.json`
- `rayfronts/annotations/raw_annotations/SnowyVillage.json`
- `rayfronts/annotations/transformed_annotations/SnowyVillage_t_x-145_y20_z-2.5_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/SnowyVillage_t_x-152_y-80_z-2_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/SnowyVillage_t_x-200_y-80_z-2_o_x0_y0_z0.json`

## 488c4117dc — 2025-09-06 — Seungchan (airstation-01) — no keyword

**vlfm manager**

https://github.com/seungchan-kim/RayFronts/commit/488c4117dcaaedc13fed583ae0da4978e84c6d91

+139 / −58 in 3 files:

- `rayfronts/behaviors/vlfm_behavior.py`
- `rayfronts/mapping_server_vlfm.py`
- `rayfronts/vlfm_manager.py`

## 18b1649e9a — 2025-09-06 — Seungchan (airstation-01) — KEYWORD fix

**snowyvillage car annot fix; downtownwest**

https://github.com/seungchan-kim/RayFronts/commit/18b1649e9a8a75c94913eca01831d53c5c22b2bf

+306 / −165 in 34 files:

- `rayfronts/annotations/oracle_paths/DowntownWest_t_x-120_y0_z0_o_x0_y0_z0_fire hydrant.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x-120_y0_z0_o_x0_y0_z0_food cart, trash bin.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x-120_y0_z0_o_x0_y0_z0_food cart.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x-120_y0_z0_o_x0_y0_z0_fountain, food cart.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x-120_y0_z0_o_x0_y0_z0_fountain, trash bin.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x-120_y0_z0_o_x0_y0_z0_fountain.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x-120_y0_z0_o_x0_y0_z0_trash bin.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x0_y0_z0_o_x0_y0_z0_fire hydrant.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x0_y0_z0_o_x0_y0_z0_food cart, trash bin.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x0_y0_z0_o_x0_y0_z0_food cart.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x0_y0_z0_o_x0_y0_z0_fountain, food cart.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x0_y0_z0_o_x0_y0_z0_fountain, trash bin.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x0_y0_z0_o_x0_y0_z0_fountain.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x0_y0_z0_o_x0_y0_z0_trash bin.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x2_y-60_z0_o_x0_y0_z90_fire hydrant.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x2_y-60_z0_o_x0_y0_z90_food cart, trash bin.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x2_y-60_z0_o_x0_y0_z90_food cart.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x2_y-60_z0_o_x0_y0_z90_fountain, food cart.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x2_y-60_z0_o_x0_y0_z90_fountain, trash bin.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x2_y-60_z0_o_x0_y0_z90_fountain.json`
- `rayfronts/annotations/oracle_paths/DowntownWest_t_x2_y-60_z0_o_x0_y0_z90_trash bin.json`
- `rayfronts/annotations/oracle_paths/SnowyVillage_t_x-145_y20_z-2.5_o_x0_y0_z-90_car.json`
- `rayfronts/annotations/oracle_paths/SnowyVillage_t_x-145_y20_z-2.5_o_x0_y0_z-90_water tower, car.json`
- `rayfronts/annotations/oracle_paths/SnowyVillage_t_x-152_y-80_z-2_o_x0_y0_z90_car.json`
- `rayfronts/annotations/oracle_paths/SnowyVillage_t_x-152_y-80_z-2_o_x0_y0_z90_water tower, car.json`
- `rayfronts/annotations/oracle_paths/SnowyVillage_t_x-200_y-80_z-2_o_x0_y0_z0_car.json`
- `rayfronts/annotations/oracle_paths/SnowyVillage_t_x-200_y-80_z-2_o_x0_y0_z0_water tower, car.json`
- `rayfronts/annotations/raw_annotations/SnowyVillage.json`
- `rayfronts/annotations/transformed_annotations/DowntownWest_t_x-120_y0_z0_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/DowntownWest_t_x0_y0_z0_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/DowntownWest_t_x2_y-60_z0_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/SnowyVillage_t_x-145_y20_z-2.5_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/SnowyVillage_t_x-152_y-80_z-2_o_x0_y0_z90.json`
- `rayfronts/annotations/transformed_annotations/SnowyVillage_t_x-200_y-80_z-2_o_x0_y0_z0.json`

## e82ab72435 — 2025-09-07 — Seungchan (airstation-01) — no keyword

**added shipyard env annotations**

https://github.com/seungchan-kim/RayFronts/commit/e82ab72435ee1deaf4bcbd8fe5277d3598d5599e

+1202 / −0 in 25 files:

- `rayfronts/annotations/oracle_paths/Shipyard_t_x0_y-100_z-0.5_o_x0_y0_z0_crane.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x0_y-100_z-0.5_o_x0_y0_z0_ship construction, crane.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x0_y-100_z-0.5_o_x0_y0_z0_ship construction.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x0_y-100_z-0.5_o_x0_y0_z0_ship, crane.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x0_y-100_z-0.5_o_x0_y0_z0_ship, white silo.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x0_y-100_z-0.5_o_x0_y0_z0_ship.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x0_y-100_z-0.5_o_x0_y0_z0_white silo.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x55_y50_z-0.5_o_x0_y0_z-90_crane.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x55_y50_z-0.5_o_x0_y0_z-90_ship construction, crane.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x55_y50_z-0.5_o_x0_y0_z-90_ship construction.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x55_y50_z-0.5_o_x0_y0_z-90_ship, crane.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x55_y50_z-0.5_o_x0_y0_z-90_ship, white silo.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x55_y50_z-0.5_o_x0_y0_z-90_ship.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x55_y50_z-0.5_o_x0_y0_z-90_white silo.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x70_y-100_z-0.5_o_x0_y0_z90_crane.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x70_y-100_z-0.5_o_x0_y0_z90_ship construction, crane.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x70_y-100_z-0.5_o_x0_y0_z90_ship construction.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x70_y-100_z-0.5_o_x0_y0_z90_ship, crane.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x70_y-100_z-0.5_o_x0_y0_z90_ship, white silo.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x70_y-100_z-0.5_o_x0_y0_z90_ship.json`
- `rayfronts/annotations/oracle_paths/Shipyard_t_x70_y-100_z-0.5_o_x0_y0_z90_white silo.json`
- `rayfronts/annotations/raw_annotations/Shipyard.json`
- `rayfronts/annotations/transformed_annotations/Shipyard_t_x0_y-100_z-0.5_o_x0_y0_z0.json`
- `rayfronts/annotations/transformed_annotations/Shipyard_t_x55_y50_z-0.5_o_x0_y0_z-90.json`
- `rayfronts/annotations/transformed_annotations/Shipyard_t_x70_y-100_z-0.5_o_x0_y0_z90.json`

## 130679b194 — 2026-03-02 — Seungchan (airstation-01) — no keyword

**re-initialize raven:rayfronts**

https://github.com/seungchan-kim/RayFronts/commit/130679b1940e8a3822efcb31c11160efa4464143

+5 / −5 in 3 files:

- `README.md`
- `run_docker.sh`
- `run_mapping_server.sh => run_mapping_server_rosnode.sh`

## 81802708ba — 2026-03-02 — Seungchan (airstation-01) — no keyword

**modify readme for raven rayfronts**

https://github.com/seungchan-kim/RayFronts/commit/81802708bad75e27d6b4e89b1f9624d1f1620959

+10 / −130 in 1 files:

- `README.md`

## d16861e28c — 2026-03-03 — Seungchan (airstation-01) — no keyword

**remove rf title in readme.md**

https://github.com/seungchan-kim/RayFronts/commit/d16861e28ceea1367a7109dd9ccb1017fa1e381e

+0 / −27 in 1 files:

- `README.md`

## 58c465c279 — 2026-04-27 — Seungchan (airstation-01) — no keyword

**updated desktop.dockerfile for 5070,80,90 series**

https://github.com/seungchan-kim/RayFronts/commit/58c465c279fd2c1809438b9b0175fcb8d58d0e65

+9 / −7 in 1 files:

- `docker/desktop.Dockerfile`

## 039f18ce3a — 2026-04-27 — Seungchan (airstation-01) — no keyword

**modified depth topic for isaac-sim config**

https://github.com/seungchan-kim/RayFronts/commit/039f18ce3acc9e012d249ec497a57b2c9cfef4ef

+1 / −1 in 1 files:

- `rayfronts/configs/dataset/ros2isaacsim.yaml`

## cd5f9abfc7 — 2026-04-27 — Seungchan (airstation-01) — no keyword

**added scikit-learn in dockerfile**

https://github.com/seungchan-kim/RayFronts/commit/cd5f9abfc773ed12e3be0697434cb1c25af7820c

+2 / −1 in 1 files:

- `docker/desktop.Dockerfile`

## f6f030b289 — 2026-04-29 — Seungchan (airstation-01) — no keyword

**removed pitch offset camera; radio large->base; tune the parameters for rayfronts**

https://github.com/seungchan-kim/RayFronts/commit/f6f030b28920e26182b850b26be13af5d2862fe6

+15 / −12 in 3 files:

- `rayfronts/configs/encoder/naradio.yaml`
- `rayfronts/datasets/ros.py`
- `run_mapping_server_rosnode.sh`

## 27597ae39a — 2026-04-29 — Seungchan (airstation-01) — no keyword

**rayfronts container name consistently**

https://github.com/seungchan-kim/RayFronts/commit/27597ae39a89ecdf9864655b6f71c6528145cd7f

+2 / −1 in 1 files:

- `run_docker.sh`

## 30a8f42d7e — 2026-04-30 — Seungchan (airstation-01) — no keyword

**rayfronts default image [448,448]**

https://github.com/seungchan-kim/RayFronts/commit/30a8f42d7eb2182be6c3cd5a7c96726d2d6ca45b

+4 / −5 in 3 files:

- `background.txt`
- `rayfronts/behaviors/voxel_behavior.py`
- `run_mapping_server_rosnode.sh`

## 7aaad2849d — 2026-04-30 — Seungchan (airstation-01) — no keyword

**input_text -> prompt; test.txt -> background.txt; behavior_manager clean**

https://github.com/seungchan-kim/RayFronts/commit/7aaad2849d26c2f5f7391961e2540e8580c0bf46

+12 / −15 in 4 files:

- `background.txt`
- `input_prompt.py`
- `rayfronts/behavior_manager.py`
- `rayfronts/configs/default.yaml`

## b9318139be — 2026-05-04 — Seungchan (airstation-01) — no keyword

**removed comments**

https://github.com/seungchan-kim/RayFronts/commit/b9318139be8ab9c1fe6552c16f789a071f5d5c47

+1 / −71 in 4 files:

- `rayfronts/behaviors/frontier_behavior.py`
- `rayfronts/behaviors/lvlm_behavior.py`
- `rayfronts/behaviors/ray_behavior.py`
- `rayfronts/behaviors/voxel_behavior.py`

## cc98a9fd84 — 2026-05-05 — Seungchan (airstation-01) — no keyword

**xyz axes directions for annotations**

https://github.com/seungchan-kim/RayFronts/commit/cc98a9fd8463a2c959d6c4dbf8d5c36c801443fc

+6 / −6 in 1 files:

- `mission_checker.py`

## 9a8d7aca9e — 2026-05-12 — Seungchan (airstation-01) — no keyword

**added retroneighbor**

https://github.com/seungchan-kim/RayFronts/commit/9a8d7aca9ecc0ab1e95b5e74ae6b2d3f7c167d69

+2477 / −0 in 1 files:

- `rayfronts/annotations/raw_annotations/RetroNeighborhood.json`

## 1327728b48 — 2026-05-12 — Seungchan (airstation-01) — no keyword

**3d annotation box with strong edges and transparent color planes**

https://github.com/seungchan-kim/RayFronts/commit/1327728b480dd7c16ca01cfbec7630f90f0b0f53

+49 / −21 in 1 files:

- `annotation_viz.py`

## aede16bf22 — 2026-05-12 — Seungchan (airstation-01) — KEYWORD fix

**fix on launch_raven inputs**

https://github.com/seungchan-kim/RayFronts/commit/aede16bf22e4c3cbccf690acd61589eb97149f7a

+5 / −35 in 1 files:

- `annotation_viz.py`

## c6de5834ea — 2026-05-12 — Seungchan (airstation-01) — no keyword

**drone init x,y,z,qx,qy,qz,qw inputs**

https://github.com/seungchan-kim/RayFronts/commit/c6de5834ea872b21898304e6eed03759724a3aac

+15 / −8 in 1 files:

- `annotation_viz.py`

## 232f1532a9 — 2026-05-12 — Seungchan (airstation-01) — no keyword

**annotation_viz v1**

https://github.com/seungchan-kim/RayFronts/commit/232f1532a9b983272cb0d0ba65703fc98c479682

+154 / −0 in 1 files:

- `annotation_viz.py`

## 26270c5527 — 2026-05-18 — Seungchan (airstation-01) — no keyword

**removed transformed annotations, oracle paths; updated raw_annotations->annotations**

https://github.com/seungchan-kim/RayFronts/commit/26270c5527b3663fe38f77831c36d33f08f4f4fc

+197 / −20309 in 271 files:

- `annotation_viz.py`
- `rayfronts/annotations/{raw_annotations => }/AbandonedCity.json`
- `rayfronts/annotations/{raw_annotations => }/AbandonedFactory.json`
- `rayfronts/annotations/{raw_annotations => }/ConstructionSite.json`
- `rayfronts/annotations/{raw_annotations => }/DowntownWest.json`
- `rayfronts/annotations/{raw_annotations => }/FireAcademy.json`
- `rayfronts/annotations/{raw_annotations => }/MilitaryBase.json`
- `rayfronts/annotations/{raw_annotations => }/ModernCityDowntown.json`
- `rayfronts/annotations/{raw_annotations => }/RetroNeighborhood.json`
- `rayfronts/annotations/{raw_annotations => }/Shipyard.json`
- `rayfronts/annotations/{raw_annotations => }/SnowyVillage.json`
- `rayfronts/annotations/abandonedcity_annotate_origin.py`
- `rayfronts/annotations/abandonedfactory_annotate_origin.py`
- `rayfronts/annotations/constructionsite_annotate_origin.py`
- `rayfronts/annotations/downtownwest_annotate_origin.py`
- `rayfronts/annotations/fireacademy_annotate_origin.py`
- `rayfronts/annotations/militarybase_annotate_origin.py`
- `rayfronts/annotations/moderncitydowntown_annotate_origin.py`
- `rayfronts/annotations/neighborhood_annotate_origin.py`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_building.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_bus stop, yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_car, bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_yellow motorhome, car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y0_z0_o_x0_y0_z0_yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_building.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_bus stop, yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_car, bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_yellow motorhome, car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x0_y80_z0_o_x0_y0_z-90_yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_building.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_bus stop, yellow motorhome.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_car, bus stop.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_yellow motorhome, car.json`
- `rayfronts/annotations/oracle_paths/AbandonedCity_t_x5_y-60_z0_o_x0_y0_z90_yellow motorhome.json`
- … 231 more

## 624a4a8c6c — 2026-05-19 — Seungchan (airstation-01) — KEYWORD fix

**mode text visualizer fix; trajectory length only**

https://github.com/seungchan-kim/RayFronts/commit/624a4a8c6c6e9c46d35f1be416de42e8a0b77fe9

+23 / −215 in 3 files:

- `annotation_viz.py`
- `mission_checker.py`
- `rayfronts/mode_text_visualizer.py`

## 6a405e47fa — 2026-05-19 — Seungchan (airstation-01) — no keyword

**removed cars from AF annot**

https://github.com/seungchan-kim/RayFronts/commit/6a405e47fa629dcef12a9da635edc35f8bfdb0d0

+1 / −61 in 1 files:

- `rayfronts/annotations/AbandonedCity.json`

## abcc6f69b9 — 2026-05-22 — Seungchan (airstation-01) — KEYWORD fixed

**fixed yaw orientation for annotation visualization**

https://github.com/seungchan-kim/RayFronts/commit/abcc6f69b90eeb3c7570097721ffbebcbbeab53c

+10 / −20 in 1 files:

- `annotation_viz.py`

## 9cb58fd49d — 2026-05-22 — Seungchan (airstation-01) — no keyword

**fire academy new annotations**

https://github.com/seungchan-kim/RayFronts/commit/9cb58fd49dd95a8640aa98446d00b6f70c23ac08

+130 / −330 in 1 files:

- `rayfronts/annotations/FireAcademy.json`

## 8d838d79e4 — 2026-05-23 — Seungchan (airstation-01) — KEYWORD fix

**query label deletion/re-add problem fix**

https://github.com/seungchan-kim/RayFronts/commit/8d838d79e444a035b71865034e4c33020b981fc8

+4 / −0 in 1 files:

- `rayfronts/mapping_server_rosnode.py`
