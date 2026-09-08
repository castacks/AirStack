# Candidate commits

Every commit in the window whose message matches the fix-keyword regex (plus all others, --all-messages). Read the diff before classifying; the message alone is `low` confidence.

## 10ea40ee0b — 2025-08-15 — N47IN — no keyword

**Initial commit**

https://github.com/castacks/DontFoolMeTwice/commit/10ea40ee0bca352b80d8fb7a7648076a0695f397

+8336 / −0 in 39 files:

- `.gitignore`
- `CMakeLists.txt`
- `assets/adjusted_nominal_spline.json`
- `config/combined_segmentation_config.yaml`
- `launch/resilience.launch.py`
- `launch/resilience_with_segmentation.launch.py`
- `package.xml`
- `resilience/__init__.py`
- `resilience/__pycache__/__init__.cpython-310.pyc`
- `resilience/__pycache__/base.cpython-310.pyc`
- `resilience/__pycache__/drift_calculator.cpython-310.pyc`
- `resilience/__pycache__/historical_cause_analysis.cpython-310.pyc`
- `resilience/__pycache__/naradio.cpython-310.pyc`
- `resilience/__pycache__/naradio_processor.cpython-310.pyc`
- `resilience/__pycache__/narration_manager.cpython-310.pyc`
- `resilience/__pycache__/pointcloud_manager.cpython-310.pyc`
- `resilience/__pycache__/prompt_templates.cpython-310.pyc`
- `resilience/__pycache__/risk_buffer.cpython-310.pyc`
- `resilience/__pycache__/simple_descriptive_narration.cpython-310.pyc`
- `resilience/__pycache__/utils.cpython-310.pyc`
- `resilience/__pycache__/yolo_sam_detector.cpython-310.pyc`
- `resilience/base.py`
- `resilience/detection/__init__.py`
- `resilience/detection/yolo_sam_detector.py`
- `resilience/drift_calculator.py`
- `resilience/historical_cause_analysis.py`
- `resilience/naradio.py`
- `resilience/naradio_processor.py`
- `resilience/narration_manager.py`
- `resilience/pointcloud_manager.py`
- `resilience/prompt_templates.py`
- `resilience/risk_buffer.py`
- `resilience/simple_descriptive_narration.py`
- `resilience/utils.py`
- `resilience/yolo_sam_detector.py`
- `scripts/calibrate_drift.py`
- `scripts/main.py`
- `scripts/narration_display_node.py`
- `scripts/standalone_segmentation.py`

## a6b86967ad — 2025-08-18 — N47IN — no keyword

**Added refined naradio-heatmap, accurate cause embeddings**

https://github.com/castacks/DontFoolMeTwice/commit/a6b86967ad396f2db3be3ece8f0f0aff5acdd855

+2569 / −1852 in 16 files:

- `README.md`
- `launch/resilience.launch.py`
- `launch/resilience_with_segmentation.launch.py`
- `resilience/__init__.py`
- `resilience/__pycache__/__init__.cpython-313.pyc`
- `resilience/__pycache__/naradio.cpython-310.pyc`
- `resilience/__pycache__/naradio_processor.cpython-313.pyc`
- `resilience/__pycache__/simple_descriptive_narration.cpython-313.pyc`
- `resilience/historical_cause_analysis.py`
- `resilience/naradio.py`
- `resilience/naradio_processor.py`
- `resilience/risk_buffer.py`
- `resilience/yolo_sam_detector.py`
- `scripts/__pycache__/main.cpython-313.pyc`
- `scripts/main.py`
- `scripts/standalone_segmentation.py`

## bbd42fbf59 — 2025-08-21 — N47IN — no keyword

**Semantic voxel mapping wip**

https://github.com/castacks/DontFoolMeTwice/commit/bbd42fbf591a7e3bf5bc3f9d11655b1c903f8f0b

+1392 / −419 in 9 files:

- `config/combined_segmentation_config.yaml`
- `launch/semantic_mapping_complete.launch.py`
- `launch/semantic_octomap.launch.py`
- `resilience/__init__.py`
- `resilience/__pycache__/semantic_info_bridge.cpython-310.pyc`
- `resilience/semantic_info_bridge.py`
- `resilience/semantic_voxel_mapper.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## 47d8e2f521 — 2025-08-21 — N47IN — no keyword

**Naradio improvements + Octomap Integration**

https://github.com/castacks/DontFoolMeTwice/commit/47d8e2f521a85aa7f30e80428970fcd3fab5a7bb

+1037 / −53 in 8 files:

- `CMakeLists.txt`
- `config/combined_segmentation_config.yaml`
- `config/octomap_config.yaml`
- `launch/octomap_voxel_mapper.launch.py`
- `resilience/__pycache__/octomap_manager.cpython-310.pyc`
- `resilience/naradio_processor.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## acb08de6c3 — 2025-08-22 — N47IN — KEYWORD fix

**Voxel Mapping works, need to fix drift**

https://github.com/castacks/DontFoolMeTwice/commit/acb08de6c36cbf49c444e86bd6a1d21b330d5633

+525 / −139 in 4 files:

- `scripts/__pycache__/depth_octomap_node.cpython-310.pyc`
- `scripts/__pycache__/main.cpython-310.pyc`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## 1316a11539 — 2025-08-22 — N47IN — no keyword

**Semantic voxel mapping wip, hotspots working**

https://github.com/castacks/DontFoolMeTwice/commit/1316a1153991ea683a6662af8569752d10bc4e70

+1149 / −150 in 18 files:

- `resilience/__init__.py`
- `resilience/__pycache__/__init__.cpython-310.pyc`
- `resilience/__pycache__/drift_calculator.cpython-310.pyc`
- `resilience/__pycache__/historical_cause_analysis.cpython-310.pyc`
- `resilience/__pycache__/naradio_processor.cpython-310.pyc`
- `resilience/__pycache__/narration_manager.cpython-310.pyc`
- `resilience/__pycache__/risk_buffer.cpython-310.pyc`
- `resilience/__pycache__/semantic_hotspot_helper.cpython-310.pyc`
- `resilience/__pycache__/semantic_info_bridge.cpython-310.pyc`
- `resilience/__pycache__/semantic_voxel_mapper.cpython-310.pyc`
- `resilience/__pycache__/voxel_mapping_helper.cpython-310.pyc`
- `resilience/__pycache__/yolo_sam_detector.cpython-310.pyc`
- `resilience/naradio_processor.py`
- `resilience/semantic_hotspot_helper.py`
- `resilience/semantic_info_bridge.py`
- `resilience/voxel_mapping_helper.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## 5bbd2c56ad — 2025-08-23 — N47IN — KEYWORD fix

**Drift fix, will clean and optimise**

https://github.com/castacks/DontFoolMeTwice/commit/5bbd2c56adb7fb5b5398f4a23798061934222a53

+618 / −99 in 5 files:

- `config/combined_segmentation_config.yaml`
- `resilience/semantic_info_bridge.py`
- `scripts/__pycache__/depth_octomap_node.cpython-310.pyc`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## 4a7eb9a51d — 2025-08-26 — N47IN — no keyword

**Perfected Voxel Mapping**

https://github.com/castacks/DontFoolMeTwice/commit/4a7eb9a51d0e3f522d824ea995a81a78dc764c5f

+280 / −877 in 6 files:

- `resilience/__pycache__/base.cpython-310.pyc`
- `resilience/__pycache__/naradio.cpython-310.pyc`
- `resilience/__pycache__/prompt_templates.cpython-310.pyc`
- `resilience/semantic_info_bridge.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## 1bf07f4764 — 2025-09-02 — N47IN — no keyword

**Voxel centric GP analysis**

https://github.com/castacks/DontFoolMeTwice/commit/1bf07f4764d0553cfdcedfd21f25072bf5913c6c

+885 / −0 in 1 files:

- `scripts/voxel_gp.py`

## 4f7d284b78 — 2025-09-02 — N47IN — no keyword

**3D GP**

https://github.com/castacks/DontFoolMeTwice/commit/4f7d284b78bb68e8e6295cddf8b3294da6ab5e0c

+602 / −0 in 1 files:

- `scripts/postprocess_3dgp.py`

## 310fd867d9 — 2025-09-02 — N47IN — KEYWORD fixes

**mapping fixes and GP**

https://github.com/castacks/DontFoolMeTwice/commit/310fd867d9aabad20da23aace404adb8796960dd

+597 / −17 in 3 files:

- `resilience/narration_manager.py`
- `scripts/depth_octomap_node.py`
- `scripts/postprocess_2dgp.py`

## 1179104418 — 2025-09-03 — N47IN — no keyword

**Async per-voxel GP fitting**

https://github.com/castacks/DontFoolMeTwice/commit/1179104418416083d08a53a8aba42b73864532c4

+894 / −239 in 3 files:

- `resilience/__pycache__/gp_fit_utility.cpython-310.pyc`
- `resilience/gp_fit_utility.py`
- `scripts/voxel_gp.py`

## 85d157cc96 — 2025-09-03 — N47IN — no keyword

**true resilience, HA eliminated**

https://github.com/castacks/DontFoolMeTwice/commit/85d157cc964d21efe5ae76524e5948273fa57b4e

+90 / −635 in 1 files:

- `scripts/main.py`

## 4975305145 — 2025-09-03 — N47IN — no keyword

**true resilience, HA eliminated**

https://github.com/castacks/DontFoolMeTwice/commit/49753051450e32f7be16d98a3ca15ad6a5d69dbe

+340 / −91 in 5 files:

- `resilience/naradio_processor.py`
- `resilience/risk_buffer.py`
- `resilience/semantic_info_bridge.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## 68268bac3c — 2025-09-03 — N47IN — no keyword

**multi cause handling**

https://github.com/castacks/DontFoolMeTwice/commit/68268bac3ce9b8896a5a1ea07fbdbdf0753f0824

+719 / −135 in 7 files:

- `resilience/__pycache__/naradio_processor.cpython-313.pyc`
- `resilience/__pycache__/semantic_info_bridge.cpython-313.pyc`
- `resilience/naradio_processor.py`
- `resilience/semantic_info_bridge.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`
- `scripts/voxel_gp.py`

## c1f3acb9f1 — 2025-09-04 — N47IN — no keyword

**external path handling enabled**

https://github.com/castacks/DontFoolMeTwice/commit/c1f3acb9f1f5fe9809053c26bad264ec0e4d5026

+227 / −469 in 1 files:

- `README.md`

## 84f3743adb — 2025-09-04 — N47IN — no keyword

**external path handling enabled**

https://github.com/castacks/DontFoolMeTwice/commit/84f3743adb7d71def9f961a0798f0b3016ad16a5

+55 / −59 in 4 files:

- `CMakeLists.txt`
- `config/main_config.yaml`
- `resilience/path_manager.py`
- `scripts/main.py`

## 6d188040cf — 2025-09-04 — N47IN — no keyword

**updated configs and path management**

https://github.com/castacks/DontFoolMeTwice/commit/6d188040cf55134cdfc1ab49993264e22bea89db

+1173 / −331 in 16 files:

- `CMakeLists.txt`
- `README.md`
- `config/combined_segmentation_config.yaml`
- `config/main_config.yaml`
- `config/mapping_config.yaml`
- `config/octomap_config.yaml`
- `launch/resilience_with_segmentation.launch.py`
- `launch/semantic_mapping_complete.launch.py`
- `resilience/__init__.py`
- `resilience/naradio_processor.py`
- `resilience/narration_manager.py`
- `resilience/path_manager.py`
- `resilience/semantic_info_bridge.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`
- `scripts/publish_plan.py`

## cd3528066c — 2025-09-08 — N47IN — KEYWORD patch

**real-time GP patch, cause pcd**

https://github.com/castacks/DontFoolMeTwice/commit/cd3528066ca7e935002a9e59308b0cec115818d5

+60 / −137 in 3 files:

- `resilience/semantic_info_bridge.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## ec2d692496 — 2025-09-08 — N47IN — no keyword

**clean-up**

https://github.com/castacks/DontFoolMeTwice/commit/ec2d6924969d1ace17ca51e3e4d250dd5abae975

+7 / −249 in 2 files:

- `resilience/semantic_info_bridge.py`
- `scripts/main.py`

## 414480eaf4 — 2025-09-10 — N47IN — no keyword

**proper path handling + FLU convention**

https://github.com/castacks/DontFoolMeTwice/commit/414480eaf47ccea21f64617a9418e8313bc2a137

+1044 / −50 in 6 files:

- `config/main_config.yaml`
- `resilience/narration_manager.py`
- `resilience/path_manager.py`
- `scripts/main.py`
- `scripts/postprocess_3dgp.py`
- `scripts/trajectory_analysis.py`

## ceb22e27aa — 2025-09-10 — N47IN — no keyword

**mature GP fit**

https://github.com/castacks/DontFoolMeTwice/commit/ceb22e27aaa21bf211f45d34acdb8d87fa237a1a

+394 / −0 in 1 files:

- `scripts/depth_octomap_node.py`

## 0e284952f2 — 2025-09-10 — N47IN — no keyword

**mature GP fit**

https://github.com/castacks/DontFoolMeTwice/commit/0e284952f22da19ed7cf4ec83f0ab8710c252f86

+127 / −9 in 4 files:

- `resilience/risk_buffer.py`
- `resilience/semantic_info_bridge.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## 5f65b08fe1 — 2025-09-10 — N47IN — no keyword

**realtime GP, partial**

https://github.com/castacks/DontFoolMeTwice/commit/5f65b08fe146b8d945d18c83ffb1bbd8fc4c2e26

+1132 / −57 in 5 files:

- `resilience/__init__.py`
- `resilience/semantic_info_bridge.py`
- `resilience/voxel_gp_helper.py`
- `scripts/analyse_gp.py`
- `scripts/depth_octomap_node.py`

## 4035af6ed1 — 2025-09-11 — N47IN — no keyword

**evolving costmaps beta**

https://github.com/castacks/DontFoolMeTwice/commit/4035af6ed11ead3ac17859194366ae65ed96ae62

+1023 / −110 in 20 files:

- `config/main_config.yaml`
- `resilience/__pycache__/__init__.cpython-310.pyc`
- `resilience/__pycache__/drift_calculator.cpython-310.pyc`
- `resilience/__pycache__/gp_superposition_manager.cpython-310.pyc`
- `resilience/__pycache__/historical_cause_analysis.cpython-310.pyc`
- `resilience/__pycache__/naradio_processor.cpython-310.pyc`
- `resilience/__pycache__/narration_manager.cpython-310.pyc`
- `resilience/__pycache__/path_manager.cpython-310.pyc`
- `resilience/__pycache__/pointcloud_manager.cpython-310.pyc`
- `resilience/__pycache__/risk_buffer.cpython-310.pyc`
- `resilience/__pycache__/semantic_hotspot_helper.cpython-310.pyc`
- `resilience/__pycache__/semantic_info_bridge.cpython-310.pyc`
- `resilience/__pycache__/semantic_voxel_mapper.cpython-310.pyc`
- `resilience/__pycache__/voxel_gp_helper.cpython-310.pyc`
- `resilience/__pycache__/voxel_mapping_helper.cpython-310.pyc`
- `resilience/__pycache__/yolo_sam_detector.cpython-310.pyc`
- `resilience/gp_evolution_visualizer.py`
- `resilience/gp_superposition_manager.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## a5ffb2c24b — 2025-09-11 — N47IN — no keyword

**standalone resilience without mapping**

https://github.com/castacks/DontFoolMeTwice/commit/a5ffb2c24b26227785074fafdf042ccbc553c296

+313 / −137 in 5 files:

- `resilience/__pycache__/simple_descriptive_narration.cpython-310.pyc`
- `resilience/narration_manager.py`
- `resilience/pointcloud_utils.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`

## d149d25776 — 2025-09-11 — N47IN — no keyword

**FLU mature?**

https://github.com/castacks/DontFoolMeTwice/commit/d149d257765190e9807c34b0c72816d88aea8303

+461 / −0 in 1 files:

- `scripts/traj_comparison.py`

## 85ceb9a682 — 2025-09-12 — N47IN — no keyword

**costmap mature**

https://github.com/castacks/DontFoolMeTwice/commit/85ceb9a68217931c0f001cd82787dbcdaab096d2

+529 / −948 in 3 files:

- `resilience/gp_evolution_visualizer.py`
- `resilience/gp_superposition_manager.py`
- `scripts/depth_octomap_node.py`

## e1a99f3279 — 2025-09-14 — Navin Sriram — no keyword

**Update README.md**

https://github.com/castacks/DontFoolMeTwice/commit/e1a99f32799a11d52f73d8526fc197f7c80a91b4

+16 / −14 in 1 files:

- `README.md`

## 504e117db2 — 2025-09-14 — Navin Sriram — no keyword

**Update README.md**

https://github.com/castacks/DontFoolMeTwice/commit/504e117db28c8ebeb6433d3088ff46e094c47d09

+4 / −2 in 1 files:

- `README.md`

## cf0197ebf0 — 2025-09-14 — N47IN — no keyword

**TARo code**

https://github.com/castacks/DontFoolMeTwice/commit/cf0197ebf022cb0d1fa29f7d8fee11936bcc5f81

+580 / −1045 in 9 files:

- `README.md`
- `launch/octomap_voxel_mapper.launch.py`
- `launch/resilience.launch.py`
- `launch/resilience_with_segmentation.launch.py`
- `launch/semantic_mapping_complete.launch.py`
- `launch/semantic_octomap.launch.py`
- `scripts/depth_octomap_node.py`
- `scripts/main.py`
- `scripts/narration_display_node.py`

## 9a580c0892 — 2026-01-03 — Andrew Jong — KEYWORD error,fix

**Bump cmake version to fix min version error; also add ros2 build folders to gitignore**

https://github.com/castacks/DontFoolMeTwice/commit/9a580c08927d4fb681bf7a4b32a94c0cd8f5cd71

+6 / −2 in 2 files:

- `.gitignore`
- `CMakeLists.txt`

## 4ec60f84a2 — 2026-01-12 — N47IN — no keyword

**merge all changes**

https://github.com/castacks/DontFoolMeTwice/commit/4ec60f84a2416b17ff45e0aef4c613355c49cc62

+2 / −3 in 1 files:

- `.gitignore`

## 1ec9c0044d — 2026-01-12 — N47IN — no keyword

**merge all changes**

https://github.com/castacks/DontFoolMeTwice/commit/1ec9c0044d2c0c6d5104f1c3ce4ed37c85a51454

+23835 / −10443 in 203 files:

- `CMakeLists.txt`
- `RayFronts/.gitignore`
- `RayFronts/.gitmodules`
- `RayFronts/CONTRIBUTING.md`
- `RayFronts/LICENSE`
- `RayFronts/README.md`
- `RayFronts/assets/abstract_fig.jpg`
- `RayFronts/assets/example1.jpg`
- `RayFronts/assets/example2.jpg`
- `RayFronts/assets/example3.jpg`
- `RayFronts/assets/logo.gif`
- `RayFronts/assets/method_teaser.gif`
- `RayFronts/compile.sh`
- `RayFronts/docker/desktop.Dockerfile`
- `RayFronts/docker/jetson.Dockerfile`
- `RayFronts/environment.yml`
- `RayFronts/experiments/README.md`
- `RayFronts/experiments/patches/concept_graphs.patch`
- `RayFronts/experiments/patches/hovsg.patch`
- `RayFronts/experiments/semseg_configs/replica_conceptfusion.yaml`
- `RayFronts/experiments/semseg_configs/replica_concpgr.yaml`
- `RayFronts/experiments/semseg_configs/replica_naclip.yaml`
- `RayFronts/experiments/semseg_configs/replica_naradio.yaml`
- `RayFronts/experiments/semseg_configs/replica_trident.yaml`
- `RayFronts/experiments/semseg_configs/scannet_conceptfusion.yaml`
- `RayFronts/experiments/semseg_configs/scannet_concpgr.yaml`
- `RayFronts/experiments/semseg_configs/scannet_naclip.yaml`
- `RayFronts/experiments/semseg_configs/scannet_naradio.yaml`
- `RayFronts/experiments/semseg_configs/scannet_trident.yaml`
- `RayFronts/experiments/semseg_configs/tartanair_conceptfusion.yaml`
- `RayFronts/experiments/semseg_configs/tartanair_naclip.yaml`
- `RayFronts/experiments/semseg_configs/tartanair_naradio.yaml`
- `RayFronts/experiments/semseg_configs/tartanair_trident.yaml`
- `RayFronts/experiments/srchvol_configs/base.yaml`
- `RayFronts/experiments/srchvol_configs/rayfronts_0.yaml`
- `RayFronts/experiments/srchvol_configs/rayfronts_10.yaml`
- `RayFronts/experiments/srchvol_configs/rayfronts_20.yaml`
- `RayFronts/experiments/srchvol_configs/sempose_0.yaml`
- `RayFronts/experiments/srchvol_configs/spherical_semfronts_10.yaml`
- `RayFronts/experiments/srchvol_configs/spherical_semfronts_20.yaml`
- … 163 more
