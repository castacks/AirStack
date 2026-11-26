# Isaac Sim readiness assessment

**ISAAC SIM PILOT NOT YET READY.** Source-level assessment dated 2026-11-26. Runtime state is reported in [compute and telemetry](COMPUTE_AND_TELEMETRY.md). AirStack pins Isaac Sim 5.1.0, Ubuntu-compatible ROS 2 Jazzy and PX4 v1.16.1 for this container path. Pegasus also exposes independent launch and ROS 2 integration surfaces. AirStack also carries separate AirSim and simple simulator paths. The Isaac-facing surface is the Pegasus application with vehicle backend, physics step and optional ROS 2 bridge. The checks below use the repository’s pinned version. [AirStack Dockerfile](../../../simulation/isaac-sim/docker/Dockerfile.isaac-ros) · [NVIDIA 5.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) · [NVIDIA 5.1 ROS installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)

| Capability | Classification | Evidence and constraint |
|---|---|---|
| Version and Jazzy/PX4 build configuration | Supported and verified | Verified pinned Dockerfile and launch surfaces at source level. Image build and GPU startup remain untested. |
| RGB camera, depth and segmentation | Supported but unverified | 5.1 camera and ROS 2 publication APIs expose camera products. Intrinsics, pixel centers, depth convention and output transfer encoding require fixture checks. [NVIDIA camera API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.camera/docs/index.html) |
| Opaque geometry, motion and occlusion | Supported but unverified | USD transforms and render products can supply first-visible geometry. Ray distance and axial depth require explicit conversion. Render filtering and subpixel coverage affect edge labels. |
| Calibrated native 8–14 µm radiometry | Unknown pending inspection | No qualified native camera path found in the inspected AirStack 5.1 surfaces. |
| Planck, emissivity, reflected background, band response, thermal detector | Requires custom model | Use separate spectral model driven by geometry, material and environmental state with dimensional metadata. |
| Visual fog / volumetrics | Supported but unverified | RTX volumetric controls are renderer features. Comparable coefficients, energy behavior and exact 5.1 extension support remain unmeasured. [NVIDIA RTX volumetrics](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/rtx-renderer_common.html) |
| Quantitative RGB attenuation, aerosol scattering and LWIR gas/dust transport | Requires custom model | M11 numerical transfer references do not qualify the renderer's visual fog as a radiometric reference. |
| Stochastic environmental evolution and replay | Requires custom model | Explicit realization seeds, independent innovations, time advance order and snapshot replay are required. |
| Multispectral synchronization, ROS optical transforms and masks | Requires adapter | Bind stamps, intrinsics, extrinsics, masks and detector ranges to a declared pair sequence. [NVIDIA ROS camera transforms](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.html) |
| ROS 2 image publication and simulation clock | Supported but unverified | NVIDIA documents camera and clock graphs. AirStack per-robot namespace and QoS require end-to-end checks. [NVIDIA ROS cameras](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera.html) |
| Dataset export | Requires adapter | Replicator supplies native annotation writer surfaces. Thermal sidecars, label isolation, checksums and exact pair semantics need an ORE-MS writer. [NVIDIA Replicator](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_overview.html) |
| Driver, wall throughput and renderer determinism | Unknown pending inspection | GPU inventory exists. Pinned image digest, compatibility run, memory and latency measurements remain open. |

| Criterion | Disposition | Basis |
|---|---|---|
| IS01 | CONDITIONAL | Qualified numerical subsets with AC02/AC05 open and unresolved model-form limits. |
| IS02 | CONDITIONAL | Interface proposal specifies clock, frames and camera contracts. No integration test. |
| IS03 | PASS | Native geometry/rendering and custom spectral/environmental provenance are specified. |
| IS04 | PASS | LWIR, atmospheric, scattering and detector limits are explicit. |
| IS05 | CONDITIONAL | Source tag plus replay plan. Image digest, stage bundle and renderer replay remain open. |
| IS06 | NOT TESTED | Hardware inventory only. |
| IS07 | CONDITIONAL | Proposed minimal scene and measurable checks below. Freeze after allowable review. |
| IS08 | NOT TESTED | M11 authorization covers assessment. Pilot decision remains pending. |
