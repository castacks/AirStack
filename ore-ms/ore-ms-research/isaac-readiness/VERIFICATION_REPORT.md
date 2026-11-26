# ORE-MS-ISAAC-READY-001-R1 verification report

**Disposition: NOT READY — REMEDIATION REQUIRED.** The repaired OSMO pod produced live camera and map data. The pristine Isaac image still imports NumPy 2.4.6, while the working container depends on a manual downgrade. A fresh workflow launch and controlled fixture verification remain open.

| Gate | Disposition | Evidence |
|---|---|---|
| V01 | PASS | Local and live remote Git matched; M11 subtree is unchanged from its pinned checkpoint. [Baseline](BASELINE_AND_FIX_AUDIT.md) |
| V02 | PASS | Both source fixes are on the remote branch; runtime downgrade separated from the local image, which imports 2.4.6. [Provenance](IMAGE_PROVENANCE.md) |
| V03 | CONDITIONAL | Workflow SHA-256 recorded, one GPU confirmed and three local image IDs captured. Outer digest, fresh scheduling and two registry digests unavailable. [Workflow](OSMO_WORKFLOW_AUDIT.md) |
| V04 | FAIL | Original autostart failed. Three manually repaired services ran, but the pristine Isaac image still has the incompatible dependency and two headless subprocesses failed. [Boot](BOOT_VERIFICATION.md) |
| V05 | CONDITIONAL | One GPU sustained the ambient scene, with 3,185 MiB sampled VRAM use and 4–7% GPU utilization. Startup and fixture peaks unmeasured. [GPU](GPU_FEASIBILITY.md) |
| V06 | NOT TESTED | The 480 × 300 camera transport ran, but no 33 × 25 controlled depth/geometry comparison was executed. [Geometry](CAMERA_GEOMETRY.md) |
| V07 | CONDITIONAL | Domain 1 saw 238 monotonic clock samples; domain 0 saw zero. Pause/reset and paired stamp behavior remain open. [Clock](CLOCK_AND_ROS_CONTRACT.md) |
| V08 | CONDITIONAL | ROS delivered RGB8, 32FC1 depth, K, pose, state and an 853-point map marker. Optical calibration and acquisition alignment unmeasured. [Data validation](FOXGLOVE_DATA_VALIDATION.md) |
| V09 | CONDITIONAL | Two HTTP 101 handshakes and live payloads. External 1011 timeout reported, steady external reconnect unverified. [Connectivity](FOXGLOVE_CONNECTIVITY.md) |
| V10 | CONDITIONAL | Screenshot shows a right camera frame; ROS and WebSocket data independently observed. Displayed time stayed at epoch, and matched visual geometry was untested. [Data validation](FOXGLOVE_DATA_VALIDATION.md) |
| V11 | FAIL | Same image and running container have different NumPy runtimes. No pinned runtime or stage replay was demonstrated. [Reproduction](REPRODUCIBILITY.md) |
| V12 | CONDITIONAL | Preserved build, render, namespace and timeout evidence. Clean cold-start regression and long-duration tests remain open. [Failures](FAILURE_TELEMETRY.md) |
| V13 | PASS | Frozen M11 unchanged. No multispectral, flight-control or later milestone work undertaken. [M11 criteria](ISAAC_ENTRY_CRITERIA.md) |
| V14 | FAIL | Reference freeze prepared for 2,475 rays. Image and fresh startup remediation plus runtime fixture checks are prerequisites. [Recommendation](READINESS_RECOMMENDATION.md) |
