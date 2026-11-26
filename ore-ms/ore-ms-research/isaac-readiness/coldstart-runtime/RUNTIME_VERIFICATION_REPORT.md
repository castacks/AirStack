# Phase B runtime verification

**COLD START FAILED — STOP AND REPORT BLOCKERS**

The fresh OSMO task `airstack-dev-314 / workspace` reached SSH and Docker at the expected Phase A source revision. The registry rejected the robot image pull. Isaac and GCS were created and never started.

| Gate | Status | Evidence |
|---|---|---|
| B01 | PASS | Workflow and workspace task matched local OSMO metadata. Fresh AirStack clone and Docker initialization observed. |
| B02 | PASS | Research branch and checkout matched the Phase A SHA and live remote before delivery. |
| B03 | CONDITIONAL | Compose, the created Isaac container and local RepoDigests matched the qualified manifest. Outer workspace image identity and a running Isaac container were unavailable. |
| B04 | CONDITIONAL | Digest guard present and pulls observed. Original Compose invocation unavailable. |
| B05 | FAIL | Robot image pull failed. Isaac and GCS never started. Task stdout unavailable for the full unattended timeline. |
| B06 | NOT TESTED | Running Isaac and writer workload unavailable. |
| B07 | NOT TESTED | Sensor publication unavailable. |
| B08 | NOT TESTED | Robot runtime, MAVROS, odometry and map unavailable. |
| B09 | NOT TESTED | Robot-domain runtime clock and TF unavailable. |
| B10 | NOT TESTED | GCS-domain runtime clock and TF unavailable. |
| B11 | FAIL | Workspace-local TCP 8766 connection refused. |
| B12 | NOT TESTED | External Mac visualization unverified. |
| B13 | NOT TESTED | No healthy steady-load interval. Idle GPU and container state sampled. |
| B14 | PASS | Startup failure, gaps and blockers preserved in this package. |
| B15 | FAIL | Evidence committed locally. The workspace has no GitHub push credential and the GitHub integration denied the write. Remote branch remained at Phase A during verification. |

[Provenance](WORKFLOW_PROVENANCE.md) · [Startup timeline](COLD_START_TIMELINE.md) · [NumPy and sensors](ISAAC_NUMPY_SENSOR_VERIFICATION.md) · [ROS time and TF](ROS_TIME_TF_VERIFICATION.md) · [Foxglove](FOXGLOVE_CONNECTIVITY.md) · [Resources](RESOURCE_STABILITY.md) · [Blockers](REMAINING_BLOCKERS.md)
