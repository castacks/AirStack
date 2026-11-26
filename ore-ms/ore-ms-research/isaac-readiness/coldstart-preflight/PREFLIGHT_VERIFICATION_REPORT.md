# ORE-MS-OSMO-COLDSTART-001-R1 Phase A

**PREFLIGHT CONDITIONAL — USER ACTION REQUIRED.** The committed startup path is qualified offline. Validate the actual Mac-local workflow before user-controlled submission.

| Gate | Disposition | Evidence |
|---|---|---|
| P01 | PASS | Starting branch, HEAD and remote reviewed, together with prior readiness and remediation evidence. |
| P02 | PASS | No Mac-local file or checked-in workflow YAML was changed. Local rclone customization remains user-managed. |
| P03 | CONDITIONAL | Repository entrypoint applies `git clone --recursive --branch` for absent checkouts and the remote research ref was verified. Actual fresh clone and baked entrypoint untested. |
| P04 | PASS | Offline launch block, real CLI and Compose passed the intended image through each tested process boundary. |
| P05 | PASS | Compose resolved the exact qualified digest for livestream in the fixture. Fresh registry access untested. |
| P06 | PASS | Offline trace emitted `up --no-build -d`. A simulated Docker failure remained a launch failure with no fallback command. |
| P07 | PASS | Default robot and GCS image references were identical across all tested cases. |
| P08 | CONDITIONAL | Reference requests one OSMO GPU; offline Compose resolves one simulator GPU and one robot. Mac schema validation and actual scheduling remain open. |
| P09 | PASS | Exact validation, submission, workflow ID, forwarding and evidence instructions recorded. |
| P10 | PASS | No OSMO command or live Docker lifecycle operation was executed. |
| P11 | PASS | Report, source and offline evidence submitted on the research branch. The delivery records the verified remote commit. |

[Startup audit](STARTUP_PATH_AUDIT.md) · [Image verification](IMAGE_PROPAGATION_VERIFICATION.md) · [YAML review](WORKFLOW_YAML_REVIEW.md) · [Mac handoff](USER_DEPLOYMENT_HANDOFF.md) · [Remaining blockers](REMAINING_BLOCKERS.md)
