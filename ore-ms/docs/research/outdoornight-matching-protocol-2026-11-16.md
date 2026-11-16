# OutdoorNight reference-free matching protocol

16 November 2026, America/New_York; base a41fb5640 plus input-qualification extension.
Prospective plan before any matching on OutdoorNight. Reuses source/architecture
reading from the completed FlashLight/SmokeRoom studies; no fresh paper-body
reading or author-system reproduction is claimed.

## Question and fixed population

Does thermal supply additional essential/cheirality-supported pair proposals when
RGB has no supported proposal, and at what additional matching cost? The primary
endpoint is the number of added supported proposals from the unchanged RGB-first
rule over all971 declared half-second anchors. Report fixed RGB and thermal
controls, all failures, missing inputs, exact calls and projected matching time.
No reference-error, correct-motion, recovery-success or trajectory endpoint exists.
A greater output count can include wrong estimates; no sensor ranking is required.

Use the [completed whole-export qualification](outdoornight-qualification-findings-2026-11-16.md):
26,371 images,1,942 sensor intervals, initial thermal interval unsupported by the
unchanged25ms endpoint gate. Both SIFT and learned methods produce3,884 rows.
All rows and1,942 frontend/anchor choices remain, with no successful-prefix or
condition/error-selected trimming. Fixed thirds use floor(index*3/971), independent
of availability or appearance. These are time bins, not measured darkness labels.

## Unchanged mechanics

Original RGB `MACVO_FrontendCov.pth` and thermal `MACT_Offroad.pth`, no updates;
restore/check bytes against EXP112's member manifest. Restore exact compiler
image sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09
using EXP106 base and EXP112 delta, checking archive and every consumed member.
Use deterministic cuBLAS workspace`:4096:8`, no TF32, deterministic Torch.

Reuse `subt_export_comparison.py`, native grayscale conversion, own OutdoorNight
MEI RGB/pinhole thermal calibration and4,096-point query grid. Learned temporal
flow uses unchanged1px bidirectional-cycle gate. SIFT uses native-size images,
unchanged detector and mutual0.75ratio matching. Lift calibrated rays into the
virtual focal400 camera; RANSAC1px/0.999probability/1,000iterations, RNG0, minimum
eight cheirality inliers for status`ok`. No model, grid, threshold or geometry fit.

Request RGB first. Retain an`ok` RGB proposal; otherwise request thermal iff its
input is supported and retain it iff`ok`; otherwise abstain. Fixed thermal and
fixed RGB are controls. Status-only routing cannot use reference values. Retain
sensor, own source/target paths and timestamps. Conjugate rotations to IMU axes;
unit translation remains at its target camera origin, no metric lever-arm transport
or composed mixed-frame trajectory. RGB-first output count equals the status union
by construction; it does not establish intelligent quality selection or accuracy.

## Outputs, cost and stopping

Report every frontend/policy over all anchors and fixed thirds: output count,
availability fraction, longest consecutive no-output run (anchor count), matcher
calls and projected seconds. For each sensor report full status distribution,
match-count and cheirality-inlier quartiles, including failed supported calls.
For fallback report requested thermal calls, added outputs and failed attempts.
All accuracy quantities are explicitly unavailable, not zero-success estimates.

Calls count only qualified input pairs, including failed matching attempts. A
learned call includes both temporal directions. Projected lazy matching seconds
sum measured supported RGB calls plus requested supported thermal calls, with
CUDA synchronization at learned timing boundaries. SIFT feature-cache reuse differs
under a lazy schedule; timings are approximate projections, not robot latency,
energy, resident/startup memory or acquisition savings. Full fixed control arms
are actually computed for comparison; do not claim a live lazy execution run.

Before full execution verify all gray hashes, whole-input completion/audit and
source/calibration/model/protocol pins. Qualify both native image sizes with
bitwise repeated learned flows and unchanged states; retain camera roundtrip and
old SmokeRoom synthetic lifting equivalence tests and selector truth table.
Freeze runtime and source before full predictions. Lock all predictions, then
choices, before summaries. Preserve failed execution attempts without overwrite.

Independent auditor checks full population, prediction status and proposal
geometry, routing, sensor/frame/stamp bindings, every denominator/third/run/call/
cost, and unavailable accuracy semantics without importing runner calculations.
Use CPU regression fixtures and deliberate semantic corruptions. Save figures,
protocol, source/runtime/checkpoint pins, manifests and verified local packet.
Stop after this bounded comparison; no tuning or accuracy claim follows the result.
