# Real lighting and smoke: frontend availability characterization

16 November 2026. Completed populations and results below distinguish frontend
availability from correspondence truth and pose/trajectory accuracy. This is an
existing-baseline characterization, with no new method, training or rescue tuning.

Follow-up completed: [spatial/temporal support](deployment-spatial-support-findings-2026-11-16.md)
shows that the minimal-pose availability gains below do not establish a broad,
sustained geometry advantage. Preserve the original endpoints and consult the
follow-up before interpreting availability as persistent scene coverage.

## What this study can establish

The [lighting protocol](deployment-lighting-protocol-2026-11-16.md) tested all
three selected NGCLAB bright/changing/dim recordings, and the separate
[real-smoke protocol](deployment-smoke-protocol-2026-11-16.md) tests the full
SubT-MRS SmokeRoom export. Fixed half-second intervals span each recording.
SIFT and the released RGB/thermal learned matchers share a monocular geometry
solver. These are temporal frontend diagnostics, not full MAC-VO/MACThermal stereo
reproductions, sensor-fusion results or metric VO trajectories.

**The lighting accuracy endpoint did not qualify for interpretation.** Under the
author extractor's raw-mocap-as-camera contract, even bright RGB has large
translation-direction disagreement. Preserve all the primary results rather
than turning solver coverage into an accuracy claim. A reference-frame/calibration
contract problem is a plausible explanation, not a proven diagnosis. No camera
transform, clock offset or threshold was fitted to improve these outcomes.

The independently checkable secondary outcome is frontend availability: an
interval supplies at least eight cheirality inliers under the fixed essential
solver. This is evidence of usable geometric support, not proof that the tracks
or recovered pose are correct. A consistent hallucination can pass such checks.
The smoke study prospectively chooses reference-free availability because its
supplied integrated-odometry frame/provenance remains separately qualified.

## Completed availability results

Each entry is supported intervals / all declared intervals (percent). Missing
sensor input remains in the denominator; pose-reference gaps do not suppress
image matching. Support means ≥8 cheirality inliers under the frozen geometry.

| Condition | SIFT RGB | SIFT thermal | Learned RGB | Learned thermal |
|---|---:|---:|---:|---:|
| Bright | 88/136 (64.7%) | 65/136 (47.8%) | 108/136 (79.4%) | 113/136 (83.1%) |
| Changing light | 81/147 (55.1%) | 43/147 (29.3%) | 106/147 (72.1%) | 122/147 (83.0%) |
| Dim | 3/138 (2.2%) | 54/138 (39.1%) | 110/138 (79.7%) | 120/138 (87.0%) |
| Whole SmokeRoom recording | 353/838 (42.1%) | 249/838 (29.7%) | 500/838 (59.7%) | 494/838 (58.9%) |

Learned thermal adds 10.9 percentage points of geometric support in changing
light and 7.2 points in dim light over learned RGB. The much larger SIFT dim gap
(2.2% RGB versus 39.1% thermal) shrinks with learned matching, preserving the
importance of the stronger RGB baseline. Bright SIFT favors RGB (64.7% versus
47.8%); learned matching changes that ranking (79.4% versus 83.1%).

Across the whole SmokeRoom recording, learned RGB is 59.7% and thermal 58.9%.
The prospective final third is RGB 208/279 (74.6%) versus thermal 223/279 (79.9%):
thermal +5.4 points there. The first third is mostly stationary and must not be
interpreted as smoke-motion failure; all thirds remain included. Learned match
medians in the final third are RGB 2473 versus thermal 1585. Higher match count
does not automatically imply more pose-solver support.

In dim light, learned thermal supports 18 intervals without RGB support; RGB
supports 8 without thermal. SmokeRoom has 51 thermal-only and 57 RGB-only supported
intervals. Their hindsight union is 551/838 (65.8%), versus 500/838 RGB alone.
This is an availability ceiling, not an implemented fusion/selector improvement
or a pose-accuracy oracle. Model-specific false support remains possible.

Global contrast is not an established modality selector: SmokeRoom has no anchors
in the predeclared RGB p95–p05 <40 bin, despite visibly obscured sections in the
fixed-time contact sheet. Bright foregrounds can preserve global dynamic range
while distant structure disappears. Static motion also confounds geometry-support
versus contrast. All contrast bins and paired outcomes are saved; no favorable
window or new threshold was chosen from outcomes.

All-frame lighting medians: RGB p95–p05 contrast 235 bright, 153 changing and 30
dim; learned accepted-match medians remain about 3560, 3449 and 3255. Thermal
medians are about 2385, 2301 and 2061. This measured dim setting retains usable
RGB signal and is not a zero-lux experiment. Thermal normalization and camera
viewpoint are additional confounds; do not generalize the ranking to all darkness.

The evidence points to a frontend-dependent availability benefit. Next characterize
spatially distributed persistent tracks using these cached predictions, while
qualifying a camera-reference transform independently before full pose/trajectory
claims. No selector is trained or validated by these descriptions.

Figures and data are in the local evidence packet: `availability.png/.pdf`,
`fixed-time-examples.png`, `smoke-fixed-grid.png`, `availability-summary.json`,
`availability-relationships.json`, `availability-paired.json`, and
`smoke-fixed-thirds.json`. Figures are scientific plots from actual inputs,
not generated scene illustrations.

## Inputs, controls and limitations

All 13,517 NGCLAB RGB/thermal images and all 22,800 smoke images were decoded and
hashed before predictions. Three complete lighting bags total 23,790,789,628 bytes;
smoke ZIP is 9,248,440,073 bytes. Sources and SHA receipts are preserved locally.
No bulk data or checkpoints enter Git. The selected images are exposed development
inputs; training overlap and wider generalization are not established.

RGB uses native grayscale. Thermal mono16 uses each frame's min/max to produce
uint8, with the recipe frozen before inference. Thermal values and contrast after
that transform are not directly comparable to radiometric RGB or lux. Both SIFT
and learned matching use the same prepared per-modality images. Model weights
are modality-specific in the learned comparison; SIFT supplies a common frontend
control. Query policies also differ between SIFT and learned grids.

Native image sizes are 640×480 for lighting and smoke RGB, 640×512 for smoke
thermal. Raw pixels remain distorted; accepted correspondences are converted
using the published camera models before geometry. Smoke MEI/pinhole full-grid
round-trips are within 1.2e-12 pixels. These synthetic checks validate the adapter,
not the physical calibration or reference-frame binding.

Both learned checkpoints run in frozen FP32 deterministic inference with no TF32,
unchanged state dictionaries and bitwise repeat checks at native sizes. Runtime:
image `sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`,
Torch 2.7.1+cu128, OpenCV 5.0.0, NumPy 2.2.6, RTX PRO 5000 Blackwell. The existing
loader performs its documented legacy RGB-key migration; thermal loads without
migration. No threshold, exposure, checkpoint or model search followed results.

## Condition identity and static intervals

A 24-time evenly spaced contact sheet confirms visible obscuration during part
of SmokeRoom (for example around 292–310 seconds), with structure still visible
in thermal. It also shows a long stationary prefix, a moving person and clear
segments. The dataset name does not mean every image is smoky. No manual smoke
window was selected for scoring: report the full recording, the prospectively
fixed time thirds and the fixed RGB-contrast bins. Contrast can change because
of motion/lighting/scene as well as smoke. Raw camera viewpoints differ.

Stationary or nearly pure-rotation pairs can have many valid correspondences
while failing translation cheirality because there is insufficient parallax.
Therefore accepted-match availability and essential-pose support are reported
separately. Neither is a trajectory-tracking failure/recovery metric. Dynamic
foregrounds can also support incorrect dominant-motion estimates. In particular,
never describe the stationary prefix as failed smoke odometry merely because
`recoverPose` rejects translation.

## Why pose accuracy remains unresolved

NGCLAB mocap has gaps up to 3.69 seconds; no interpolation crosses a >50 ms bracket.
Before matching, common reference/motion-qualified sets were frozen at 121 bright,
122 changing-light and 127 dim anchors. All 421 lighting anchors still received
image matching, including intervals without pose reference. Missing reference is
not an algorithm tracking failure. Accuracy and all-declared lower bounds have
separate denominators.

The author extractor at commit `6d4bf30ac28d69e9e28f8f634185e0829f39ab9b` writes
raw `/vrpn_client_node/RigidBody/pose` directly as ground truth and documents the
RGB optical-center convention. Its code applies no marker-to-camera transform.
We followed that contract without fitting one. The thermal-to-RGB exported
rotation required an explicitly reported closest-SO(3) correction (Frobenius
0.000827); original bytes and translation are preserved. This small normalization
does not explain large bright RGB discrepancies.

For example, on 121 common bright intervals, learned RGB supplies 98 solver
outputs but zero pass joint ≤5° rotation / ≤15° directed translation. Conditional
median axis-aware rotation error is 4.71°, translation-direction error 123.64°.
Rotation-magnitude disagreement is much smaller (about 0.24° over all referenced,
supported bright learned-RGB intervals): consistent with an unresolved coordinate
binding, but also not enough to diagnose it. Do not choose a camera-axis flip or
fit hand-eye calibration from these predictions. The repository's public
[world-axis question](https://github.com/NGCLAB/multi-spectral-dataset/issues/12)
had no reply in the inspected snapshot. This is a bounded unresolved contract,
not proof that the dataset's ground truth is wrong.

Primary joint successes are retained exactly: bright SIFT RGB 1/121, thermal 0/121;
learned RGB 0/121, thermal 1/121. Changing light: SIFT 0/122 vs 1/122; learned 0/122
vs 3/122. Dim: SIFT 0/127 vs 0/127; learned 1/127 vs 2/127. These numbers do not
support a deployment pose-accuracy ranking while the reference convention is open.

## Evidence and recovery

Lighting evidence: ignored `results/deployment-characterization/`; smoke:
`results/deployment-smoke-frontend/`. Source runners import frozen primitives
rather than changing the synthetic study. Independent auditors recheck source/input
pins, associations, reference construction where applicable, geometry packets and
all score/count denominators. They do not certify physical truth or reproduce the
learned matcher independently. The prospective lighting observable plan and its
original joint-outcome summaries remain available even though the pose endpoint
is unqualified; availability relationships are explicitly secondary descriptions.

Qualification failures are preserved: missing deterministic cuBLAS environment,
a duplicate qualification-write attempt, a slow initial ZIP intake, and the OpenCV5
iterative-undistortion API change. These were fixed before the corresponding real
predictions, preserving thresholds and populations. No successful-prefix
substitution, author contact, Box upload or Git publication occurred.

Final counts: 36,317 decoded images; 5,036 locked predictions (1,684 lighting and
3,352 smoke); 28,613 lighting audit checks, 41,123 smoke checks and 322
independent derived-availability checks passed. The
small availability-analysis writer failure was preserved and corrected without
changing predictions. The prepared-input/result recovery packet is at
`/root/ore-ms-artifacts/deployment-characterization-2026-11-16/`, with member hashes
and readback verification. It contains prepared pixels and checkpoint/source pins;
large original bag/ZIP files and checkpoints remain separate local assets.
