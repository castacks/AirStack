# Controlled continuity sequence: missing motion versus measured reattachment

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus working sources.
One frozen execution complete; no model training or method tuning.

## Result

All72planned pair fits converge. All four post-gap poses are available in each
arm of each of six artificial sequences. Retained-keyframe cross-gap tracking
and a visual measurement to an old keyframe keep errors small in this controlled
overlap setting. A restart preserves a useful new local trajectory, but simply
attaching it to the last pre-gap pose misses the intervening motion.

This demonstrates the difference between resumed local output and recovery of
an old coordinate frame. It is not a new recovery algorithm, physical thermal
accuracy result, full native VO run or ORB-SLAM3 reproduction.

## Frozen setup and controls

Six sequences reuse the three already-exposed day/night/rain thermal textures
and plane4m/two-depth4/8m geometry. Ten stereo frames each,120views. Each frame
has optical Y rotation0.15deg times frame index and center[.01,0,.03]m times index.
Frames3–5 are withheld; first return6. Last pre-gap frame2 to return6 has exact
center displacement126.491mm and rotation0.6deg. No sensor-noise or physical-time
model is imposed. This is a small-motion, overlapping-view assay.

Released MACT checkpoint strict-loaded, unchanged native selector/covariance
bodies, same existing optical frozen-weight IRLS pair harness, same identity
pair initialization and200query cap. Covariance head, means, solver, anchor,
geometry, selection seed and interruption remain fixed. This harness composes
pair estimates explicitly; it does not execute the complete historical native
state/reset/motion-initializer/postprocessor machinery.

| Arm | Information used | Pose frame |
|---|---|---|
| Adjacent uninterrupted | All adjacent pairs0→1 through8→9 | Original world |
| Retained across gap |0→1→2→6→7→8→9, using real pre/post-gap image matching | Original world |
| Local restart |Identity at6, then6→7→8→9 | New, disconnected local frame |
| Hold-pose splice |Estimated old pose2 multiplied by restarted local poses; implicitly zero gap motion | Original world, diagnostic assumption |
| Old-anchor reattachment |Visual0→6 estimate, then6→7→8→9 | Original world |

The old anchor is explicitly provided: no place-recognition retrieval, wrong-place
negative assay or calibrated recovery acceptance rule is evaluated. Interrupted
arms never use estimates whose endpoints are withheld frames. Post-gap adjacent
estimates are shared between arms; equal measurements are reused exactly.

## Original-frame accuracy

Translation RMSE in millimetres over all four fixed post-gap frames6–9:

| Sequence | Adjacent uninterrupted | Retained across gap | Hold-pose splice | Old-anchor reattachment |
|---|---:|---:|---:|---:|
| day-plane4m | 16.259 | 9.163 | 125.455 | 4.769 |
| day-two-depths | 12.906 | 5.137 | 128.876 | 2.816 |
| night-plane4m | 30.198 | 15.246 | 123.863 | 7.205 |
| night-two-depths | 7.761 | 4.874 | 126.838 | 2.037 |
| rain-plane4m | 12.506 | 5.763 | 126.497 | 4.679 |
| rain-two-depths | 19.661 | 9.300 | 130.767 | 6.245 |

Retained-gap and old-anchor controls have lower translation and rotation RMSE
than adjacent uninterrupted composition in all six sequences. This is a positive
bounded result, not evidence that outages improve odometry: frame spacing,
per-pair geometry and number of composed estimates change together. These
controls do not isolate which of those effects causes the improvement.
There is no new-method advancement gate or across-population generalization test.

![Original-frame post-gap errors](../../results/continuity-sequence/scoring-001/continuity-comparison.png)

## Local restart is assessed in its own frame

| Sequence | Local translation RMSE, mm | Local rotation RMSE, deg |
|---|---:|---:|
| day-plane4m | 4.028 | 0.01842 |
| day-two-depths | 2.838 | 0.06612 |
| night-plane4m | 7.719 | 0.10209 |
| night-two-depths | 1.743 | 0.04199 |
| rain-plane4m | 2.010 | 0.02561 |
| rain-two-depths | 4.812 | 0.07833 |

These local errors compare against the true relative motion starting at6, and
include the restart identity frame. They must not be ranked against the
original-world RMSE table as interchangeable scores. The restart supplies no
measured transformation back to the old frame. The hold-pose splice makes that
missing connection visible: its123.863–130.767mm RMSE is near the126.491mm gap
displacement, despite the useful resumed local estimates.

The six identical-frame identity-pair contexts have translation errors0.703–
3.249mm and rotation errors0.00114–0.04735deg. These are actual zero-motion
estimation errors, not bitwise-repeat failures. Nonzero pair error is relevant
to accumulated estimates but does not identify the sole cause of the spacing
result. Every null is retained, not subtracted from estimates.

## Qualification, integrity and limits

Before learned inference,11,913checks pass independently reconstructed rays,
plane/atlas support, sampled bilinear quantization and analytic pose-chain
fixtures. Exact cross-gap and anchor transforms recover the original frame;
a failed bridge leaves subsequent retained-chain poses unavailable. A rigid
change to an unconnected post-gap map leaves its internal measurements unchanged,
qualifying the unknown global transform. This is an algebraic observability
control, not a learned no-overlap recovery experiment.

After prediction lock,13,762checks pass (including repeated rendering checks):
fixed population/input-batch hashes, native selection and packet reconstruction,
all72fits' stationarity/information, arm-specific edge composition, missing-state
semantics, reference-frame errors and complete denominators. No producer, scorer,
auditor or tolerance amendment after freeze. The first render launcher omitted
the existing secondary cv2/scipy runtime path and failed before any rendering;
that environment correction was recorded before all learned inference.

Artificial planar atlases, limited motion, overlap and a supplied correct anchor
limit recovery claims. No natural outage, calibration change, unknown place,
IMU integration, false-recovery detection, dynamic scene, real-time deadline or
independent physical reference was tested. All three texture sources are exposed
development inputs, with no independent recording-level generalization claim.

## Connection to original papers and disposition

MACThermal Chapter6 names continuity/stable-calibration and thermal-inertial
recovery as openings. This assay isolates one of their evaluation requirements:
a resumed local estimate needs a measured connection to preserve the old frame.
[ORB-SLAM3](https://arxiv.org/html/2007.11898v2) already supplies relocalization,
inertial bridging and map-merging mechanisms; a fixed-anchor visual measurement
is an established baseline, not our novelty. See the
[source/asset intake](continuity-feasibility-findings-2026-11-14.md).

Complete this exact comparison without changing interruption severity, frame
spacing, anchor, geometry or solver to manufacture a stronger claim. Preserve
its positive bridging results. A future spacing/persistent-estimation study
would need controls that separately vary the number and length of edges; a
recovery method would additionally need overlap/false-association behavior.
Those are different hypotheses, not automatic next variants. Other original-paper
gap families remain open.

Render freeze: `experiments/continuity-sequence-render-freeze-2026-11-14.json`.
Estimator/auditor freeze: `experiments/continuity-sequence-estimator-freeze-2026-11-14.json`.
Raw images, dense fields, packets, all fit receipts, lock, scores, CSV and figures:
`results/continuity-sequence/`. No synthetic truth is read by the capture function;
truth is used in rendering/qualification and after prediction lock for scoring.
