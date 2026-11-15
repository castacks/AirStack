# Contribution decision after the selection, continuity and sensor-time studies

14 November 2026 UTC. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus existing working changes.
Synthesis and bounded primary-source review; no new inference, training or scores.
This decision supersedes older next-step recommendations, preserving their history.

## What the completed work contributes

The defensible deliverable is a controlled empirical study of how uncertainty
changes meaning between a learned matching output and the estimator that uses it.
Three distinctions have direct local evidence:

| Question | Supported result | Scope that must stay attached |
|---|---|---|
| Does lower estimator spread mean higher accuracy? | In day-two-depths, query regeneration increases translation variance 21.76× while reducing exact-target translation RMSE 25.08%. Other five scenes worsen. | Six artificial scenes and an imposed half-LSB ensemble; not physical noise. |
| Is a matching uncertainty readout a pose-accuracy ranking under sensor integration? | Memory reduces median matching covariance in all six moving cases; translation and rotation error each rise in four. Both counts persist with fixed query IDs. | One virtual timing profile, three texture proxies, pairwise pose harness; no exact integrated-pixel flow truth. |
| Does an additional recovery gate help an already successful baseline? | Native/geometry controls accept all 12 positive and no 24 negative candidates; the cycle gate drops one positive without rejecting another negative. | Constructed world identity; negatives yield zero proposals, so conditional ranking is undefined. |

These are reproducible comparisons and useful controls. Publication novelty has
not been established. The existing frontend succeeds on all planned timing fits
and the constructed recovery assay. Neither that success nor the mixed learning
results closes the original papers' broader research openings.

## Prior work that changes the claim

[MBA-VO](https://arxiv.org/html/2103.13684v1), section3, already incorporates
exposure integration and local camera motion into a direct tracker. Its backend
uses deblurred keyframes. The pinned released implementation samples virtual
poses using capture/exposure times and splines; do not equate this source with
the paper's two-pose description. Generic image-formation-aware VO is established.

[CUAHN-VIO](https://arxiv.org/html/2208.13935v1), VII-E, already discusses blur
increasing predicted uncertainty while mean predictions remain useful, reducing
their EKF influence. This is author interpretation in its evaluated setup. Our
thermal experiment instead finds lower aggregate matching covariance and mixed
pose errors; different outputs, targets and sensors prevent a direct replication
or contradiction claim. The general blur/uncertainty/task connection is prior art.
Its released update body uses network covariance in the measurement gain.

The inspected CUAHN paper is arXiv2022v1. The official repository cites the journal
article as Robotics and Autonomous Systems185:104866,2025; the journal version
was not newly read. Existing TRNeRF/TRGS-SLAM reviews already establish thermal
response/readout-aware restoration and estimation. Existing DNLS/DISK/NG-RANSAC
reviews establish generic task-trained weighting/selection. A new head, scalar
covariance adjustment, reblurred residual or pose loss alone is not a demonstrated
research distinction.

Six released README/source snapshots across the two new repositories are hashed
in [source pins](../../results/contribution-synthesis/primary-source-pins.json).
Inspection is targeted and static; no author system or checkpoint was executed.

## What each study leaves open

| Original opening | What our work adds | What it does not answer |
|---|---|---|
| MAC-VO V1 / MAC-I² I1 persistent estimation | Supplied-anchor continuity and matched-count reference controls | Continuous probabilistic state reuse, natural retrieval or map merging |
| MAC-VO V2 / MAC-I² I2,I4 uncertainty assumptions | Separate conditioning models, selection variability and exact-target errors | Physical conditional coverage, sequential reliability or a new missing covariance term |
| MAC-VO V3 selection | Standard information and matched learning controls, with positive cases | A generally superior selector; the tested small policy recipe alone is closed |
| MAC-VO V4 / MACThermal T2 supervision | Qualified virtual image integration and explicit timestamp-specific pose targets | Correct observed-pixel flow labels for integrated images or improved thermal training |
| MACThermal T1 continuity | Successful controlled connection baseline and explicit abstentions | Natural outages, changing calibration, qualified thermal–inertial recovery |
| MACThermal T3 / MAC-I² I3 sensor transfer | Identifies the difference between borrowed and measured sensor parameters | Cross-sensor adaptation or calibrated real-camera transfer |
| MACThermal T4 multispectral representation | Prior RGB/thermal weighting controls | Modality-invariant representation and new cross-modal matching |
| MACThermal T5 active perception | No direct experiment | Motion/camera/resource allocation for perception |
| DNLS training/compute/dynamics openings | Existing limited pose-learning controls | General scale identifiability, dynamic-scene learning or compute-efficient training |

The [original gap map](original-paper-gap-map-2026-11-13.md) retains author versus
interpretation labels. This table is our evidence assessment, not an assertion
that later literature has left every original opening untouched.

## Next increment selected: qualify evidence for sensor transfer

Before extending the exposed virtual timing experiment, establish whether the
available real thermal data can support a sensor-transfer evaluation. This is
an input-feasibility task tied to T2/T3, not a new accuracy experiment or a claim
that the other gaps are lower-value. It adds evidence the nine virtual cases
cannot provide and avoids selecting another severity from their observed scores.

The finite output must be a provenance table for available sequences: camera and
raw/prepared grid, timestamp convention, exposure/response/readout provenance,
preprocessing order, independent pose/IMU reference and its alignment convention,
and recording-level split availability. Distinguish measured, author-specified,
borrowed, nominal and unknown entries. Use the actual loader and data payload;
do not infer a response time from the existing 8ms virtual parameter or turn
nominal geometry into physical truth. Existing input-contract and continuity
intakes should be reused, not rerun under another name.

Stop after one bounded inventory and readiness decision. If physical quantities
remain unknown, report precisely which claim cannot be evaluated and retain a
nominal-data option with that label. Do not launch inference, fit clocks, estimate
sensor constants from the exposed errors, obtain a large dataset, or promote a
method during this intake. A later method comparison needs a specific distinction
from MBA-VO/TRNeRF/TRGS-SLAM and separate geometry/appearance/task controls.

## Evidence and report

The updated [technical report](uncertainty-interfaces-report-draft-2026-11-13.md)
incorporates the later studies rather than leaving the old conditioning direction
as the lead method claim. [Evidence ledger](../../results/contribution-synthesis/evidence-ledger.json)
hashes the linked local findings and latest audit/score receipts. Existing
numerical audits are not rerun for this documentation-only synthesis. No new
algorithm, physical calibration or full published-system reproduction is claimed.
