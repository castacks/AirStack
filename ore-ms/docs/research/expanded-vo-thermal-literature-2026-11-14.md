# Additional VO/thermal references and next contribution decision

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus preserved working changes.
Targeted primary-source discovery/review; not a systematic literature survey.
No new estimator experiment or training. Existing numerical recipes remain closed.

## What this changes

The latest matched-count paths show that reference choice affects accuracy,
with translation/rotation preferences that differ across scenes. These are
correct-anchor, overlapping-view artificial tests. Before building a reference
policy, compare how existing thermal SLAM systems obtain and validate old-map
connections. Their results already establish substantial capability; our tests
do not demonstrate that those systems fail.

Two particularly relevant additions are TI-SLAM and TRGS-SLAM. Thermal loop
closure, probabilistic graph factors, geometric verification and relocalization
already exist. The useful next question is whether a candidate connection is
trustworthy outside our supplied-correct-anchor setting. This is a research
interpretation, not a newly demonstrated failure or verified field-wide gap.

## Thermal sources

| Paper | Established mechanism and relevance | Reading depth in this pass |
|---|---|---|
| [DeepTIO](https://arxiv.org/abs/1909.07231v2) | Thermal-inertial learned odometry with hallucinated RGB features and selective feature fusion; relevant to representation and multimodal learning beyond fixed-output weighting | Primary abstract; no new method/code reproduction |
| [TI-SLAM: Graph-based Thermal-Inertial SLAM with Probabilistic Neural Networks](https://arxiv.org/html/2104.07196v1) | Probabilistic odometry, thermal embedding loop detection, geometric rejection and uncertainty-weighted pose graph; direct overlap for generic thermal uncertainty/loop-closure proposals | Targeted backend VI, representation VII-B1 and limitations VIII; author README and selected MATLAB loop/rejection/backend source |
| [TP-TIO](https://arxiv.org/abs/2012.03455) | Learned ThermalPoint, radiometric tracking and optimization-based inertial fusion | Primary abstract recheck, existing deeper local reviews preserved |
| [Self-TIO](https://ieeexplore.ieee.org/document/10803066/) | Self-supervised16bit feature extraction and hybrid KLT/learned tracking, including NUC/aggressive motion evaluation; direct training/representation reference | Primary indexed abstract; previous author repository availability review retained |
| [ETIO](https://arxiv.org/abs/2210.10033v2) | Edge images, adaptive KLT and inertial/sliding-window estimation; useful alternative thermal representation baseline | Primary abstract only; code not checked |
| [ThermalSfMLearner](https://arxiv.org/abs/2103.00760v3) | Thermal/visible consistency supplies depth/ego-motion self-supervision; missing dense thermal labels do not rule out learning | Fresh primary abstract recheck; existing author-checkpoint/transfer experiments remain completed |
| [TRGS-SLAM](https://arxiv.org/html/2603.20443v1) | 2026preprint: continuous-time thermal-inertial3DGS, microbolometer-aware rendering and recovery; sensor-artifact and persistent-map reference | Targeted3.5keyframes,3.6mapping/relocalization,4.1–4.2, AppendixK; project/README and selected source branches, not full-system audit |

## General VO sources that connect to our observations

| Paper | Why reference it | Fresh depth |
|---|---|---|
| [Direct Sparse Odometry](https://arxiv.org/abs/1607.02565v2) | Joint inverse-depth/pose optimization and photometric calibration: persistent geometry and image formation already have established foundations | Primary abstract only |
| [D3VO](https://arxiv.org/abs/2003.01060v2) | Learned depth/pose/photometric uncertainty inside direct VO; learned weighting and brightness modeling are prior art | Primary abstract recheck; historical reading retained |
| [DPVO](https://arxiv.org/abs/2208.04726v2) | Recurrent sparse patch matching coupled with differentiable BA; baseline for persistent correspondences and compute | Primary abstract recheck; earlier residual/code review remains historical |
| [PNEC](https://arxiv.org/abs/2204.02256v1) | Anisotropic feature uncertainty affects rotation; useful when translation/rotation preferences differ, without asserting it explains our result | Primary abstract recheck; previous local rotation controls remain completed |

DROID-SLAM, ORB-SLAM3, DNLS, Good Feature, Deeper-DICE and the MSO thesis
remain relevant existing references in the living registry. None is newly
reproduced by this intake. ThermalSfMLearner's local transfer failure and the
small selection/weighting recipes do not close the corresponding paper families.

## Specific paper limitations and overlap

**TI-SLAM.** SectionVIII explicitly identifies scale accuracy, offline execution,
backend covariance/loop-rejection parameter choice and adaptation across sensor
intrinsics/extrinsics as limitations. Its backend already checks geometric
consistency among loop proposals before optimization. Thus a new thermal loop
or covariance-weighted graph is not, by itself, an increment. Its source release
calls itself simplified and uses Python plus MATLAB with an older TensorFlow
stack; code availability is not a qualified current-runtime reproduction.

**TRGS-SLAM.** AppendixK explicitly bounds random past-keyframe sampling by
small-scene overlap and its relocalization by limited drift, and identifies
predefined thermal scaling thresholds as an online limitation. Sections3.5–3.6
already use translation/visible-Gaussian overlap criteria, stronger past-keyframe
weighting and SSIM-triggered state rollback/relocalization. These assumptions
connect directly to our current correct-anchor/overlap boundary. They are not
proof of a missing mechanism or a newly observed failure in TRGS-SLAM.

Its released README also distinguishes generated pseudo-ground-truth poses from
independent physical truth, and specifies camera/IMU/calibration inputs. The
TRNeRF data are a possible thermal-inertial source, not an already downloaded or
qualified physical reference. Do not silently replace our reference problem
with another estimator-generated trajectory and call it ground truth.

## Next finite research step

Recommend a **connection-validity assessment before another reference-selection
method**. Candidate question: does learned match/pose uncertainty distinguish
valid thermal old-map connections from false connections beyond established
geometric verification? This is an evaluation hypothesis; no new failure,
confidence model or novelty is established.

A defensible protocol would need known-overlap positives and explicitly qualified
wrong-place/no-association negatives, separate correspondence acceptance from
true pose error, and preserve every rejected/unavailable case. Compare native
fit/support criteria against robust geometric verification and cycle/trajectory
consistency inspired by TI-SLAM. Adding generic RANSAC or cycle checks is already
prior art, not the proposed contribution. Any uncertainty readout is evaluated
conditionally after those controls, without treating low residual or convergence
as a correct association.

Do not yet launch training, invent an uncertainty score, copy published numeric
thresholds across incompatible units, or sweep the exposed reference indices.
First qualify the positive/negative scene contract and baseline interfaces;
only then bind one finite numeric comparison and stopping rule. This next step
addresses a concrete boundary of our existing evidence, not a failure already
observed in the authors' systems. Thermal-specific representation, cross-sensor
transfer, sensor-physics modeling and active perception remain distinct options.

## Source and code provenance

TI-SLAM source commit:
`bd5eaecd39a41a780d2ed5a549c169f57fb5cbd6`.

Inspected README; embedding cosine-distance proposal generation;
`spatial_consistency.m` cyclic geometric validation; backend settings/source
entry point. Not a full review of every MATLAB/network function. No code imported
or executed. The first master-branch request failed; the official main branch
was then resolved and pinned.

TRGS-SLAM source commit:
`07ba901afb9d9296f2e4399240d9fd4dc6d2398c`.

README/data prerequisites and pseudo-ground-truth generation description;
targeted `slam.py` mapping sampling/SSIM rollback and `trajectory/keyframes.py`
translation/IoU selection/random sampling. No renderer/spline implementation,
checkpoint, runtime, dataset payload or benchmark reproduction checked here.

The repository stores only bibliographic/source-hash metadata in
`results/expanded-literature-review/source-review-manifest.json`. Author code
snapshots/paper text remain outside repository and Box packet. Search-only leads
(TEVIO, online-photometric thermal SLAM, FTI-SLAM and other new listings) are not
used as method evidence or declared gaps without deeper verification.
