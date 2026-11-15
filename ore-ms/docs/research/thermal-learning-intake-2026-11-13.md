# Thermal-learning direction: method comparison and finite first assay

13 November2026, America/New_York. Source/code intake and protocol design only.
The previous conditioned temporal-window configuration stays closed. The user
asked to proceed through the original-paper gap shortlist; this step examines
thermal learning rather than tuning that solver.

## Outcome

Retain thermal correspondence learning as a distinct direction. Do not present
photometric augmentation, cycle consistency, shared encoders or self-supervised
uncertainty as new mechanisms. A suitable published uncertainty-learning baseline
is now identified and source-inspected: U²Flow. The immediate next deliverable
is its bounded inference/target assay, not a new architecture or another VO fit.
No published-system reproduction or candidate training has run in this step.

## Method-level overlap

- [CMRW v1](https://arxiv.org/html/2506.03148v1),3/6: modality-specific encoders,
  shared matching transformer and intra/cross-modal random-walk losses learn
  correspondence without photometric constancy. Intra-modal supervision helps
  avoid poor cross-modal convergence. The inspected limitation concerns domains
  without shared distinctive structures. Its author repository still contains
  a code-coming-soon README; this pass verifies that page, not a fresh code-body
  release. A self-supervised shared representation is already prior work.
- [TartanMatch v1](https://arxiv.org/html/2610.14513v1),III–IV: thermal intensity
  is replicated into three channels, while the shared backbone learns flow and
  covisibility from supplied/geometry-derived/refined labels. Thermal is not
  given the learned input projection used for other modalities. The pinned
  public tree has42entries and inference/model tests; no training pipeline was
  identified in that tree. The existing local within-thermal flow-replacement
  trial is not a cross-modal representation-training experiment. Neither
  covisibility nor a plausible warp is independent matching truth.
- [Learning by Analogy](https://openaccess.thecvf.com/content_CVPR_2020/papers/Liu_Learning_by_Analogy_Reliable_Supervision_From_Transformations_for_Unsupervised_Optical_CVPR_2020_paper.pdf)
  already uses transformed clean predictions to supervise challenging augmented
  images. Relevant method excerpt inspected, not a full-paper/code audit.
- [Sun-RAFT/Muun-RAFT](https://cv-stuttgart.github.io/Reviving-Unsupervised-OpticalFlow/)
  paper excerpts and pinned Sun-RAFT loss/augmentation source describe self-
  supervised flow with appearance/spatial transformations. Standard augmentation
  consistency is thus a mandatory baseline rather than a new method claim.
- [U²Flow v1](https://arxiv.org/html/2604.10056v1),3.3/A.2/B.4, explicitly trains
  uncertainty from the L1 difference between transformed clean predictions and
  augmented-input predictions. Its released loss optionally detaches this
  difference from flow learning. AppendixB.4 already separates appearance and
  spatial augmentation, with spatial dominating on its evaluated RGB benchmarks.
  That result motivates testing the thermal domain; it does not establish a
  thermal failure or warrant merely repeating its ablation as novel.

The original MACThermal augmentation and the prior raw-patch, matcher-transfer,
small stereo-residual and shared-weight trials remain separate completed
configurations. None trained the published U²Flow system or a new thermal
representation. Their results do not rule out this broader direction.

## Source/checkpoint readiness

| Asset | Checked evidence | What remains |
|---|---|---|
| U²Flow | Author source commit `152bb38d90621c34902943763c00a77ecef8725f`; README, trainer, loss modules, requirements downloaded and inspected. | README links Drive checkpoints; exact checkpoint bytes/state and inference compatibility not verified. No GitHub release assets returned. |
| Sun-RAFT | Author source commit `912e2b44c600fc733363b57a8852e9e1801e4c1f`; README, loss and augmentation modules inspected. Releasev1.0.0 lists three21103087-byte checkpoints. | Metadata only, no checkpoint download/state validation or execution. |
| CMRW | Author README still says code coming soon. | No executable official reproduction available from that inspected release. |
| Existing runtime | Docker lists AirStack images, not the exact recovered MAC compiler image. Isolated NumPy/SciPy environment is available. | A historical GPU runtime receipt is not current neural-runtime readiness. No torch import, model installation or training attempted here. |
| Thermal inputs | Prior fixed raw input hashes/manifests and recoverable packets exist. Current persistent-window recovery contains track/tensor data, not the required raw image payload at its image paths. | Restore only a declared small image packet and bind source hashes before decoding/inference. No new recording campaign required for a controlled warp assay. |

Old framework requirements need a separately qualified isolated runtime;
do not silently swap dependency versions and call it an exact reproduction.
Downloading model assets and running interface smoke would be engineering
qualification; matching an author's benchmark additionally requires their data,
configuration and metric protocol.

## First finite assay: actual error versus predictive inconsistency

Question: **on thermal images with known imposed geometry, does augmentation
inconsistency track actual correspondence error, or can a prediction remain
consistent but wrong?** This is a target-validation assay for a published learning
signal, not an assertion that the problem is newly discovered.

Use one fixed released checkpoint per model: original MACThermal as application
context and official U²Flow as the self-supervised uncertainty baseline. Their
architectures differ, so this comparison cannot isolate training objective alone.
Sun-RAFT is an additional future learning-control candidate, not a third automatic
experiment in this intake.

Proposed controlled input population: raw first20 thermal-left frames from each
already exposed day/night/rain prefix, with complete recordings kept together.
Generate source/target pairs from one frame with a fixed known spatial mapping;
apply geometry-preserving appearance changes separately. Include identity, a
nonzero displacement, appearance-only change and their combination. The same
visibility mask and geometric labels must apply to all methods. Fix the actual
transforms, ranges, interpolation and image preparation before any model run.
Neither a clean model prediction nor the candidate's own matches are labels.

Measure all eligible pixels' endpoint-error mean/RMSE, fixed1px/3px success,
finite-output coverage, uncertainty/error ranking and predictive inconsistency.
Score clean/appearance-altered pairs separately so success cannot come from
ignoring image displacement. Preserve ambiguous/textureless and invalid pixels
in explicit denominators. If dropout/rejection is introduced later, compare
error-versus-coverage at identical retained fractions, never a favorable mask.

A synthetic warp of a real image supplies exact imposed correspondence, not
truth for natural inter-frame thermal motion. Appearance perturbations are
controlled stressors, not physically validated NUC/weather simulation. No
claim of physical temperature calibration or fresh-record generalization.

**Execution gate:** author inference output semantics, image-size/flow scaling,
checkpoint identity, no-op preparation, known coordinate mapping and padding/
visibility must pass fixed mechanics checks first. Then freeze one run and
numeric endpoints, without checkpoint, temperature-range or frame searches.
This document is not that numerical freeze. No performance thresholds are
chosen from unobserved scores; the scientific question is descriptive target
validity, not promotion of an unimplemented model.

**Stopping decision:** finish and report that one target assay. Consistently wrong
predictions would motivate a separately designed supervision mechanism; ordinary
augmentation sensitivity alone would not. No automatic new confidence head,
VO integration or architecture ladder. A new method requires a mechanism beyond
the existing transformation-consistency and uncertainty-loss formulations.

## New dataset lead: provenance matters

Harshith Mohan Kumar, *Deep Thermal Optical Flow*, UCR master's thesis2025,
[institutional record](https://escholarship.org/uc/item/8fb1r3k8), describes aligned
RGB/thermal capture and RGB-model-generated flow supervision. Institutional
abstract and indexed thesis excerpts inspected. This is pseudo-ground-truth,
not independently measured thermal flow. Public dataset/checkpoint release and
exact calibration/evaluation are unverified. A direct PDF download returned
zero bytes and could not be parsed; no full-thesis reading is claimed. No data
was downloaded. This lead does not close our reference-quality limitation and
is not a reason to start a separate acquisition project.

## Reproducibility record

Small source copies/trees were downloaded outside the repository, never imported
or executed. `experiments/thermal-learning-source-intake-2026-11-13.json` records
exact source hashes, inspected files and reading depth. No model weights or new
image/reference payload were downloaded; no synthetic/real model outcome,
parameter update, new training dataset or held-out assignment occurred.
