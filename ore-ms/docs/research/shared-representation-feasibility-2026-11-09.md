# Shared-representation compatibility check and scope decision

2026-11-09, America/New_York. Bounded read-only assessment requested to keep the work on the original MS question. Base project commit `293b666c4`; the preceding literature note is an uncommitted documentation update at the start of this check. No model weights, dataset images or new reference labels were downloaded; no training, inference, matching scores or VO experiment ran.

## Decision

**Retain the shared-representation papers as related work; do not start a matcher integration or representation-training branch now.** TartanMatch is a technically accessible candidate, but no evidence in this check establishes that replacing the matcher would improve the project's cross-camera reliability prediction. A runnable model alone does not justify another experimental chain.

The main question remains whether uncertainty/context predicts relative RGB–thermal geometric error and can support better VO. The [original handoff](../../MS_RESEARCH_HANDOFF.md), sections 2–4, distinguishes that question from baseline execution and makes downstream integration contingent on a useful reliability signal. The completed fixed-rule evaluation remains [STOP_INSUFFICIENT_EVALUATION](decisive-uncertainty-findings-2026-11-06.md), with no demonstrated matched benefit over RGB. It is not reopened by this literature review.

## What was checked

| Candidate / prerequisite | Observed result | Consequence |
|---|---|---|
| TartanMatch implementation | Public [author repository](https://github.com/castacks/tartanmatch), pinned locally at `b3f65ffdf9723a028d82f36ab0beb8457d0def72`. Code license BSD-3-Clause; README assigns weights CC BY-NC-SA 4.0. | Existing inference code can be inspected without developing a replacement. No installation performed. |
| TartanMatch checkpoint | Public [model repository](https://huggingface.co/theairlabcmu/TartanMatch), revision `27662ae4c0438fd901cbc80836f027b1985385f6`; API reports ungated. Checkpoint HEAD returned HTTP200 and 1,713,083,888 bytes. | Availability verified by metadata only. Bytes, tensor contents and inference were not validated. |
| TartanMatch inputs/outputs | `model.py` exposes `predict`; RGB/thermal are three-channel uint8 or floats in [0,1]. Output is original-coordinate dense flow and covisibility. | Mechanical image compatibility is plausible; no covariance or relative-camera-error head is supplied. Covisibility is not calibrated geometric-error uncertainty. |
| CMRW implementation | [Author repository](https://github.com/ayshrv/cmrw) at `7b1f7b2f6d91e2bfad718d719ecac6dd6db87fa4` tracks only README, stating code is coming soon. | Reference only. Do not reimplement the paper as a side project. |
| Local compute/runtime | RTX PRO 5000 Blackwell, 48,935MiB reported. Host Python lacks torch, numpy, safetensors and huggingface_hub. Listed Docker images are AirStack images, not the previously recovered MAC runtime. | GPU availability does not establish runnable compatibility. No dependency/runtime build attempted. |
| Local project data | `ore-ms/data` and `ore-ms/results` are absent in this checkout. Earlier source reports document recoverable Box artifacts. | Any run would require selecting/restoring a bounded development packet, rather than claiming the historical data is currently staged. |
| Existing evaluation target | The existing MS2 adapter preserves official source pairing and thermal preparation; physical frame/clock verification remains false. Prior common-target motion conversion is nominally qualified, not independently physical. | A learned cross-camera flow field would not validate timing, visibility or the physical reference target. |

Source-level facts above are from the pinned repositories and current local inspection. The small source clones are outside the project at `/root/ore-ms-reference-inspection`; they were read, not imported or executed. A manifest in `experiments/shared-representation-feasibility-2026-11-09.json` records inspected source hashes.

The [TartanMatch paper](https://arxiv.org/html/2610.14513v1) lists a mixed real/synthetic training suite and separate evaluation datasets. MS2 and TartanDrive3 are not named in its training table. That is not a certificate that particular recordings or upstream pretraining never overlap. TartanAir is synthetic and should not be conflated with TartanDrive3. Exact checkpoint lineage remains to be resolved for any future generalization claim.

## Why inference is not the next main-work step

Our existing motion experiment already has nominal common-frame/interval comparisons; absence of a learned RGB–thermal embedding is not its demonstrated execution blocker. The final test's joint eligibility excludes unsupported, missing/nonadjacent, termination-only, invalid-projection and invalid-reference cases. A shared matcher might change correspondence quality, but this review does not show it repairs those exclusions or yields predictive cross-camera uncertainty.

Switching the frontend would change the estimator outputs and uncertainty contract together. An apparent gain could come from a stronger matcher rather than improved reliability estimation. Establishing that distinction would require matched estimator controls, a defined uncertainty target and separate recording-level evaluation. Training shared/private encoders now would expand into a new method-development project before this justification exists.

The [earlier shared-point audit](../../experiments/EXP-055-ms2-cross-modal-feasibility.md) found candidate overlap but did not certify visibility or common physical identity. Learned matches could provide candidate correspondences; their predictions must not become the truth against which their own reliability is judged.

## Keep the next deliverable finite

The next main-work deliverable is a supervisor-facing decision using the [characterization report](characterization-report-2026-11-09.md): useful within-camera ranking; no validated cross-camera adaptive benefit for the tested route; shared representations as a possible future hypothesis. Include the completed coverage stop and the matched failures. This is ready from existing evidence and needs no new model run.

The proposed future hypothesis, if separately justified, is narrower than “build a joint space”: **local cross-modal correspondence features add predictive value for relative geometric error beyond existing uncertainty and simple camera/context baselines.** It would need an independent error target and recording separation; raw feature similarity or a plausible warped image would not establish that value.

Reopen technical work only when all of these are concrete:

1. A named failure of the existing reliability model that the proposed feature is expected to address, tied to the original error target rather than general matching quality.
2. An existing pretrained model and a bounded already-exposed input packet with defensible geometric scoring support; no new dataset campaign or estimator rewrite to create the first result.
3. A frozen incremental-value comparison against the existing uncertainty/context and fixed-camera controls, plus a clear stopping rule and separate recording-level evaluation if development evidence warrants it.

If those are not available, keep the idea in related work. No further paper sweep, model download, threshold search, calibration repair or architecture implementation follows automatically from this check.

## Verification boundary

The repository and checkpoint metadata are checked observations; model performance and integration feasibility remain untested. A GPU or public checkpoint is not a positive scientific outcome. No new project partition is assigned, no recording is relabelled, and production defaults and previous experiment decisions are unchanged.
