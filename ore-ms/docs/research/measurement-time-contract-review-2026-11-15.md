# Measurement-time review: preserve the supplied-pair benchmark

15 November 2026. Branch `ore-ms-orb-development`, baseline `53afa19d16` plus local working documents. Targeted source review, not a new experiment.

## Decision

**The public sources do not establish a stronger physical measurement-time contract.**
The dataset documents synchronization and its loaders use supplied image indices.
We can continue the explicitly declared supplied-pair benchmark. We cannot certify
exposure times from the exported timestamp values or turn the duration probe into
a correction. This closes this source-check task; it does not require stopping all
RGB–thermal research until the acquisition details become available.

The completed [4 November review](ms2-clock-evidence-findings-2026-11-04.md)
already reached this limit. Today's bounded check confirms both official repository
HEAD revisions remain unchanged; no full issue/tree rescan was repeated. The
[current source record](../../results/measurement-time-review/run-001/source-review.json)
contains retrieval time, commit IDs, loader hashes and relevant lines.

## What the primary sources establish

The [official data-format page](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format)
describes rectified, synchronized streams and poses projected into each sensor's
coordinate system. The [setup page](https://sites.google.com/view/multi-spectral-stereo-dataset/setup)
describes external synchronization for RGB/NIR stereo and a left-thermal sync
signal for thermal stereo. These support the authors' acquisition description.
They do not define the event and clock domain written to the camera timestamp files.

The [original loader](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/blob/0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284/dataloader/MS2_dataset.py)
and [benchmark loader](https://github.com/UkcheolShin/SupDepth4Thermal/blob/f6725678f9b5dba669ea1f0739506a613f2fb2fb/dataloaders/loaders/MS2_dataset.py)
sort image paths, select corresponding indices and reuse a sample index across
requested modalities. Neither loader reads timestamp text. Both revisions match
the prior review. This establishes an implementation convention, not a measured
cross-camera exposure residual or proof of perfect physical simultaneity.

## Operational contract

| Use | Permitted interpretation | Required limit |
|---|---|---|
| Existing fixed-weight and learned-weight results | Empirical results on the stated supplied-pair inputs and nominal reference | Preserve all failures, tradeoffs and original numbers; no claim of certified physical synchronization |
| Further supplied-pair fusion evaluation | Use unchanged image IDs, calibration, solver and reference conventions for every arm | Explicit synchronization assumption; no timestamp-based rescaling or outcome-driven re-pairing |
| Duration probe | Posthoc sensitivity of projection to reference-motion magnitude | No claim of estimated lag, actual exposure duration, corrected pose accuracy or deployable fix |
| Physical asynchronous fusion | Requires acquisition-supported event/clock definitions, mapping and residual bounds | Currently unqualified; exported timestamps alone are insufficient |
| Claims about thermal's environmental contribution | Need independent corroboration beyond nominal pose gain/conflict | Timing sensitivity, depth error and other causes remain unresolved |

In particular, differing **recorded** durations do not establish differing physical
exposure intervals. The large residual reductions show that the diagnostic is
sensitive to motion magnitude; scaling translation can also compensate depth bias.
The [fixed-group diagnosis](fixed-group-diagnosis-findings-2026-11-15.md) should be
read with that distinction. Its image checks remain useful evidence that some
harmful groups are photometrically consistent, without proving geometric accuracy.

## What would change this decision

The exact data-owner question is already prepared in the 4 November review:
which event/clock each timestamp file represents, how exported image IDs relate
to exposures/triggers, and the applicable cross-camera mapping and residual bounds
for the three named recordings. Acquisition/export code or an authoritative
answer could qualify a physical-time estimator. No message was sent.

Absent such new evidence, stop repeating public clock searches and do not select
a clock correction from pose scores. Continue engineering under the explicit
supplied-pair contract if evaluating practical benchmark improvement. The current
results still show small, mixed improvements and no established added value from
the richer learned weighting layer. This review supplies no new accuracy claim.

## Verification

Both official repository HEADs match the prior pinned revisions. Both current
loader texts contain zero timestamp mentions; the saved relevant lines confirm
sorted image lists, same-index stereo selection and shared modality sample IDs.
Official setup/data-format pages were read again. No images, model weights,
references, clocks, correspondence packets, predictions or scores were changed.
