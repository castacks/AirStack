# Natural-motion evaluation: available geometry versus correspondence truth

13 November2026, America/New_York. Label-provenance review and fixed support
inventory, after the known-warp thermal assay. No natural-motion model inference,
reference-pose value parsing, new training or correspondence-error scores here.

## Decision

The existing assets support a finite **provided-geometry correspondence
consistency diagnostic** on natural thermal pairs. They do not presently supply
certified visible temporal correspondence truth. Do not describe this distinction
as “no labels exist” or treat it as a blanket block on thermal learning.
A separate proposed protocol below preserves the available evidence without
calling model/pipeline-derived labels independent physical ground truth.

## Existing local evidence and newly verified availability

Restored the exact60 own-thermal projected-depth maps for the already selected
20day/night/rain frames from the SHA-verified November12 depth-transfer packet.
Each PNG matches both the archived member hash and dataset-record hash. All60
raw image hashes were verified by the preceding assay. The existing recovery
has60 same-ID RGB reference-pose files; their bytes are hashed but their values
were not parsed during this intake. No additional recording/large source archive
or new pose/depth product was requested.

Depth numeric support was frozen before decoding: native256x640,uint16,
zero invalid,positive code count only. No resize, dilation, interpolation,
confidence gate or model-selected pixel set.

| Recording | Frames | Positive source depth pixels,total | Per-frame minimum–maximum | Mean native image fraction | Adjacent candidate pairs |
|---|---:|---:|---:|---:|---:|
| Day |20|159674|7125–8522|4.873%|19|
| Night |20|187027|9264–9405|5.708%|19|
| Rain |20|123322|5593–6809|3.763%|19|

These are source-depth pixels, NOT verified temporal correspondences. The
[previous depth-factor admission](depth-label-retrieval-findings-2026-11-09.md)
found almost no joint depth support at particular native-selected feature
locations. That result does not imply sparse maps have no usable pixels anywhere.
The proposed diagnostic starts from original depth-supported source pixels,
not from the model's selected features; its target support remains to be measured.

The [single-scan projection review](native-depth-proxy-findings-2026-11-05.md)
and [clock review](ms2-clock-evidence-findings-2026-11-04.md) already bound their
physical interpretation. Source/target visibility, exported time semantics,
per-point uncertainty and calibration provenance remain unresolved. Do not rerun
clock searches, fit offsets or read another archive to erase those limitations.

## Published evaluation labels: what this source check establishes

[CMRW v1,4 and AppendixA.2](https://arxiv.org/html/2506.03148v1) reports manually
annotated Thermal-IM RGB/thermal pairs across5timesteps,100pairs/1000points.
For KAIST, it reports RGB-sequence CoTracker-derived correspondences, including
thermal-to-thermal evaluation. NYU uses PIP++ tracks with visual inspection.
Thus “published correspondence annotations” does not mean all labels are manual
or independently measured. The Thermal-IM passage does not by itself establish
an independently manual thermal-only temporal-flow set or a current release.
The inspected [author repository](https://github.com/ayshrv/cmrw) still has a
code-coming-soon README; the project-page request failed. No evaluation annotation
payload was obtained. Source statements, availability and reference quality
remain separate questions. This is a targeted section reading, not full-paper
or annotation-dataset reproduction.

The [Deep Thermal Optical Flow institutional abstract](https://escholarship.org/uc/item/8fb1r3k8)
already states its labels use RGB-model predictions. It remains a useful training
lead, not a newly verified independent evaluation reference. Generic object-
tracking boxes or camera trajectories likewise do not supply per-pixel flow.

## Proposed finite protocol: supplied-geometry disagreement on57pairs

This is a reviewable specification, NOT an execution freeze. Numeric source,
calibration and runtime pins must be bound before inference. No current result
is asserted, no new capture/annotation campaign or protected payload assigned.

**Population:** all19adjacent pairs in each existing20-frame thermal prefix,
57pairs total. Keep original integer times and source identities. Use nominal
same-ID rig/pose grouping as an explicit inherited benchmark assumption, without
claiming measured physical simultaneity or estimating a clock correction.

**Model:** the single qualified released U²Flow Sintel checkpoint. For each
natural pair, predict original preparation and fixed targetgamma1.6,114forward
calls. Same128x320preparation and12iteration CPUFP32 interface as the preceding
assay. No checkpoint, frame or appearance-family search. Predictions lock before
any pose-derived endpoint labels are constructed. Model output never defines
reference eligibility. A zero-flow field is diagnostic context, not a full
competing VO method. No MACThermal superiority claim without its own comparator.

**Reference geometry:** let P_t be the provided world-from-RGB camera matrix
under the already-qualified saved convention and E be nominal thermal-from-RGB.
For own-thermal source pixel u with positive supplied source depth z,

    X0 = z K_thermal^-1 [u,v,1]^T,
    X1 = E P_(t+1)^-1 P_t E^-1 X0,
    u1 = project(K_thermal X1).

This is a static-scene, nominal-pair projection proxy, not a measured moving-
object correspondence. Validate transform directions,metre units,rig convention
and original own-camera calibration against the existing adapter/source records.
Do not silently use separately sensor-initialized poses as one global frame.
An input-convention mismatch stops this configuration without pose/time fitting.

At evaluation half-resolution, source sample coordinate is
`(u+0.5)/2-0.5` per OpenCV pixel centers and reference displacement is
`(u1-u)/2`. Bilinear interpolation samples MODEL flow only; reference-depth
holes remain holes. No reference-depth resizing or filling.

**Three fixed reference-support tiers, all reported:**

1. Positive native source-depth code, target positive z and projected pixel
   inbound; deduplicate rounded target cells by nearest projected source depth.
   Record behind-camera/out-of-bounds/collision counts explicitly.
2. Tier1plus nonzero native target-depth at `floor(u1+0.5)`; do not expand the
   pixel radius or interpolate missing reference values.
3. Tier2plus depth agreement `abs(z_projected-z_target)<=0.1m+0.02*z_target`
   and backward reprojection distance<=1native pixel. These are predeclared
   engineering consistency tolerances, not measured sensor error bounds or
   certified visibility. No tolerance sweep or score-based label pruning.

Tier1is broad geometry disagreement; Tier2adds target-map availability;
Tier3is a more constrained diagnostic. None certifies static-point identity,
occlusion or physical accuracy. Preserve all tiers rather than selecting the
one with favorable model scores. Support outcomes are currently unknown.

**Endpoints:** per-pair support/rejections, provided-geometry endpoint disagreement
mean/RMSE,1/3evaluation-pixel agreement, original scalar-alpha/disagreement ranking
and clean-versus-gamma predictive consistency on identical reference supports.
Retain every one of57pair slots, with explicit null metrics for empty supports;
record-supported macro means with denominator counts, not pixel pooling across
recordings. No physical calibration or natural-ground-truth-EPE wording.

**Stopping decision:** one frozen run if input conventions pass. Report all57pairs,
including empty/low-support cases and inconsistent references. No automatic
longer sequence, threshold/floor, correspondence selection, training fit or
uncertainty recalibration. Large disagreement alone would not identify a model
failure: reference geometry/timing/dynamics are competing explanations. A later
method needs a specific defensible endpoint beyond this diagnostic.

## Intake verification and retained scope

Exact archive/member/depth hashes are in
`results/natural-flow-reference-intake/inputs/metadata.json`; support protocol
is `experiments/natural-flow-reference-intake-freeze-2026-11-13.json`, results in
`results/natural-flow-reference-intake/report.json`. Old role labels in the
November10 adaptation dataset are historical; no new split/holdout is assigned.
All three recordings remain previously exposed development inputs.

Independent Pillow-versus-OpenCV verification passes **246 checks**: exact depth
hashes, schema, per-frame positive counts/fractions and recording aggregates.
No endpoint construction, pose-value parsing or model execution occurred.
