# Natural thermal motion against nominal supplied geometry

13 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus source-frozen working changes.

## Result and decision

The released U²Flow baseline produces useful natural-motion predictions under
this provided-geometry diagnostic: mean displacement disagreement is lower than
zero flow on day and rain in every support tier. Night geometry has very small
motion, and zero flow is slightly closer. The controlled-warp baseline success
remains positive evidence. No training, model adaptation or recipe search occurred.

Small appearance-induced prediction differences coexist with substantial
provided-geometry disagreement in day and rain. Existing scalar alpha ranks that
disagreement more strongly than clean/gamma predictive consistency in those
records. This is descriptive evidence about these diagnostics, not a calibrated
uncertainty result or proof of actual matching error. Reference geometry, nominal
timing, dynamics and visibility are competing explanations. This experiment
neither closes the original thermal-learning gap nor justifies a new head on its own.

## Frozen population and convention qualification

All 19 adjacent pairs from each previously exposed 20-frame prefix: 57 pairs,
114 CPU FP32 forwards, clean and fixed target gamma 1.6. The exact released Sintel
checkpoint and author source are unchanged; 12 iterations per forward, all 1,368
upstream uncertainty tensors finite; strict 106-tensor load and post-run weight
identity pass. All predictions locked globally before pose values or endpoints
were constructed. No protected split, new source recording or model training.

The [official pinned loader](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/blob/0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284/dataloader/MS2_dataset.py)
supports the inherited sequence-first pose interpretation. Its `set_extrinsics`
function was executed in isolation on each saved calibration and matches the
thermal-from-RGB composition to 1e-12. The [official data format](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format)
specifies depth code/256 in metres and left-camera projected depth. These support
the nominal benchmark convention, not proof of physical exposure or visible-point
identity. No pose direction, scale, time or mounting was fitted from model results.
78 pre-inference analytic/convention checks pass: all 60 intrinsics, extrinsic
composition, identity, translation direction, world-frame conjugation, empty
support, collision choice and half-pixel sampling. All 60 pose matrices later
pass the declared rigidity screen without rotation projection.

The [intake contract](natural-flow-reference-contract-2026-11-13.md) defines the
three tiers. Tier 1 uses positive source depth, front/inbound projection and
nearest-depth target-cell collision removal. Tier 2 requires target depth at the
rounded pixel. Tier 3 also applies the fixed 0.1m+2% target-depth tolerance and
1 native-pixel backward reprojection check. Backward projection uses that rounded
target pixel's own depth; none of these checks certifies correspondence identity.
A pre-execution amendment explicitly separates geometry support from bilinear
model-sampling support at source coordinates `(u+0.5)/2-0.5`. No extrapolation,
depth interpolation or model-based reference selection. Border exclusions are
0/0/0 points for day, 112/20/19 for night and 0/0/0 for rain, for tiers 1/2/3.

## All tiers and recordings

|Record|Tier|Geometry points|Scored points|Supported pairs|Clean mean px|Gamma mean px|Zero-flow mean px|
|---|---|---:|---:|---:|---:|---:|---:|
|day|tier1|135914|135914|19/19|2.650485|2.649316|3.796029|
|day|tier2|21504|21504|19/19|1.753514|1.758694|2.621095|
|day|tier3|16852|16852|19/19|1.659041|1.666194|2.460240|
|night|tier1|176969|176857|19/19|0.039058|0.040456|0.036585|
|night|tier2|106342|106322|19/19|0.038272|0.039709|0.036108|
|night|tier3|101755|101736|19/19|0.038742|0.040260|0.036645|
|rain|tier1|83472|83472|19/19|8.400243|8.392906|10.557888|
|rain|tier2|6917|6917|19/19|6.144947|6.093838|7.891263|
|rain|tier3|3290|3290|19/19|6.973755|6.860850|8.609658|

Values are equal-pair means of per-pair endpoint-displacement disagreement at
128x320 evaluation resolution. All metric entries above have 19/19 valid pairs;
all 57 pair slots remain in each tier. Point totals are support counts, not
independent sample counts. Tier 3 per-pair support ranges day 173–1696, night
4079–6947 and rain 92–259. Reference populations differ substantially across
conditions and tiers; this is not a causal comparison of camera or weather quality.
No single tier was chosen after seeing scores. The fuller per-pair records also
retain RMSE and 1/3-pixel agreement; macro RMSE is the mean of per-pair RMSE,
not a pooled-pixel RMSE.

## Ranking and appearance consistency on tier 3

|Record|Clean alpha vs geometry disagreement|Gamma alpha vs geometry disagreement|Predictive consistency vs clean geometry disagreement|Predictive consistency mean px|
|---|---:|---:|---:|---:|
|day|0.745877|0.735151|0.146979|0.043681|
|night|0.325821|0.349343|0.337397|0.009139|
|rain|0.622472|0.622913|0.413191|0.296827|

Correlations are equal-pair means of within-pair Spearman correlations. The
clean-alpha and consistency columns use the same clean geometry-disagreement
endpoint and support. Every correlation above has 19 valid pairs. Alpha is the
released scalar log-uncertainty output, not a calibrated 2D covariance.

Among gamma predictions with geometry disagreement greater than 1 pixel, the
per-pair fraction also having clean/gamma difference below 0.1 pixel averages
92.527% day and 49.604% rain. Night has no such points: its conditional fraction
is null with 0 valid pairs, not zero. These are **consistent disagreements with a
proxy**, not demonstrated consistently wrong physical correspondences. Appearance
consistency by itself cannot resolve whether a model or this reference is correct.

![All pairs and strict-tier support](../../results/natural-flow-geometry/run-001/natural-flow-geometry.png)

## Independent audit and reproducibility

The independent auditor uses Pillow depth decoding, absolute thermal camera
poses/world-frame point transport, scalar dictionary collision resolution and
SciPy dense-field interpolation. It independently checks all support masks,
rejection counts, endpoint projections, sampled predictions, scalar metrics and
recording macro denominators. It also compares every prepared source/target to
the preceding locked known-warp assay's canonical image. **5,635 checks pass**
over all 57 pairs and 60 pose files. Maximum absolute numeric difference across
all audited arrays/scalars is 6.10e-7, under the declared relative/absolute audit
tolerances (large-coordinate arrays included); no audit retry or scorer change.

Execution freeze: `experiments/natural-flow-geometry-freeze-2026-11-13.json`
(SHA256 `dbee341cb85617db0590cd20c793beb8825aec0cddedf62ba3b06628a14a98a5`),
237 source/input/checkpoint pins. Results and logs:
`results/natural-flow-geometry/`; prediction lock, per-pair geometry arrays,
`scores.json`, independent audit and figure are retained. Source:
`tools/natural_flow_geometry.py`, `tools/qualify_natural_flow_geometry.py`,
`tools/audit_natural_flow_geometry.py`. Runtime inherits
`environment/u2flow-thermal-assay-qualified-requirements.txt`.

Close this fixed diagnostic as completed, without threshold/severity/frame sweeps
or automatic learning. Any subsequent thermal-learning experiment needs a distinct
target and an evaluation source that can distinguish actual matching error from
reference disagreement. No true-natural-flow EPE, physical calibration, VO gain,
MACThermal superiority or new-method novelty is established here.
