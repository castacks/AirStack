# EXP142: camera-local full-record provided-reference evaluation

Date: 2026-11-05 America/New_York. Parent EXP141 / 79339e882.

Evaluate the six existing latest declared EXP141 DROID outputs on the complete
three development recordings. Selection is inherited from EXP141's capacity and
layout declarations, before reference outcomes. Preserve original failures and
their complete source populations. No new inference, fitting, reserve access or
EXP133 benchmark promotion.

Use each modality's own left-camera recorded nanosecond times and its exact
same-stem `odom/<sequence>/<rgb|thr>/<stem>.txt` provided reference. Interpret the
absolute matrices as camera-to-initial under the official loader's sequence-first
image convention. Do not mix clocks or cameras, interpolate, substitute stems,
choose pose direction from scores, or fit scale/time/extrinsics. This is a provided
convention comparison, not verified physical synchronization or independent GT.
The official loader pairs poses by image stem and returns absolute sequence-first
poses: https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/blob/0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284/dataloader/MS2_dataset.py#L291
The dataset describes navigation-derived left-camera references:
https://sites.google.com/view/multi-spectral-stereo-dataset/data-format

Freeze all prediction, manifest, canonical-plan, execution, scorer, study, auditor
and test hashes before reading reference matrix values. The historical EXP040
recovery packet was downloaded during source discovery before this freeze; it
contains previously development-exposed odometry archives. No reference members
were opened then. This is not prospective reference-payload acquisition or heldout
evaluation. Archive and member hashes are pinned using its saved manifest before
extraction; only RGB/thermal members of these three development records may enter
the new reference namespace. Preserve the full source axis, including missing or
nonrigid references as explicit exclusions. Never discard estimator slots.

Use the unchanged EXP133 v2 scorer: SE3-only ATE without scale, origin-aligned ATE,
approximately 1-second RPE (endpoint 1–1.15 seconds; no intervening gaps >150 ms),
source/pose/reference/RPE coverage, segment/reset/terminal status, and its explicit
engineering-success definition. Existing rigidity tolerance is 1e-5, with only
proper-SO3 roundoff projection. Degenerate SE3 alignment remains null. Report all
six streams and separate equal-recording means per modality, without choosing a
winner or retuning from the results. Reference invalidity, cadence gaps and runtime
failure remain visible alongside scores. No sensor/compute-matched adaptive claim.

Validation: analytic adapter and scorer controls before reference values; independent
raw-reference Kabsch/origin and relative-motion recomputation; exact population,
timestamp and source-ID bindings; semantic mutations of outputs, reference axis,
scale and false physical-time claims must fail. Predictions and scorer hashes must
remain unchanged after scoring. Source and reference SHA manifests accompany all
results. Preserve the original EXP133 gates, empty active validation/test partitions
and two reserves.
