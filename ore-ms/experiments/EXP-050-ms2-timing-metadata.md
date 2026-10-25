# EXP-050: Recorded timing metadata and the scope of benchmark evaluation

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `55bd3c4fb`.
**Exact exposure-time semantics are not a prerequisite for every current experiment.** Follow MS²'s documented provided-pair protocol for benchmark-reference evaluation; retain timing discrepancies as diagnostics and qualifications. The inherited2ms stereo/30ms cross gate is not a validated universal MS² acceptance criterion and should not veto all benchmark work or make every experiment wait for an author reply. This clarifies scope; no existing gate flag, pairing, score, source file or default changes.

## What timing is needed for

- Current per-modality provided-pair stereo/reference diagnostics can proceed under the published benchmark contract. We are not yet comparing RGB and thermal predictions at verified identical physical points/times, so exact RGB–thermal simultaneity is not a prerequisite for each separate camera's descriptive result.
- Precise motion-sensitive physical correspondence/pose claims need sufficiently justified image/reference alignment for their claimed accuracy. Alignment may include documented motion compensation; sensor acquisition times need not all be identical. It is not necessary to demand perfect timestamps before reporting benchmark results with their assumptions.
- Static scene geometry substantially reduces motion-induced timing ambiguity, making CATS a complementary reference check when access becomes available. Static scenes still require appropriate calibration, visibility and labels.

The official [MS² setup](https://sites.google.com/view/multi-spectral-stereo-dataset/setup) documents an external synchronizer for RGB/NIR and thermal stereo synchronization from the left thermal camera. It also describes estimating each sensor pose according to its timestamp and reference-cloud processing. Its intended acquisition/pairing protocol supports benchmark use; it does not define what every exported timestamp marks. A recorded time difference is not automatically a measured exposure delay or uncompensated reference error. The [CATS setup](https://bigdatavision.org/CAT/setup.html) documents software-triggered cameras and static scenes for slow laser scanning. Source HTML/URL/hash/date snapshots are preserved.

Illustratively,5m/s×20ms=0.1m of motion; projected error additionally depends on direction, rotation, depth and calibration. This is not a measured error bound for our data, because actual exposure delay and physical motion uncertainty remain unresolved.

## Fixed metadata sensitivity check

Before inspecting these associations, freeze four existing20-ID windows: night opening, day opening, day middle1936–1955 and day late3873–3892. Keep single-scan primary, merged/filtered sensitivities and own/common support unchanged. No new inference, acquisition, clock fit, re-pairing, gate threshold or sample substitution.

Primary predictor is absolute recorded right-minus-left stereo timestamp difference for the scored camera modality; primary endpoint is the image's median normalized L2 provided-reference disagreement. Secondary predictors are camera-left versus same-index left/right LiDAR packet time differences, separately. Those packet times are context metadata: the contributing points' scan times, motion compensation and exact label association are not established here. We do not equate a packet-time offset with actual depth-label timing error.

Secondary endpoints are per-image median normalized absolute vertical residual, covariance-trace median and within-image uncertainty/error rho. Cross-modal left-camera separation is retained descriptively, not used as a causal predictor for each modality's stereo residual. Preserve20 per-image scalars per group; no pixel pooling, cross-window pooling, p-values, causal claims or outcome-selected thresholds.

This yields960 endpoint rows (repeated depth/support views of160 image/modality cases) and576 descriptive correlations:8 predeclared primary correlations plus568 secondary/sensitivity associations. The repeated variants/supports are not additional independent observations. All endpoints and timestamp associations remain available; missing/constant fields would be reported explicitly rather than repaired.

## Primary single-scan own-support associations

Each coefficient is Spearman over20 adjacent image summaries. The metadata need not represent exposures; large rank correlation does not establish a material physical effect, especially for microsecond-scale RGB offsets. All results are exploratory on previously inspected recordings.

|Window / modality|Recorded stereo offset min / median / max (ms)|Stereo-offset versus L2 rho|LiDAR-left packet offset versus L2 rho|LiDAR-right packet offset versus L2 rho|
|---|---:|---:|---:|---:|
|night-opening / RGB|0.002759 / 0.008755 / 0.009191|0.090|0.086|0.036|
|night-opening / Thermal|22.888212 / 34.611671 / 45.470387|0.481|-0.191|-0.313|
|day-opening / RGB|0.050135 / 0.055551 / 0.061183|0.119|-0.150|-0.012|
|day-opening / Thermal|0.169192 / 4.585782 / 9.766359|-0.257|0.238|0.084|
|day-middle / RGB|0.009518 / 0.060766 / 0.088370|-0.099|0.170|0.480|
|day-middle / Thermal|0.030618 / 5.320448 / 63.965475|-0.018|0.074|0.006|
|day-late / RGB|0.014347 / 0.023119 / 0.031643|-0.583|0.182|-0.141|
|day-late / Thermal|0.053103 / 1.840407 / 6.421360|-0.307|-0.068|0.561|

The primary stereo-offset associations change sign/magnitude across windows. Thermal is+0.481 at night and−0.257/−0.018/−0.307 across the day windows. RGB includes−0.583 in the late window despite offsets only0.014–0.032ms. This is not a consistent monotonic physical-delay/error relationship. Scene evolution, scheduling, reference labels and serial dependence remain possible confounders; no causal explanation is established.

A null association cannot prove exposure timing harmless, and a positive/negative association cannot establish what timestamp events mean. The decision to continue provided-pair benchmarking comes from task scope and documented dataset intent, not from accepting/rejecting a threshold based on these correlations. LiDAR packet associations are also mixed and do not validate or invalidate the reference motion-compensation pipeline. All576 outputs remain in the report; no preferred offset/endpoint is selected.

![Recorded offsets versus per-image reference disagreement](../results/ms2-timing-metadata/run-001/primary-associations.png)

Standalone PNG/PDF plots preserve all20 points per primary panel with no fitted lines. Axis labels deliberately say recorded time, not exposure delay. Exact source IDs,all4 camera times and packet-time values remain in the endpoint records.

## Validation and standalone replay

Independent scalar validation checks original source-ID/time joins, integer differences, endpoint extraction, tied ranks, quantiles, availability and complete960-row/576-correlation coverage. All18,916 controls pass,33 frozen inputs remain unchanged and1,902 historical source/result hashes pass. No numerical results from EXP046–049 change.

An isolated CPU replay mount containing only the scalar-report/manifests/timestamps/code and this analysis passes17,014 checks with `--skip-historical`. It contains no raw images, prediction arrays, weights, GPU computation or old archive snapshots. The flag explicitly omits historical-payload preservation checks and does not replace the prior array validations. Full validation remains the recorded authoritative preservation audit.

From `MAC-VO-main`, in the existing smoke image with AirStack mounted at `/root/AirStack`:

```bash
python -m Scripts.Setup.ms2_timing_metadata \
 --root /root/AirStack/ore-ms --output /path/to/fresh-run
python -m Scripts.Setup.validate_ms2_timing_metadata \
 --root /root/AirStack/ore-ms --run /path/to/fresh-run \
 --output /path/to/fresh-validation.json
```

Full historical checks require EXP043,EXP044-complete,EXP046,EXP047,EXP048,EXP049 manifests under `/root/ore-ms-recovery` and their restored payloads. For the tested input-only replay, explicitly add `--skip-historical`. All paths must be fresh. CPU NumPy/SciPy in the existing smoke runtime; independent tied ranks/quantiles reuse the earlier pure scalar validator helpers. Startup driver diagnostics remain in logs; no model or GPU is used.

Artifacts `results/ms2-timing-metadata/` include960 joined rows,all576 associations,timing summaries,frozen registry,both validation reports,plots and source snapshots. Box target `backups/2026-10-25-exp-050` includes all33 scalar/timestamp input files needed for standalone replay plus code/notes/figures and incremental Git bundle after EXP049. Models/full images/archives/packages/tokens are excluded. Previous recoveries remain necessary only for the complete historical preservation check.

## Practical decision

Continue uncertainty/error benchmarking under documented provided pairing rather than waiting for perfect timing documentation. Retain the exported timestamp discrepancy and all old gate flags, and scope claims to provided references. More precise physical metrology, temporal/VO interpretation and same-time cross-modal comparisons require alignment assumptions appropriate to the claim; they do not require every experiment to stop until those questions are answered. No source or calibration correction is justified by this metadata association check.

CATS/source replies remain useful parallel evidence. This analysis neither makes them unnecessary nor turns their current absence into a universal project blocker. No additional data upload is needed for this check.
