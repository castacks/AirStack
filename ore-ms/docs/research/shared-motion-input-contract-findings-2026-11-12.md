# Bounded real-input decision for shared RGB–thermal motion

12 November 2026. Branch `ore-ms-orb-development`. Metadata calculation frozen
at `8cb397ee9`. First20 already-exposed groups000000–000019 of
MS² night `_2021-08-13-21-18-04`; exactly19 adjacent intervals. No images,
feature arrays, prediction scores or pose/depth reference values opened.

**Decision: STOP_THIS_STRICT_SHARED_INTERVAL_CONFIGURATION.** Nominal spatial
geometry is compatible, but no selected RGB/thermal recorded interval has equal
endpoints. The existing adapter explicitly leaves physical clock semantics
unverified. This capture cannot be passed unchanged into the current synthetic
prototype's exact-common-interval interface. No clock fitting, nearest-time
pairing, data acquisition, alternate recording, trajectory-model implementation
or training follows this check.

This is a compatibility decision, not a new synchronization discovery. The
[existing EXP139 clock review](ms2-clock-evidence-findings-2026-11-04.md) already
reported the issue on first101 groups in three exposed recordings. We checked
one concrete prefix against the new prototype and did not repeat that search.

## What is available and compatible

The paired manifest's source SHA256 matches the100-group archived source
manifest; all selected source stems, paths and80 integer nanosecond timestamps
agree across the two representations. All four20-element timestamp streams are
strictly increasing. All80 referenced image files exist locally; this check
neither decodes nor rehashes their pixels. Metadata availability is not a fresh
pixel/feature parity qualification.

Across the20 rows, each camera's native K and positive baseline match the
existing calibration JSON. Rectified stereo right transforms have identity
rotation, negative horizontal translation and zero vertical/axial translation.

| Camera | Native image size | Focal length fx=fy, px | Baseline, m |
|---|---:|---:|---:|
| RGB | 1224×384 | 764.5138705813054 | 0.29918420530585865 |
| Thermal | 640×256 | 387.78695052062903 | 0.3038995142414822 |

Using RGB-left optical coordinates as a nominal body anchor, compose the supplied
NIR-to-camera calibration:
`R_thermal_from_rgb = R_nir2thr R_nir2rgb^T`,
`t_thermal_from_rgb = t_nir2thr - R_thermal_from_rgb t_nir2rgb`.
Translations are converted from millimetres to metres exactly once. The composed
translation is approximately `[0.0114624, 0.3297751, 0.5682268]`m; rotation
determinant1.0 and maximum orthogonality error2.22e-16. These checks establish
nominal source consistency, not independently measured physical mounting.

The native loader uses camera-local identity `T_BS`; that identity does not
encode the cross-camera rig transform. It also uses native NED conventions,
whereas the synthetic prototype uses optical XYZ. A future binding must
explicitly convert camera coordinates and apply the rig transform; concatenating
native point arrays or treating both identity anchors as one body is invalid.
Binding feature arrays was deferred because the temporal admission already fails.

## Recorded-time evidence

No timestamps were converted to floating point before endpoint comparisons.
Offsets are signed thermal-left minus RGB-left and right minus left. Values
below summarize exported metadata; they are not physical exposure error bounds.

| Diagnostic | Minimum, ms | Median, ms | Maximum, ms |
|---|---:|---:|---:|
| Cross-camera left offset | -11.756187 | -2.807753 | 60.563806 |
| RGB stereo offset | -0.009191 | -0.008755 | -0.002759 |
| Thermal stereo offset | -45.470387 | -32.585796 | 37.557153 |
| Thermal-left minus RGB-left interval duration | -53.156908 | 0.043112 | 63.427183 |

Exact shared RGB/thermal left endpoint intervals: **0/19**. Every original
interval and offset is retained in the JSON report. No thresholds were tuned,
intervals removed, offsets estimated or source IDs substituted. The paired
manifest's `physical_frame_or_clock_verified` flag remains false.

The [official data-format page](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format)
describes rectified, synchronized streams and supplies calibration conventions.
The [setup page](https://sites.google.com/view/multi-spectral-stereo-dataset/setup)
describes external RGB/NIR synchronization and thermal-left-generated stereo
synchronization. These support the published supplied-pair benchmark convention.
They do not newly establish what event each exported timestamp represents or
supply a capture-specific residual timing bound. Recorded-time disagreement
does not prove that the hardware exposures were unsynchronized.

## Scope decision: avoid a timing detour

Exact timestamp equality is **our current prototype's interface restriction**,
not an author-declared requirement for every MS² benchmark study. This finding
does not invalidate the dataset, its supplied-pair evaluations or previously
reported camera-local results. It does not rule out multi-sensor VO research.

Close this direct, strict recorded-interval binding. Do not repeatedly audit
clocks or acquire new payload to rescue it. Retain the successful numerical
prototype and nominal spatial contract as reusable work.

A separate, scientifically narrower experiment could explicitly adopt the
authors' supplied same-ID pairs as **nominal simultaneous benchmark groups**,
preserve all actual timestamp fields and report that modeling assumption.
It must not advertise verified physical simultaneity, overwrite clocks or
silently bypass the current guard. Its first endpoint should be one fixed-weight
joint-estimation comparison against RGB-only before any learned head. Such an
experiment needs its own declared input adapter and controls; it has not been
executed or promoted by this failed direct-binding gate. A timing-aware physical
estimator would be a larger method change and is not selected here.

## Verification and retained artifacts

The metadata script completes on its first execution. Assertions check source
hash ancestry, fixed stems, duplicate timestamp/path representations, monotone
streams, native K/baseline identity and rectified stereo conventions. Source and
input hashes are unchanged after the run. No new unit tests mirror these checks;
no estimator, model or production data loader changed.

Protocol: `experiments/shared-motion-input-contract-freeze-2026-11-12.json`.
Report: `results/shared-motion-input-contract/run-001/report.json`.
The report retains all20 rows,19 intervals, nominal rig transform and80 file
availability entries. This bounded check supplies no training, calibration,
latency, held-out generalization or VO accuracy claim.
