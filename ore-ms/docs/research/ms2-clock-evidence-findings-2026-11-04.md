# EXP139: source-clock evidence remains unresolved after a bounded refresh

2026-11-04 America/New_York. Parent EXP138 `91ec26b49`; source/metadata protocol
freeze `ded06878d` precedes the new prefix calculation. No native inference,
scientific reference scoring, clock correction, training or reserve access occurs.

The source refresh does not justify promoting EXP138's identity-clock assumption
to verified physical synchronization. The authors document synchronization hardware
and the loaders document an index-based pairing convention. The inspected sources
still do not define the exposure/device/host event and clock domain represented by
the exported camera timestamp text files. The benchmark remains execution-not-ready.
This is a bounded negative finding, not proof that acquisition code or clock evidence
exists nowhere, nor evidence that the actual exposures were unsynchronized.

## Primary sources refreshed

The [extended paper's sensor-system section](https://arxiv.org/html/2503.22060v1#S3.SS1)
describes external RGB/NIR synchronization, a thermal-left-generated stereo sync
signal and a software trigger starting the two systems. This documents the authors'
acquisition architecture; it does not identify the event stamped into each exported
`img_*_timestamp.txt`, its clock conversion, or per-record residual timing bounds.
The [dataset site](https://sites.google.com/view/multi-spectral-stereo-dataset)
also describes synchronized data, without resolving those export semantics.

The freshly resolved official repository revisions match EXP042:

| Repository | Current resolved revision | Tree blobs | Selected source files | Issues / comments |
|---|---|---:|---:|---:|
| MS2-MultiSpectralStereoDataset | `0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284` | 12 | 2 | 17 / 16 |
| SupDepth4Thermal | `f6725678f9b5dba669ea1f0739506a613f2fb2fb` | 137 | 7 | 3 / 1 |

The [original loader](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/blob/0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284/dataloader/MS2_dataset.py#L181)
and [benchmark loader](https://github.com/UkcheolShin/SupDepth4Thermal/blob/f6725678f9b5dba669ea1f0739506a613f2fb2fb/dataloaders/loaders/MS2_dataset.py#L200)
sort left/right PNG lists and select the same index, without reading timestamp text
or applying a clock correction in these loaders. That is evidence for preserving
the published provided-pair convention, not proof of simultaneous exposure.

Nineteen byte-preserved source snapshots include current commit metadata, complete
nontruncated recursive path lists, all Markdown and loader/relevant-name text files,
all available issue bodies/comments, paper v1 HTML and the dataset home page.
Issue/comment pagination finishes below 100 records; each issue's declared comment
count matches the collected comments. All 20 issues and 17 comments are read.
They concern access, image values, calibration, code releases, IMU availability
or benchmark differences; none inspected resolves the timestamp-writing semantics.

This phase does not rescan every implementation file in the benchmark, third-party
forks, the prior DeepIR tree, or acquisition systems for other datasets. It does not
repeat EXP042's complete-night timestamp analysis. The earlier paper/supplement,
manufacturer and access-reply findings remain historical evidence, not fresh checks.
No email, public issue, comment or author request is sent.

## Original 101-frame prefixes, four timestamp fields each

Freeze original stems `000000–000100` from the same exposed day, night and rain
manifests used by EXP137. Read only metadata: 303 source rows and 1,212 exact integer
nanosecond timestamps. There are no new images, pose-reference values, nearest-time
matches, frame substitutions, offset/rate fits or outcome-based source selections.

The signs below are RGB-right minus RGB-left, thermal-right minus thermal-left,
and thermal-left minus RGB-left. The inherited 2 ms stereo and 30 ms cross-left
limits are recorded-time diagnostics, not measured physical exposure tolerances
or new eligibility gates. Counts retain all 101 source IDs per condition.

| Condition | RGB stereo min / median / max (ms) | Within 2 ms | Thermal stereo min / median / max (ms) | Within 2 ms |
|---|---:|---:|---:|---:|
| Day | −.079329 / .044375 / .061183 | 101/101 | −9.766359 / .169192 / 67.269485 | 33/101 |
| Night | −.025103 / −.014079 / −.002759 | 101/101 | −45.800686 / 20.622077 / 39.421384 | 0/101 |
| Rain | −.086039 / −.074311 / −.058687 | 101/101 | −62.227406 / −2.567759 / 8.513392 | 13/101 |

| Condition | Cross-left min / median / max (ms) | Within 30 ms | Thermal stereo sign changes /100 | Cross-left sign changes /100 |
|---|---:|---:|---:|---:|
| Day | −36.503429 / −26.593531 / 45.701577 | 26/101 | 51 | 91 |
| Night | −66.080466 / 2.080388 / 60.733472 | 94/101 | 84 | 32 |
| Rain | −60.981350 / 1.923666 / 65.044403 | 95/101 | 58 | 25 |

No RGB/thermal left timestamp matches exactly: 0/101 points and 0/100 endpoint
intervals in each condition. Both source cadences are irregular, with original RGB
adjacent intervals spanning approximately 66.65–266.65 ms across these prefixes;
thermal-left intervals span approximately 51.46–205.76 ms. Nominal capture rate does
not justify overwriting the exported time axis with a uniform one.

For any fixed constant shift c, max(abs(d_i−c)) is at least half the observed signed
range. Thermal stereo lower bounds are day38.517922, night42.611035, rain35.370399 ms;
cross-left lower bounds are day41.102503, night63.406969, rain63.0128765 ms. Thus a
single additive shift cannot place every recorded pair within the respective
inherited diagnostic bound in these prefixes. No shift is fitted or applied.
These are bounds on recorded metadata, not estimates of exposure delay, proof of
clock drift, or grounds for concluding the hardware synchronization failed.

This uncertainty matters for the stereo baseline inputs themselves as well as
cross-camera temporal transport. Finite ORB/DROID output and baseline scaling do
not establish correct simultaneous stereo geometry for these exports. The supplied
same-ID experiment protocol can still be reported explicitly as such; its physical
interpretation cannot be strengthened by suppressing these discrepancies.

## Concrete evidence needed next

The saved evidence ledger separates documented conventions from unresolved claims:

| Question | Current evidence | Needed to strengthen the claim |
|---|---|---|
| Which event is exported? | Unresolved | Timestamp-writing acquisition/export implementation or author definition for all four camera files |
| Which clocks are used? | Unresolved | Device/host domain, epoch, units, conversions and configuration |
| How do RGB and thermal clocks correspond? | EXP138 uses an explicit assumption | Applicable per-record mapping and measured bounds from acquisition evidence |
| Do same-ID stereo frames share an exposure/trigger? | Hardware architecture and same-index loader documented | Export-to-trigger/frame correspondence and explanation of recorded discrepancies |
| Is the physical common frame correct? | Provided numeric convention qualified in EXP135 | Independent calibration evidence; not resolved by time transport |
| Are DROID poses available causally? | Current outputs are offline backend/filler results | Per-frame output receipts/failure status from an appropriately instrumented causal path |

A public-source refresh cannot manufacture these missing acquisition facts. Do not
repeat this same search as the next experiment without a new source. Useful further
engineering can use a separately declared provided-pair protocol, while preserving
these limitations; it must not silently amend the frozen physical-time benchmark.
Full-recording runtime/failure qualification remains separate unfinished work.

Prepared question for the data owner, **not sent**:

> For the MS² day `_2021-08-06-11-23-45`, night `_2021-08-13-21-18-04` and rain
> `_2021-08-06-16-19-00` exports, what event and clock does each RGB/thermal
> `img_left_timestamp.txt` and `img_right_timestamp.txt` represent? Which acquisition
> and export code/configuration maps device, message and host times to these files?
> How are same-ID stereo frames tied to common exposures/triggers, and what mapping
> and residual timing bounds relate RGB and thermal across each recording?

## Validation and recovery

A separate standard-library scalar auditor passes 2,698 checks over original source
identities, CSV integer equality, offsets, exact medians, extrema, sign changes,
cadence, equal-endpoint counts, Fraction-based shift bounds and public snapshot
hashes. Six copied-output mutations (timestamp, ordering, offset, source count,
median and false clock verification) are rejected. All 28 EXP139 pins and all 23 parent EXP138 pins are unchanged. No existing
publisher, native adapter, saved estimate, benchmark protocol, test/validation
assignment, reserve or default is modified. No GPU or new model inference is used.

Protocol: `experiments/ms2-clock-evidence-2026-11-04.json`.
Sources: `results/clock-evidence/source-001/{registry,evidence-ledger}.json`.
Metadata/audit: `results/clock-evidence/run-001`.
Recovery: `backups/2026-11-04-ms2-clock-evidence-exp139`.
The source/metadata replay needs Python 3.11+ only, not images, weights or Docker.
