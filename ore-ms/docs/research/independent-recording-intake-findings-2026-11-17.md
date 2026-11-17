# Three new-recording candidates screened; independent paired-pose gate remains unmet

17 November 2026 America/New_York. Completed the
[bounded intake protocol](independent-recording-intake-protocol-2026-11-17.md),
following the [baseline decision](baseline-recommendation-2026-11-17.md).
This is source and actual-payload qualification, not a model/pose experiment.
Branch `ore-ms-orb-development`, committed base `4408b7fb8` plus pinned local sources.

## Decision

**None of the three screened candidates qualifies for independent RGB–thermal
pose validation.** BlockVisual is a new paired-input candidate with no qualified
reference here. LongCorridor and MultiFloor have supplied pose files but no
thermal stream in their complete extracted ZIP member inventories. Keep those
findings distinct from transport success and from a global reference-absence claim.
Do not run a model or relabel existing exposed records as independent validation.

| Candidate | Listed RGB frames | Listed thermal frames | Listed LiDAR frames | Supplied reference rows examined | Readiness |
|---|---:|---:|---:|---:|---|
| BlockVisual | 8,624 | 15,512 | 0 | None in inspected extension inventory | Bounded paired-input candidate only |
| LongCorridor | 6,720 | 0 | 2,777 LAS | 1,385 | Cannot test paired thermal comparison |
| MultiFloor | 10,017 | 0 | 4,139 LAS | 2,061 | Cannot test paired thermal comparison |

These sequences were selected before payload reads from the remaining unexposed
extension candidates, not by matcher/error results. Local experiment/doc search
found no prior model-outcome study for these exact names. This is novelty to the
current experiment series, not certified exclusion from checkpoint training.
Primary-release metadata was also read as context; its archives were not sampled.

## Actual input and reference checks

The current official [sequence table](https://superodometry.com/datasets) and
public [extracted extension folder](https://drive.google.com/drive/folders/1E4EFVY-Bnef7sSBprnfL5Z49HAc89dLr)
were read through the Google Drive connector. All three complete ZIP central
directories were then accessed using the existing strict `HTTPRangeFile` helper:

| Candidate | Exact archive bytes | Directory members | Directory/text transport bytes |
|---|---:|---:|---:|
| BlockVisual | 8,770,779,847 | 24,143 | 2,790,250 |
| LongCorridor | 3,888,497,930 | 9,504 | 1,053,776 |
| MultiFloor | 5,578,358,447 | 14,163 | 1,588,541 |

Every HTTP range enforces206status, exact Content-Range/length/total and a receipt
SHA. Each retrieved timestamp list passes independent SHA/CRC, strict monotonicity,
full stream count and consecutive PNG/LAS ordinal checks. Complete directories
establish listed stream absence/presence, not full compressed-archive integrity
or decoding of every image. Each candidate stays below16MiB transport budget.

Four prespecified BlockVisual endpoint PNGs (first/last of each camera) pass ZIP
CRC, SHA, PNG header and Pillow verification: RGB640×480/8bit and thermal640×512/
16bit. Full timestamps span about359.26sRGB and359.17sthermal with overlapping
nominal support. Samples do not certify whole-span decoding, time calibration,
blocked-interval severity, stop/start coverage or a calibrated no-light condition.
No own-camera projection or whole-span matching qualification is claimed.

The two small reference ZIPs were actually fetched through Drive: MultiFloor
1,243,727bytes; LongCorridor124,939bytes. All five non-directory members pass CRC
and SHA verification. Their CSVs have timestamp, world/body position `p_w_b_*`
and quaternion `q_w_b_*` columns. Rows are finite and strictly time ordered;
maximum quaternion norm deviations are6.83e-7and6.50e-7respectively. Their nominal
camera/reference spans overlap, but no physical body-to-camera/clock/interpolation
contract is inferred from overlap or headers. Reference maximum gaps are about
403.44msMultiFloor and201.72msLongCorridor; no old high-rate reference gate is
silently applied or relaxed. Neither ZIP supplies the missing thermal stream.

The cached extension-reference inventory used by the protocol contains four
entries (SmokeRoom, FlashLight, MultiFloor, LongCorridor), with no BlockVisual.
This is scoped to that preserved inventory, not a claim that BlockVisual truth
cannot exist elsewhere. Current maintainer
[issue9](https://github.com/yuanjun-gao/ICCV2023_SLAM_Challenge/issues/9)
confirms the older duration descriptions were wrong and BlockVisual should have
no LiDAR; the extracted member inventory agrees. Issue8's old missing-bag reply
is retained but does not override the now-present public raw archive. Issue36
concerns **BlockLiDAR**, not a BlockVisual reference, and cannot be substituted.
No closed OutdoorNight/Gascola reference search was restarted.

## Why a new recording is not independent truth

Fresh targeted reading of the current
[SubT-MRS v5 paper, section3.2](https://arxiv.org/html/2307.07607v5)
identifies reference trajectories as a fusion of map, visual-odometry, LiDAR and
IMU constraints. The older v2 section5.3 also describes visual constraints for
geometrically degraded cases. A surveyed map and supplied trajectory therefore
do not by themselves certify sensor-independent camera/IMU pose truth for these
files. This is a dependence qualification, not evidence that supplied trajectories
are numerically wrong or unusable as nominal benchmarks.

The current project page, v5 publication metadata, targeted ground-truth sections,
folder receipts and relevant maintainer comments are saved. This is fresh targeted
source/paper reading, not a full-paper review or exact reference-generator source
reproduction. Prior nominal SmokeRoom/FlashLight results remain as labelled;
OutdoorNight remains its separate nominal gyro tier. None is promoted to an
independent physical-validation population by this intake.

## Verification and retained failure

The on-disk consistency checker passes92checks: frozen source/selection identities,
range bounds/budgets, complete timestamp/member ordinals, fetched-member hashes/
CRCs, native PNG schemas and reference numeric/time structure. This is a passed
transport/schema audit alongside a **failed readiness gate**, not a full independent
pose qualification. One initial checker failure is retained: its LiDAR suffix
filter omitted the release's `.las` files, causing timestamp/count mismatch.
The original checker/log are preserved; the corrected checker recognizes LAS.
No input, budget, frame/reference criterion or model outcome was changed.

Evidence is ignored under `results/independent-recording-intake/`, including
selected text/PNG members, complete small reference ZIPs, all inventory/range/
member receipts, source snapshots, payload locks, failed and corrected checks,
and separate per-candidate readiness fields. No full sensor ZIP, raw bag, model
call, fitted clock/axes/bias, pose score, outreach, private-data request or upload.
A member-readback-verified packet is under
`/root/ore-ms-artifacts/independent-recording-intake-2026-11-17/`.

## Next boundary

Close this three-candidate screen. BlockVisual could support a separate input-only
or nominal-gyro study after full raw/export/calibration qualification, but that
would not satisfy the independent-pose objective and is not authorized by this
intake's outcome. A next independent-validation attempt needs a distinct paired
multispectral release with an explicit external reference/frame chain, or a
separately qualified reference-generation procedure excluding the evaluated
camera stream. Preserve dependence on any shared IMU explicitly. Do not repair
this gate by choosing a favourable crop/window, fitting axes/clocks, borrowing
calibration or treating a reference filename as proof.

A concrete next intake is camera-independent LiDAR-reference feasibility for the
already identified M2P2 `r2c2_night_1` recording: inspect reference-generation
source, calibration and shared-sensor dependencies before downloading its sensor
payload or running an estimator. The earlier catalog check found no ready
trajectory; this would be a new generation-contract assessment, not a repeat of
that listing or a claim that a LiDAR-derived trajectory is already qualified.
