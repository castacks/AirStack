# Rig/reference contract and prospective recording protocol

2026-10-30, America/New_York. Branch `ore_ms`, base `2ea470272`.
The [readiness audit](evaluation-readiness-2026-10-30.md) identified missing
physical and prospective-evaluation evidence. This follow-up reconstructs the
nominal rig contract from its exact archived sources and specifies the next
evaluation requirements. It does not declare a gate ready for training or assign
unseen recordings to a test set.

## Reconstructed source contract

The supplied `data/calibration/yamaha.yaml` was restored from the existing EXP075
recovery archive, verified against its manifest and pinned member SHA256
`031c199619916bf1466a807d9ed91896d58b1428f0bdf74951dabc3c1b59e27d`.
No alternate calibration was downloaded or substituted. The graph uses the
existing interpretation `T_parent_from_child`, with quaternion order xyzw.
Comments saying “frame to transform from” do not override the explicitly audited
matrix convention. Historical reversed export labels are documented in
[calibration notes](../calibration.md).

| Quantity | Exact supplied path / interpretation | Evidence boundary |
|---|---|---|
| RGB optical anchor | `vehicle -> multisense/head -> multisense/left_camera_optical_frame` | Nominal static chain; not a newly measured mounting. |
| Thermal optical anchor | `vehicle -> thermal_left/optical_frame` | Supplied anchor retained even with MSA stereo preprocessing. |
| Rectified native camera basis | Optical chain multiplied by `R_rect.T * OPTICAL_FROM_NED` | Camera-info R and native basis belong to each prepared stream. Raw optical NED TF alone omits rectification. |
| NovAtel IMU | `vehicle -> novatel/imu_frame`; nominal translation `[−0.508, −0.05, −0.152]`m, identity rotation | Describes the named IMU frame; no equivalence to GNSS antenna or solution origin follows. |
| GNSS solution origin | Export metadata names `gps_frame` | No such node/path in this supplied YAML; absence is limited to this graph. |
| Pose reference | Interpolated SO `sensor_init -> vehicle` | Recorded SLAM proxy; reference-assisted interval bridges remain offline evaluation operations. |

The new `Scripts/Setup/rig_reference_contract.py` independently composes source
graph paths and the rectification/basis factors, then compares against all434
original-arm saved matrix rows from EXP084–086. All six camera/window matrices
match exactly (maximum absolute element difference0). The inherited bounded
nearest-SO(3) rectification policy changes a serialized element by at most
`2.6584852208522136e-8`; this is the existing serialization-roundoff handling,
not a new calibration fit. Quaternions use the existing SciPy normalization.
Source/protocol/evaluation/proof ancestry and18 distinct input hashes are checked
before/after. Two CPU Docker runs produce byte-identical contract JSON.

Artifacts: `results/rig-reference-contract/run-001/{contract,repeat}.json`.
The output records every original YAML edge, selected graph path, rectification
source, numerical discrepancy and unresolved provenance flag. It confirms source
consistency, not physical frame accuracy. No new pose loss is computed; no
production geometry is changed. EXP086 numerical-audit qualifications remain
those recorded in the preceding readiness audit and original experiment.

## Evidence still needed to complete the contract

| Field | Current source-backed status | Smallest useful additional evidence |
|---|---|---|
| Applicable camera/body mounting | Supplied nominal anchors; owner supports MSA stereo applicability | Recording configuration/TF snapshot or owner mapping identifying applicable body/optical anchors and any changed mounting. Stereo calibration applicability alone does not define the body anchor. |
| GNSS axes/orientation | Owner reports UTM with approximate X/Y flip | Explicit signed axis mapping, handedness, zone and orientation convention; no fitted swap from VO scores. |
| GNSS physical origin | `gps_frame`, separate named IMU transform | Receiver solution-origin/lever-arm setup and applicable body transform, including whether already compensated. |
| Reference clock/latency | Export timestamps; measured camera offsets | Actual publisher timestamp code/config and acquisition synchronization description. |
| Reference quality/dependencies | Owner reports direct GNSS output, jitter, RTK on some runs; SO non-use tentative | Recording-specific solution/fix/covariance status if saved, and SO/receiver aiding inputs. Missing logs stay missing. |
| Checkpoint training overlap | Unknown for both evaluated recordings | Exact checkpoint recording/split list and preprocessing/finetuning recipe; matching image size is insufficient. |

The [owner-reply record](source-replies-2026-10-22.md) remains the attribution
source. No new owner statement has been inferred from numeric reconstruction.

## Prospective evaluation specification

This is a **design specification**, not a completed executable freeze. Future
recording IDs, a distinct candidate method, online interval policy and numeric
deadline remain unassigned. No new evaluation recording may be described as
prospectively tested under this document until those fields and all executable
sources/parameters are frozen before accessing its scientific payload.

1. **Partition unit and development ledger.** Treat the complete recordings
   `20251027/night_mac_salon_warehousefig8` and `20251027/0_salon_so_day` as
   development-exposed, including their unprocessed portions. Treat other already
   inspected project recordings according to their existing access ledger; this
   two-recording list is not an exhaustive project holdout ledger. Keep adjacent
   frames, clips and modalities of any recording in one partition. Allocate
   future development, validation and test recording IDs by metadata before
   content/label inspection. A recording used to design or debug the method
   becomes development. Keep checkpoint-training membership separately labelled
   known/unknown; unseen by this project does not imply unseen by checkpoint.
2. **Target and camera output.** Output a body-frame relative motion on one
   declared interval with provenance: chosen expert, original sensor endpoints,
   result-availability time and any sensor-causal transport. Target translation
   loss is `||translation(inv(D_reference) * D_predicted)||` in metres; rotation
   geodesic in degrees is secondary and reported separately. An independent
   applicable reference is required for a physical-accuracy claim. If only SO
   is available, retain the explicit proxy scope. Never pass the reference
   endpoint bridges to the deployed predictor or output adapter.
3. **Asynchronous output and deadline.** Before test access, specify the output
   clock, acquisition-to-output timestamp relation, numeric deadline, permitted
   buffering/transport inputs, stale-result rule and abstention behavior. Source
   result times must precede the corresponding decision deadline. Thermal
   intervals with differing endpoints are not silently relabelled as RGB
   intervals. A reference-free transport implementation must be independently
   exercised on development data before being frozen. Processing-only selection
   and a deadline measured in separate expert-process clocks are insufficient.
4. **Candidate and fitting freeze.** Declare a falsifiable distinction from the
   failed uncertainty/geometry/solver/closure controls, exact inputs, model,
   training loss and all hyperparameters. Restrict fits and selection to
   development/validation recordings. Freeze checkpoint hashes, calibration,
   preprocessing, source/image IDs, seeds, training membership, features and
   all parameters before test payload access. No candidate is selected here.
5. **Baselines and availability.** Include fixed RGB, fixed thermal and a
   deterministic availability fallback (RGB preference, thermal only when RGB
   unavailable), under the same interval/output/deadline adapter. Keep the
   original camera estimates fixed when isolating a gate. Label-assisted oracle
   is only a separate bound. Publish missing, unsupported, invalid and late
   counts, with exact interval IDs. Report both the union coverage and exact
   matched comparisons. A method may not improve its headline error merely by
   discarding harder intervals; an error comparison never substitutes for a
   coverage comparison.
6. **Aggregation and decision.** Report each test recording separately, both
   camera baselines, fallback and candidate on exact matched intervals; include
   per-recording RMSE/coverage and latency distributions. Use an equal-recording
   macro-average for the summary, rather than letting long clips dominate.
   For a claim of consistent primary improvement, require lower translation
   RMSE than each predeclared fixed-camera and fallback baseline on every test
   recording's matched population, alongside candidate coverage at least that
   of fallback on the full declared population. Publish failures, ties and empty
   cohorts; empty comparisons cannot pass. Rotation and timing regressions stay
   visible and prevent an unqualified overall-improvement claim. A weaker
   outcome may support a narrower descriptive claim, not retroactive relaxation
   of the criterion. With few recordings, report descriptive outcomes without
   claiming population statistical significance.
7. **Access and execution record.** Save the full declaration, input/recording
   ledger, code/config/image hashes and immutable method artifacts before first
   test access. Log access, failures and all predeclared outcomes. Debugging on
   a test recording consumes its prospective status for subsequent candidates.
   Preserve original declarations and publish amendments with timing. Reserved
   MS² captures and additional CATS scenes remain unopened by this step; static
   CATS disparity evaluation cannot validate temporal pose selection.

## Focused source request — draft only

The following is ready to copy into existing owner discussions; no message was
sent and no remote-account action is required for this source reconstruction.

> For the October27,2025 ATV recordings `night_mac_salon_warehousefig8` and
> `0_salon_so_day`, could you identify the applicable camera-to-vehicle TF/config,
> and what physical origin `gps_frame` represents? We have a separate nominal
> `vehicle -> novatel/imu_frame` transform and will not assume they are equal.
> For the GNSS export, an explicit axis/orientation mapping, timestamp assignment,
> receiver lever-arm setup and which of these recordings had RTK would resolve
> the main reference questions. If saved, the relevant solution/fix/covariance
> logs and receiver aiding/SO input configuration would help. For
> `MACT_Offroad.pth`, could you provide the training/finetuning recording list and
> preprocessing recipe, including whether either October27 recording was used?
> A repository/config pointer or explanation is useful even if logs are absent.

## Reproduction and recovery

From `MAC-VO-main` inside a CPU container with ore-ms mounted at `/workspace`:

```bash
python -m Scripts.Setup.rig_reference_contract --root /workspace --output /workspace/results/rig-reference-contract/new-run/contract.json
```

Only previously inspected metadata/calibration and saved reports are read. No
inference, label fitting, new recording access or setting change. Recovery:
`ore_ms:ore-ms/backups/2026-10-30-rig-reference-contract`; incremental Git base
`2ea470272`, with EXP075/084–086 source/data context as full-reconstruction
dependencies. The recoverable nominal source contract is complete; missing
physical provenance and the future method/test assignment are explicitly pending.
