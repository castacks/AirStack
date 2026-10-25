# MS project: RGB–thermal geometric uncertainty

Study whether RGB and thermal uncertainty estimates are comparable and predict
which modality supplies more reliable geometric measurements. The first stage
is baseline reproduction and uncertainty/error characterization; adaptive
fusion follows only if the measurements support it.

Start with [MS_RESEARCH_HANDOFF.md](MS_RESEARCH_HANDOFF.md) for the approved
research scope, decisions, resource pointers, and verified progress.

## New bounded execution checks

Use [run_td3_execution_v2.py](MAC-VO-main/Scripts/Setup/run_td3_execution_v2.py) for new runs. Inspect `tracking_diagnostics.support_state` and original unsupported-pair counts as well as raw execution status. These preserve tracking failures before interpolation changes stored flags. [EXP-033](experiments/EXP-033-gnss-and-tracking-reporting.md) documents usage and bitwise pose-equivalence controls; earlier runners remain available for historical reproduction.

## Repository layout

- `MAC-VO-main/`: extracted MAC-VO source, recovered FlowFormer/DPVO dependencies,
  and environment/verification scripts. See its upstream README for the pipeline.
- `models/macslam/`: extracted checkpoints, excluded from Git.
- `data/archives/`: original model and dataset ZIPs, excluded from Git.
- `data/calibration/yamaha.yaml`: supplied extrinsics source, excluded from Git.
- `data/tartandrive3/`: extracted recordings, excluded from Git.
- `experiments/`: reproducibility notes and measured findings.
- `results/`: generated manifests, arrays, plots, and logs, excluded from Git.
- `docs/research-proposal.pdf`: research proposal.
- `MAC-VO-main/Scripts/Setup/README.md`: environment and Box staging instructions.

## Verified starting point

`MACT_Offroad.pth` strictly loads all 428 checkpoint entries into FlowFormerCov.
The MACT frontend runs on the RTX PRO 5000 Blackwell GPU using Torch 2.7.1 and
CUDA 12.8. Synthetic inference at 128x128 and 480x640 produces finite flow and
positive covariance. Bounded 10-frame camera-local RGB and thermal odometry
execution is now exercised; calibration validity and dataset accuracy remain
unverified. See [EXP-003](experiments/EXP-003-bounded-vo.md) for execution evidence
and [EXP-004](experiments/EXP-004-moving-reference.md) for the first moving-window
comparison against recorded SLAM. Raw uncertainty recording and separate-window
scale checks are complete in [EXP-006](experiments/EXP-006-raw-uncertainty.md):
a single thermal scale does not transfer uniformly, so no correction is adopted.
[EXP-007](experiments/EXP-007-thermal-failure.md) narrows the late thermal failure
and confirms that this backend directly weights image/disparity residuals.
[EXP-008](experiments/EXP-008-backend-ablation.md) replays both objectives on
identical inputs: removing target disparity does not resolve the late failure.
[EXP-009](experiments/EXP-009-weight-ablation.md) finds strong pixel-weight
sensitivity: fixed weights improve late thermal translation but worsen rotation.
[EXP-010](experiments/EXP-010-weight-influence.md) locates strong upper-image
precision concentration in late thermal. Native replay in
[EXP-011](experiments/EXP-011-precision-mass.md) retains most of the translation
gain with total u/v precision preserved, alongside the rotation penalty.
[EXP-012](experiments/EXP-012-channel-mass.md) isolates the channels: u-only
redistribution gives a smaller late thermal rotation penalty than redistributing
both, but still needs separate-clip and geometry validation.
[EXP-013](experiments/EXP-013-separate-clip-validation.md) tests the frozen rule
on a separate night recording: benefits do not transfer consistently, and RGB
tracking fails in two selected windows. Thermal completes all three windows.
[EXP-014](experiments/EXP-014-night-uncertainty.md) records raw uncertainty
without changing those outputs: RGB uncertainty rises strongly in failed
windows and candidate selection collapses. This is a selection-coupled
availability signal, not independent uncertainty calibration.
[EXP-015](experiments/EXP-015-night-geometry.md) adds fixed-grid geometric
residuals: RGB disagreement rises strongly in both failed windows, while
within-pair uncertainty/error ranking remains inconsistent. Keep defaults.
[EXP-016](experiments/EXP-016-temporal-context.md) tests within-window
association after motion/flow/depth controls: signs vary and temporal
repeatability is weak. No general additional temporal signal is established.
[Verified paper notes](docs/research/literature-2026-10-20.md) ground target
definitions and the next thermal preprocessing provenance audit.
[EXP-017](experiments/EXP-017-thermal-provenance.md) reproduces all18 sampled
night thermal images exactly under OpenCV4.10. Saved stereo matrices also
reproduce, but physical body/stereo geometry remains inconsistent; night
cheirality does not justify a blanket baseline-sign flip.
[EXP-018](experiments/EXP-018-stereo-sign.md) retains signed stereo flow on
12 fixed first-frame inputs and reproduces every saved depth array bitwise.
Thermal model signs agree with feature displacements across day/night;
raw-to-processed topic ordering reproduces exactly, while physical calibration
and dense-field validity remain unresolved.
[EXP-019](experiments/EXP-019-spatial-stereo.md) audits a fixed grid: both
recordings share upper-negative/lower-positive thermal predictions, while
night fitted features concentrate in the upper image. This makes the apparent
feature sign contrast spatially selection-dependent. Physical calibration
remains unresolved. No weighting default has changed.
[EXP-020](experiments/EXP-020-calibration-reconciliation.md) verifies our
camera-info usage and tests concrete stereo corrections. Both a free-pose fit
and a body-center-constrained rotation fit fail their validation criteria.
Rejected candidates and source-convention exports are preserved separately;
a known-target thermal stereo sequence or independent applicable calibration
was requested to resolve the remaining geometry. Serial numbers are not required.
[EXP-021](experiments/EXP-021-msa-calibration-audit.md) audits the subsequently
supplied MSA source calibration: all24 fixed image checks pass without fitting.
Its fisheye model differs from the current producer; applicability and metric/body
accuracy remain unconfirmed. No original calibration or VO output is replaced.

The older `macvo:box` image cannot execute on Blackwell. Use `macvo:blackwell`:

```bash
cd /root/AirStack/ore-ms/MAC-VO-main
docker build --progress=plain --network=host -t macvo:blackwell \
  -f Docker/Dockerfile.blackwell .
docker run --rm --gpus all \
  -v /root/AirStack/ore-ms:/workspace -w /workspace/MAC-VO-main \
  macvo:blackwell python Scripts/Setup/verify_mact_checkpoint.py \
  --forward --height 480 --width 640
```

Replace the host mount path if the project is restored elsewhere. Avoid
installing the legacy `requirements.txt` into this image: its xFormers pin
would downgrade the framework. The recovered frontend uses native attention.

## Checkpoints and data

The shared CMU Box bundle is `macslam.zip`. Keep it in `data/archives/` and
extract it into `models/` to produce `models/macslam/MACT_Offroad.pth`, `MACVO_FrontendCov.pth`, and
`MACVO_posenet.pkl`. Archive SHA-256:

```text
77ad168672019f94d8e48ebb087cb109b938bcf8784e3df480070cd5f437f00c
```

AirLab processed data: `/offroad/datasets/tartandrive3`; raw bags:
`/offroad/rosbags`. Prioritize `20251027` or `20251106` for RGB-stereo and
thermal-stereo. These paths are not mounted in the current workspace.

The `20251027/1_salon_mac_cov_day` clip is staged locally, with camera streams,
timestamps, intrinsics, recorded TF, and SLAM inputs. Both RGB and thermal pass
real-image frontend execution. The supplied `yamaha.yaml` is staged under
`data/calibration/`; camera calibration reconciliation is in progress. See the
latest notes under `experiments/` before running geometry evaluation.
Thermal normalization and the checkpoint's training split are
still unknown; three-channel input compatibility alone does not establish the
original preprocessing recipe.

## Storage and shutdown

OSMO workspace files and local Docker images are temporary. Keep model/data
archives and outputs in persistent storage. Box remote `ore_ms` is configured;
the user has saved its config in the OSMO generic credential `ore-ms-box`.
Future workflows must map that credential and restore the private rclone
config; see the setup guide. Tokens and checkpoint/data files are excluded
from Git. Base64 encoding is not encryption.

Commit code and notes before ending a session, then push to an authorized
repository or download a Git bundle to persistent storage. A local commit
does not survive deletion of the workspace.

This project is tracked inside AirStack on the `ore_ms` branch. From the
AirStack root, recover its pinned dependencies with:

```bash
git submodule update --init --recursive -- \
  ore-ms/MAC-VO-main/Module/Network/FlowFormer \
  ore-ms/MAC-VO-main/Baseline/DPVO
```

To restore a downloaded `ore-ms.bundle`:

```bash
git clone ore-ms.bundle ore-ms
cd ore-ms
git submodule update --init --recursive
```

Then restore the checkpoint bundle and rebuild the GPU image. DPVO source is
present, but its compiled extensions and execution have not been verified.

MSA source-image follow-up: [EXP-022](experiments/EXP-022-msa-source-images.md)
checks the subsequently supplied thermal JPEG pair. Both all242 mutual matches
and214 F inliers pass the retained image criteria without calibration fitting.
This supports separate raw/depth/VO replay; metric/body/timing accuracy and
October applicability remain unconfirmed. Existing inputs/defaults unchanged.

[EXP-023](experiments/EXP-023-msa-preprocessing-vo.md) regenerates raw thermal
images with MSA calibration and runs the same six20-frame windows. All240
old-image replay controls pass. Day VO proxy errors improve substantially;
night results remain mixed. MSA is the experimental baseline for subsequent
geometry diagnostics; production defaults and previous results are unchanged.

[EXP-024](experiments/EXP-024-msa-uncertainty-geometry.md) recomputes
original/MSA thermal uncertainty against the same SLAM proxy across all six
windows. Day reprojection errors fall, but spatial ranking does not become a
consistent temporal reliability score. Late-night epipolar association remains
negative, and depth/visibility sensitivity weakens late reprojection ranking.
No new inference, weights or fusion rule. MSA remains the experimental baseline.

[EXP-025](experiments/EXP-025-salon-so-day-validation.md) stages the added
`20251027/0_salon_so_day`:1335 synchronized groups/~163s, no timestamp overlap
with earlier two recordings. Six RGB/MSA-thermal20-frame runs complete. Thermal
translation proxy RMSE0.185/0.198/0.084m; RGB0.381/0.122/0.130m, with RGB lower
rotation error in all3. Source camera metadata matches;120 old thermal replays
exact. GNSS-labelled data are present, but public converter maps their name
to `/odometry/filtered_odom`; upstream sensor/fusion/accuracy/timing provenance
requires confirmation. Keep SLAM-proxy interpretation and unchanged defaults.

[EXP-026](experiments/EXP-026-long-salon-so-day.md) extends the added
day capture to3 fixed100-frame RGB/MSA-thermal windows, excluding old samples.
Strict150ms continuity failed middle/late; preregistered250ms revision retained
actual gaps (9.9/10.2/10.0s). All6 attempts complete with no interpolation.
Near-static early spatial uncertainty/reprojection ranking is negative; moving
windows positive. Image/translation/rotation uncertainty associations differ,
and accumulated drift versus per-step errors can rank cameras differently.
SLAM/body anchor provisional;21 tests pass. No defaults/weights changed.

[EXP-027](experiments/EXP-027-night-low-motion.md) checks late-night
thermal low motion with independent LK tracking on fixed saved source pixels.
LK0.053px/model0.083px,model-LKerror0.103px on95.54% valid grid tracks;
LK-SLAM/predicted-deptherror0.925px. Texture/region sensitivity persists; no
exact decoded frame duplication. The0.0085m VO vs0.261m SLAM discrepancy remains,
but network-alone visible-motion suppression is unsupported. Reference/timing/
depth/calibration causes unresolved; no tuning. Exact bag interval
2025-10-27 22:13:40.207–22:13:42.107 EDT.7 tests/222 source hashes pass.

[EXP-028](experiments/EXP-028-reference-timing.md) finds no indexed timestamp
or SLAM-export mismatch on the fixed windows. Late-night filtered odometry
reports0.0161m displacement versus SLAM vehicle0.2608m, with zero mux/teleop
commands; its upstream sensors, independence and accuracy remain unconfirmed.
A55-case fixed reference-time sweep retains the late-night discrepancy; no
offset is fitted or adopted.11 tests/402 hashes and independent checks pass.
Keep MSA and all VO defaults; resolve reference provenance before using this
window as a physical pose-error or fusion-calibration target.

The follow-up [filtered-odometry source audit](docs/research/filtered-odometry-provenance-2026-10-21.md)
identifies a historical NovAtel GPS/IMU origin and the legacy `novatel_span`
package lead. Exact October2025 producer, aiding inputs, clock behavior,
quality and gps_frame geometry remain unverified. Public candidate drivers
must not be substituted for recording-specific source; no reference is promoted
to ground truth and no new VO experiment is claimed.

[EXP-029](experiments/EXP-029-stereo-image-consistency.md) checks18 saved stereo
pairs without a pose reference or new inference. Independent patch matching
supports68/1,008 late-night thermal grid points with0.423px median model
disagreement; most locations remain unvalidated. Stereo variance/discrepancy
associations are weak and selection-dependent. A recorded NCC precision issue
is stabilized without threshold changes;9 tests/97 hashes and independent
numeric controls pass. Keep MSA/defaults and the pending reference-source gate.


[EXP-030](experiments/EXP-030-temporal-angular-consistency.md) checks342 saved temporal pairs in nominal angular units without a pose reference. Median spatial uncertainty/LK-disagreement rho ranges−0.187–0.572 for thermal and0.361–0.594 for RGB. Late-night thermal remains negative despite94–97% LK support; normalization is not physical confidence calibration. Seven tests,2,394 independent ray/covariance checks,95 exactly reproduced prior LK pairs and729 frozen hashes pass;402 earlier audit hashes unchanged. Keep defaults and pending provenance gates. Box recovery: `backups/2026-10-21-exp-030`.


[EXP-031](experiments/EXP-031-new-night-validation.md) stages the new `night_mac_salon_warehousefig8`:1,500 paired groups, no overlap with three prior recordings. Three fixed first-eligible20-frame windows,751 prepared hashes. All three MSA-thermal attempts and early RGB complete without interpolation; middle/late RGB retain0 matches and fail, with0 independent LK support. Preserve failures and unavailable errors. Thermal median spatial angular uncertainty/disagreement rho0.005/−0.223/0.529; operational availability differs from calibrated reliability.24 tests,600 identical image decodes,798 scalar/finite-difference checks,114 pairs/160,056 points and267 analysis hashes pass; prior729/402/97 hashes unchanged. Rebuilt GPU image f429af4afa23 has recorded transitive version differences; CPU analysis unchanged. Keep defaults/provisional calibration/reference gates. No more upload needed now: unused intervals of this archive suffice for a next bounded study. Box recovery `backups/2026-10-22-exp-031`.


[EXP-032](experiments/EXP-032-long-night-validation.md) extends the new night recording to three nonoverlapping100-group midpoint blocks (14.9/9.9/11.0s), preserving30/0/1 gaps>150ms. All thermal attempts have0 pre-refinement lost pairs; RGB has99/97/99 out of99. Middle/late RGB raw execution passes are interpolation-dominated; existing MotionInterpolate aliases/clears boundary flags, understating original losses (97→94,99→95). Preserve reports/settings; audit raw counts/logs. Thermal median spatial uncertainty/LK-disagreement rho0.407/0.397/0.537; temporal/gap effects vary, not physical confidence calibration.25 tests,4,158 scalar/finite-difference checks,594 pairs/833,976 points,54 strata and3,041/1,229 hashes pass; prior751/267/729/402/97 unchanged. Box recovery `backups/2026-10-22-exp-032`. Next software task: pre-refinement diagnostics with unchanged poses. For future numerical RGB/thermal error comparison, request another night recording with visible RGB detail; names first. No more data needed to reproduce this audit.

[EXP-033](experiments/EXP-033-gnss-and-tracking-reporting.md) completes GNSS scalar translation cross-checks on15 fixed cases x3 offsets and a versioned reporting fix. New `run_td3_execution_v2.py` preserves pre-refinement support/flags even on zero-observation failure; original runner/postprocessor unchanged. Three GPU controls reproduce5,753 arrays bitwise; middle RGB correctly exposes97 original unsupported pairs vs94 stored flags.13 tests pass. GNSS original late-night0.0161m vs VO0.0085m/camera SLAM0.2610m; canonical early and new short late also show reference-dependent disagreement. Longer thermal GNSS31.71/32.52/31.47m vs VO26.45/31.72/26.74m retains early/late shortfall. Magnitudes from distinct physical origins are not camera-pose errors or calibrated GT; no axes/latency/scale/lever arm fitted.45 comparisons/2,340 queries validated, max bracket41.56ms;828 final hashes,291 reporting-control hashes and previous scientific hashes unchanged. GNSS phase001 source preserved after scope-label correction; phase002 arrays bitwise identical. Owner axes/RTK/origin/timing and checkpoint train split still pending. No more image upload needed for these checks. Box `backups/2026-10-22-exp-033`.

[EXP-036](experiments/EXP-036-night-batch-screen.md) batch-screens the four
remaining available October27 night archives. Most sampled RGB is very dark;
redcourse-waypointskip1 has a brief illuminated late candidate (10.56% median
LK support), requiring sustained-support and timing checks before VO. The
source-image screen includes569 selected images and456 pairs; no inference,
calibration or weighting change. No further upload is needed for this candidate.

[EXP-037](experiments/EXP-037-redcourse-lit-interval.md) extends that redcourse
candidate by100 RGB acquisitions on each side. Only6/1800 tracks survive the
original20-frame interval; a short preceding block retains74. Current stereo/
cross-modal timing gates yield no continuous paired window. No learned VO was
run; temporal pairing needs separate justification before a numerical comparison.

[EXP-038](experiments/EXP-038-mac-relative-ranking.md) directly tests saved MAC
raw temporal covariance ranking on114 paired day transitions. Nominal-angular
ranking agrees with lower MAC–LK disagreement on108/114 versus112/114 for
alwaysRGB; six thermal switches help1/hurt5. A full-grid uncertainty sensitivity
helps on one switch. This is population-sensitive matching-disagreement evidence,
not physical reliability calibration or a full3D/pose covariance test. No training,
new inference or fusion change.

[EXP-039](experiments/EXP-039-mac-recording-replication.md) repeats those frozen
rules on57 saved pairs from the different1_salon_mac_cov_day capture. Both
angular rules selectRGB throughout;all matching-disagreement targets also favor
RGB,so they equal the fixed baseline without testing successful thermal switches.
Ratio association changes sign across recordings. No inference/training/tuning.

[EXP-040](experiments/EXP-040-public-data-audit.md) audits public paired-data
access and independent reference suitability. MS² is preferred; user-supplied scripts now verify access. STheReO calibration/poses are preserved, but images hit Drive quota;
reference gaps reach7.4s and thermal baseline conventions need review. TartanRGBT's
MACVO poses cannot independently validate MAC. 2,753 STheReO and222,538 MS² schema/hash controls pass;22 source rotations flagged;
no model run or dataset-readiness claim. Box `backups/2026-10-23-exp-040`.

[EXP-041](experiments/EXP-041-ms2-night-pilot.md) acquires the fixed MS² night
sequence and preserves20 IDs with stereo RGB/raw16-bit thermal, three depth
variants and camera poses.5355 source controls plus37 independent timing controls
pass. Recorded thermal stereo offsets span15.2–49.8ms throughout10131 frames;
no pair passes the inherited2ms gate,including nearest-right pairing. Official
hardware-sync description does not resolve recorded timestamp semantics. No MAC
inference; resolve timing before a motion-sensitive comparison. Box
`backups/2026-10-23-exp-041` preserves the bounded pilot,not40GB archives.

[EXP-042](experiments/EXP-042-ms2-timestamp-semantics.md) checks primary acquisition
descriptions and both official MS² loaders. They use same-ID stereo pairing but
do not define exported exposure-time semantics. Thermal offsets flip sign8810
times;constant clock shifts and fixed±1-index shifts cannot satisfy the inherited
2ms diagnostic throughout.10174 controls pass,no correction/inference. Prepared
20-ID source-input manifest preserves all four times and raw pixels. Generic
loader would collapse619 thermal levels to5 and fabricate timestamps; implement
MS²-specific/model-compatible input handling next. Box2026-10-23-exp-042.

[EXP-043](experiments/EXP-043-ms2-mac-adapter.md) implements a real MS² MAC
sequence adapter. Rawuint16 thermal and both recorded camera times are retained;
model frames require an explicit audited percentile/CLAHE/bilateral recipe.
All80 source images and120 depth maps pass independent full-array checks;
14 tests/1087 controls pass with real CPU Torch/PyPose. Clip,batch and resize
preserve metadata;reference labels remain explicit. No model inference,timing
correction or default changes. Checkpoint training preprocessing remains
unconfirmed. Box `backups/2026-10-23-exp-043`.

[EXP-044](experiments/EXP-044-ms2-frontend-smoke.md) runs pretrained RGB on the
fixed MS² pilot:39 calls each for RGB384×1224 and thermal256×640 all pass shape,
finiteness,positive covariance and independent saved-array validation. All80
model-input tensors match archived producer replay;14 adapter tests pass. The
original checkpoint archive is hash-verified after shared-link recovery. Runtime
compatibility only,not physical accuracy. Full Box recovery:2026-10-24-exp-044-complete.

[EXP-045](experiments/EXP-045-ms2-reference-protocol.md) audits MS² pose/timing/calibration metadata and freezes a provided-pair single-scan-depth diagnostic before scoring. Both pose-direction interpretations remain explicit; physical timing/reference gates remain unresolved. Existing stereo outputs can support separately labelled per-modality projected-reference diagnostics, without a physical cross-modal accuracy claim.

[EXP-046](experiments/EXP-046-ms2-stereo-depth-proxy.md) executes the fixed MS² stereo/depth diagnostic: primary within-image uncertainty/proxy-error rho medians0.353RGB/0.453thermal, falling to0.120/0.385 on common depth-product support. All120 cases and16,380 independent checks pass. The result is sensitive to support/reference choice and establishes neither physical covariance calibration nor a cross-modal winner; no inference or defaults change.

[EXP-047](experiments/EXP-047-ms2-depth-spatial-controls.md) adds fixed depth/spatial controls: primary own-support joint within-bin coefficients0.179RGB/0.388thermal with~99.5% retained. Common-support RGB is weaker and eligibility-sensitive. All720 controls and41,416 independent checks pass; no physical calibration, cross-modal winner or default change follows.

[EXP-048](experiments/EXP-048-ms2-day-replication.md) replicates the fixed proxy protocol on a distinct moving MS² day pilot. Primary rawrho0.252RGB/0.392thermal; joint within-bin coefficients0.130/0.149 (night0.179/0.388). All78 frontend fields,120 proxy cases and720 controls validate. Association is positive but varies across captures; no physical calibration/camera-winner claim. Full day archives are now local, so further fixed intervals need no new upload.

[EXP-049](experiments/EXP-049-ms2-day-later-windows.md) tests fixed middle/later intervals of the existing day recording while CATS is unavailable. Primary joint coefficients are0.149RGB/0.209thermal and0.131/0.307; positive median proxy association persists with variable magnitude. All80 new stereo fields,240 proxy cases and1440 controls validate, with original source IDs/times preserved and no inference/analysis default changes.
