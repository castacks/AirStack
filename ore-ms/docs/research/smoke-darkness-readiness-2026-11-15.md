# Smoke and darkness: real data are available, but evaluation targets differ

15 November2026. This finite source/asset assessment follows the user's two thesis
links. Initial metadata intake was followed by the input/runtime qualification below;
no stereo-VO trajectory has been scored. The later
[fixed synthetic SIFT assay](smokeseer-relative-pose-findings-2026-11-15.md) is complete:
clean RGB90.04%versus smoky RGB49.44%primary success across ten scenes; thermal
73.61%versus smoky RGB44.74%in nine matched-camera scenes. This is a synthetic
monocular baseline result, not a new-method or physical smoke claim. It closes
an environmental-coverage omission in the contribution review, not the broader
question of thermal usefulness. Daylight results do not settle smoke performance.

## Current synthetic conclusion after learned comparison

The [existing learned frontend control](smokeseer-learned-findings-2026-11-16.md)
is complete across the full frozen population. SmokyRGB97.92%and thermal98.53%
primary success largely remove the SIFTgap; both-camera failures fall217→2in
nine identical-camera scenes. This is not an unmet failure beyond that baseline.
CleanFicusregression remains visible. Physical smoke/complete-darkness evidence
and Gascola's export contract are separate unresolved questions. No new method,
training or real-smoke trajectory claim follows from the synthetic study.

## Complete synthetic input preflight

The [prospective relative-pose protocol](smokeseer-relative-pose-protocol-2026-11-15.md)
now declares all ten train splits, three arms (smoky RGB, supplied clean RGB and
thermal),3495images and1155adjacent pairs per arm. The full public ZIP downloaded
successfully. Its member index matches the earlier bounded-range index; all
selected3495members pass ZIP CRC32/SHA256, every PNG chunk CRC and full OpenCV
lossless decode. Every declared pair passes pose conversion, chronology and
binding checks. No pairs have zero reference translation. Independent raw-image
hash, coverage, pose-algebra and frozen-pin audit passes15081checks.

Runtime: pinned restored image, OpenCV5.0.0/NumPy2.2.6, no predictions. Selected
images comprise1165uint8grayscale thermal,2090uint8RGBA RGB/clean and240uint16RGBA
Mars RGB/clean images. Preserve full-depth normalization and fixed black alpha
composition. libpng emitted duplicate `eXIf` ancillary metadata warnings during
successful decoding; CRC and pixel decode gates passed. No metadata stripping or
input rewriting was used to pass qualification.

Reference adjacent rotation spans4.800001–33.807318degrees, median6.551918degrees.
Identity rotation passes the planned5degree threshold on149/1155pairs (12.9004%);
keep scene-specific identity controls. These are rendered camera paths, not
verified sensor timing or physical real-time VO. The clean-RGB pose binding is a
released-loader assumption; it is not independently checked render provenance.
The earlier header-only limitations remain historical, superseded for the selected
train population by complete decoding here; unselected test images were not decoded.

Input/protocol freeze and receipts are under `results/smokeseer-preflight/`.
Local verified evidence and original ZIP are preserved under
`/root/ore-ms-artifacts/smokeseer-preflight-2026-11-15/`. The subsequent fixed SIFT diagnostic and independent audit are complete in the
[findings](smokeseer-relative-pose-findings-2026-11-15.md). No parameter search or
model training occurred. Preflight success alone did not establish pose accuracy.

## SmokeSeer synthetic camera qualification

The public synthetic ZIP is accessible through bounded HTTP ranges. Its central
directory lists5055members; all40camera JSON files (3,144,238bytes) were retrieved
with per-member ZIP CRC32and SHA256verification. This is a partial archive intake,
not a full3,609,993,464byte ZIP integrity check. All3330camera assignments across
ten scenes have finite homogeneous transforms with proper rotations within1e-5,
and every referenced PNG is listed in the directory. No image predictions, model
execution, training or pose-error score occurred in this intake.

The pinned author loader interprets transforms as Blender camera-to-world, flips
Y/Z for optical convention and derives focal length from horizontal field of view
and image width. Render scale has not been established as metres. The `frame`
field is animation time, not a physical exposure timestamp; test files contain
repeated values and novel camera views. Never deduplicate or join poses on this
field, or treat the supplied test order as a verified chronological VO stream.

By unique filename ordinal, nine of ten training scenes have identical RGB/thermal
poses, animation times and field of view. Caterpillar differs (camera-centre
separation up to1.577579render units); do not call its views co-located. Five test
scenes also differ: Ficus,Forest,Hotdog,Lego andMars. Filename equality alone is
not evidence of matching rays. No same-modality stereo pair was established;
these assets do not directly satisfy our MAC-VO/MACThermal stereo input contract.

The first training PNG header in each scene/modality was checked (20samples):
all800×800, thermal8-bit grayscale; RGB8-bit RGBA except Mars16-bit RGBA.
IHDR CRCs pass, but these are header checks, not whole-image CRC/decode checks or
proof of consistent format across every frame. A silent8-bit conversion would
needlessly change Mars input; define preprocessing before any predictions.

**Camera-intake decision, now executed:** these assets support a separate synthetic
monocular relative-pose smoke diagnostic. Full selected-image integrity/decoding,
motion preflight and the fixed baseline execution subsequently passed; see findings. It is not a real-smoke or metric stereo-VO
benchmark. Before inference, freeze scene population, all adjacent pairs, camera
conversion, image normalization, matched controls, pose endpoints and stopping
rules. Include every declared scene; handle the known camera differences in the
protocol rather than removing failures after seeing scores. Do not assume clean
image counterparts, physical thermal radiometry or a calibrated stereo baseline.
No new method is justified by asset availability alone.

Evidence: `results/smokeseer-synthetic-intake/` contains camera receipts, ZIP index,
all40JSON files, final camera audit, sampled PNG-header audit and pinned loader.
The earlier animation-key comparison is retained as an invalid-association
diagnostic; the final audit uses unique filename ordinals. A verified local
archive is under `/root/ore-ms-artifacts/smokeseer-synthetic-intake-2026-11-15/`.

## Contract audit: condition identity also needs confirmation

A bounded follow-up inspected all12reachable release commits and public calibration/
timestamp/RGBissue discussions. The calibration file has only one published revision;
no earlier rectification export, P1/P2or per-sequence time contract was found.
Do not repeat the same source search without new information.

Important correction: the primary paper's TableIIIlabels Gascola-1as **day**, with
Gascola-2day+smoke,Gascola-3static smoke andGascola-5night smoke. Earlier wording
in this assessment inferred smoke from the collection-level description and was
too strong. The public archive has three folders with2835/2075/1735pairs, unlike
the paper's five Gascola sequences/counts. The paper-to-export mapping is not
verified. Do not call the already-qualified `gascola_1` a smoke test, or silently
switch to another folder and assume its label. Input/runtime checks remain valid.

Maintainer [RGB-release comments](https://github.com/firestereo/FIReStereo/issues/7#issuecomment-3435567518)
state that RGB exports were not released because fisheye rectification/thermal
association were unfinished; a later comment offers raw ROS data on request.
This explains the missing published RGB stream. It does not establish calibrated
paired RGBaccess, and no request has been sent. The [rectified-Pdiscussion](https://github.com/firestereo/FIReStereo/issues/10)
remains unresolved in the inspected replies; [time association](https://github.com/firestereo/FIReStereo/issues/11)
also has no inspected answer. Raw YAML plus visual epipolar plausibility would
not uniquely establish exported focal lengths or image transforms.

### Maintainer request prepared locally, not sent

> We are preparing a thermal-stereo VO assessment using the public Gascola ZIP.
> Could you clarify these three release-specific points?
>
> 1. How do `gascola_1/2/3` (2835/2075/1735pairs) map to the paper's five Gascola
> sequences? Which exports contain moving-camera smoke, and which contain day,
> night or static-camera scenes?
> 2. Are these PNGs raw,undistorted or stereo-rectified? Please provide their exact
> P1/P2(or K1/K2,R1/R2,Tand output crop/resize) and baseline units, or the export
> code/config that generates them from the published calibration.
> 3. What are the timestamp units and frame association? Does each row describe
> both exposures or only one camera; were frames dropped or subsampled?
>
> If available, a per-sequence calibration/timing/condition manifest would suffice.
> We are not treating SLAM estimates as independent smoke ground truth.

This is the concrete remaining input contract. No owner outreach, calibration fit,
new inference or VOcapture occurred in the follow-up. Source/comments/history
receipts are under `results/gascola-export-contract-audit/`.

## Current Gascola1 qualification: inputs and frontend pass; VO calibration unresolved

The complete Gascola1prefix was restored by a strict HTTP206range:2177759008bytes,
not the full5.15GBarchive. All5671selected members pass ZIPCRC/SHA checks, including
5670PNG images. Every PNG chunk CRC passes. All2835left/right pairs decode as
640×512uint16; filenames and the2835monotone source timestamp rows match. Full-
archive SHA remains unverified; the selected-byte and per-member proofs are explicit.

The exact archived GPU runtime `sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`
was reconstructed from verified EXP106base/EXP112delta blobs. All2782model-packet
members pass hashes; originalMACT checkpoint SHA remains
`eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a`.
CUDA matrix computation passes on the RTXPRO5000Blackwell. Historical driver-init
stderr remains in logs alongside successful exit codes; no runtime settings changed.

Two fixed frontend calls—source0left/right stereo and source0left→source1left—pass
at512×640 with finiteFP32flow and positive covariance. All428checkpoint entries
load with no migration,removed keys or initialized buffers. Three source images
have bitwise-identical Pillow/OpenCVdecodes. The existing named percentile/CLAHE/
bilateral recipe is reused without rectification; checkpoint-training recipe
compatibility remains unverified. This is geometry-free runtime qualification,
not VO, depth correctness, successful tracking or smoke robustness evidence.
Saved arrays/source checks pass; no longer sequence or accuracy score is inferred.

The full VO capture is **not launched** because export geometry is not qualified.
The provided YAML contains distortion parameters, but lacks explicit rectified
P1/P2for these PNGs. The [open calibration discussion](https://github.com/firestereo/FIReStereo/issues/10)
distinguishes raw and rectified parameters and has no inspected final resolution.
That issue concerns the depth export; it does not prove Gascola is miscalibrated.
It reinforces the need to establish this archive's exact transformation chain.
The [timestamp association discussion](https://github.com/firestereo/FIReStereo/issues/11)
concerns other split data; our Gascola count match passes but units/exposure meaning
are not thereby physically established. No guessed Pmatrices or second undistortion.

Concrete remaining information: (1)whether GascolaPNGfiles are raw,undistorted or
stereo-rectified and their corresponding left/right projection matrices; (2)the
units and left/right acquisition meaning of each timestamp row. No author was
contacted. Once established, a separate whole-span VO protocol can bind the
adapter and frozen endpoints. No new method or trajectory gain is claimed here.
Evidence is under `results/gascola1-baseline-qualification/`; source images are
under `data/firestereo/gascola_1/`, outsideGit. Earlier metadata-only observations
below describe the intake phase, not the now-restored pixel/runtime state.

## Dataset decision

| Source | Confirmed asset/evidence | Suitable next use | Accuracy boundary |
|---|---|---|---|
| SmokeSeer real | Drive listing exposes1.70GBreal.zip and separate Bathroom/RedContainer RGB/thermal videos. Release loader consumes COLMAP-style poses. | Smoke robustness, cross-modal registration or reconstruction study after input inspection. | Not a ready metric-VO truth source; publication explicitly reports unsynchronized cameras and no true real-data ground truth. |
| SmokeSeer synthetic | Listed3.61GBsynthetic.zip; authors describe Blender parameters and rendered RGB/thermal sequences. | Conditional exact-pose controlled smoke assay after verifying archive contents and camera model. | Files not downloaded; not proof of physical thermal behavior or real-world generalization. Not directly our four-camera stereo-fusion input. |
| FIReStereo Gascola | Public5.15GBarchive reachable; central directory confirms3thermal sequence folders. Gascola1contains2835left/right PNG-name pairs and2835timestamp rows. | Thermal-stereo candidate after verifying exported sequence condition identity. | Inspected archive has timestamps and images, no trajectory reference or RGB stream. No automatic ATE/fusion claim. |
| FIReStereo Frick | Night/rain collection; published manifest lists thermal/depth,SLAM reconstruction and LiDAR/IMU bags. Reachable144.7MBFrick1reconstruction includes traj.txt,time.log,scans.pcd. | Secondary night/rain thermal-VO supplied-reference assessment. | SLAM-derived reference is not independent metrology; trajectory-to-image association/frame/time/scale not qualified here. Not verified zero illumination. |
| FIReStereo Firesgl | Manifest lists thermal images of prescribed-fire scenes; release describes8bitlogging unlike other16bitcollections. | Later smoke/fire transfer case after recipe/format qualification. | No matched RGB or reference product established from inspected release; avoid silently applying16bit assumptions. |

The two datasets are not plug-in replacements for our RGB-stereo plus thermal-stereo
fusion capture. FIReStereo documents thermal stereo/LiDAR/IMU. SmokeSeer uses RGB
and thermal video with estimated cross-modal geometry, not verified two-camera
stereo within each modality. Do not duplicate one image to fabricate stereo.

## Source findings and reading depth

[SmokeSeer v1](https://arxiv.org/html/2509.17329v1), targeted3.5,4.1,Appendix8:
poses are obtained with learned SfM and cross-modal registration. Real cameras
are unsynchronized, and real ground truth is unavailable. Those estimated poses
cannot certify independent odometry accuracy. The synthetic release is a distinct
controlled option. This is targeted reading, not full-paper/thesis or figure review.

[FIReStereo v1](https://arxiv.org/html/2409.07715v1), targetedIII-C/D and release
metadata: reference products derive from Faster-LIO and calibration. Smoke corrupts
LiDAR enough that smoky depth maps were not generated. This does not prove every
smoky trajectory is unusable, or that no additional reference exists; it prevents
promoting an unspecified LiDAR product to certified smoke ground truth.
The [thermal thesis](https://publications.ri.cmu.edu/3d-thermal-perception-for-autonomous-navigation-in-visually-degraded-unstructured-environments)
reports complete-darkness autonomy; we have not reproduced that deployment or
verified lights-off/lux evidence for our own near-black recordings.

[SmokeSeer release](https://github.com/cmu-ci-lab/SmokeSeer/tree/3fb5c0b45c9446343d62e560e6cab6b536171927)
commit3fb5c0b45c9446343d62e560e6cab6b536171927: README and targeted pose-loader
branches inspected; no build or execution.
[FIReStereo release](https://github.com/firestereo/FIReStereo/tree/54cb48b606e4f5b84219931ca6322a70264367ce)
commit54cb48b606e4f5b84219931ca6322a70264367ce: README,data_description,download
manifest/downloader and calibration inspected read-only. Downloader was not executed.

## Actual access and metadata checks

SmokeSeer's Drive listing and both video subfolders were read with the connected
Google Drive skill. The initial pass verified listing/downloadability metadata only. The later synthetic
qualification above retrieves bounded ZIP payloads; no real videos or full ZIP
were downloaded, and complete-archive integrity remains unverified.

FIReStereo server returns HTTP200to HEAD and HTTP206to bounded range requests.
Only ZIP central directories for Gascola/Frick1and two small text members were
retrieved: Gascola1timestamps and Frick1trajectory format. No full archives,
image data or point clouds were extracted. Central directory hashes and both
text-member CRC32/SHA256checks are preserved. Full archive SHA256is not verified.

Gascola1left/right filenames match exactly across2835pairs. Timestamp rows are
strictly increasing and match that count; physical exposure synchronization and
timestamp units are not established by these checks. Frick1traj.txt has2108eight-
column rows; format alone does not qualify pose conventions or reference quality.
The published download manifest lists reconstruction/bags for Frick/Hawkins,
not Gascola/Firesgl; no global absence claim about unpublished assets is made.
The release requests a research survey; none was submitted and no author contacted.

## Finite next evaluation and baselines

Prioritize **export identity/calibration confirmation before the real-smoke baseline**,
then preserve the selected, explicitly smoke-labelled sequence in full chronology. Restore the declared thermal preprocessing,
checkpoint,calibration and runtime before inference. Preflight all stereo/timestamp
bindings; do not pick favorable smoke intervals. A bounded initial interface check
must be labeled engineering, not the scientific smoke result.

Compare native MACThermal against an applicable thermal-stereo baseline under
matched sensor inputs. RGB-only/fused claims require actual corresponding RGB;
RGB is not established in this archive. VIO baselines require the same IMU supply
and separate reporting. SmokeSeer reconstruction or offline learned SfM is context,
not a real-time stereo-VO equivalent. Reconstruction PSNR is not trajectory error.

Without a qualified smoke pose reference, valid first endpoints are pipeline
completion, supported adjacent outputs, lost-track/reset counts and continuity
under fixed existing settings. These are algorithmic support, not proven accurate
motion or obstacle safety. Require independently qualified reference data before
metric accuracy claims. SmokeSeer synthetic may support exact-pose diagnostics,
but its camera arrangement must be validated rather than forced into our stereo API.

Separately qualify Frick night/rain supplied-reference evaluation, keeping exposure
claims conservative. To test physical complete darkness, additional recording
provenance identifying illumination state is needed; a night label is insufficient.
Before requesting new uploads, inspect existing assets and owner documentation.

Stop the metric-VO branch if reference/frame/time evidence cannot be qualified;
retain useful smoke operational tests without inventing truth. Do not close smoke,
thermal supervision or representation research merely because dense labels are
absent. A new method still requires a measured failure beyond strong existing
baselines, with one mechanism and a separate frozen evaluation contract.

## Duplicate-work check and additional prior art

The history contains TartanAir's syntheticRGB `gascola/Easy/P001` in
[EXP100](../../experiments/EXP-100-training-coverage-comparison.md). It is not
FIReStereo's real thermal Gascola smoke capture. FIReStereo-named preprocessing
was reused on other datasets; that does not constitute a Gascola smoke VO run.
No recorded real-Gascola VO execution was found in the inspected research notes,
experiment records or relevant commit summaries. That initial intake was metadata-only; subsequent input/runtime and synthetic
camera qualifications are separately recorded above.

[TP-TIO](https://publications.ri.cmu.edu/tp-tio-a-robust-thermal-inertial-odometry-with-deep-thermalpoint)
is already in our registry: sectionIII-C was inspected in the November12tracking
intake. A local translation-only radiometric patch prototype was executed and
failed its day/night/rain accuracy gate; it did not implement ThermalPoint or
IMU fusion and did not reproduce TP-TIO or its smoke experiments. The current
institutional abstract explicitly includes smoke evaluation. TP-TIO is relevant
prior art/baseline context; a sensor-matched comparison must separate its inertial
input from camera-only MACThermal. No new full-method/code reading occurred here.

[AdveRsarial Calibration between Modalities](https://publications.ri.cmu.edu/adversarial-calibration-between-modalities)
was not found in the prior registry or execution records. This pass reads only
the institutional abstract/metadata. ARC learns an input adapter for an existing
source-modality neural model; it is not camera intrinsic/extrinsic synchronization
calibration. Thermal-input adaptation is relevant to representation/low-label
learning, but no pose/correspondence geometry preservation or VO benefit is
established by this abstract. A generic small adapter before a frozenRGB model
already has prior-art overlap; do not propose it as new or treat ARC as tested.

## Preservation

Source snapshots,access receipts,Drive listings,ZIP directories,text-member hashes
and validation remain under `results/smoke-darkness-intake/`, ignored by Git.
Paper registry records exact reading depth. Initial intake performed no model execution; the later two-call frontend qualification
is explicit above. No new training,benchmark accuracy score,external message,form
submission or Box upload occurred.
