# Data selected for RGB–thermal deployment characterization

16 November 2026. Data discovery and bounded payload qualification, **no new VO
accuracy results**. The user selected characterization: quantify when RGB,
thermal and existing fusion help, including null and negative cases. A new
algorithm or a failure unexplained by existing baselines is not an admission gate.

Execution update: [real-data findings](deployment-characterization-findings-2026-11-16.md)
now supersede this discovery-stage readiness assessment. Downloads and decoding
work, but the lighting camera/reference convention does not yet support a physical
pose-accuracy interpretation; frontend availability is reported separately.

## Decision

Use **NGCLAB desk1 halfsphere bright / changing / dim** for the first controlled
lighting comparison, and **SubT-MRS SmokeRoom plus the extracted FlashLight
release** for real smoke and illumination transitions. Keep **M2P2 night** as the
outdoor follow-on. The inspected downloads work without an author request; this
does not yet certify whole-sequence integrity or independent trajectory truth.

| Selected population | Why it answers the question | Verified locally | Remaining qualification |
|---|---|---|---|
| NGCLAB desk1-halfsphere, -ic, -dim | Same scene/task family across bright, changing and dim illumination; full motion-capture reference documented for desk recordings | Three actual ROSbag file headers and complete connection/chunk indexes; RGB, LWIR and motion-capture streams; RGB/LWIR calibration downloaded and ZIP CRC checked | Full image/reference decoding, rigid-body-to-camera reference binding, timestamp association and camera projection |
| SubT-MRS SmokeRoom | Real smoke with visible and thermal image exports | Archive index, both complete timestamp lists and first image per camera; reference/IMU ZIP fully downloaded and CRC checked; intrinsics/extrinsics downloaded | Supplied integrated-odometry provenance and frame, camera models, explicit malformed-matrix handling, whole-span association and image integrity |
| SubT-MRS extracted FlashLight | Real changing illumination, with corresponding supplied reference | Same bounded member checks; exported image times overlap the downloaded reference | Use the extracted variant; raw-bag ZIP appears to contain a different recording. [Whole-export nominal qualification completed](flashlight-qualification-findings-2026-11-16.md); raw header/independent physical truth and zero-lux remain unverified |
| M2P2 r2c2_night_1 | Outdoor low-light deployment, passive sensors and independent LiDAR stream | Public catalog and actual 30-second bag index, with visible/thermal images, IMUs, LiDAR and GNSS | Establish independent pose reference; filtered/wheel odometry is not automatically ground truth. Do not infer measured lux from directory names |

These recordings have one thermal camera. They are **not drop-in thermal-stereo
inputs for MACThermal**. Compare appropriate monocular RGB, monocular thermal and
cross-spectral baselines under a common sensor budget; list extra IMU/depth/stereo
inputs in separate arms. NGCLAB desk recordings have no IMU connection. Keep
Kinect depth as reference-only unless a separate RGB-D baseline is declared.

## CMU paper search and actual download trail

Inspected the user's [CMU thermal search](https://publications.ri.cmu.edu/?title=&author=&abstract_keyword=thermal&type=&start_year=&end_year=)
and its smoke-keyword counterpart. Saved both pages and extracted title links.
This is a targeted lead screen, not a systematic full-paper review.

The strongest new trail is the CMU Super Odometry / resilient-perception work to
the [official dataset page](https://superodometry.com/datasets) and
[SubT-MRS paper](https://arxiv.org/abs/2307.07607). Its Sensor Fusion Challenge
extension explicitly lists SmokeRoom, OutdoorNight and FlashLight with RGB,
thermal and IMU. The linked raw, extracted and reference folders were inspected,
then actual archive members were retrieved; availability is not inferred from a
paper's dataset promise.

[TP-TIO](https://superodometry.com/tptio) also links public data. Its thermal folder
contains `2019-12-13-1.zip` (12,493,860,977 bytes). A separately named RGB corridor
folder is not proof of synchronized same-recording RGB. Condition identity and
reference coverage remain unqualified, so it is a secondary lead.

Existing MACThermal, SmokeSeer and thermal-navigation reading is reused rather
than counted as new experiments. FIReStereo Gascola-1 remains day-labelled and
blocked on its export contract; SmokeSeer results remain synthetic relative-pose
controls, not real-smoke trajectories. The 2014 smoke-odometry abstract is another
lead, with no public payload established in this bounded search.

## NGCLAB: selected controlled lighting cohort

Primary sources: [official release](https://github.com/NGCLAB/multi-spectral-dataset),
[data catalog](https://github.com/NGCLAB/multi-spectral-dataset/blob/master/data.md),
[calibration](https://github.com/NGCLAB/multi-spectral-dataset/blob/master/calibration.md)
and [dataset paper](https://arxiv.org/abs/2007.00622).
The documented visible/LWIR pair is hardware-triggered. Desk recordings have full
motion capture; the release's complex/outdoor recordings have weaker endpoint
references and are not interchangeable accuracy cohorts. Dim is not a certified
zero-lux condition. Half-sphere motion is preferable to the available rpy-only
rotation controls for a study including translation.

| Recording | Canonical public file | Actual bytes | RGB / thermal / mocap messages |
|---|---|---:|---:|
| bright | [desk1-halfsphere.bag](https://drive.google.com/file/d/1aH2vDYbGu6PE29JIgwCtiP9hgCaiBr2R/view) | 8,187,849,538 | 2188 / 2187 / 7377 |
| changing | [desk1-halfsphere-ic.bag](https://drive.google.com/file/d/1ib14-3QLFW9J62ItX-AYptdNa_uymJXX/view) | 8,479,290,630 | 2355 / 2355 / 7674 |
| dim | [desk1-halfsphere-dim.bag](https://drive.google.com/file/d/1uv-8UkJGATEkL8Ustph2MxxI0Rqoj-sz/view) | 7,123,649,460 | 2216 / 2216 / 7926 |

Each complete connection/chunk index was parsed from public HTTP byte ranges,
including counts consistent with its file header. Actual intervals are about
68.35, 73.59 and 69.24 seconds. Topics include `/camera/image_raw`,
`/optris/thermal_image`, `/vrpn_client_node/RigidBody/pose`. Actual NUC-flag topic
is `/optris/flag_state` (12 messages bright, 4 changing), absent in dim; do not
assume the catalog's differently named topic or infer zero NUC events from absence.
The bright file's HTTP size differs from the catalog's stated 5.18 GB; use the
actual access receipt. A connector 413 on these large bags was a size limit;
public range access succeeded. No full bags or image payloads were downloaded.

The small visible–LWIR calibration ZIP and standard camera text were downloaded,
hashed and checked. Full pixel encoding, per-frame availability and the binding
between mocap rigid body and evaluation camera remain next-step qualifications.

## SubT-MRS: smoke and transitions

Official linked folders: [raw bags](https://drive.google.com/drive/folders/1bV5oCLrpVoc6xKcIduUcco47nG0jPH6i),
[extracted images](https://drive.google.com/drive/folders/1E4EFVY-Bnef7sSBprnfL5Z49HAc89dLr),
[extension references](https://drive.google.com/drive/folders/1bSGDe1FFQYEjs20P8L15pTx7T0HCn2L0).
The following are archive counts, not validated synchronized pairs:

| Export | ZIP bytes | RGB / thermal images | Download ID |
|---|---:|---:|---|
| SmokeRoom | 9,248,440,073 | 10,290 / 12,510 | `140UJgOCa-2itHUZQfwOK1zISD8XRGw51` |
| FlashLight | 5,070,560,080 | 6,652 / 8,063 | `1_lxsaQN_s6MEedVsDhF6Z51Nf7xTGR_m` |
| OutdoorNight | 9,634,557,274 | 11,925 / 14,446 | `1QSwNaIFjEKvikcoyDeUtVDcnR8Wvc2ax` |

All six selected raw/extracted ZIP central directories were accessible. Both full
timestamp lists and first PNG in each camera directory were fetched for all three
exports: 12 selected members passed ZIP CRC and SHA receipts. Sample PNG headers
show 640×480 8-bit RGB and 640×512 16-bit grayscale thermal. This is bounded payload
evidence, not a full-archive integrity check or a smoke-density measurement.

Full downloaded smoke and FlashLight reference ZIPs contain IMU and
`integrated_odom/odometry_data.csv`: respectively 83,667 and 53,768 poses, strictly
increasing timestamps. Pose columns are timestamp, xyz, quaternion xyzw. Nominal
export/reference overlap is:

| Export | RGB within reference / total | Thermal within reference / total |
|---|---:|---:|
| SmokeRoom | 10,278 / 10,290 | 12,498 / 12,510 |
| FlashLight | 6,625 / 6,652 | 8,032 / 8,063 |

Startup images precede reference support. Declare eligibility and report this
support before predictions. Different native camera rates mean equal indices
are not synchronized pairs. Nominal timestamp overlap establishes neither clock
accuracy nor camera/reference coordinate binding; do not fit offsets to outcomes.

**Important release distinction:** extracted FlashLight timestamps and reference
are March 2023; the raw FlashLight ZIP's bag filename is July 2023. Select the
extracted variant for qualification and do not silently swap in the raw bag.
OutdoorNight has real sensor payloads but no matching reference in the inspected
extension or primary-reference listings. This is not an archive-global absence
claim; defer its pose-accuracy arm until a matching reference is established.

Downloaded SmokeRoom intrinsics specify RGB MEI and thermal pinhole projection.
The published thermal-to-IMU 4×4 matrix has bottom row `[0,0,0,0]`; RGB's is valid.
Preserve the original. An explicit adapter may construct a homogeneous transform
from the supplied rotation/translation after validating the convention and
rotation; document that normalization instead of silently editing calibration or
fitting it to reference. Resolve applicability to FlashLight separately.

Finally, an `integrated_odom` filename does not independently establish truth.
The paper describes a reference-generation framework; verify the provenance,
frame and uncertainty of these particular exported trajectories before claiming
independent pose accuracy. Until then label metrics as supplied-reference results.

## M2P2: outdoor follow-on

Sources: [project](https://people.cs.gmu.edu/~xiao/Research/M2P2/),
[paper](https://arxiv.org/abs/2410.01105),
[public Dataverse](https://dataverse.orc.gmu.edu/dataset.xhtml?persistentId=doi:10.13021/ORC2020/SP577T).
The inspected V4.1 catalog contains 684 unrestricted bags under CC0, including
night/evening directories. Public file 5496, `r2c2_night_1_2025-03-18_21-21-13_chunk0000.bag`,
serves actual ROSbag bytes (2,918,515,263 bytes total). Its full index confirms
299 images per RGB camera, 285 thermal images, 300 LiDAR messages and two IMUs
across about 30 seconds. Filtered/wheel odometry is present but is not certified
independent ground truth. Establish the LiDAR/reference procedure before accuracy
comparisons. The dataset paper reports no-light coverage; this probe alone does
not establish that this particular chunk has measured zero lux.

ViViD++ currently directs downloads through an academic-email request form, so it
is not the first no-request alternative. Cross-spectral SNU's inspected share
only exposed day data; that access limitation no longer blocks the selected study.

## Next execution contract and evidence recovery

First qualify full-span inputs, reference binding and projection for the selected
cohorts. Use existing format helpers where possible; no model run is implied by
this discovery. Then freeze a finite comparison of appropriate RGB-only,
thermal-only and existing fusion baselines. Measure pose error where reference
qualifies, tracking coverage/failures, recovery time and computation. Keep bright
controls and all predeclared conditions, including cases where thermal adds cost
or hurts. Relate outcomes to observable contrast, texture, saturation and thermal
changes; associations are not an already-validated deployment selector. Report
smoke, dim light and changing illumination separately. Resolve training overlap
and development exposure before claiming generalization.

Raw receipts, downloaded reference/calibration payloads, bounded index/member
probes and their helpers are under ignored
`ore-ms/results/deployment-data-search/`. Preserve both the failed HTTP503 member
attempt and successful retry. Large sensor archives were not downloaded. The
verified local recovery packet is
`/root/ore-ms-artifacts/rgb-thermal-deployment-data-2026-11-16/`; its manifest pins
members and hashes. No author contact, Box upload or Git publication was performed.
