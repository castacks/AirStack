# OutdoorNight inputs qualify; pose reference remains unavailable

16 November 2026, America/New_York; base a41fb5640 with recorded qualifier/auditor
extensions. Completed the [prospective input protocol](outdoornight-qualification-protocol-2026-11-16.md).
No matching, model execution, training, camera selection, clock/axis/extrinsic fit
or pose-accuracy scoring occurred.

## Decision

The complete extracted OutdoorNight release qualifies for **nominal input-only
comparison**, with one explicitly unsupported thermal interval. It does not yet
support a supplied-reference pose-accuracy arm. Do not treat exported IMU
orientation or input completeness as independent motion truth.

| Full-span check | Result |
|---|---:|
| Raw extracted ZIP bytes | 9,634,557,274 |
| RGB / thermal images, all decoded and independently redecoded | 11,925 / 14,446 |
| RGB span | 485.839804128 s |
| Half-second anchors / sensor intervals | 971 / 1,942 |
| Input-qualified sensor intervals | 1,941 |
| Exported IMU rows | 96,715 |
| Matching reference poses / pose-supported cohorts | 0 / none |
| Independent qualification audit checks | 139,667 passed |
| Reference-present/null regression and semantic tests | 5 passed |

Archive SHA256:
`7604ff76b2b6d315a3f3e5f4df6d7824f1103daaa2d1c92ab69af0b4fae1d717`.
All288 strict HTTP206 ranges verify exact position/length/total and have separate
SHA receipts; the assembled archive is hashed. This is observed transport and
content integrity, not publisher-checksum authentication.

## Dates, calibration and missing reference

The [official sequence row](https://superodometry.com/datasets) identifies
OutdoorNight as SP1, RGB/thermal/IMU, night illumination. Its own linked
[intrinsics](https://drive.google.com/file/d/1hbIyPUJ24YSyX1vISjkMUeqX5K0rrSkU/view)
and [extrinsics](https://drive.google.com/file/d/1Zkb4FybZBx2skEXxYnffL4jNneTmd8pQ/view)
were fetched and hashed. RGB is640x480 uint8BGR/MEI; thermal640x512 uint16/pinhole.
Both unchanged4,096-point ray roundtrips have maximum error below1.2e-12px.
Published camera-to-IMU upper3x4 is preserved; thermal's zero homogeneous bottom
row is explicitly replaced with[0,0,0,1]. Only sub1e-5 SO(3) roundoff is allowed;
there is no new calibration estimate or fit.

RGB timestamps run2023-02-27T00:04:01.550667Z to00:12:07.390471Z. Thermal starts
at00:04:02Z. The first thermal anchor misses its initial endpoint by449.333024ms
and remains unsupported under the unchanged25ms gate. All971 anchors remain.
Stamps are strictly increasing and all PNG ordinals bind one-to-one to timestamp
lists. Equal ordinal numbers across the two different camera rates are not pairs.
The IMU is finite, strictly stamped and quaternion-normalized; its maximum gap is
34.992ms. Without reference poses, no gyro/reference consistency test is performed.

The inspected [extension reference folder](https://drive.google.com/drive/folders/1bSGDe1FFQYEjs20P8L15pTx7T0HCn2L0)
lists four files for FlashLight, SmokeRoom, MultiFloor and LongCorridor, with no
OutdoorNight file. The [primary reference listing](https://drive.google.com/drive/folders/1KyM_OO14G6z4NeDZdEfa7hntLLhfdAPp)
has nine entries and no OutdoorNight file. The extracted archive contains images,
two timestamp lists and IMU, without odometry. These are scoped inventory findings,
not proof that a reference cannot exist elsewhere.

Bounded central-directory access to the official raw ZIP identifies six bags named
2023-02-26-19:04:01 through19:11:29. Those filename clock values are compatible
with the export's UTC span under UTC-5; filenames do not establish timezone,
image-header matching, physical exposure timing or reference identity. No raw bag
was decoded and no raw sensor/reference frame claim follows.

## Observed conditions before predictions

Seven fixed equally spaced RGB-anchored times span the entire recording, with
nearest thermal images. The samples show indoor views at the start and finish,
and outdoor views in between. Outdoors the thermal image retains ground, trees,
buildings and lamp structures; RGB is dominated by bright near-rig foreground,
glare and a visible dot pattern, with limited distant structure in those samples.
This supplies a relevant visibility contrast, but not a quantitative no-light or
causal sensor-physics label. The full recording is retained; no darker or easier
window was selected from prediction errors.

Across all RGB frames, global gray mean ranges100.29–148.20 (median116.81), and
p95-p05 ranges190–215 (median197). Bright near-field content can dominate these
statistics despite weak distant-scene visibility. They are not ambient lux or an
established sensor selector. Thermal per-image min/max normalization is unchanged;
normalized thermal brightness is not a radiometric comparison with RGB. Raw
thermal observed range is18073–22686. Fixed time-third summaries are saved without
claiming that thirds isolate illumination or scene difficulty.

## Implementation, verification and recovery

The existing generic qualifier and independent auditor now accept explicit null
`reference_archive`. The reference-backed fixture retains pose labels and passes
the unchanged gyro gate; null mode emits empty reference arrays/cohorts, false
pose support and null unperformed IMU agreement. Three semantic mutations are
rejected: invented reference support, wrong sensor endpoint and invented IMU
agreement. No path-only inference runner was added; older frozen captures retain
their historical source hashes and are not reissued under the extended source.

The independent auditor redecodes every source PNG, checks grayscale bytes/hashes,
timestamps, calibration, IMU arrays, all sensor endpoints and missing-reference
semantics. These checks establish input binding, not physical camera calibration
or pose accuracy. Source and archive pins are rechecked after qualification.
Host CPU runtime: Python3.12.3/NumPy2.5.5/OpenCV5.0.0/SciPy1.18.1/PyYAML6.0.3;
this does not establish readiness of the historical learned/GPU runtime.

Ignored evidence is under `results/outdoornight-qualification/`; raw ZIP and gray
arrays are under `data/deployment-characterization/`. A local recovery packet
under `/root/ore-ms-artifacts/outdoornight-qualification-2026-11-16/` retains
source/configuration, receipts, full manifests, arrays and fixed-time sheet with
readback-verified member hashes; raw images/gray arrays are external pinned inputs.
No Box upload, author outreach or Git publication is performed in this stage.

Qualification stops here. Next finite study can characterize **reference-free
matching support, availability and cost** on all971 anchors, using the existing
SIFT/learned baselines with a separately frozen plan and exact runtime/model
qualification. It cannot claim correct motion or thermal recovery accuracy. A
pose-accuracy extension requires a matching independently qualified trajectory;
do not substitute another recording, fit a reference or tune on these images.
