# Natural transition reference check

22 November 2026. Check public inputs for a same-recording visible–thermal
handoff through natural degradation and recovery. Qualifying the interval
requires usable camera and IMU calibration, both camera streams through the
transition, evidence of visible tracking degradation, and a reference tied
to the tracked body across the full scoring interval.

## PARK

The public release repository and KISS-IMU remain at the commits recorded
in the [PARK provenance audit](diter-plusplus-park-intake-2026-11-19.md).
The current public DiTer++ issue inventory supplies no PARK reference-frame
or thermal calibration clarification. KISS-IMU's public repository has no
release assets and its author documentation names a processed PARK layout
without mapping its pose CSV and IMU to recording-level source frames.
These checks leave the existing excerpt useful for field frontend work.
The conflict between thermal intrinsics and the pose-to-IMU binding
remains unresolved. [DiTer++ source](https://github.com/sparolab/DiTer-plusplus/tree/884384764e45e681252a305f862cc6de3a9f5195) · [KISS-IMU data layout](https://github.com/sparolab/KISS-IMU/blob/572bfbe29e90bcff55dc1ddf90205f071851f818/examples/dataset_layout.md)

## M2DGR Door

The previously inspected [room sequences](m2dgr-room-reference-intake-2026-11-17.md)
are separate bright and dark recordings. Door 01 is a stronger scenario
on paper because the same recording crosses from outdoors to indoors and
back outside. Door 02 covers entry only. The paper describes a Leica
tracker following a prism on the robot and reports separate visible and
thermal SLAM baselines on Door 01. Those overall baselines supply no
qualified handoff window. [Paper, Sections III and IV](https://arxiv.org/html/2112.13659) · [Author sequence listing](https://sjtu-visys.github.io/M2DGR/#dataset-sequences)

Both downloaded TUM files match their LFS object sizes and SHA-256 hashes.
All rows have strictly increasing timestamps and finite positions. Every
quaternion is four zeros. Door 01 has 2,214 position rows and a 107.10 s
maximum gap. Door 02 has 1,011 rows and a 450 ms maximum gap. The author
calibration file lists forward RGB, thermal and IMU transforms relative to
LiDAR. The bag endpoints both returned HTTP 403 to 32-byte requests. The
door crossings cannot be placed on the reference timeline from this
intake, and the position-only reference omits attitude and the verified
prism-to-estimator-body binding. [Pinned release](https://github.com/SJTU-ViSYS/M2DGR/tree/44acebee11a750768fee140bdcfce52209860f4b)

## Brno winter drive

Before inspecting payloads, choose the shortest author-listed
evening drive, rainy-suburban sequence `3_1_4_2` (listed as six minutes).
A verified suffix range recovered the ZIP directory and nine reference,
timestamp, IMU and calibration-pointer members passed member CRC.
The supplied calibration pointer resolves to public `v1` commit
`49b5baf9003fbea63792f2b69f3de811b035ad6a`. It provides RGB and thermal
intrinsics and nominal camera, IMU and antenna transforms. The release
describes its mounting calibration as approximate. [Pinned recording inventory](https://github.com/Robotics-BUT/Brno-Urban-Dataset/blob/608673413782a956c0681be7fbf6672cb2f7cfca/recordings_table.md) · [Calibration](https://github.com/RoboticsBUT/Brno-Urban-Dataset-Calibrations/tree/49b5baf9003fbea63792f2b69f3de811b035ad6a) · [Winter dataset paper](https://doi.org/10.1016/j.dib.2021.107667)

Over 375 s, the released sidecars contain 3,750 forward RGB stamps,
11,234 thermal stamps, 150,150 IMU rows and 7,474 GNSS rows, with no
repeated or backward timestamps. Nearest thermal system timestamps are
within 18.92 ms of every RGB stamp, with a median absolute difference
of 8.36 ms. This check does not determine exposure timing. GNSS brackets
up to 100 ms cover 3,730 of 3,750 RGB timestamps. The longest GNSS gap is
510 ms. The GNSS position file has latitude, longitude, altitude and
heading with no per-row fix quality, covariance, or full attitude.
The weather and time-of-day tags do not locate a visibility transition.
No video payload or estimator was run. The sequence is accessible for
further screening and remains unqualified for metric recovery.

### Auxiliary reference check

Before inspecting the auxiliary sidecars, use full recording coverage and
independence from the estimator's IMU as reference requirements. Six ZIP
members passed CRC after bounded range reads with a matching archive ETag.
All six streams have finite values and strictly increasing timestamps.
The 150,150 IMU attitude rows have maximum quaternion norm error of
\(1.01\times10^{-6}\) and span 3,749 of 3,750 RGB timestamps and all 11,234
thermal timestamps. The delta quaternion sidecar has identical timestamps.
The authors describe both as products of the onboard IMU. They provide a
diagnostic for that sensor and cannot supply independent attitude truth
for an estimator using the same IMU. [Winter extension data formats](https://pmc.ncbi.nlm.nih.gov/articles/PMC8683662/)

The auxiliary IMU GNSS spans only 1,671 RGB timestamps, starting 208.07 s
into the drive. All 8,366 altitude values are zero. For 8,364 auxiliary
samples with a Trimble sample within 100 ms, the median horizontal
separation is 440.04 m using the released system timestamps and a
local spherical approximation. The separation does not identify its
cause. The Trimble sidecar also contains 1,660 zero azimuth values among
7,474 rows, with no validity field to interpret them. These products
leave the independent body-pose requirement unmet. Close this
reference route for metric recovery unless additional provenance or
an independent pose product becomes available. The forward RGB and
thermal videos account for 9.12 GB of compressed members and were not
retrieved for this reference check.

Reproduction inputs, range and hash receipts, and the audit script and
JSON output are under `/root/ore-ms-artifacts/brno-3_1_4_2/`.

## JPL Mars Yard

The authors' night characterization consists of repeated thermal flights
from before sunset to after sunrise, with flat-field correction before
each flight. Their visible-camera comparison is a separate daytime
reference flight. The public README lists thermal imagery, IMU, raw
OptiTrack and filtered OptiTrack/IMU pose for the thermal recordings.
The paper reports raw OptiTrack outliers near sunset and sunrise and
uses the same onboard IMU to filter the evaluation reference.
[Paper, Section V](https://data.nasa.gov/docs/datasets/public/jpl-mars-yard/IROS19_1819_FI.pdf) · [NASA release](https://catalog.data.gov/dataset/thermal-inertial-uav-night-dataset)

The first 2 MiB of the published 06:30 flight returned HTTP 206. The
ROS bag connections recovered from this prefix include thermal imagery,
raw IMU, raw OptiTrack and filtered pose. The two bounded requests for
the 08:41 daytime comparison archive timed out, so its topic inventory
is unverified. The published comparison uses separate flights and does
not establish a paired degradation and return interval. Keep it as
prior evidence for thermal flight across times of night. No full bag
or estimator was run. The downloaded README and paper, prefix,
sanitized response headers and connection audit are under
`/root/ore-ms-artifacts/jpl-mars-yard-intake/`.

None of the inspected paths supplies a verified full reference through
a measured visible degradation and return interval. No handoff replay
or recovery score was produced. Source pins, bounded request receipts,
reference files, CRC and hash checks, and timestamp diagnostics are under
`/root/ore-ms-artifacts/natural-transition-intake/`.
