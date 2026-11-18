# ViViD++ and CART pose-reference intake

This is a follow-up to the [M2P2 registration result](m2p2-night-registration-and-support-2026-11-17.md)
and [M2DGR room intake](m2dgr-room-reference-intake-2026-11-17.md). The target
remains an independent, time-bound pose reference for the existing RGB–thermal
comparison.

## ViViD++ handheld indoor

The [paper](https://arxiv.org/html/2204.06183#S3) documents six-degree-of-freedom
external mocap, marker-to-camera hand-eye calibration and a published approach
to sequence time offsets. Indoor sequences include bright, dark and flashlight
conditions, including a varying-flashlight sequence. Outdoor handheld uses
LiDAR SLAM and the driving set uses RTK position. The indoor handheld set is
the closest match to the existing comparison. The authors' offsets and frame
chains still need to be checked against the actual release.

The [official download page](https://visibilitydataset.github.io/4_download.html)
requires an educational-domain request. No request was submitted and no
ViViD++ bag or mocap payload was obtained.

## CART Duck aerial night

The [paper and supplement](https://arxiv.org/abs/2403.08997) describe recorded
UAV position estimates, VN100 orientation and camera trigger streams. The
[official night split](https://github.com/aerorobotics/caltech-aerial-rgbt-dataset/blob/dc1adecf72f58f0191e337321c20ede2ace0e49f/splits/thermal_splits/temporal/night.txt)
names two Duck sequences. The [CaltechDATA release](https://data.caltech.edu/records/cks6g-ps927)
served verified byte ranges for both bags. Decoded both full bag footers and
sampled the first, middle and last target-bearing storage chunks by stream.

| Sequence suffix | File bytes | Color images | Thermal images | UAV local poses | VN100 IMU |
|---|---:|---:|---:|---:|---:|
| 18-20-21 | 29,008,628,685 | 4,398 | 51,233 | 17,602 | 158,313 |
| 19-55-11 | 35,693,033,968 | 5,189 | 60,960 | 20,772 | 186,796 |

Sampled local pose headers use `map`. Sampled sensor frames include `vn100`,
`boson_camera` and `camera`. All six sampled UAV GPS raw fixes report
fix type 3 (ordinary 3-D fix), so the presence of an RTK-capable receiver
does not establish fixed RTK on the tested portions. This sample cannot
rule out other fix modes elsewhere. The complete index shows timing and
navigation streams, with independent sub-degree camera pose accuracy,
exposure binding and whole-record RTK coverage still unverified. Do not
download either full bag for the current accuracy comparison on this evidence.

Bounded scripts, headers, footers and metadata samples are under
`/root/ore-ms-artifacts/cart-intake/`. No full bag, model run or Box upload
was performed for this intake.
