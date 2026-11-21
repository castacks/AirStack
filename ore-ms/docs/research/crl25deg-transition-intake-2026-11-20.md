# CRL25DEG light-transition and reference intake

Public `shellby-0225-train-lab` and `shellby-0225-train-loop1` bags
are accessible by exact HTTP byte range. The complete ROS connection
and chunk indexes verify front-camera, FLIR Boson and IMU streams
in each bag. The release describes the outdoor training loop as
an open field route, and a separate test loop as traversing field and
forest. Only training sequences are listed in the public download folder.
[Pinned dataset release](https://github.com/comrob/CRL25DEG_dataset/tree/60d2af8cbee8d698467825e831cf9d63c77eacbd),
[public data folder](https://drive.google.com/drive/folders/1ef0k0JzQpKvGQkLGCsqh9FhuewvpQVYq).

| Training bag | Duration | Front RGB / FLIR / main IMU messages | Reference check |
| --- | ---: | ---: | --- |
| Lab | 71.0 s | 3,087 / 2,128 / 7,100 | 1,479 total-station position rows, all quaternion fields zero, maximum timestamp gap 0.800 s |
| Outdoor loop | 569.3 s | 24,781 / 17,076 / 56,934 | 4,554 finite ordered rows, with a 100.799 s gap from 1740152516.127 to 1740152616.926 UTC seconds |

At 10, 30, 50, 70 and 90% of each bag's span, inspect one indexed
chunk preceding the target and select the nearest available frame
per camera within that chunk. Missing images in a single chunk are
reported as missing from this sample. The outdoor 10% sample shows
shadowed trees and a grayscale median of 6/255. At 30% and 50% the
samples show an open field, at medians 140 and 124. The lab RGB
samples have medians 78–108 wherever a frame was returned.

On the outdoor follow-up grid from 2% to 22% in 2% increments, the
RGB median decreases from 34 to 6 and rises to 75 at 20%.
The main reference gap covers the dark samples and much of the
observed increase. The separate `*_noisy.txt` export has a maximum
gap of 0.107 s. All 4,554 primary rows match the noisy export exactly
at shared timestamps. The bag index contains no raw GNSS fix or status
topic, so that agreement cannot qualify the additional rows. Use neither interpolation across the gap
nor the noisy export as a substitute. Sparse image values and
view changes do not establish tracking loss or recovery.

Headers, complete indexes, bounded chunks, reference files,
scripts and scout output are in `/root/ore-ms-artifacts/crl25deg-intake/`.
This intake used no pose estimator. Metric scoring of the early
outdoor interval requires an independently qualified position reference
through the missing span and a camera/IMU timing and frame contract.
