# ETH3D visible-light transition scout

Downloaded the public training sequence `einstein_global_light_changes_1`
in mono format. The archive passed ZIP CRC verification. All 872 RGB
frames decoded; timestamps were strictly increasing over 32.108 s, with
a maximum interval of 36.864 ms.
[ETH3D release](https://www.eth3d.net/slam_datasets).

Measure the grayscale pixel 95th percentile per frame and report
one-second medians. Detect up to 300 Shi–Tomasi corners per frame
(quality 0.01, spacing 8 pixels) and count adjacent-frame pyramidal LK
correspondences with successful forward and backward status and at most
one pixel round-trip error. Use the full recording and retain zero counts.
This measures image-space support, with fresh corners each frame.

At elapsed second 2, median image p95 was 44 and median track count 283.
In bins 3–13, median image p95 stayed at 2, with median counts from 66 to
107. At second 14 the values recovered to 46 and 267. Only 2 of 871
adjacent pairs had zero accepted tracks. A second dim interval appears
at seconds 23–24. This is a recorded visible-light stress control. The
scout does not measure full estimator loss, relocalization, metric error,
or thermal benefit.

The supplied reference has 2,531 rows and spans the image endpoints,
but its longest internal gap is 1.37 s. Under a 1/30 s bracketing limit
only 688 of 872 image times have support. Any metric replay would need
interval-specific reference eligibility before scoring. No pose estimator
was run.

Archive, scout source and per-frame CSV are under
`/root/ore-ms-artifacts/eth3d-light/`. OpenCV 4.11.0.86 and NumPy
2.2.6 were used. Archive SHA-256 is
`44cdf876f9c06ce498750e0624269ba9efd72877230b1ca4b10167d0b416b96b`.
