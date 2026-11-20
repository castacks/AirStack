# IRS RTVI Fast Dark 2 × 2 thermal pooling

Reducing thermal images from 640 × 512 to 320 × 256 raised metric
position RMSE from 0.171 m to 0.277 m at the same camera cadence.
The pooled configuration met the coverage and output timing gates
and missed the accuracy gate.

## Fixed comparison

Use the [qualified Fast Dark thermal replay](irs-thermal-input-budget-intake-2026-11-19.md)
with a 20 Hz initialization prefix followed by 10 Hz at 25.25 s from
the first image. Apply nonoverlapping 2 × 2 box averages with integer
half-up rounding to all 1,122 images, including the initialization
prefix. The whole-run change permits one fixed camera model. Carry
over the same image and IMU event order, bag times, camera headers,
and all 35,560 serialized IMU messages. Read back the pooled images
and verify data dimensions, hashes, and 3,366 sample pixel sums.

Adjust the equidistant camera matrix using the half-pixel-center map
`x' = (x − 0.5) / 2` with the same rule for y. The replay log confirms
focal lengths of 202.49 and 202.53 pixels and a principal point of
(159.275, 125.67). Keep distortion coefficients, extrinsics, camera
time shift, feature cap, and all pixel-domain tracking settings
fixed. A fixed patch width therefore covers a larger angular region
in the pooled arm.

Use the frozen [startup study scorer](irs-thermal-cadence-startup-2026-11-19.md)
with a separate yaw and translation fit for each arm on the same
early samples, holding scale fixed. Preflight reproduced all 98 early and 612 post-cutoff
reference-supported timestamps. Require at least 98% shared support
in both intervals, position RMSE at most 0.35 m and 120% of the
native-resolution cadence arm, a first corrected pose within 150 ms
of first input, and a maximum corrected pose gap within 150 ms.

## Results

| Image input | Position RMSE on 611 common timestamps | First corrected pose | Largest corrected gap | Offline loader wall time |
| --- | ---: | ---: | ---: | ---: |
| Native 640 × 512 | 0.171 m | 50.01 ms | 116.68 ms | 3.745 s |
| Pooled 320 × 256 | 0.277 m | 50.01 ms | 116.68 ms | 2.571 s |

Both arms emitted 1,120 corrected poses and matched all 98 early
and 611 of 612 post-cutoff reference-supported timestamps. Pooled
position RMSE increased by 62% and exceeded the 0.205 m accuracy
limit. The published pose timestamp checks measure sensor time.
The independent audit reproduced the scores and timing metrics.

The pooled arm used 75% fewer pixels per frame, and its single
offline loader run was 31% faster. The timing includes bag I/O
and solver work, with pooling performed beforehand. These results
close this fixed 320 × 256 configuration for Fast Dark. They leave
other scales, changes to patch settings, and pooling only after
initialization untested.

The frozen protocol, pooled calibration, input and output hashes,
direct captures, logs and independent scorer are under
`/root/ore-ms-artifacts/irs-budget-intake/spatial/`.
