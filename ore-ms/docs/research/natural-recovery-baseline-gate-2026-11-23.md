# Admission gate for a natural recovery recording

23 November 2026. The [thesis results](rgb-thermal-thesis-results-2026-11-20.md)
retain the scoped thermal availability and accuracy gains, the
recovery failures, and the cadence tradeoff. The remaining question
is whether a fixed RGB-to-thermal fallback improves stateful
trajectory recovery during an independently referenced field transition.

## Qualify the recording and the clear control

1. Require accessible same-recording RGB, LWIR, IMU, and an
   independent pose product. Verify pose-body and sensor frame
   bindings, clock semantics, and reference coverage through the
   lead-in, degradation and return. If a pose product uses the same
   IMU as the estimator, qualify the dependency before claiming
   independent accuracy.
2. Inspect the recording for measured visible degradation and return
   using fixed image or track diagnostics. Record how the interval
   was selected. An interval chosen after a scout is development
   evidence. Keep a separate recording for unchanged transfer.
3. Freeze the sensor-only initialization method, the clear lead-in
   control interval, expected input and output support, scoring
   queries and alignment rule. Set translation, rotation and
   continuity bounds from the reference quality and the proposed
   recovery effect. Keep the 0.15 m bound for the already tested
   short-range indoor control. Declare a context-specific bound
   before evaluating a different platform.
4. Run RGB+IMU on the lead-in. If it fails, retain the result and
   close that candidate for recovery under the declared estimator
   configuration. For a passing candidate, compare continuous
   RGB+IMU, continuous thermal+IMU and one fixed fallback on
   the same interval. Report pose accuracy on common support,
   output coverage and gaps on full support, and measured compute
   under the same runtime and replay conditions.

## Public recording intake

**KAIST Multi-Spectral Day/Night.** The authors describe co-aligned
RGB and LWIR and a synchronized export with OXTS RT2002 positions,
global orientation, motion channels and signal quality. The capture
hardware is listed at 100 Hz and the synchronized file at 25 Hz.
Day, dusk and dawn are recording labels and supply no image-verified
degradation and return interval. The author repository's raw and
rectified data entries are placeholders. Its dataset and thumbnail
links redirect to sign-in. The public task suite offers processed
benchmark subsets, and this intake did not locate a recording-level
download with the documented motion and reference channels. Even
with that archive, check whether the inertial inputs and pose share
the RT2002 measurements. [Paper, Sections II–III](https://gorokee.github.io/jsyoon/JaeShin_homepage/kaist_multispectral.pdf) ·
[Author repository](https://github.com/unizard/kaist-allday-dataset) ·
[Benchmark suite](https://sites.google.com/view/multispectral/home)

**CMHT autonomous.** The published sensor and label inventories
provide paired RGB/thermal, LiDAR, radar and LiDAR-associated
GPS/IMU, plus object tracks. The documented reference annotations
are per-object boxes and identities. The release description gives
no independent trajectory bound to the sensor rig. Host timestamps
are used for synchronization. Night and rain recordings are listed
as conditions, with no qualified same-run recovery interval.
This route stops at the reference gate. [Paper, Sections 3–4](https://pmc.ncbi.nlm.nih.gov/articles/PMC12175244/) ·
[Dataset DOI](https://doi.org/10.20383/103.01024)

Neither candidate reached the RGB+IMU gate. No full archive or
estimator run was warranted from the material located in this intake.
