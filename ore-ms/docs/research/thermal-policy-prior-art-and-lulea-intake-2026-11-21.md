# Thermal update admission and independent recording intake

21 November 2026. This intake asks whether an available thermal motion update
can be judged accurate enough to admit during observed visible degradation.
The existing controlled-dimming transfer and the failed IRS dual-camera
qualification are recorded in the [research handoff](../../MS_RESEARCH_HANDOFF.md).

## Comparisons motivated by prior work

| Work | What it contributes to the experiment design |
| --- | --- |
| [Brunner et al.](https://doi.org/10.1002/rob.21464) | The published abstract describes selective visible–thermal SLAM and compares it with single modalities and unfiltered combination under smoke and lighting changes. Those comparisons are prior art. Exact admission rules and reference methods remain unchecked because the full paper was unavailable. |
| [Khattak et al.](https://arxiv.org/pdf/1903.01656) | The previously reviewed methods use entropy to select image regions and feed visible and thermal landmarks to an inertial filter. The mine experiment reports an endpoint against a surveyed marker. This motivates measuring observation quality and scoring recovery throughout the interval, alongside any endpoint. |
| [Zhao et al., SuperLoc](https://arxiv.org/abs/2412.02901) | Sections on alignment risk and active fusion use LiDAR geometry to predict weak pose directions and selectively incorporate external odometry. This motivates evaluating rotation and translation separately when testing confidence signals. The LiDAR method supplies no validated camera admission cue. |

Together these papers motivate a comparison between **unconditional thermal
admission and conditional admission on the same eligible update set**. A
degradation cue determines when thermal should be considered. The candidate
admission score must use data available by the decision time. A held-out
reference can score accepted and rejected updates against a declared error
tolerance. Compare risk at matched coverage and report the resulting misses,
latency, and cost. Recent history is a candidate input whose incremental value
requires a same-frame control. The papers establish precedent for selective
fusion and direction-dependent confidence. No estimator or policy was run here.

## Luleå underground transition candidate

Koval et al., Table 3, identify illumination changes in the B–C traverse of
two separately recorded sensor sets. Set 2 starts to receive illumination
around 85 s and has UWB positions only in the first 30 s and from 160 s.
It has T265 grayscale fisheye, thermal, and IMU. Set 3 starts to receive
illumination around 84 s and provides D455 RGB, thermal and IMU, with no
UWB in the stated sensor configuration. UWB on set 2 gives point positions,
with a gap across its lighting transition. It gives no attitude reference
for that interval. The measurements from one traverse cannot be treated as
reference for the other. [Author paper](https://doi.org/10.1016/j.robot.2022.104168)

Verified the byte ranges and ZIP central directories from the published data.
Each archive needed one 1 MiB suffix. Set 2 contains seven bags, including
three B–C members. Set 3 contains seventeen bags, including nine B–C
members. Both central directories contain bag payloads only. The separate
URDF archive matches the published MD5 and passes member CRC. It supplies
fixed mounting transforms, including D455, T265, Boson and Decawave.
The URDF does not supply optical intrinsics or a trajectory reference.
Bag internals, image timing and calibration topics were outside this check.
[Public release](https://zenodo.org/records/6948960)

Set 3 B–C remains a candidate for field frontend characterization. The
inspected release does not qualify either set for independent RGB–thermal
metric recovery across the lighting boundary. No full bag was retrieved.
Range receipts, inventories, URDF and hashes are under
`/root/ore-ms-artifacts/lulea-subt-intake/`.
