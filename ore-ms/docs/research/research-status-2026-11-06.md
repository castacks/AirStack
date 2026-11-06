# RGB–thermal VO research status

2026-11-06 UTC, after EXP151. [Latest findings](cross-recording-stereo-depth-findings-2026-11-06.md).

Recoverable RGB/thermal ORB and DROID full-record development baselines exist. Thermal tracking support is generally more continuous, while its translation error remains higher. The initialized native constructor repair in EXP150 removes a demonstrated memory-dependence bug and leaves the exposed night travel shortfall near 0.61.

EXP151 extends native depth diagnosis to day: thermal native/projected-LiDAR depth ratio 0.865507 day versus 0.826784 night; reference-only consistent subsets 0.871988/0.874161. The projected reference remains sparse and physically unqualified. This pattern extends across two exposed recordings, without identifying a cause or correction.

Patch-gap/error association is modest and weaker in day: median within-image rho −0.114880 versus −0.191699 night, negative 15/20 and17/20. Descriptor cues are weak/inconsistent. All 80pairs have identical two-pass geometry and 2,938 native-cost reconstructions pass. No calibrated confidence, selector or fusion improvement is validated.

Next test fixed depth/spatial confounders before a confidence policy, and continue acquisition/export timing, visibility and calibration provenance work. Defaults/reserves/EXP133 physical/online gates remain unchanged; no scale/time fit or learned/adaptive promotion.
