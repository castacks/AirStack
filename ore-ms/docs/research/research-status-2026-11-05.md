# RGB–thermal VO research status

2026-11-05 America/New_York, after EXP152. [Latest findings](stereo-ambiguity-controls-findings-2026-11-05.md). EXP151 filenames retain their earlier host-UTC date 2026-11-06.

Recoverable RGB/thermal ORB and DROID development baselines exist. Thermal tracking support is generally more continuous while translation errors remain higher. EXP150 fixes a native constructor memory-dependence defect, with night thermal travel/reference still near 0.61. EXP151 extends shorter native/projected-depth ratios across day/night (0.865507/0.826784; reference-consistent 0.871988/0.874161). Those are conditional sparse-projection comparisons, without an identified physical cause or fitted correction.

EXP152 tests the modest patch-gap cue under fixed depth/location groups. Primary 30-point joint controls have no supported image. Predeclared 10-point sensitivity gives thermal all-primary joint coefficients −0.066791 day/−0.043283 night, retaining median 32.1%/33.3% of points in defined images. Signs are mixed (8/15,8/14 negative), and consistent-reference results vary. All original raw correlations reproduce. The evidence does not support promoting the cue to a confidence or camera-selection policy.

Prioritize supported reference/correspondence and acquisition/export timing, visibility and calibration validation before confidence learning or adaptive fusion. Defaults/reserves/EXP133 physical/online gates remain unchanged. No new inference/acquisition, scale/time fit or learned/adaptive promotion occurred in EXP152.
