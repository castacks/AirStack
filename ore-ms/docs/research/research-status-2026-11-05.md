# RGB–thermal VO research status

2026-11-05 America/New_York, after EXP149. [Latest findings](native-match-ambiguity-findings-2026-11-05.md).

The project has operational, recoverable RGB/thermal ORB and DROID development baselines. Full day/night/rain execution and qualified supplied-reference scores exist. Thermal ORB generally retains more tracking support, while its translation error remains higher; support alone is not a comparative accuracy guarantee.

The main diagnosed pattern on a fixed exposed night prefix is thermal ORB travel compression (~0.61 of supplied reference). Finalization has little effect. Native stereo match counts are substantial. Sparse projected-LiDAR comparison shows shorter thermal depths (~0.827 ratio), persisting in reference-only consistent neighborhoods (~0.874). Physical timing/projection/visibility/calibration qualification remains unresolved.

Initialized native replay identifies a modest candidate quality cue: separated patch-cost gap has median within-image rho −0.192 with thermal relative depth-proxy error, and −0.233 in the existing consistent subset. Descriptor margins are inconsistent. The 40-pair test reproduces all 1,398 evaluated historical points, with two unrelated first-frame differences. This is exposed, conditional association rather than validated confidence or camera-selection/fusion benefit.

A source-order issue was found: native stereo construction assigns baseline after the matcher uses it. The replay explicitly initializes geometry; historical SLAM code was not repaired or re-scored. Next isolate that repair with original/patched controls, then validate the candidate cue across recordings and reference conditions. No new architecture, learned selector, adaptive gain or production change is adopted. Existing reserves/defaults and EXP133 physical/online readiness gates remain unchanged.
