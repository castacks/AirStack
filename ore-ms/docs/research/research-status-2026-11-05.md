# RGB–thermal VO research status

2026-11-05 America/New_York, after EXP150. [Latest findings](stereo-baseline-initialization-findings-2026-11-05.md).

Recoverable RGB/thermal ORB and DROID full-record development baselines exist. Thermal ORB generally retains more tracking support while translation errors remain higher; support alone does not imply better accuracy.

The fixed exposed night prefix showed thermal travel compression near 0.61, substantial native stereo matches, and conditional sparse projected-LiDAR depth ratios below one. Physical timing, projection, visibility and calibration qualification remain unresolved. EXP149 found a modest patch-cost ambiguity/error association, without validated confidence or fusion benefit.

EXP150 isolates a concrete native constructor defect: baseline was read before assignment. The minimal repair makes synthetic stereo geometry independent of storage patterns. An unchanged-source clean rebuild is byte-identical to the original library. Six original/rebuilt/fixed RGB/thermal captures complete. Repaired thermal common final travel/reference is 0.613906, translation RPE 2.8326 m versus RGB 0.6435 m. One native realization per arm does not establish a causal accuracy gain. Historical outcomes and the default runtime remain unchanged.

Next qualify initialized stereo/depth evidence across recordings and obtain acquisition/export timing and visibility/calibration provenance before a correction or confidence policy. No learned selector, adaptive fusion gain, heldout/full-record/online qualification, reserve/default change or EXP133 promotion has been adopted.
