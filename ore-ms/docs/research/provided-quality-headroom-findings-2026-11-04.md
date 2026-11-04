# EXP130: quality headroom on provided exports

2026-11-04, branch `ore_ms`; parent `4da8557cd`, scoring/protocol freeze `6324a366f`.

All 323 fixed jointly eligible intervals pass the provided-reference support checks. Retrospective selection has rotation headroom against recorded SLAM, while translation headroom is small. This is a development proxy result with unverified physical mounting and sensor-clock equivalence.

| Matched metric | Fixed RGB mean | Fixed thermal mean | Oracle mean | Reduction from best pooled fixed |
|---|---:|---:|---:|---:|
| Translation (m) | 0.24859978 | 0.12710241 | 0.12693197 | 0.134% |
| Rotation (degrees) | 0.25021140 | 0.23224070 | 0.16556426 | 28.710% |

Translation reduction is 0.00017044 m (0.17044 mm) per interval, against a best-fixed mean of 0.12710241 m. RGB has lower translation discrepancy on 59 rows and thermal on 264; winner counts alone would overstate the useful translation improvement.

For rotation, RGB wins 108 rows and thermal 215. Allowing an oracle to choose the best fixed camera separately for each window lowers the comparator mean to 0.21076723 degrees. Per-interval selection still lowers that mean to 0.16556426 degrees: 0.04520297 degrees, or 21.447%. Thus some pooled gain comes from different windows preferring different cameras, and some remains within windows. Both comparators are retrospective.

| Window | Joint rows | RGB rotation mean (deg) | Thermal rotation mean (deg) | Oracle rotation mean (deg) | Gain over window best fixed |
|---|---:|---:|---:|---:|---:|
| night_thermal_so_salon/early | 95 | 0.041592 | 0.016566 | 0.015959 | 3.662% |
| night_thermal_so_salon/late | 38 | 0.046424 | 0.011472 | 0.011451 | 0.187% |
| night_thermal_so_salon3/middle | 95 | 0.253219 | 0.326229 | 0.208646 | 17.603% |
| night_thermal_so_salon3/late | 95 | 0.537338 | 0.442235 | 0.333733 | 24.535% |

Translation and rotation prefer different cameras on 157/323 rows. They remain separate targets; no weighted combination or overall best-camera label is inferred.

![Frozen matched metrics](../../results/provided-quality-headroom/run-001/quality-headroom.png)

The 323 rows are 95 salon/early, 38 salon/late, 95 salon3/middle and 95 salon3/late. All 380 original availability rows remain the coverage population; the 57 thermal-only rows do not enter this matched quality oracle. All 323 matched rows scored, zero excluded. EXP129 thermal availability remains 380/380 with zero coverage headroom. No new native inference, scheduler replay, model fitting, reserve access or candidate change occurred. EXP128 MSA exact-replay failure remains failed; these exact provided-export inputs form a separate cohort.

The two source archives explicitly declare `sensor_init -> vehicle`. Metadata/timestamps were acquired before predictions; reference pose payloads were acquired only after the 323 nominal-body, stationary-twist predictions were locked. Exact integer-ns source timestamps, actual provided rectification matrices and the unchanged Yamaha nominal graph determine the comparison. No fitted alignment, direction, scale, time offset or reference bridge enters prediction. Source timestamp streams contain large gaps elsewhere; every scored endpoint has support and a bracketing gap at most the frozen 150 ms. Linear translation and quaternion SLERP follow the existing loader, with finite/unit/monotonic validation and no extrapolation.

Bounded ZIP member acquisition retrieved 25,368,655 bytes of reference pose text plus frame/timestamp metadata, verified by ZIP CRC and SHA256. The source is interpolated recorded SLAM, and its independence from camera inputs is unresolved. It is not a physical accuracy or GNSS/RTK result. Original network-training overlap is unknown. The four windows from two exposed recordings do not establish generalization or statistical significance.

Validation passed 5,245 independent checks, including five semantic cases, plus eight independent nominal graph/rectification/basis checks. The main audit reconstructs native motion provenance, SE(3) interval projection through independent matrix log/exponential, reference interpolation and endpoint losses, matched populations, and oracle summaries. The supplemental graph audit was added after seeing scores to check source composition; it changes no predictions or scoring rules. Scoring and the core auditor ran from the pre-pose freeze. The protocol pins 2,780 sources/inputs. CPU runtime emitted the known driver-init warning but completed with exit code zero.

Prediction SHA256: `d0babc63ac312a571d1922571b7690e6958d09bdfaae5404089378f2c48f3f0c`. Score SHA256: `8d34b19c71fa3e93d381b686789123e96287187be1f5516cfc1c065bb79ac39d`. Artifacts: `results/provided-quality-headroom/run-001/` (`prediction-lock.json`, `predictions.json`, metadata/pose intake locks and CRCs, `metadata-audit.json`, `scores.json`, `audit.json`, `frame-audit.json`, figures). [Protocol](provided-quality-headroom-protocol-2026-11-04.md).

The next useful development experiment is a predeclared causal signal audit focused on rotation: can already available uncertainty/health information rank the two outputs beyond fixed-camera and simple condition baselines? Freeze feature timing, target and partition before fitting. A selector choosing between completed outputs can use current completed-motion signals at the output boundary; a sparse pre-input scheduler must use earlier history and cannot consume target-motion uncertainty that does not yet exist. The current dense oracle proves neither contract feasible. Existing saved journals support a first bounded signal audit; independent-condition benefit and physical accuracy still need the previously documented reference/clock/mounting and recording evidence.
