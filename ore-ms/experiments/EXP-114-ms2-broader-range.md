# EXP114: broader day/night/rain geometry and covariance comparison

2026-11-02 America/New_York. Sampling ac2cb6615; native/scorer f67473b04; pre-label persistence declaration1e9cda98e. [Full findings and figures](../docs/research/ms2-broader-range-findings-2026-11-02.md).

900pairedframes, nine100groupwindows, three already-exposed MS² recordings; 36expert executions/3600frame executions. All891decision slots retained,889reference-scoreable,852jointly scoreable; RGB/T translation wins718/134. A grossly nonrigid source pose invalidates two intervals under the frozen rule. Original RGB, geometry, recovery and information-rank decisions all identical:882RGB/9abstain. Fixed primary fails on every recording; no model/threshold/architecture selected or sensor skipped.

| Recording | Jointly scoreable pairs | RGB / geometry / information rank T RMSE (m) | Count T RMSE (m) | Thermal T RMSE (m) | RGB / geometry R RMSE (deg) |
|---|---:|---:|---:|---:|---:|
| day | 292 | 0.106984 | 0.106984 | 0.182225 | 0.047707 |
| night | 271 | 0.048920 | 0.057728 | 0.440083 | 0.063381 |
| rain | 289 | 0.284965 | 0.285069 | 0.485106 | 0.031645 |

Supplementary covariance persistence is variable; posthoc fixed dt² rescaling improves adjacent rho17/18series, median0.163→0.742. Future-error association still varies; exposure-clock semantics remain unverified. This grounds a timing-comparability investigation, not an adopted memory model or VO improvement.

99tests; 47034arrays exact off/on with zero diagnostic drift. Independent source/calibration23162checks; native/control/reference audit204599numericchecks,1764H/1736transports; covariance5832; posthoc cadence738. All source/weight/reference pins unchanged. Unknown checkpoint training overlap; two reserves untouched. Backup backups/2026-11-02-ms2-broader-range; model/compiler runtime prerequisite EXP112, base runtime EXP106.
