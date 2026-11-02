# EXP110: retrospective RGB–thermal covariance lead-time diagnostic

2026-11-02, America/New_York. Frozen selection `e35cef011`, source-transform
clarification and producer `8e86e7f36`. No fitting or candidate adoption.

Thermal covariance serial ordering persists to the tested 1 s future-pair-start
lead (rho 0.595 / 0.257 / 0.660), while its association with future independent-LK
disagreement is 0.622 / −0.137 / 0.058 across three windows of one exposed night
recording. Persistence is not a consistent future-risk signal. RGB target support
is tiny; sustained RGB failure does not provide valid comparative error labels.

Across the three exposed EXP091 recordings, current native trace-gap versus future
translation/rotation proxy associations and a fixed three-lag average vary by
window. No consistent averaging advantage, calibrated cross-modal forecast,
pre-inference latency benefit, physical pose improvement or architecture choice.

[Full findings, all populations and limits](../docs/research/covariance-lead-time-findings-2026-11-02.md).
Protocol: `docs/research/covariance-lead-time-protocol-2026-11-02.json`.
Results: `results/covariance-lead-time/run-001/`. Two boundary/statistic tests pass;
independent source/timing/statistic reconstruction passes with four rejected
semantic mutations. Zero inference/updates/new raw data/reserve access; original
models and production defaults retained. Backup:
`ore_ms:ore-ms/backups/2026-11-02-covariance-lead-time`.
