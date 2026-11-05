# EXP142: complete-record camera-local provided-reference scores

2026-11-05 America/New_York. Parent EXP141 / 79339e882; freeze c1e5f4a5c.
[Full findings](../docs/research/provided-reference-full-record-findings-2026-11-05.md).

All six saved latest declared DROID streams retain 55,618 finite offline poses.
All 52,276 reference-qualified approximately-one-second RPE pairs are scored;
ten nonrigid reference poses remain explicit exclusions. RGB SE3-only ATE is
23.150/22.856/39.069 m in day/night/rain, versus thermal 24.218/59.439/133.915 m.
Thermal has slightly lower day translation RPE, with higher rotation RPE.

Fifteen tests pass. Independent geometry and raw TAR/member closure total
379,949 checks / 39 rejected mutations. Predictions and scorer remain unchanged.
Historical reference-packet download preceded the new freeze; matrix reads follow
it. Three exposed development records only, not heldout or prospective payload
evaluation. No new inference, scale/time fitting, reserves or defaults.

Own-camera provided conventions retain unresolved physical timing/calibration
limits; no physical accuracy, causal availability or adaptive benefit is established.
Next: frozen complete-record ORB context with the same inputs and metric contract.
Backup: backups/2026-11-05-provided-reference-full-record-exp142.
