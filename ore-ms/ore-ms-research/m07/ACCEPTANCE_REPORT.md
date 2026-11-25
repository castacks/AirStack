# M07 acceptance report

- **Experiment** ORE-MS-EXP-PER-001
- **Directive** M07-R1
- **Recommendation** ACCEPT WITH CONDITIONS, limited to scalar effective burden under independently constrained range, material, and admitted path-source conditions. Project Director approval remains pending.

A 401-cell framewise Bayesian inverse estimates dimensionless path-average burden b∈[0,1]. Attenuation, path radiance, detector settings, prior and noise assumptions are explicit. Known and unknown nuisance information are identified in the [scientific model](SCIENTIFIC_MODEL.md) and [data contract](DATA_CONTRACT.md). RGB and LWIR each support the constrained scalar in the matched synthetic setting, with joint rank one. With independent unknown extinction coefficients and unknown range, two bands have rank two in three parameters. Known thermal crossover, occlusion, and rails can reduce available information to zero. See [structural and local observability](OBSERVABILITY.md).

The matched holdout provides 256 independent scored realizations per mode. RGB/LWIR/joint MAE was 0.03339/0.12947/0.03235, compared with prior MAE 0.24026. The 96 fixed M06 holdout time samples gave RGB MAE 0.000654. They use an externally specified range and target condition. Unknown M06 thermal path emission precludes ordinary LWIR recovery. Deliberate air mismatch produced substantial undercoverage in the controlled inverse. See the [holdout analysis](HOLDOUT_ANALYSIS.md) and [verification](VERIFICATION_REPORT.md).

All applicable gates passed and the sequential test is outside this framewise implementation. M01–M06 checksums and regression replays passed. AC14 closed upon M07 authorization. AC10 remains open for unknown-range/extinction and latent-driver identification. Separate validation, model-discrepancy, and Director review gates are recorded in the [acceptance register](../ACCEPTANCE_CONDITIONS_REGISTER.md). No M08 forecast, filter, training or fusion work was started.
