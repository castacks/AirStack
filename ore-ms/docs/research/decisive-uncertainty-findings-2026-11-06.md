# Decisive whole-record uncertainty test: stopped without validated benefit

2026-11-06 America/New_York. Execution freeze `3ec9ed240`; unchanged EXP131 angular rule, exact recovered MAC/MACT runtime and weights. One attempt per expert, complete original recordings, no source/parameter/threshold substitutions.

The frozen primary outcome is **STOP_INSUFFICIENT_EVALUATION**. Night joint reference-qualified support is 89.68%, below the predeclared 95%. This is an evaluation-sufficiency stop, not a formal rejection of the uncertainty hypothesis. The retained matched comparisons also provide no adaptive benefit over RGB: the rule selects RGB throughout the matched day intervals and is slightly worse at night. Day oracle opportunity is below the required 10%. No passing pooled score or alternate rule rescues these gates.

All four continuous native runs completed. The original source axes contain 5,695 day groups and 5,234 night groups (43,716 camera images). All 21,858 expert-frame outputs and failures/support flags are preserved. Both predictions locked at 2026-11-06T07:58:26.556334+00:00 before either new RGB odometry archive was acquired. Native execution used the original run() API without the 100-frame smoke CLI bound or window resets.

| Recording | All adjacent intervals | Matched intervals | Joint coverage | Rotation oracle headroom | ≥95% coverage / ≥10% headroom |
|---|---:|---:|---:|---:|---|
| day | 5694 | 5416 | 95.12% | 9.04% | PASS / FAIL |
| night | 5233 | 4693 | 89.68% | 12.20% | FAIL / PASS |

The following are descriptive means on each recording's identical matched support. They do not override the insufficient two-record evaluation. Rotation is adjacent-motion error in degrees; translation is metres, using the frozen nominal common-target/provided-reference convention.

| Recording | RGB rotation | Thermal rotation | Angular rule rotation | RGB translation | Thermal translation | Angular rule translation |
|---|---:|---:|---:|---:|---:|---:|
| day | 0.046486 | 0.131621 | 0.046486 | 0.167096 | 0.455033 | 0.167096 |
| night | 0.050481 | 0.149451 | 0.051148 | 0.158298 | 0.603736 | 0.165873 |

Day angular rule equals RGB and cannot meet the required 5% improvement against RGB or the day metadata baseline. Night angular rotation is 1.32% worse than RGB; translation is 4.79% worse (within the separate 5% translation guard). Thus neither recording meets the candidate-benefit gate. Metadata is RGB by day and thermal by night. Raw uncertainty remains diagnostic only; its unchanged scores are retained in the outcome JSON.

| Expert | Completed poses | Unsupported native transitions | Observer errors | Elapsed minutes |
|---|---:|---:|---:|---:|
| day rgb | 5695 | 0 | 0 | 37.51 |
| day thermal | 5695 | 240 | 0 | 13.95 |
| night rgb | 5234 | 3 | 0 | 35.15 |
| night thermal | 5234 | 493 | 0 | 12.11 |

Native unsupported flags are preserved separately from joint eligibility: missing/nonadjacent/unsupported/termination-only motion, invalid common-target projections and invalid reference endpoints are retained in the full denominator. Exact exclusion reason counts and every excluded source ID are in the saved scores and outcome. The selector can use the sole valid expert for availability; union availability does not replace the declared both-expert accuracy coverage gate. All reference pose qualification reports are retained.

RGB has lower matched rotation and translation proxy error in both recordings, and fewer native unsupported transitions. These data do not support a general claim that thermal is more reliable. Thermal executed faster on this hardware and input configuration, but this completed-output selector ran both experts; it demonstrates no compute saving. Runtime does not establish accuracy.

Stop this fixed-rule evaluation route and report the coverage/method characterization. Do not relax coverage/headroom, retune the signal, reinterpret interpolated/termination-only output as native support, rerun for better outcomes, or reuse these recordings as fresh validation. Patch-gap confidence remains closed and thermal-bias microdiagnostics parked. The two recordings are now exposed for future development.

Scope: provided RGB reference proxy and declared common optical-frame/zero-offset unit-rate clock model, with original stationary-twist transport; physical synchronization/frame truth unverified. Unknown checkpoint training overlap; project-unexamined status was qualified only within audited recorded lineage before this test. Adjacent-motion results do not establish full-trajectory or online scheduling performance.

Validation: development replay matches all 19 saved intervals / 18 eligible projections; six analytic gate fixtures; independent decision reconstruction passes 41248 checks; source/model/full-native counts and post-prediction reference gating independently pass. Native and scoring first attempts completed without quality retries. Both original sync archives pass full Box downloaded-byte SHA256 checks.

Artifacts: `results/decisive-uncertainty-evaluation/run-001`, frozen plan and outcome JSONs in `experiments/`, native captures/receipts, prediction locks, RGB references, all scores/exclusions and read-only audit sources/results. Recovery: `ore_ms:ore-ms/backups/2026-11-06-decisive-uncertainty-evaluation`; original sync archives are separate objects, runtime/models are the exact prior recovery prerequisites.
