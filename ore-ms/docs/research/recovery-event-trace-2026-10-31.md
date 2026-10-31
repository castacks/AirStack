# Saved RGB recovery outlier: prefix, motion and gate trace

2026-10-31 UTC; branch `ore_ms`; source commit `bfce0015e4e009e130bc97d9a56e170da9f3cddf`.

The known outlier was present in the received native RGB motion, before interval projection. Current availability accepted that motion after two unsupported pairs; the fixed history rule withheld RGB. The combined learned gate received the prior-support flags, but its night-excluded training rows had no such recovery pattern and the corresponding coefficients were zero. This explains the saved replay decisions. It does not identify the physical failure cause or justify a new rule.

This follows the [matched evidence synthesis](temporal-gate-findings-2026-10-31.md) and [EXP091 fixed gate comparison](../../experiments/EXP-091-temporal-gate-development.md). The target is the already-exposed `EXP089/middle/96`, not a new test. The trace includes pairs 94, 95 and 96 of `td3/20251027/night_mac_salon_warehousefig8`. All original choices, losses and fit artifacts remain unchanged.

## Health and support at the original decisions

| Pair | Expert | Selected / retained | Occupied cells / 9 | Original need_interp | First matching motion available |
|---:|---|---:|---:|---|---|
| 94 | rgb | 0 / 0 | 0 | true | no |
| 94 | thermal | 200 / 192 | 9 | false | yes |
| 95 | rgb | 0 / 0 | 0 | true | no |
| 95 | thermal | 172 / 165 | 8 | false | yes |
| 96 | rgb | 18 / 18 | 3 | false | yes |
| 96 | thermal | 177 / 168 | 9 | false | yes |

Pairs 94/95 have no RGB optimization submission or matching first motion write; their original RGB health marks interpolation required. Thermal remains supported. Pair 96 has complete RGB health, no execution error, 18 retained points and a first matching motion write whose target support flag is false for need_interp. Raw availability therefore considers both cameras available and uses its fixed RGB preference.

This availability contract checks current target support, interval match and received motion. It does not require the preceding pair to be supported. At the same observed prefix, the existing two-interval history rule sees RGB unsupported on pair 95, thermal supported on both 95 and 96, and chooses thermal (`only_thermal_confirmed`). This is the original rule, not a repair introduced by this trace.

## First-write evidence and controller ordering

Target decision: timeline sequence 1379, controller timestamp `316069988274075` ns. RGB submission 1 is event 193 (timeline 1360); health is event 195 (1362); first motion write is event 197 (1376). The later of RGB health/motion receipts is `316069981142449` ns, 7.131626 ms before the decision. Thermal health/motion events are 483/485; its later receipt is `316069507201101` ns, 481.072974 ms before the decision. These are differences on the same saved controller clock, not live latency measurements or sensor/controller synchronization.

RGB first-write parent/frame indices are 96/97. The captured parent camera-map pose at that write is identity; its committed target translation is approximately (-0.160389, -0.693684, -0.676581) in the recorded camera convention. The preceding pair’s last observed health marks its new frame 96 as needing interpolation. The trace does not substitute a later interpolated parent pose. Inverse-parent × committed-target reconstructs the recorded camera delta, and nominal body conjugation reconstructs the saved native body motion.

The recorder also saved the optimizer pose before writing, but `optimizer_convergence` and `pose_covariance` are null. A written result and a false target need_interp flag are not a convergence certificate or calibrated pose uncertainty. Neither the identity parent nor the sparse support alone proves a physical cause.

The controller prefix already contains newer-sensor-interval pair-end events: RGB event 198 (timeline 1377) and thermal 488 (1371). Both arrived before the pair-96 decision. The declared current/two-prior feature contract nevertheless excludes those newer intervals; the trace verifies their sequence IDs are absent from the gate’s health evidence. Receipt causality and which sensor intervals a model permits are distinct constraints.

## Interval projection did not create the RGB outlier

RGB native and requested intervals are identical: `[1761615394809864998, 1761615394909902096]` ns, duration 100.037098 ms. Its duration ratio is exactly 1 and outside-native horizon is 0. The stationary-twist projection differs from the native body matrix by at most 3.34e-16 in any entry. Its native body translation norm is 0.956979 m under the nominal convention. The large RGB increment therefore predates time scaling.

Thermal native interval is `[1761615394802999973, 1761615394903000116]` ns. Its duration ratio is 1.00036954947, outside-native extent 6.901980 ms, within the declared one-native-duration horizon. Independent SE(3) matrix logarithm/exponential reproduces this projection. The nominal acquisition clocks and physical transforms are still unverified.

The saved recorded-SLAM proxy losses are RGB 1.003081 m / 50.187642 degrees and thermal 0.040678 m / 1.337601 degrees. These are existing model-relative proxy errors, not a new physical ground-truth measurement. Model, frame, sensor-clock and live-deadline admission flags remain unverified.

## What the fixed gate observed

RGB current retained/selected and inbound/selected fractions are both 1 despite the absolute count of only 18; the seven-coordinate feature schema does not include absolute counts. Occupancy 3/9 is visible. RGB prior two support flags are both 0, while current support is 1. Thus the primary gate did receive the recovery pattern; the observations were not hidden by a missing feature read.

Each of the four gate families used its fit that excludes this entire night recording. Its 144 eligible donor rows come from the two day recordings. None has an unsupported immediately preceding RGB pair. In the combined model, all three RGB support coordinates are constant 1 in training, normalized with scale 1, and have exactly zero coefficients:

| Coordinate | Event value | Training min/max | Coefficient | Margin contribution |
|---|---:|---|---:|---:|
| rgb/lag0/supported | 1 | 1 / 1 | 0 | 0 |
| rgb/lag1/supported | 0 | 1 / 1 | 0 | -0 |
| rgb/lag2/supported | 0 | 1 / 1 | 0 | -0 |

The health-only control also includes prior support; uncertainty-only and
current-only omit those prior support coordinates by design. Fixed-model margins
and choices on the same target are:

| Family | Margin | Action |
|---|---:|---|
| combined | 11.291418 | RGB |
| uncertainty | 2.016984 | RGB |
| health | 41.094619 | RGB |
| current | 12.816137 | RGB |

The complete affine decomposition is saved, including all train-only ranges,
normalizers, standardized values, coefficients and contributions. For example,
the combined model’s current RGB occupancy contributes +9.368941 to the RGB
margin on this event, while its previous RGB retained fraction contributes
-9.741408. These are correlated-feature algebraic contributions, not causal
feature effects. The final combined margin is +11.291418. It is a cost-sensitive
score, not a calibrated correctness probability.

The donor support flags cannot acquire a recovery response when they never vary
in training. This finding identifies a specific coverage limitation of this
fixed fit. It does not establish that more capacity, more repeated windows,
absolute counts or another threshold would generalize. The negative development
result is preserved; no family or feature set was replaced.

## Validation, decision and next bounded step

Existing production policy/model/feature code was replayed on the exact three
prefixes. Discrete fields agree exactly with the frozen results; maximum numeric
replay discrepancy is 3.34e-16. The initial attempt lacked the module path; an
exact-float equality check then rejected floating-point roundoff across runtimes.
Both attempts are retained. The successful comparison uses a 1e-12 absolute
numeric tolerance and exact discrete identities, with no changed decisions.

A separate auditor imports no production policy, feature or projection code. It
reconstructs current/previous availability from source prefixes, quaternion pose
matrices, inverse-parent deltas, nominal body conjugation, SE(3) logm/expm
projection, all feature coordinates and all four margin decompositions. It checks
3,375 recursive values/schema bindings; maximum independent matrix discrepancy
is 2.56e-15. Eleven semantic mutations are rejected, including late receipts,
prior support, first-write identity, parent pose, projection ratio, gate
features, excluded-recording model, training recovery count, margin contribution,
newer-interval exclusions and missing pair. Key reordering passes. Status:
`PASS_INDEPENDENT_RECOVERY_TRACE`. No new unit suite was added for this bounded
analysis; scripts and raw validation are preserved.

No gate, support policy or production default was changed. The existing
[motion admission boundary](arbiter-motion-output-2026-10-30.md) remains in force;
these nominal projections are not promoted to physically admitted motions. No
neural inference, fit, new recording, reserve access or owner message occurred.

The next bounded repository step is a predecessor-evidence inventory across the
existing frozen prefixes: bind the exact first-write parent pose/index to its
last observed support evidence, distinguish unsupported, absent and supported
predecessors, and count which candidate motions each case affects. This should
establish what a separate motion-state contract can actually verify. It must not
turn this single known outlier into an automatically adopted rejection rule or
an outcome-selected evaluation cohort. Recording-specific physical/reference
provenance, checkpoint membership and live timing remain separate open inputs.

Artifacts: `results/recovery-event-trace/run-001/{trace.py,audit.py,protocol.json,trace.json,validation.json}`.
Both scripts ran in CPU Docker image
`sha256:af124653ef9cd76660bb8a91f5ae609d6488d41fc78269fe03b3f484c5d35fd1`,
with one BLAS/OMP thread. The replay script needs MAC-VO-main on PYTHONPATH;
the independent auditor does not import it. Source/script/output hashes bind the
entire run. Recovery: `ore_ms:ore-ms/backups/2026-10-31-recovery-event-trace-final`,
incremental history based on `bfce0015e`, bounded metadata, trace/audit scripts,
outputs and lab records. No images, checkpoints, credentials or runtime binary
are included.
