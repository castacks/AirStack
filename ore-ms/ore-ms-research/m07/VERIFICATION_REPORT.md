# M07 verification report

The public model, private generation registry, observation contract and acceptance criteria were frozen before evaluation. Verification uses analytic equal-output constructs, independent RGB spectral quadrature/inversion, a separate Jacobian check, independent holdout scoring, and an isolated deployed estimator. This is procedural separation within one development effort.

Run `python3 verify.py "$out"` after generation and estimation as in [reproduction](REPRODUCTION.md).

| Gate | Independent reference and measured result | Frozen tolerance | Outcome |
|---|---|---|---|
| V01 inversion | Known-range optical depth inverse error 1.39e−17 m⁻¹. Independent spectral red-band inversion max error 4.44e−16 burden. | 2e−12 m⁻¹ and 2e−10 burden | PASS |
| V02 structural ambiguity | Matched range/extinction, attenuation/source and thermal ε/T counterexamples. Maximum difference zero. Crossover, blocked and rail conditions demonstrated. | 2e−10 equal-output residual | PASS |
| V03 observability | Analytic scaled Jacobians and centered differences agree within 2.72e−10. Single unknown range/extinction rank 1/2, unconstrained two-band rank 2/3, known burden rank 1/1, degenerate probes rank 0. | 2e−6 derivatives, expected ranks at 1e−8 relative | PASS |
| V04 recovery | Matched holdout MAE 0.03339/0.12947/0.03235 for RGB/LWIR/joint, absolute biases 0.000142/0.01281/0.000563 and zero hard abstentions. M06 fixed RGB max error 0.001636. | Bias 0.035/0.08/0.035; MAE 0.065/0.21/0.065; M06 max 0.006 | PASS |
| V05 coverage | Matched empirical 95% interval coverage 94.92%, 96.48%, 94.53%. Mismatched-air results substantially under-cover and are retained separately. | The three matched values within [0.88,0.995] | PASS matched |
| V06 modality contribution | Probe information sum 278.09446+32.38002=310.47448 within numerical tolerance, still rank 1. Joint MAE decreased vs RGB; M06 joint exactly equals RGB outputs. | 2e−10 relative additivity, joint MAE ≤RGB+0.02, conditional M06 equality | PASS |
| V07 degeneracy | Both blocked arms, nuisance constraints withheld, and saturated controls abstain. Crossover LWIR abstains, severe RGB has 256 weak cases, missing/invalid paths handled. Deliberate mismatch worsened joint MAE to 0.04752. | Exact prescribed flags and preserve mismatch failure evidence | PASS |
| V08 temporal filter | Framewise independent replay at 0/6/12 s. | Sequential filtering optional under M07-R1 | NOT TESTED |
| V09 leakage prevention | 6,144 features and 24,576 mode outputs. Disjoint realization partitions, forbidden field mutation unchanged, extra column rejected, standalone inference ran with labels/private generation absent. | Exact partitions and feature allowlist | PASS |
| V10 replay and regression | All serialized M07 outputs reproduced byte for byte. M01–M06 manifests and regression suites passed, roadmap invariants preserved. | Exact content and scope | PASS |

The [full metrics](evidence/metrics.csv), [observability spectrum](evidence/observability.csv), [oracle controls](evidence/oracle.csv), [verification log](evidence/verification.log), and [holdout analysis](HOLDOUT_ANALYSIS.md) retain numeric values and failure modes. This verifies synthetic conditional recovery. Formal independence and real-world validity remain open.
