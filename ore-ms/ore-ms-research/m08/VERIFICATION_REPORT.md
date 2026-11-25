# M08 verification report

Generate, predict, run the separate oracle and score with [the reproduction commands](REPRODUCTION.md), then invoke `python3 verify.py "$out"`. Exact OU formula and midpoint calculations are in the independent `reference.py`. Five automated tests cover V01–V09. V10 was checked through full replay, archive manifests, prior harnesses and repository scope. All results remain synthetic and within one development effort.

| Gate | Independent reference and measured result | Frozen tolerance | Outcome |
|---|---|---|---|
| V01 OU moments | Formula error 1.11e−16 and semigroup error 2.22e−16. On 4096 draws, conditional mean and variance errors 0.01306 and 0.01722. | 2e−12 analytical, 0.07 Monte Carlo | PASS |
| V02 sufficient state | Reflected pair starts at b=0.5 each. At 6 s paired predictive mean difference magnitude 0.16372. Independent midpoint error 1.11e−16. | 2e−12 equality and quadrature, difference >0.02 | PASS |
| V03 initialization | Reconstruct unchanged M07 means to zero measured difference for RGB/LWIR/joint; example SD shrank from 0.19590 to 0.03344 with ESS 369.63. Privileged h=0 oracle is exact. | 2e−12 means, narrower SD, exact oracle identity | PASS |
| V04 space and time | Periodic transport identity, Monte Carlo spatial covariance error 0.005395. At 48 s decay 0.0003355, variance gap 1.125e−7 and mean interval width 0.7081 vs 0.1581 at issue. | 2e−12 transport, covariance 0.07, decay ≤0.001, variance gap ≤1e−6 | PASS |
| V05 physical mapping | All sample values and reference transmissions admissible. M02/M03 benchmark transmission errors zero. | [0,1], reference error ≤2e−12 | PASS |
| V06 calibration and scores | Matched joint coverage 93.75/93.75/96.88/96.88/95.31%. Independent two-point CRPS, binned log score, AUC and Brier arithmetic checks passed. | Coverage [0.83,1] by horizon and scoring checks | PASS matched only |
| V07 baseline matrix | 21,120 deployable rows and 960 oracle rows. Persistence/climatology marginal repeats exact. Same-rank joint/RGB near tie and prior/physics near tie preserved. | Complete matrix and exact persistence | PASS |
| V08 mismatches and guards | Prespecified fallback/abstention statuses passed. Wrong dynamics at 2 s had 69.53% coverage, wrong air at issue 85.16%. Both failures were retained. | Exact guard decisions and reported failure comparisons | PASS handling |
| V09 leakage | 64/128 independent realization split, isolated CLI without oracle/private truth, ID-only change preserved samples, future column rejected. | Exact split and allowlist | PASS |
| V10 replay and scope | All eight data outputs matched byte for byte; M01–M07 archive hashes and replays and roadmap invariants passed. | Exact evidence and no scope expansion | PASS |

[Verification log](evidence/verification.log) · [Baseline and mismatch results](BASELINE_ANALYSIS.md) · [Replay log](evidence/replay.log) · [Prior regressions](evidence/prior_regressions.log)
