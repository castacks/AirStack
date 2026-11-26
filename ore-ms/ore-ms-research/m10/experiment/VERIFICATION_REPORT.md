# Verification report

| Gate | Criterion | Independent or replay evidence | Disposition |
|---|---|---|---|
| V01 | Frozen population, frontends, calibration, labels and B03 score preserved | Preflight replay from archived observations and exact raw output manifest. | PASS |
| V02 | All realization groups retained and split disjoint | PF1 independent audit and adaptive scorer enumerate all groups and scenarios. Calibrator rejects wrong split cardinality or group ranges. | PASS |
| V03 | Development-only fit and calibration-only cutoff and probability mapping | Pre-holdout receipt binds protocol, model, source, prediction and calibration inputs. Holdout checks receipt before staging observations. | PASS |
| V04 | Bounded deterministic policy, association, fallback and dependence | Hand-calculated fixtures cover clip bounds, convex weights, independent score, residual dependence, conflict, both missing, and a bracketed scalar optimizer reference. | PASS |
| V05 | Isolation, acquisition, masks and forbidden fields | Isolated Linux process denies reads against a real scoring file. Score mutation leaves identical bytes. Six illegal input fields are rejected. Four equivalence families repeat exactly. | PASS |
| V06 | Safety reference | Independent zero-event Wilson upper endpoint agrees to 1e−12 for every nonprivileged arm in both primary cells. Negative cases and outcomes preserved. | PASS for primary cells. Distractor and 4/20 claims withheld. |
| V07 | Scorer and statistics | Independent rational squared distances, grouped resampling and nearest-correct candidate enumeration reproduce the primary estimate and oracle ceiling. Max CI discrepancy 4.17e−17. | PASS |
| V08 | Context eligibility | M07, M08 and M09 admit 0/6,912 each under the immutable corpus contract. Context fallbacks match E03. | PASS for admission. Effect studies unsupported. |
| V09 | Reproducibility and historical integrity | Two clean M10-R1 runs reproduce 11 scientific outputs byte for byte. M01–M09 regressions and RG1 integrity pass. | PASS |
| V10 | Authorized scope | Changes are scoped to M10-R1 and one roadmap progress row. No downstream experiment was begun. | PASS |

Run the freeze and holdout commands in the reproduction instructions to recheck the full gate set. Timing and raw test duration lines are measured outputs with run-to-run variation. [Independent verification](evidence/verification.json) · [Historical regression log](evidence/prior_regressions.log) · [Reproduction](REPRODUCTION.md)
