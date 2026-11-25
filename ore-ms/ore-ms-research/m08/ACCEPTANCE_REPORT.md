# M08 acceptance report

- **Experiment** ORE-MS-EXP-PER-001
- **Directive** M08-R1
- **Recommendation** ACCEPT WITH CONDITIONS, limited to the declared synthetic fixed-path conditional model. All ten verification gates passed. Director review, calibrated robustness under process/nuisance mismatch, and external and physical validation remain open.

The nonlinear path average cannot serve as a sufficient Markov state. M08 therefore propagates a distribution over all five M05 coefficients, weighted by the current M07 likelihood when eligible. It outputs burden and distinct RGB/LWIR transmission distributions over 0, 2, 6, 18 and 48 s, with separate privileged oracle forecasts. The [model](SCIENTIFIC_MODEL.md), [sufficiency analysis](STATE_SUFFICIENCY.md) and [data contract](DATA_CONTRACT.md) document the assumptions and permitted information.

The 192-realization population includes 128 independent holdout realizations. At 2 s joint burden CRPS was 0.08063 versus 0.11455 for climatology and 0.10234 for persistence. Joint and RGB-only were practically tied. Joint burden interval coverage ranged 93.75–96.88% when matched. Wrong dynamics reduced 2 s coverage to 69.53%. Wrong-air issue coverage and poor warning sensitivity were preserved. [Baseline and calibration analysis](BASELINE_ANALYSIS.md) · [Verification report](VERIFICATION_REPORT.md)

The live roadmap records the M07 restricted acceptance and AC16 closure. AC10 and AC17 remain open, and the register adds M08 decision and model-discrepancy gates. M01–M07 history is unchanged. M09 awaits explicit authorization. No learned forecasting or fusion was started.
