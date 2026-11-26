# Reliability definitions

M10 task reliability is **the calibrated probability of a correct visible-target localization, given permitted issue-time inputs, a stated operating regime, and a declared decision/coverage rule**. Report it separately per modality and for the resulting decision, with both modalities' joint errors measured on matched realizations. It is not yet available from prior milestones. Reliability under missing or abstained cases must remain distinguishable from score conditional on issuance.

| Quantity | Definition | Current evidentiary status |
|---|---|---|
| Environmental transmission | Fraction of direct radiance transmitted along a defined path and wavelength. | M02/M03 modeled and M08/M09 forecast under qualified synthetic constraints. Strong transmission can concern a nearer foreground while the target is entirely hidden. |
| SNR | Signal level divided by an explicitly defined noise level under detector calibration and exposure. | Controlled detector parameters and residual-based diagnostics, with clipping caveat. M02's bright haze case clipped 49.59% of channels despite 35.42 dB code-residual SNR. [M02 acceptance](../../m02/ACCEPTANCE_REPORT.md) |
| Saturation | Detector rail censoring. | Observable code/flag proxy. True pre-rail signal and exact lost contrast are unavailable from the clipped output. |
| Geometric visibility | Target first-hit support relative to in-frame target projection. | Exact synthetic labels for scoring. Future or physical visibility requires independent evidence. High transmission cannot establish it. |
| Radiometric contrast | Target minus background radiance or code contrast, tagged with units, modality, spectral band and detector configuration. | Supported in known synthetic matched regions. Contrast measured on candidate regions with unknown association needs task calibration. Thermal crossover can erase it at unit transmission. |
| Estimation uncertainty | M07 distribution for current constrained burden. | Conditional matched-case calibration. Small interval width does not show physical model validity or local identifiability. |
| Predictive uncertainty | M08/M09 horizon-specific marginal uncertainty for path burden or reference transmission. | Time- and model-conditional and can undercover under shift. It describes neither actual future target visibility nor a task-level error rate. |
| Expected downstream performance | Frequency/probability of correct task outcomes, conditional on available evidence and declared coverage. | Requires prospective M10 calibration and evaluation. Physical use also requires later validation. |

Only integrity flags and independently constrained nuisance contracts are immediately decision-eligible as gates. Contrast, SNR and environmental estimates can serve as experimental covariates. To turn a covariate into a reliability weight, calibrate against the downstream event on disjoint data, measure common-mode residuals, and verify performance at matched coverage. Use separate denominators for available pairs, supported localizations and all test opportunities.
