# M02 acceptance report

- **Experiment** ORE-MS-EXP-PER-001
- **Directive** M02-R1
- **Recommendation** ACCEPT M02 WITH CONDITIONS. Verification supports the synthetic visible-light baseline under its declared assumptions. Physical camera and atmospheric validity and organizationally independent verification remain open. Project Director acceptance is pending.

[M02 requirements traceability](REQUIREMENTS_TRACEABILITY.csv), [scientific model](SCIENTIFIC_MODEL.md), [frozen protocol](EXPERIMENT_PLAN.md), [verification report](VERIFICATION_REPORT.md), and [reproduction instructions](REPRODUCTION.md) document the implementation and evidence. The [Research Roadmap Mandate](../RESEARCH_ROADMAP_MANDATE.md) controls subsequent authorization.

## Executed experiments

Each configuration contains 18 fixed cases and 26,136 channel records. The primary configuration covers clear conditions, extinction and distance sweeps, out-scattering and path radiance, isotropic and forward-weighted phase, layer order, illumination, noise and combined conditions. The separate headroom diagnostic repeats the matrix at a higher full well. Exact values for all arms are in the [primary metrics](evidence/primary_metrics.csv) and [headroom metrics](evidence/headroom_metrics.csv).

| Primary comparison | Result |
|---|---|
| Increasing pure extinction and distance | Direct radiance fell in each band. SSIM fell from 0.9953 at low extinction to 0.6003 at high extinction. Short and long range SSIM were 0.9929 and 0.6003. High extinction and long range had the same optical depth at every wavelength. |
| Matched out-scattering, isotropic source, and forward-weighted source | Green contrast fell from 0.6153 to 0.5643 and 0.5041. SSIM rose from 0.8838 to 0.9112 and 0.9391 as path radiance moved digital brightness closer to the reference. |
| Layer reversal | Direct radiance remained equal while path radiance changed. Green contrast moved from 0.5918 to 0.5835. |
| Noise-only and alternate seed | Code-residual SNR was 34.42 and 34.61 dB. |
| Dim illumination with haze and noise | SNR was 23.21 dB, SSIM 0.1562, and equivalent input radiance RMSE 0.2113 W·m⁻²·sr⁻¹. |
| Bright illumination with haze and clipping | 49.59% of channels upper-clipped and SSIM was 0.6164, despite a 35.42 dB residual SNR. |
| Thick scattering | Maximum scattering optical depth was 3.995 with a straight-path repeated-interaction proxy of 0.908. The result documents where the approximation is poorly suited. |

The primary clear reference upper-clipped **33.06%** of samples. Consequently pure attenuation increased measured green contrast from 0.4929 in clear to 0.6155 at high extinction. The separate, predeclared headroom run eliminated clear upper clipping, preserved all transport radiances and gave near-constant pure-attenuation green contrast (0.6155 clear, 0.6155 high extinction). In the unclipped matched phase comparison, green contrast still fell from 0.6152 without a path source to 0.5638 isotropic and 0.5042 forward-weighted. Contrast, SSIM and code-domain SNR therefore describe different effects in this scene.

## Scope and outstanding work

M01's source, acceptance tests and archived evidence were preserved. Only the live roadmap's authorization/status references and the M02 directory changed. The task catalog and dependency edges were retained exactly. Measured spectral response, low-flux noise, multi-angle and multiple scattering, and physical aerosol validation remain unresolved. Later milestones were not initiated.
