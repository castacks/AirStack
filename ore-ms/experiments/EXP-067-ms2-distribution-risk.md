# EXP-067: distribution-aware disparity-risk assumption audit

Date: 2026-10-28 America/New_York. Starting commit52b53e3a0, branch ore_ms.

No distribution-aware replacement is adopted. A parameter-free lognormal assumption modestly improves some first-order comparisons but worsens day; Gamma sensitivity is more severe, including both-camera infinite-risk cases. Mean/variance outputs and positivity alone cannot identify inverse-depth risk. These are exploratory comparisons on already examined captures, not new prospective evidence.

## Declaration and numerical method

See the [full mathematical specification](../docs/research/disparity-risk-assumptions-2026-10-28.md) for assumptions, reciprocal-moment derivations, counterexample and primary documentation links. The design was recorded before comparison scores. Four EXP066 complete-field populations, original20 IDs each,80frames/106562both-valid pairs, all finite observed errors and all anchors retained. The two final official-test captures remain untouched. No fit, new labels/inference/download, clipping or changed point estimates/defaults.

Hypothetical true disparity mean and variance equal the predicted disparity and horizontal variance output; this unverified interpretation is an assumption, not a result. With c=v/mu² and original first-order trace U, lognormal risk is U*(1+3c+c²). Gamma risk is U*(1+2c)/((1-c)*(1-2c)) for c<.5, otherwise infinity. They are assumed squared point errors around the unchanged ray-based estimate, not merely transformed variances. Lower score chooses camera; exact finite ties=.5; Gamma one-infinite chooses finite, both-infinite=.5 mixture. Stable log scores avoid finite overflow; assumed infinity has a separate mask. No depth/disparity floor or replacement cap.

## Observed comparisons

Primary midpoint squared3D risks, with mean-distance sensitivities. Ratios below1 favor the assumed-family selector. All entries are medians of20 paired per-frame ratios; B/E/W means strict better/exact equal/strict worse. No pooled capture risk.

| Reused capture | Lognormal / first-order squared risk (B/E/W) | Gamma / first-order squared risk (B/E/W) | Lognormal / first-order mean distance | Gamma / first-order mean distance |
|---|---:|---:|---:|---:|
| exp063_rain | 0.9763 (17/3/0) | 0.9449 (19/1/0) | 0.9856 | 0.9662 |
| exp064_day | 1.603 (1/0/19) | 329.4 (0/0/20) | 1.054 | 2.309 |
| exp064_night | 0.9955 (20/0/0) | 0.99 (15/0/5) | 0.9919 | 0.9791 |
| exp064_rainy | 0.9685 (16/0/4) | 1.122 (7/0/13) | 0.9802 | 0.9887 |

Lognormal worsens day19/20 squared-error frames while improving EXP063 rain17 with3ties, night20, and newer rain16 with4losses. These improvements do not establish consistency. Gamma worsens day20/20 and newer rain13/20 squared-error frames. It can improve mean distance while worsening squared error, as in newer rain; changing the primary loss after outcomes would obscure that distinction. Both endpoint anchors preserve these median directions against first-order under both losses.

Fixed-camera squared-risk comparisons remain essential:

| Capture | Lognormal / RGB | Lognormal / thermal | Gamma / RGB | Gamma / thermal |
|---|---:|---:|---:|---:|
| exp063_rain | 1.511 | 0.6495 | 1.417 | 0.6327 |
| exp064_day | 0.001278 | 1.174 | 0.4682 | 218 |
| exp064_night | 2.711 | 0.9596 | 2.683 | 0.9615 |
| exp064_rainy | 0.5783 | 0.7905 | 0.8289 | 1.056 |

In newer night, both assumptions lose to always RGB in all20 squared-risk frames despite small improvements over first-order. Full report retains original camera-scale, angular/pixel geometry, unpropagated-angular and equal-random controls, all anchors/losses and exact counts. These assumed-family comparisons are not the EXP064 fitted uncertainty-only model; methods must not be conflated.

Gamma assumed infinities are explicit:

| Capture | RGB Gamma-infinite points | Thermal Gamma-infinite points | Both-infinite pairs | Median both-infinite fraction |
|---|---:|---:|---:|---:|
| exp063_rain | 29 | 24 | 0 | 0.000% |
| exp064_day | 1566 | 3502 | 763 | 1.434% |
| exp064_night | 4344 | 5230 | 878 | 3.004% |
| exp064_rainy | 1360 | 1443 | 279 | 1.755% |

Across four captures,17498 camera-point distributions have infinite assumed squared risk, including1920 pairs where both do. Observed supplied-reference errors are finite. Both-infinite cases use the declared equal-mixture fallback; the choice is not determined by an ordering of the infinite expected risks. The large day ratio must not be described as measured infinite error, proof of a physical tail, or a uniquely implied Gamma policy. Median lognormal-versus-first-order choice disagreements are0.75%,0.82%,1.71%,2.38% in table order; small decision changes can have large squared-risk effects.

## Validation

Independent raw-frontend audit PASS:1589390 scalar/hash/count checks,242 runner input hashes and164 additional raw-frontend/calibration hashes,80 frames/106562 pairs. The auditor reconstructs mu,v and first-order geometry from original fields/pixel coordinates/calibration, computes lognormal and Gamma scores from reciprocal moments using extended precision, checks every derived score/finite mask/decision, all observed risks, strict comparisons and summary medians. Original first-order decisions reproduce saved choices exactly. This certifies implementation, not calibration or physical reference correctness.

Separate assumption audit PASS: eight finite-family numerical quadratures at c=.01,.1,.25,.49; Gamma divergence at .5 and1 with increasing truncated integrals in log coordinates;12 identical-mean/variance two-point examples; finite/infinite ties and small-c limit. The two-point construction proves lack of a mean/variance-only upper bound analytically. All are fixed mathematical fixtures, not additional independent data. Figure inspected.

Scripts: `ms2_distribution_risk.py`, `validate_ms2_distribution_assumptions.py`, `validate_ms2_distribution_risk.py`, `plot_ms2_distribution_risk.py` in `MAC-VO-main/Scripts/Setup`. Results: `results/ms2-distribution-risk/run-001/{protocol.json,report.json,assumption-validation.json,validation.json,figures/}` plus80 derived decision/score NPZs. Run using the analysis venv from MAC-VO root, `python -m Scripts.Setup.ms2_distribution_risk --root /root/AirStack/ore-ms --derived /root/AirStack/ore-ms/results/ms2-disparity-diagnostics/run-001 --output <fresh-output>`. Independent validator uses `--root ... --run <output>`; mathematical validator uses `--output <fresh-json>`.

Private EXP067 backup contains tracked code/docs, all outputs, notebook and git bundle. Parent derived fields require EXP066; original saved fields/source products require EXP063/064. No credentials/checkpoints/transport archives. Keep current defaults. Next useful work is a source-level audit of the uncertainty training objective and checkpoint semantics/provenance before selecting another distribution or loss. That audit can proceed without consuming holdouts or requesting additional paper access.
