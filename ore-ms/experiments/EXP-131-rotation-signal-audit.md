# EXP131 — Rotation signal audit

Date:2026-11-04. Branch:ore_ms. Parent:a00a87918. Pre-extraction/fitting freeze:6c7ddefc3.

Question: can available uncertainty/health signals recover EXP130 rotation headroom at the completed-output boundary? Two recording-disjoint development folds,323matched rows, sixfixed features, ridge lambda1/train-only scaling, tenmethods declared before feature extraction. Reference outcomes previously exposed; no blind/generalization claim.

Primary combinedridge selects thermal323/323, matching fixedthermal0.232241deg. No learned benefit. Fixed raw/approximateangular uncertainty rules lower rotation means to0.217604/0.220639deg but increase translation means from0.127102m to0.205861/0.225521m. Gains confined tosalon3; angularrule worsens itslatewindow. Retentionfraction difference identicallyzero. Allmethods reported, no retuning/adoption.

5744checks/10semanticmutations pass, including independent augmentedleast-squares ridge, rawfeature receipts/timing and evaluation-label poisoning. Zero invalid/fallback rows.20directpins plusEXP130ancestry. Both experts alreadyexecuted; no sparsescheduling/compute claim, no native inference/reserves/reference acquisition. EXP128MSAfailure unchanged.

[Findings](../docs/research/rotation-signal-audit-findings-2026-11-04.md) · [Protocol](../docs/research/rotation-signal-audit-protocol-2026-11-04.md)

Next audit existing saved day/night captures for unchanged-rule transfer, recipe-separated, retaining translation tradeoff and reference qualifications.
