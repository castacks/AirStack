# Fixed matched-retained-count control

16 November2026 America/New_York. Follows the
[crop ablation](pre-essential-exclusion-findings-2026-11-16.md) and its preserved
[SVD audit amendment](pre-essential-exclusion-audit-amendment-2026-11-16.md).
Known exposed development outcomes; this is a single fixed engineering control.
Does region exclusion outperform retaining the same number of matches without
choosing an image region? No blind/seed-variability/generalization claim.

Use all541FlashLight and838SmokeRoom anchors, both cameras/frontends,5516rows,
including every failed/unsupported input. Three variants: original and bottom
inherit exact parent outputs; count is new. Its required count comes from the
parent IMAGE-ONLY prefit plan's bottom retained count, not pose support/error.

## Frozen reference-free selection

For each parent row with n accepted matches and target count k (0≤k≤n), keep k
original correspondence indices ranked by ascending SHA256 bytes of UTF-8:
`pre-E-matched-count-v1|seed=0|{recording}|{frontend}|{arm}|{index}|{match_index}`.
Exact digest ties use original index. Sort the retained indices back into original
ascending order before solving. k=0 yields empty, k=n yields original. This is
one deterministic pseudorandom ranking draw per row, no pixel/flow/covariance/
reference/error criterion. No alternative seed/ranking method after outcomes.
It is a count control, not a repeatability distribution or calibrated sampling law.

Preflight every original/bottom/count row (16548total), exact counts and SHA of
little-endian int64 ascending indices. Inherit and verify all parent masks,
preflight/prediction/choice/score/audit and amendment identities. All parent
original/bottom pose fields must remain exact. Save full subset bindings before
solving. Do not regenerate parent controls or modify frozen exclusion sources.

## Fit, select, score

Reuse unchanged pose helper in exact recovered image
sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09,
CPU OpenCV5.0.0/NumPy2.2.6, one OpenCV thread; same seed0,8matches,
RANSAC.999/1px/1000iterations and8cheirality gate. No new model inference,
feature extraction, camera calibration, axes/clocks/thresholds or optimizer.
Qualified old failures remain eligible for recomputation. Unsupported inputs keep
their no-call status. Exact identical index subsets reuse the locked parent
original or bottom result, with explicit origin receipts; no error-based reuse.

Lock all16548geometry rows, then unchanged RGB/thermal/RGB-first/thermal-first
status-only choices before parsing labels. Reuse existing select/score routines
under a scoped variant-name context; do not edit frozen code or score equations.
Same own-camera reference/translation origins and all/common/moving/translation/
index-third populations. R≤1°,R≤5°,jointR≤5°/directed-t≤15°; failures remain.

Primary comparison is bottom versus count for learned fixed cameras and RGB-first
moving R1/R5 and full translation-cohort joint, separately in each recording.
Original and both frontends/orders remain controls. Save paired gained/lost correct
and added/lost outputs both versus original and count→bottom, not just net counts.
No pooling, threshold/seed sweep, significance/IID or end-to-end timing claim.

## Verification and stop

Independent audit rebuilds hash ranks/counts from prefit sources, verifies parent
control identity, all retained-index hashes/order and origin/call receipts, SVD
E/R/t consistency at1e-5, original masks/pose gates, own full pose errors, choices,
cohort counts and paired contrasts. The parent's literal rawE equality failure
remains historical and is not relabelled pass. Raw matrices are never projected
in outputs; SVD projection is only an audit reference. Synthetic controls cover
exact count, key separation, empty/full, retained order and scoped helper reuse;
rehashed semantic corruptions must fail with an unmodified-valid audit check.

Reuse prior ROI/mask/source reading; no new method claim. If crop beats this count
control, count alone does not explain this comparison, but rig versus scene,
spatial distribution, RANSAC sample effects and one-draw variability remain.
No automatic causal rig-only attribution or deployment follows either result.
Stop after complete findings/verified incremental local packet, retaining all
regressions and the previous coverage losses. No new draw/mask, training, author
outreach, Box upload, Git push or automatic commit.
