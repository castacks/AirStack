# Fixed lower-image exclusion improves moving pose results with coverage costs

16 November2026 America/New_York. Completed the
[frozen pre-essential ablation](pre-essential-exclusion-protocol-2026-11-16.md)
with the explicitly preserved
[audit-contract amendment](pre-essential-exclusion-audit-amendment-2026-11-16.md).
This is the first intervention after the rig/margin mixture diagnosis. Cached
matches/checkpoints stay fixed; remaining matches are passed to the unchanged
original essential/pose helper. No new learned inference, training, solver-setting
change or adaptive camera gate. Known exposed development/reference data.

## Primary moving result

The fixed bottom-third exclusion improves the planned learned RGB-first moving
endpoints on both records. It also improves joint rotation/translation success
on the same original translation cohorts. Whole-record availability and some
static/coarse results regress, so there is no unconditional adoption.

All counts retain failures in the original fixed population. An arrow compares
original→combined; combined is identical to bottom here because the persistent-zero
mask removes no pixels. No better arm was selected from accuracy outcomes.

| Moving learned recording | Policy | Anchors | Outputs | R≤1° | R≤5° |
|---|---|---:|---:|---:|---:|
| FlashLight | rgb | 192 | 192→192 | 176→180 | 185→192 |
| FlashLight | thermal | 192 | 168→157 | 57→66 | 110→136 |
| FlashLight | rgb_first | 192 | 192→192 | 176→180 | 185→192 |
| FlashLight | thermal_first | 192 | 192→192 | 79→97 | 134→171 |
| SmokeRoom | rgb | 236 | 221→215 | 151→182 | 181→200 |
| SmokeRoom | thermal | 236 | 231→232 | 116→177 | 190→223 |
| SmokeRoom | rgb_first | 236 | 236→236 | 155→196 | 192→221 |
| SmokeRoom | thermal_first | 236 | 236→236 | 121→180 | 195→227 |

SmokeRoom RGB-first strict success changes155/236→196/236 (+17.4percentage
points), coarse192/236→221/236 (+12.3points). FlashLight changes176/192→180/192
strict and185/192→192/192coarse. The latter is a result on this exposed supplied-
reference moving cohort, not a universal accuracy, zero-lux or held-out claim.

Joint success means own-camera R≤5° and directed t≤15° on the unchanged
both-displacements≥1cm cohort. No metric body translation or VO trajectory.

| Translation cohort | Learned policy | Anchors | Joint successes | Rate |
|---|---|---:|---:|---:|
| FlashLight | rgb | 519 | 452→465 | 87.1%→89.6% |
| FlashLight | thermal | 519 | 279→316 | 53.8%→60.9% |
| FlashLight | rgb_first | 519 | 452→465 | 87.1%→89.6% |
| FlashLight | thermal_first | 519 | 305→345 | 58.8%→66.5% |
| SmokeRoom | rgb | 483 | 349→392 | 72.3%→81.2% |
| SmokeRoom | thermal | 483 | 250→351 | 51.8%→72.7% |
| SmokeRoom | rgb_first | 483 | 356→410 | 73.7%→84.9% |
| SmokeRoom | thermal_first | 483 | 256→360 | 53.0%→74.5% |

For RGB-first, SmokeRoom joint356/483→410/483 is+11.2points; FlashLight452/519→465/519
is+2.5points. Both fixed cameras and both priority orders remain visible; thermal
still favors some coarse comparisons while RGB-first favors stricter/joint ones.
No universal sensor-priority change is implied.

## Paired gains, losses and whole-record tradeoffs

| Learned RGB-first | Endpoint/cohort | Eligible | Newly correct | Previously correct lost | Net |
|---|---|---:|---:|---:|---:|
| FlashLight | R1 | 192 | 11 | 7 | +4 |
| FlashLight | R5 | 192 | 7 | 0 | +7 |
| FlashLight | joint | 519 | 23 | 10 | +13 |
| SmokeRoom | R1 | 236 | 44 | 3 | +41 |
| SmokeRoom | R5 | 236 | 36 | 7 | +29 |
| SmokeRoom | joint | 483 | 62 | 8 | +54 |

Net gains do not mean every old correct case survives. SmokeRoom moving RGB-first
loses3strict and7coarse successes; FlashLight loses7strict successes. Full
translation-cohort joint gains include8lostSmokeRoom and10lostFlashLight cases.

| Entire recording, learned RGB-first | Declared anchors | Outputs | Common-reference R≤1° successes | Common-reference R≤5° successes |
|---|---:|---:|---:|---:|
| FlashLight | 541 | 527→523 | 495→499 | 517→520 |
| SmokeRoom | 838 | 551→519 | 452→469 | 507→503 |

Common-reference denominators are538FlashLight and837SmokeRoom; declared output
counts also retain non-common anchors. SmokeRoom gains29moving coarse successes
but loses33outside that moving cohort, for net−4whole-common coarse success.
Its output count falls32. FlashLight also loses4outputs despite net accuracy gains.
The fixed mask removes real scene constraints; static/small-motion and coverage
regressions are not replaced with an identity or other rescue policy.

FlashLight fixed thermal illustrates endpoint dependence: moving R1 increases57→66,
but whole-common R1falls324→312 and output count490→479. SmokeRoom fixed RGB
whole-common coarse460→455 also regresses. Thus this recipe is a scoped moving/
joint improvement, not a full-record dominance result for every camera/endpoint.

SIFT moving controls also change substantially:

| Recording | SIFT policy | Outputs | R≤1° | R≤5° |
|---|---|---:|---:|---:|
| FlashLight | rgb | 105→124 | 4→61 | 33→106 |
| FlashLight | thermal | 134→135 | 30→32 | 110→114 |
| FlashLight | rgb_first | 157→167 | 14→68 | 73→139 |
| SmokeRoom | rgb | 162→173 | 24→149 | 85→168 |
| SmokeRoom | thermal | 134→130 | 60→65 | 100→107 |
| SmokeRoom | rgb_first | 198→213 | 30→163 | 107→201 |

The large SIFT RGB gains show that these older modality gaps partly depend on
which constraints enter geometry; they do not justify choosing SIFT over the
stronger learned RGB-first baseline. Other SIFT/translation/third results remain
in complete summaries rather than being pooled across frontends or recordings.

## Mask qualification and execution contract

Masks were frozen from five ordinal samples per own record/camera (20images),
chosen at rounded evenly spaced positions in the complete manifests. RGB zero
exclusion keeps pixels nonzero in any sample; thermal zero mask keeps everything.
This recipe excludes exactly0pixels in both RGB recordings. It remains an
explicit no-op; no intensity threshold or sample was changed to make it effective.
It does not certify that all image margins are physically valid.

The bottom mask keeps rows0–319of480RGB and0–341of512thermal, applied at both
native endpoints through own calibration/nearest-pixel mapping. It excludes the
lower third as a coarse rig-region hypothesis and necessarily removes real scene
geometry; it is not a segmented rig or calibrated validity mask. Qualifications
were inspected before refits. No region was redesigned after scores.

Original, zero, bottom and combined all remain: zero=original and combined=bottom
in every resulting count. Original retained match order is preserved. Full22,064
variant rows were preflighted before fitting, including empty/insufficient subsets.
All5,516unmasked controls exactly reproduce42,682pose fields/arrays in the
recovered compiler image44ff9a1e6fa9… (OpenCV5.0.0/NumPy2.2.6, one OpenCV thread).
The baseline gate passes before masked execution.

All22,064geometry predictions and11,032status-only choice rows lock before score
parsing. There are5,275original and4,298additional actual essential-fit calls;
11,848masked rows reuse identical index subsets explicitly. No neural inference
or feature redetection occurs. This is post-match exclusion before E fitting,
not equivalent to masking a network input. Geometry timings are not end-to-end
lazy sensor latency, energy or matched-accuracy computation benchmarks.

## Original selection ceilings and causal limits

A descriptive comparison with the previous frozen proposal ceiling is useful:
SmokeRoom's old learned moving either-camera oracle was178strict/217coarse,
while new masked RGB-first is196/221. FlashLight's old ceiling178/188 becomes
180/192under the masked RGB-first recipe. This does not violate the old bound:
the intervention changes the geometry proposals instead of choosing among the
unchanged two. It supports investigating constraint validity before fitting a
more complex selector, without establishing new-method novelty.

The intervention changes image region, correspondence count, distribution and
RANSAC sample population together. A matched-count control was not part of this
four-arm protocol, and the zero mask is a no-op. Consequently the result does
not isolate rig removal as the sole causal component or establish optimal mask
shape. It also removes floor/scene features and may leave upper rig pixels.
No crop-width/height/intensity/seed/RANSAC sweep or mask selection was conducted.

Pixel/ROI exclusion and border/validity masks are established engineering:
[OpenCV Feature2D mask contract](https://docs.opencv.org/5.0/main_modules/classcv_1_1Feature2D.html)
and local MAC-VO matching/border-mask snippets were inspected. The current
post-match ablation is distinct from native masked feature detection or full
stereo MAC-VO/MACThermal reproduction. Prior paper reading remains reused;
no new full-paper reading or novel uncertainty/selection mechanism is claimed.

## Audit correction, retained failure and validation

The original independent audit fails its raw normalizedE=[t]×R requirement at
1e-5, starting on an exactly reproduced original control. Preserve this as FAIL,
not a successful raw essential-manifold check. In total1,684variant rows (including
no-op duplicates) fail that literal check:523original,523zero,319bottom,319combined.
The raw-deviation maximum is0.00608. Nothing in those E arrays or poses is edited.

Before inspecting accuracy summaries, the documented amendment freezes a separate
SVD-contract audit. It checks U diag(1,1,0) Vᵀ consistency with [t]×R at the SAME
1e-5tolerance, matching the decomposition contract. The diagnostic maximum gap on
the1,684rows is below8.2e-15. Tagged official OpenCV4.11source and5API behavior
were checked; the older tag is not presented as the installed5binary's exact source.
This audit tests decomposition consistency, not physical truth or exact rawE rank.

The corrected independent audit passes344,230checks on mask pixels/samples,
retained indices, source/status/masks/SVD geometry, own errors, original exact
score parity, all cohorts, four policies and paired gains/losses. The canonical
corrected10-test suite passes, including an unmodified full-valid-result test,
non-ideal-E/wrong-R synthetic controls and rehashed subset/score/cohort corruptions.
The earlier seven-test log is retained but cannot alone validate the original
full audit because that auditor also failed on the unmodified control.
Frozen original source/protocol/predictions/choices/scores remain byte-identical;
only the necessary explicit independent-audit variant changes the contract check.

Evidence: `results/pre-essential-exclusion/`, with mask qualification sheets,
preflight/parity/refit/choice/score locks, all audit attempts/amendment receipts,
full transitions and outcome PNG/PDF. A per-member verified local packet is under
`/root/ore-ms-artifacts/pre-essential-exclusion-2026-11-16/`. Source/calibration/
reference/training/physical-exposure qualifications remain inherited. No Box upload,
Git push, automatic commit or deployment adoption occurred.

Stop decision: the fixed crop is a positive moving/joint engineering result on
both exposed records with meaningful coverage/static and paired-case losses.
Keep it as a candidate baseline, not a universal replacement. Next freeze a
matched-retained-count control over the same full records, with counts determined
only by this already-frozen mask and reference-free index sampling, before
attributing the gain specifically to rig-region exclusion or testing further masks.
Do not rescue current regressions with a tuned static/identity or camera gate.
