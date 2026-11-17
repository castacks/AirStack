# Average crop gains do not consistently shorten the worst sampled failure blocks

17 November 2026 America/New_York. Completed the
[frozen temporal-output assay](temporal-output-bursts-protocol-2026-11-17.md).
Branch `ore-ms-orb-development`, committed base `4408b7fb8` plus pinned local sources.
Consumes [paired crop/count](matched-count-findings-2026-11-16.md) and
[OutdoorNight transfer](outdoornight-crop-findings-2026-11-16.md) outputs unchanged.
No new model execution, predictions, geometry fits, references or selections.

## Primary finding

The crop shortens SmokeRoom's worst full-record learned RGB-first unusable block,
but lengthens FlashLight's and OutdoorNight's relative to original. OutdoorNight
also worsens relative to the same-count control. Aggregate accuracy gains do not
establish shorter sustained bad-output periods. Preserve the crop as a scoped
candidate with explicit coverage/burst costs, not a universal replacement.

A bad angular event is missing output OR error/disagreement>the fixed bound on
original both-camera common-reference anchors. Reference gaps break runs rather
than becoming failures or successes. Availability counts missing on every anchor.
Triples below always mean original / same-count / crop; no record/tier pooling.

| Learned RGB-first, full record | Missing decisions | 5° bad decisions | Missing episodes | 5° bad episodes | Longest missing block (anchors) | Longest 5° bad block (anchors) |
|---|---|---|---|---|---|---|
| FlashLight | 14 / 14 / 18 | 21 / 21 / 18 | 5 / 3 / 6 | 11 / 7 / 6 | 4 / 9 / 6 | 4 / 9 / 6 |
| SmokeRoom | 287 / 303 / 319 | 330 / 334 / 334 | 71 / 60 / 56 | 85 / 72 / 63 | 26 / 24 / 20 | 26 / 24 / 20 |
| OutdoorNight | 131 / 157 / 149 | 191 / 202 / 182 | 40 / 35 / 44 | 83 / 67 / 67 | 9 / 18 / 20 | 9 / 18 / 20 |

FlashLight: coarse bad counts21→18improve, while longest bad run4→6worsens
(count9). Missing decisions14→18andlongest missing4→6also worsen. SmokeRoom:
longest26→20improves beyond count24, despite missing287→319andcoarse bad330→334
worsening. OutdoorNight: coarse bad191→182improves but longest9→20worsens
(count18); missing131→149andlongest9→20worsen. Fewer bad decisions or episodes
do not mean a better maximum block. Smoke's fewer episodes85→63coexist with
more coarse bad decisions; no general temporal-concentration improvement follows.

Every longest5°bad block in these nine primary cells is entirely missing outputs,
not a mixture containing returned wrong rotations. Every such worst block is
also outside the frozen paired≥2°moving cohort. Thus the full-record worst-block
tradeoff primarily reflects low/small-motion proposal availability here, rather
than demonstrating prolonged moving pose error or physical robot failure.
No identity/static output was added to repair it. All ties are retained below.

## Longest blocks and observed returns

For kbad decisions at500ms spacing, sampled first-to-last span=(k−1)*0.5s.
An immediately following good sample gives observed return lag=k*0.5s from the
first bad anchor. These quantities are not continuous physical dropout duration,
time-to-relocalization or successful VO recovery. No extrapolation at record end.

| Record | Variant | All longest5°bad index ranges (inclusive) | Length | Sampled span (s) | Observed next-good lag (s) |
|---|---|---|---:|---:|---:|
| FlashLight | original | 183–186, 188–191 | 4 | 1.5 | 2 |
| FlashLight | count | 188–196 | 9 | 4 | 4.5 |
| FlashLight | bottom | 193–198 | 6 | 2.5 | 3 |
| SmokeRoom | original | 283–308 | 26 | 12.5 | 13 |
| SmokeRoom | count | 295–318 | 24 | 11.5 | 12 |
| SmokeRoom | bottom | 299–318, 792–811 | 20 | 9.5 | 10 |
| OutdoorNight | original | 37–45, 55–63 | 9 | 4 | 4.5 |
| OutdoorNight | count | 36–53 | 18 | 8.5 | 9 |
| OutdoorNight | bottom | 27–46 | 20 | 9.5 | 10 |

These maximum blocks have observed good samples on both sides. The full artifact
also retains all edge/reference/cohort-censored episodes, including OutdoorNight's
initial availability gap. No successful-prefix substitution or selected window
replaces the full population; this table is the preregistered maximum with ties.

## Moving and strict controls

Sparse moving indices are never concatenated. A nonmoving anchor ends a moving
run with an outside-cohort censor flag, even if the next moving event is bad.
A dash means no bad episode (saved null), not an unobserved performance estimate.

| Learned RGB-first, moving | Strict1° bad decisions | Strict longest | Coarse5° bad decisions | Coarse longest | Missing decisions | Missing longest |
|---|---|---|---|---|---|---|
| FlashLight | 16 / 16 / 12 | 4 / 4 / 4 | 7 / 7 / 0 | 2 / 3 / — | 0 / 0 / 0 | — / — / — |
| SmokeRoom | 81 / 68 / 40 | 14 / 11 / 11 | 44 / 32 / 15 | 8 / 7 / 6 | 0 / 0 / 0 | — / — / — |
| OutdoorNight | 117 / 96 / 84 | 6 / 7 / 7 | 62 / 49 / 37 | 3 / 4 / 4 | 0 / 3 / 2 | — / 1 / 1 |

FlashLight crop eliminates all7original moving coarse disagreements, while whole-
record missing blocks worsen; strict longest stays4. SmokeRoom moving coarse bad
44→15andlongest8→6improve (count32bad/7longest); strict81→40and14→11improve,
with strict longest tied count11. OutdoorNight moving coarse62→37improves while
longest3→4worsens (count4), and strict117→84improves while longest6→7worsens
(count7). New moving missing events appear0→2, each a singleton. These preserve
mixed temporal evidence even within the favorable moving aggregate comparison.

Full-record learned RGB-first strict bad counts/longest are FlashLight43/4→39/6,
SmokeRoom385/26→368/20, OutdoorNight318/9→272/20. The strict endpoint therefore
does not reverse the primary worst-block direction. All original→count, original→
crop and count→crop deltas remain exact and separate.

## Fixed-camera, priority and SIFT controls

All full-record5°bad totals and maximum lengths, same triple order:

| Record | Frontend | Policy | 5°bad decisions | Longest5°bad block |
|---|---|---|---|---|
| FlashLight | learned | rgb | 24 / 26 / 21 | 11 / 11 / 11 |
| FlashLight | learned | thermal | 108 / 112 / 88 | 14 / 14 / 11 |
| FlashLight | learned | rgb_first | 21 / 21 / 18 | 4 / 9 / 6 |
| FlashLight | learned | thermal_first | 72 / 53 / 45 | 4 / 9 / 6 |
| FlashLight | sift | rgb | 261 / 344 / 193 | 20 / 31 / 17 |
| FlashLight | sift | thermal | 182 / 191 / 185 | 18 / 19 / 18 |
| FlashLight | sift | rgb_first | 158 / 172 / 115 | 8 / 13 / 10 |
| FlashLight | sift | thermal_first | 123 / 154 / 111 | 8 / 13 / 10 |
| SmokeRoom | learned | rgb | 377 / 386 / 382 | 34 / 29 / 37 |
| SmokeRoom | learned | thermal | 384 / 402 / 368 | 42 / 57 / 46 |
| SmokeRoom | learned | rgb_first | 330 / 334 / 334 | 26 / 24 / 20 |
| SmokeRoom | learned | thermal_first | 327 / 340 / 330 | 26 / 24 / 20 |
| SmokeRoom | sift | rgb | 561 / 662 / 473 | 74 / 144 / 86 |
| SmokeRoom | sift | thermal | 633 / 630 / 620 | 122 / 123 / 122 |
| SmokeRoom | sift | rgb_first | 498 / 577 / 394 | 45 / 46 / 32 |
| SmokeRoom | sift | thermal_first | 478 / 552 / 416 | 45 / 46 / 32 |
| OutdoorNight | learned | rgb | 477 / 496 / 507 | 53 / 47 / 57 |
| OutdoorNight | learned | thermal | 196 / 221 / 211 | 9 / 18 / 20 |
| OutdoorNight | learned | rgb_first | 191 / 202 / 182 | 9 / 18 / 20 |
| OutdoorNight | learned | thermal_first | 153 / 173 / 166 | 9 / 18 / 20 |
| OutdoorNight | sift | rgb | 739 / 864 / 799 | 66 / 98 / 135 |
| OutdoorNight | sift | thermal | 593 / 604 / 604 | 124 / 95 / 96 |
| OutdoorNight | sift | rgb_first | 463 / 541 / 504 | 45 / 84 / 80 |
| OutdoorNight | sift | thermal_first | 447 / 536 / 493 | 45 / 84 / 80 |

Learned FlashLight thermal-only longest14→11improves, unlike RGB-first4→6.
SmokeRoom's fixed cameras worsen (RGB34→37,thermal42→46), while either priority
order improves26→20. OutdoorNight thermal-only9→20andthermal-first9→20worsen;
RGB-only53→57also worsens (count47). One universal camera/crop choice is not
supported. Both priority orders have identical output presence because either
returns a proposal whenever at least one camera supports it; angular values differ.

SIFT retains further counterexamples: FlashLight RGB-first coarse bad158→115
improves while longest8→10worsens; SmokeRoom RGB-first
498→394andlongest45→32improve. OutdoorNight463→504and45→80both worsen under crop.
Complete strict/availability/moving histograms and censor details are saved for
all policies/frontends; these controls are not pooled into the learned result.

## Execution, audit and limitations

Preflight covers2350original anchors,28200geometry/error rows,14100status-choice
rows and56400policy-selected events. All source populations, unique keys, selected
own-camera endpoints/status/errors and exactly500000000ns shared anchor spacing
qualify before temporal summaries. No source predictions/choices/labels changed.
Reference eligibility is inherited:538FlashLight/837SmokeRoom common supplied-
reference anchors and964OutdoorNight common gyro anchors; moving192/236/242.
OutdoorNight's nominal gyro and supplied-reference tiers remain distinct.

The new adapter reuses `paired_failure_strata.runs` for maximal consecutive index
blocks and existing read/write/hash helpers. It adds source/tier/event binding,
missing/disagree/unknown/outside states, spans/censor/return receipts and summaries;
existing runs helpers do not express those temporal boundary contracts. The assay
saves432groups,22404episode rows including full/moving views, exact length
histograms, conditional return maxima and432paired delta rows. Episodes overlap
across views/policies/endpoints by design;22404is not a unique physical-event count.

The independent auditor reconstructs source-policy/event/errors and chronological
states, uses a separate sequential state machine, checks every maximal episode,
boundary/censor/span/lag, histogram/summary/delta, and reproduces all parent
marginal availability/angular totals. It passes192522checks. All8tests pass:
known clean/singleton/mixed/reference/cohort/record-edge cases, exact bounds,
unmodified full-result audit, and rehashed event, censor-flag and summary
corruptions rejected. Preflight ran4finite tests and explicitly skipped4full
result tests until outputs existed; both logs remain. Exact recovered CPU image
is pinned; no new E fits, model calls or timing-benefit claim.

Evidence: `results/temporal-output-bursts/`, full preflight/freeze/events/episodes/
summaries/deltas, all logs, and `run-001/temporal-strips.png`/PDF with input hashes.
A member-verified incremental local packet is under
`/root/ore-ms-artifacts/temporal-output-bursts-2026-11-17/`. Parent packets remain
pinned dependencies. No commit, push, Box upload, training, deployment or rescue gate.

## Decision

Close this characterization with mixed temporal results. The crop's moving/joint
accuracy gains remain real within their exposed nominal contracts, but do not
establish better worst-case pair-output availability. SmokeRoom's worst block
improves with worse total missing support; FlashLight/OutdoorNight maxima worsen.
Record low-motion and reference/cohort censoring qualifications alongside counts.
Conditional observed next-good lags must not be interpreted as uncensored VO
recovery performance. Temporal dependence, one count draw, nominal reference and
training/exposure limitations prevent IID/physical generalization claims.

Prior SubT-MRS/OpenCV/source reading reused; no new paper-body reading, uncertainty
calibration, novelty or stateful VO claim. Do not fit a crop/identity/clock/camera
rescue to this result. The next bounded action is to synthesize a deployment
baseline recommendation from the three records' accuracy, support and temporal
costs, with explicit independent-validation requirements; any subsequent new
policy or model experiment needs a separate protocol. Stop this assay at packet.
