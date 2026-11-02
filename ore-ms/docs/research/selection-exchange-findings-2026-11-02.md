# EXP109 — Saved-output selection-exchange accounting

The recurrent adaptation’s eight-record regression is explained exactly by which points replace which at the fixed selection cutoff. At 50% retention, it replaces an average **3.001% of retained points**, raising record-balanced MSE by **0.113487 px²**. The highest-loss 1% contributes +0.002228 px²; the other 99% contributes +0.111259 px². The change is predominantly outside the extreme tail. A uniform positive variance scale preserves the ranking and therefore cannot repair this within-pair selection result.

This is descriptive post-hoc accounting of already observed outcomes, not a new validation set, fitted correction or identified training mechanism. The original checkpoint remains the reference.

## Frozen inputs and scope

Diagnostic freeze `d1c46c69d` precedes the new computation. All196 saved pairs /13recordings are included: EXP106 earlier-three36pairs, EXP107 prospective-two32pairs, EXP108 expanded-eight128pairs. The same original/recurrent/final-diagonal/final-trace states underlie every saved array. Five fixed retention fractions remain10/25/50/75/100%; native valid-pixel identity, mean predictions, targets, score transform and tie handling remain unchanged. All three adaptations are compared with the original.

Only saved prediction/target/raw-uncertainty arrays and their proof files are read. No raw source images or new datasets are decoded, no model forward runs, no optimizer or fitting occurs, no checkpoint is exported, and the36-record exposure ledger is unchanged. The source arrays, model binaries, ledger, prior results and scripts are bound by214SHA256pins.

## Exact exchange identity

Let B and A be the original and adapted sets of K retained points, E=A\B the entrants and X=B\A the exits. The predictions and labels are the same for all models, so every point has one fixed squared endpoint error L. Shared terms cancel:

`MSE(A) − MSE(B) = [sum(L on E) − sum(L on X)] / K`

The entrant and exit counts are equal. At100% retention every point is retained, so all exchanges and risk differences are exactly zero. All five cutoffs are verified against the original saved scores.

The tail is fixed independently of learned scores: highest-loss q=max(1,floor(.01N)) valid points within each pair, with descending loss and ascending native pixel index for ties. Tail and non-tail contributions are signed and sum to the full change. This is reference-assisted accounting, not a label-free quality feature or proposed deployable gate.

## Eight-record focus at 50% retention

| Adaptation | Replaced retained mass | MSE change (px²) | Highest-loss 1% contribution | Remaining 99% contribution |
|---|---:|---:|---:|---:|
| Recurrent diagonal | 3.001% | +0.113487 | +0.002228 | +0.111259 |
| Final diagonal | 2.561% | +0.267528 | +0.007446 | +0.260081 |
| Final trace | 2.874% | +0.210825 | +0.003117 | +0.207707 |

These are equal-record averages of within-record pooled counts/losses. The amount of turnover alone does not determine whether risk improves: the losses of entering versus exiting points determine its sign. The same small changes in retained membership improve some earlier recordings and worsen additional recordings.

## Every recording: recurrent adaptation at 50%

| Recording | Cohort | Replaced retained mass | MSE change | Tail contribution | Non-tail contribution | Entrant mean error² | Exit mean error² |
|---|---|---:|---:|---:|---:|---:|---:|
| office | earlier_three | 2.682% | +0.000500 | +0.000000 | +0.000500 | 0.047629 | 0.028979 |
| japanesealley | earlier_three | 5.415% | -0.622513 | -0.000112 | -0.622400 | 6.410704 | 17.906161 |
| amusement | earlier_three | 2.517% | -0.000534 | +0.000000 | -0.000534 | 0.185311 | 0.206519 |
| carwelding | prospective_two | 4.776% | -0.006370 | +0.000254 | -0.006624 | 0.283739 | 0.417108 |
| endofworld | prospective_two | 3.144% | +0.000107 | +0.000169 | -0.000062 | 0.105029 | 0.101614 |
| neighborhood | expanded_eight | 1.337% | +0.015649 | +0.000000 | +0.015649 | 3.297772 | 2.127243 |
| ocean | expanded_eight | 1.890% | +0.029184 | +0.000000 | +0.029184 | 5.365210 | 3.820723 |
| office2 | expanded_eight | 1.382% | +0.003557 | +0.000000 | +0.003557 | 0.428816 | 0.171531 |
| oldtown | expanded_eight | 4.129% | -0.000103 | +0.000892 | -0.000995 | 0.191596 | 0.194088 |
| seasidetown | expanded_eight | 5.523% | -0.001607 | +0.000000 | -0.001607 | 0.042783 | 0.071885 |
| seasonsforest | expanded_eight | 2.477% | +0.001990 | -0.000051 | +0.002041 | 0.504316 | 0.423967 |
| soulcity | expanded_eight | 3.702% | -0.004884 | -0.000078 | -0.004806 | 0.319375 | 0.451300 |
| westerndesert | expanded_eight | 3.570% | +0.864107 | +0.017058 | +0.847048 | 101.372331 | 77.170085 |

Westerndesert is the largest absolute regression: 3.570% of retained points are replaced, and entrant mean squared error is101.372 versus77.170px² for exits. The resulting change is +0.864107px²; +0.847048 is outside the per-pair highest-loss1%. In Neighborhood, Ocean and Office2, no entrant or exit is in that highest-loss1% at all. Their risk regressions are fully non-tail exchanges under this fixed definition.

## All source cohorts and adapted controls

| Cohort | Adaptation | Replaced retained mass | MSE change | Tail contribution | Non-tail contribution |
|---|---|---:|---:|---:|---:|
| earlier_three | Recurrent diagonal | 3.538% | -0.207515 | -0.000037 | -0.207478 |
| earlier_three | Final diagonal | 2.837% | -0.093947 | +0.000001 | -0.093947 |
| earlier_three | Final trace | 3.146% | -0.119627 | -0.000032 | -0.119595 |
| prospective_two | Recurrent diagonal | 3.960% | -0.003131 | +0.000212 | -0.003343 |
| prospective_two | Final diagonal | 3.171% | -0.002773 | +0.000142 | -0.002915 |
| prospective_two | Final trace | 4.048% | -0.002778 | +0.000162 | -0.002940 |
| expanded_eight | Recurrent diagonal | 3.001% | +0.113487 | +0.002228 | +0.111259 |
| expanded_eight | Final diagonal | 2.561% | +0.267528 | +0.007446 | +0.260081 |
| expanded_eight | Final trace | 2.874% | +0.210825 | +0.003117 | +0.207707 |
| all_thirteen | Recurrent diagonal | 3.273% | +0.021468 | +0.001395 | +0.020073 |
| all_thirteen | Final diagonal | 2.719% | +0.142526 | +0.004604 | +0.137921 |
| all_thirteen | Final trace | 3.118% | +0.101705 | +0.001936 | +0.099769 |

The combined13-record summary is retrospective context and includes development-observed legacy data. It does not replace EXP108’s previously frozen eight-record primary or become a new independent test. All per-pair, per-record and cohort macro/micro results for all fractions/arms remain in diagnostic.json, including signed cancellation and explicit empty-exchange cases at100%.

## Scale versus spatial score changes

For any common positive scalar alpha, multiplying every point’s variance by alpha multiplies every trace by alpha and preserves its ordering and ties in exact arithmetic. Factor-two float64 controls reproduce the entire stable order exactly for all784model/pair fields; the independent auditor also verifies selection invariance at each retained mass. This result is limited to a uniform factor across points and axes. Likelihood or ellipse coverage can still change under such calibration. Different camera-specific multipliers can also change cross-camera comparisons; no thermal or cross-camera comparison is tested here.

Spatial score changes are retained separately from pair-wide score shifts: each pair reports the25/50/75%quantiles of log(trace_adapt)−log(trace_original), plus RMS after subtracting its median. The following record statistics are means of pair summaries, not pooled pixel quantiles:

| Expanded recording | Mean pair median log-trace ratio | Mean pair IQR | Mean centered log-ratio RMS |
|---|---:|---:|---:|
| neighborhood | -0.007161 | 0.217400 | 0.184602 |
| ocean | -0.203155 | 0.291988 | 0.200462 |
| office2 | +0.080176 | 0.285371 | 0.225859 |
| oldtown | -0.149890 | 0.398783 | 0.254240 |
| seasidetown | -0.116648 | 0.357126 | 0.246567 |
| seasonsforest | -0.242327 | 0.246432 | 0.178066 |
| soulcity | -0.218791 | 0.258046 | 0.211154 |
| westerndesert | +0.081157 | 0.306487 | 0.243731 |

The nonzero centered departures and changed selection membership demonstrate spatially nonuniform score changes. Their cause inside optimization, architecture or training coverage is not identified by this accounting. No scalar statistic is fitted into a quality gate, and no correction is adopted.

## Independent verification

Producer and independent auditor pass on their first attempts. The final audit passes **74,256 checks** and rejects four semantic mutations: exchange delta, retained mass, tail cost and record macro. Source pins are checked before and after the audit. The auditor reconstructs selections with partition thresholds and explicit tie completion rather than the producer’s stable full sort; it independently checks tail membership, point counts, common/entrant/exit sums, all exchange/partition identities and record/cohort aggregation. Independent sorted interpolation verifies the score-shift quantiles. Frozen numerical tolerances are2e−12 relative/absolute; no repairs or tolerance changes occur.

## Decision

Keep the original ranking as the reference. Treat probability/coverage calibration and within-pair ranking as separate goals: a uniform calibration factor may change likelihood/coverage, while this fixed within-pair selection remains unchanged. Any future ranking adaptation must justify how it preserves or improves the costs of the exchanged points across recordings, including the ordinary-error population. A tail-only description misses most of this observed selection regression. This accounting does not predict how a different training loss would change the rankings.

A separately declared calibration-only baseline could test coverage/likelihood while preserving the original ranking. It would require an explicit fit/exposure/evaluation contract and would not itself address the ranking target. No such calibration is fitted in this phase.

Scope remains observed synthetic RGB/provided-flow outputs with unknown supplied-network pretraining overlap. Adjacent pairs and pixels are not independent recording replicates. This is algebraic accounting, not causal, thermal, physical VO, population-generalization or deployment evidence. The prior seven MS² readiness requirements and protected reserves remain unchanged.

Artifacts: `results/selection-exchange-diagnostic/run-001/`. Recovery target: `ore_ms:ore-ms/backups/2026-11-02-selection-exchange-diagnostic`, incremental from EXP108 `c95ff6104`. Saved arrays/model binaries remain in their original verified packets; no duplication or new model export is needed.
