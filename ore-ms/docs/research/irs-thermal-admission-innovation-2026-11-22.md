# IRS Medium thermal admission using pre-update innovation

22 November 2026. This is a follow-up to the [track-count comparison](irs-thermal-admission-history-2026-11-21.md). Test whether image patches consistent with the IMU-propagated state predict more accurate thermal motion increments. Bloesch et al. formulate the patch intensity residual as the filter innovation in ROVIO. This experiment uses an aggregate of reduced patch innovations as a candidate output-quality score, a separate use from the native per-feature update. [ROVIO paper](https://doi.org/10.1177/0278364917728574)

At the beginning of each image update, compute the reduced two-component
alignment innovation on a scratch copy of each existing thermal feature
at its propagated coordinates. Use the native thermal aligner's intensity
offset, intensity skew, robust weighting and gradient settings. Score
the median norm of finite equations if at least three exist. A lower
value ranks higher. The score is an unnormalized photometric quantity.
It can reflect both imperfect image alignment and imperfections in the
propagated state. No future frame or pose reference enters that score.

Use the frozen Medium dimming and early recovery windows and existing
incremental Vicon–IMU error ledger. Rank by innovation and by current
accepted track count on identical eligible rows, with chronological ties.
Track count is the established post-update comparison, so it has access
to the outcomes of current feature updates. Full retention supplies the
coverage control. Report 50%, 75% and 90% coverage. The predeclared
transfer requirement at 75% and 90% in both windows is to improve at
least one error metric without worsening the other against both controls.

No changes are made to the estimator's update decisions. Offline ranking
has the full cohort available to set the cutoff, so it measures which
outputs the scores favor at matched coverage. It supplies neither a
runtime threshold nor evidence of trajectory error after rejecting updates.

## Medium result

The final logging replay reproduces all 1,747 native poses byte for byte
and all 1,747 accepted-track counts. All 240 scoring opportunities have
an innovation score. The first logging build used the scratch aligner's
defaults. It was superseded by a rebuild with the native alignment settings
before adjudicating the result. The authoritative trace uses the configured
thermal aligner settings and unchanged pose and count outputs.

| Window | Retained | Count, displacement | Innovation, displacement | Count, rotation | Innovation, rotation |
| --- | ---: | ---: | ---: | ---: | ---: |
| Dimming | 100/200 | 5.99 mm | 5.63 mm | 0.0787° | 0.0560° |
| Dimming | 150/200 | 5.86 mm | 5.60 mm | 0.0758° | 0.0654° |
| Dimming | 180/200 | 5.92 mm | 5.78 mm | 0.0732° | 0.0649° |
| Dimming | 200/200 | 5.97 mm | 5.97 mm | 0.0713° | 0.0713° |
| Early recovery | 20/40 | 4.49 mm | 3.57 mm | 0.0653° | 0.0597° |
| Early recovery | 30/40 | 4.56 mm | 4.07 mm | 0.0687° | 0.0664° |
| Early recovery | 36/40 | 4.44 mm | 3.94 mm | 0.0668° | 0.0659° |
| Early recovery | 40/40 | 4.48 mm | 4.48 mm | 0.0669° | 0.0669° |

The proposed score meets the transfer criterion on Medium. The arithmetic
audit reproduces the 240 median scores, timing relative to track logging,
and all 28 RMSE entries.

## Difficult transfer

The [historical Difficult replay](irs-rtvi-difficult-dimming-transfer-2026-11-18.md)
defines the second recording and the window boundaries. The restored bag
passed archive CRC and reproduced the historical SHA-256. The instrumented
and count-only runs match all 1,740 poses byte for byte and all 1,740 track
counts. All 240 scoring opportunities qualify. The same probe binary,
feature minimum, score, coverages, and comparison criteria were used.
A path-expression error in the evaluation wrapper was fixed before the
Difficult runs were scored. There was no replay or protocol change.

| Window | Retained | Count, displacement | Innovation, displacement | Count, rotation | Innovation, rotation |
| --- | ---: | ---: | ---: | ---: | ---: |
| Dimming | 100/200 | 6.43 mm | 3.91 mm | 0.0810° | 0.0745° |
| Dimming | 150/200 | 6.87 mm | 4.89 mm | 0.0789° | 0.0796° |
| Dimming | 180/200 | 6.52 mm | 5.33 mm | 0.0799° | 0.0789° |
| Dimming | 200/200 | 6.37 mm | 6.37 mm | 0.0801° | 0.0801° |
| Early recovery | 20/40 | 2.63 mm | 2.89 mm | 0.0672° | 0.0588° |
| Early recovery | 30/40 | 3.01 mm | 3.14 mm | 0.0669° | 0.0573° |
| Early recovery | 36/40 | 3.61 mm | 3.50 mm | 0.0650° | 0.0667° |
| Early recovery | 40/40 | 3.49 mm | 3.49 mm | 0.0701° | 0.0701° |

Difficult reproduces the displacement gain during dimming at all partial
coverages. It misses the two-component criterion at dimming 75% and at
both required early recovery coverages. At early recovery 75%, one score
reduces rotation error while the other reduces displacement error. Retain
the dimming gain as a scoped result and close the single-score rule
under the frozen criteria. It was never applied to estimator updates.
Motion difficulty can differ across selected subsets, and both recordings
use the same rig with synthetic dimming windows. This measures ranking of
thermal increments conditional on their existing state history.

Both result sets, per-frame logs, frozen and executed scorer hashes, and
audits are under `/root/ore-ms-artifacts/irs-dual-preflight/admission-innovation/`.

## Motion-matched follow-up

Check whether the dimming displacement gain is explained by selecting
easier motion. Keep the 480 original opportunities and split each
recording/window at the medians of Vicon–IMU displacement and rotation
magnitude. Retain the same counts in each of the four crossed categories
for both scores. Use the original 75% and 90% coverages and chronological
tie breaking. Compute strata from the reference after capture for this
diagnostic. The two ranking scores and frozen transfer decision are unchanged.

| Recording and window | Coverage | Count → innovation displacement RMSE | Count → innovation rotation RMSE |
| --- | ---: | ---: | ---: |
| Medium dimming | 75% | 5.83 → 5.57 mm | 0.0739 → 0.0576° |
| Medium dimming | 90% | 5.95 → 5.77 mm | 0.0731 → 0.0662° |
| Difficult dimming | 75% | 6.66 → 5.02 mm | 0.0793 → 0.0778° |
| Difficult dimming | 90% | 6.56 → 5.57 mm | 0.0792 → 0.0791° |
| Medium early recovery | 75% | 4.38 → 3.90 mm | 0.0655 → 0.0654° |
| Medium early recovery | 90% | 4.53 → 4.23 mm | 0.0668 → 0.0674° |
| Difficult early recovery | 75% | 3.30 → 3.48 mm | 0.0651 → 0.0674° |
| Difficult early recovery | 90% | 3.53 → 3.49 mm | 0.0639 → 0.0685° |

All four dimming displacement comparisons retain their direction. The
Medium innovation sets actually have slightly larger mean reference
displacements than the count sets at both coverages. On Difficult
the innovation sets have slightly smaller mean displacement and
larger mean rotation magnitude. The result survives matching broad
motion categories, with variation within each category, temporal
clustering and the fixed estimator state history still present.
Difficult early recovery still gives mixed or negative results.

The arithmetic check reconstructs the original and restricted rankings
and verifies equal per-group quotas and all 64 RMSE values. The frozen
diagnostic, 480-row motion ledger, scorer and audits are under
`/root/ore-ms-artifacts/irs-dual-preflight/admission-innovation/motion-matched/`.
