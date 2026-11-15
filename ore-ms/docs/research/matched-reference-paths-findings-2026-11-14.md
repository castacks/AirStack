# Matched-count reference paths: route choice changes accuracy

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus frozen working sources.
One finite comparison complete. No training, rerendering or adaptive policy.

## Contribution decision and hypothesis

The [continuity assay](continuity-sequence-findings-2026-11-14.md) found that
cross-gap and old-anchor estimates outperform an adjacent chain in six artificial
sequences. Frame spacing, pair geometry and composition count changed together.
That positive result does not identify a new recovery mechanism or prove that
fewer estimates explain all the gain.

Select one distinct explanatory comparison: original frame0 to return6 through
available pre-gap reference1 versus reference2. Both paths contain exactly two
estimated transforms. This fixes endpoints and composition count while reference
timing, pair spans, images and inferred geometry vary jointly. It tests whether
count alone is an adequate accuracy description, not a causal decomposition of
all remaining factors. The original interruption/model/solver recipe is unchanged.

Generic keyframing is established. Targeted local `Module/KeyframeSelector.py`
review finds AllKeyframe and UniformKeyframe; its documentation already states
non-keyframes are interpolated at finalization. MAC-VO's original paper describes
two-frame estimation and multi-frame extensions. DROID-SLAM's released frontend
uses proximity factors and image-motion distance to remove redundant keyframes.
Thus neither selecting older references nor proposing generic persistent BA is
an established new-method contribution here.

## Frozen comparison

Reuse all six previously exposed10frame synthetic sequences, fixed withheld3–5,
return6, same raw rendered inputs and model, native selection/covariance bodies,
200query cap, selection seed, identity initialization and optical IRLS settings.

- Via1:0→1→6. Reuse the locked0→1 estimate; newly estimate1→6.
- Via2:0→2→6. Newly estimate0→2; reuse the locked2→6 estimate.
- Primary endpoint: original-frame translation/rotation error at frame6.
- Secondary: original-frame RMSE over fixed frames6–9, sharing the same locked
  subsequent6→7→8→9 estimates in both arms.

Only12new three-pair frontend batches/12pose fits are added. Each primary path
uses two pair estimates; the expensive new-run total is not itself a per-arm
compute advantage. Existing one-edge0→6 and six-edge adjacent composition remain
unequal-count context, not matched controls. All frame choices and endpoints
were bound before new inference; no winner is promoted into a deployment rule.
The inputs are development-exposed and selected after prior results: this is
an explanatory follow-up, not a fresh-test confirmation or field benchmark.

## Primary results

All12new fits converge. Both paths yield the return pose in all6scenes and all
four post-gap poses in each scene. Return-frame errors:

| Scene | Via1 translation, mm | Via2 translation, mm | Via1 rotation, deg | Via2 rotation, deg |
|---|---:|---:|---:|---:|
| day-plane4m | 3.736 | 4.507 | 0.03454 | 0.05723 |
| day-two-depths | 2.783 | 2.063 | 0.04747 | 0.02782 |
| night-plane4m | 3.920 | 4.949 | 0.04077 | 0.06644 |
| night-two-depths | 0.984 | 2.870 | 0.03053 | 0.03818 |
| rain-plane4m | 1.128 | 2.759 | 0.01755 | 0.01709 |
| rain-two-depths | 3.570 | 4.194 | 0.05878 | 0.02081 |

Via1 has lower translation error in5/6scenes and lower rotation error in3/6.
Day-two-depths favors Via2 in both metrics. Rain's translation and rotation
preferences disagree at return6. These results demonstrate route-sensitive
accuracy at equal composition count on these inputs. They do not establish one
reference as consistently superior, an observable selector for the winner,
statistical significance or generalization. There are no independent noise or
recording replications supporting population-level inference.

![Matched-count return-frame errors](../../results/matched-reference-paths/scoring-001/matched-reference-paths.png)

## Secondary post-gap results

| Scene | Via1 translation RMSE, mm | Via2 translation RMSE, mm | Via1 rotation RMSE, deg | Via2 rotation RMSE, deg |
|---|---:|---:|---:|---:|
| day-plane4m | 6.946 | 7.261 | 0.04146 | 0.06085 |
| day-two-depths | 5.227 | 3.649 | 0.10763 | 0.08584 |
| night-plane4m | 10.979 | 11.926 | 0.13658 | 0.16071 |
| night-two-depths | 2.461 | 4.263 | 0.06782 | 0.07428 |
| rain-plane4m | 2.536 | 3.817 | 0.03486 | 0.02038 |
| rain-two-depths | 7.736 | 7.082 | 0.12891 | 0.09561 |

The common later estimates preserve4/4availability in each arm; their interaction
with the different return-frame errors can change the translation preference.
Report this separately from the predeclared primary frame6 endpoint. Do not
choose the metric/horizon from whichever favors one route.

## Verification and interpretation

Before inference,72perfect-transform/failure fixtures verify both complete paths
and each possible missing edge over six scenes. The source/input/parent prediction
locks and all new producer/auditor code are frozen before inference. The additional
capture uses the previous unchanged capture implementation with only the two
explicit edge requests supplied by a wrapper.

Independent audit passes682checks: parent/new prediction locks,12planned fits,
exact batch identity, native selection, packet means/covariance, stationarity,
information inverse, matched count, coordinate-aware composition, both endpoints,
error metrics and denominators. All12fits converge. First execution passes; no
source, score, tolerance or numerical parameter amendment after freeze.

This supports a bounded extension of the original-paper discussion: evaluation
of a persistent thermal estimator must account for which frames are connected,
not only how many pose increments are composed. It does not resolve observation
noise dependence, identify the source of pair error, validate calibrated pose
uncertainty or supply a new keyframe policy. It also does not show that a learned
uncertainty score predicts the preferable route.

Close this fixed comparison without a reference-index/spacing/solver sweep,
additional score head or post-hoc deployment switch. A future persistent-estimation
method must specify a distinct mechanism and assess translation, rotation and
availability against established keyframe/graph baselines. Other original-paper
families, including thermal representation, transfer and active perception,
remain separate available directions.

## Sources and artifacts

Fresh source depth: MAC-VO v2 abstract/overview plus targeted local keyframe
selector, DROID-SLAM v2 abstract plus released frontend update/keyframe-removal
branches. This is not a fresh full DROID method reading, full-system code audit,
build or reproduction. DROID source is pinned to
`2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99`, frontend SHA256`72d6396b67649c16b8eb6e5e5039bf58f916d60643fa4fa64dd316db2e20b8d1`.

Read-only author snapshot remains outside the repository/backup.

Primary sources: [MAC-VO](https://arxiv.org/html/2409.09479v2),
[DROID-SLAM](https://arxiv.org/abs/2108.10869v2),
[pinned DROID frontend](https://github.com/princeton-vl/DROID-SLAM/blob/2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99/droid_slam/droid_frontend.py).

Protocol:`experiments/matched-reference-paths-freeze-2026-11-14.json`.
Results:`results/matched-reference-paths/` with parent prediction-lock dependency
on the complete continuity-sequence packet. No previously completed result is
replaced or relabeled.
