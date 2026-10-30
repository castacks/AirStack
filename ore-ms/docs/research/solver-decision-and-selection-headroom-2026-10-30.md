# Solver decision and camera-selection opportunity

2026-10-30 UTC. Branch `ore_ms`, base `a177ba60c`.

Keep the weighted-Huber repair experimental. It fixes an identified numerical
contract, but the three frozen comparisons do not establish a general VO accuracy
gain. Return the next experiment to camera-quality information, with a fixed
measurement and comparison protocol before more inference.

## What is established

[EXP082](../../experiments/EXP-082-robust-solver-contract.md) verifies the original
weighted-step/raw-loss acceptance mismatch and failure to preserve the best state
in bounded fixtures. [EXP083](../../experiments/EXP-083-weighted-solver-repair.md)
introduces a consistent whitened-Huber objective, best-state restoration and
explicit numerical status. These are numerical-contract findings; learned
covariance calibration, physical geometry and measurement correctness remain
separate questions.

| Frozen condition | Matched original/repair per camera | Translation RMSE change RGB / thermal | Rotation RMSE change RGB / thermal |
|---|---:|---:|---:|
|[Early night, EXP084](../../experiments/EXP-084-end-to-end-solver-control.md)|19 / 19|+0.322% / +0.396%|−4.793% / −5.131%|
|[Middle night, EXP085](../../experiments/EXP-085-solver-collapse-replication.md)|2 / 99|−25.662% / −3.086%|−26.672% / +0.024%|
|[Day, EXP086](../../experiments/EXP-086-solver-day-replication.md)|99 / 99|−0.265% / −1.493%|+0.125% / +0.159%|

Negative means lower proxy RMSE. These are separate translation/rotation losses
against a nominal-body, reference-assisted recorded-SLAM proxy. No weighted
cross-condition average, mixed-unit score or physical-accuracy claim is warranted.
The large middle RGB improvement uses only two motions;97/99 transitions remain
unsupported. Every one of the337repair solves hits its ten-step cap, so numerical
convergence is not established.

Original/passthrough complete-array controls and unchanged frontend outputs
separate instrumentation from the intended solver change. EXP086 also required a
**post-inference gradient-audit refinement**: two near-cancelling components fail
the original component-relative criterion. Both failures remain in its proof.
The final independent checker uses absolute-contribution scaling plus a normwise
check; all other FD/Hessian/step/objective/acceptance checks remain unchanged.
Supplementary native fixed-state replay reproduces cost/H/g exactly at1980states.
This correction does not change inference, settings or scores, and must accompany
any account of the numerical validation.

## How much could choosing cameras help?

A new CPU audit uses only the **original arms** of EXP084–086. It retains all217
interval slots: both cameras yield120 matched scores, thermal217, RGB120. These
are first pre-refinement writes after complete capture, including terminal writes.
They are not the earlier shared-clock/deadline availability populations.

For each loss separately, an offline oracle picks the lower recorded error for
that interval, with no switching cost. The best fixed camera is also chosen in
hindsight per window/loss. Both are label-assisted analysis controls, never causal
features or deployable policies. No trajectory is stitched, no parameters fit,
and no unsupported pose receives zero error.

| Both-supported population | N | Best fixed translation camera / RMSE m | Oracle translation RMSE m | Potential reduction | Oracle rotation reduction vs best fixed |
|---|---:|---|---:|---:|---:|
|Early night|19|Thermal /0.009260253|0.009189644|0.762%|7.518%|
|Middle night|2|Thermal /0.052767988|0.052767988|0.000%|2.322%|
|Day|99|RGB /0.031673337|0.028521889|9.950%|2.605%|

The day translation bound is an absolute0.003151448m RMSE reduction. It is an
opportunity under this proxy, not evidence that the labels resolve a physical
3mm improvement. Early-night translation opportunity is just0.000070609m against
thermal-only. Choosing the condition's best fixed camera already explains much
of the apparent benefit of adaptation.

Removing any terminal write from the matched population leaves18/1/98pairs.
Translation opportunity becomes0.807%/0%/9.544%; rotation7.709%/0%/2.609%.
This processing-only sensitivity does not prove shared-clock causal availability.
Union coverage retains19/99/99slots. Fixed-camera/oracle error comparisons use
exact matched IDs; RGB's sparse middle scores are never compared with thermal's
99-pair RMSE as if populations matched.

Translation oracle choices are5RGB/14thermal early,0RGB/2thermal middle,
63RGB/36thermal day. The strict winner repeats on8/18adjacent early comparisons
and53/98day comparisons; the two middle ordinals are nonconsecutive and contribute
no lag1 statistic. Rotation winners repeat14/18early and94/98day, where RGB wins
97/99day intervals. Exact ties and missing ordinal gaps break runs. These are
label-history descriptions, not predictive history features or significance tests.
They do not establish that a two-interval persistence rule can predict pose quality.

Independent NumPy verification checks908population scalar observations, all217
slot records, cohort/winner/run bookkeeping and7208exhaustive binary policies on
fixed chronological blocks. Six semantic mutations reject wrong oracle scores,
missing rows, fabricated ground-truth/causal claims, availability and persistence.
Artifacts: `results/solver-selection-headroom/run-001/`; all input report/proof/
protocol SHA256 chains and the EXP086 exceptions are preserved. No new neural
inference, training, held-out/reserved access, solver tuning or default change.

## Decision

Retain the isolated repair and its evidence as a numerical-control contribution.
Do not promote it as the RGB–thermal selection method or a convergence/accuracy
solution. Availability fallback is supported on the known collapse case and
matches thermal-only there. Quality selection when both cameras track remains
unresolved; existing covariance/residual/geometry preferences have not transferred
consistently in [EXP078–081](../../experiments/EXP-080-factor-proxy-comparison.md).

The next bounded experiment should measure **forward/backward temporal flow
closure** as additional sensor-causal quality evidence. The accompanying
[fixed protocol](flow-cycle-quality-protocol-2026-10-30.md) keeps original motions,
fixed baselines and both losses, adds reverse matching only, and tests all three
known windows. It is an unexecuted exploratory protocol. Stronger physical or
prospective claims still require independent reference/rig provenance and a
separately frozen recording-level evaluation.

## Reproduction and recovery

In a CPU container with ore-ms mounted at `/workspace`, from `MAC-VO-main/`:

```bash
python -m Scripts.Setup.solver_selection_headroom --root /workspace --output /workspace/results/solver-selection-headroom/new-run
```

The new output directory must not exist. The frozen analysis input list includes
all three original reports, protocols/proofs and the EXP086 native replay proof.
Backup target: `ore_ms:ore-ms/backups/2026-10-30-solver-decision-headroom`.
Its incremental source bundle requires commita177ba60c; it includes the frozen
source report metadata needed to rerun this analysis. Full raw scientific
reconstruction remains available through EXP084/085/086 and their prerequisites.
No checkpoints, images, credentials or new inference are required for this bound.
