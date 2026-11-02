# Research steering — RGB–thermal first, sensor generalization as the target

2026-11-02, America/New_York. This is a research-scope and evidence assessment,
not a new experiment, selected architecture, training protocol or novelty claim.

## User steering

The primary research concerns RGB–thermal geometric uncertainty and its useful
translation into shared estimation, with eventual generalization to other sensors.
MAC-VO/MACThermal are initial testbeds. Improving RGB MAC-VO by itself is supporting
work, not the endpoint. More RGB environments establish condition/domain coverage;
they do not establish generalization to a different sensor modality.

LSTM covariance memory, an agent, and an additional layer were suggested as
possible tools, not grounded directions or instructions to build them. None is
selected. Future findings may motivate a pivot; a pivot needs a specific observed
failure, an explanatory hypothesis, a discriminating test and a measurable target.

## Evidence that constrains the next choice

| Existing evidence | What it supports | What it does not establish |
|---|---|---|
| EXP009–012: covariance-only and precision-mass/channel controls materially change conditional thermal translation and rotation. | Covariance intervention has downstream leverage; spatial/channel redistribution matters. | A generally superior correction, independent physical trajectory accuracy, or a joint RGB–thermal fusion gain. |
| EXP056–070: within-camera signal coexists with cross-camera selection failures; constant scale/bias corrections and simple geometry-risk fits do not transfer consistently. | Common geometric targets and cross-modal relative reliability remain central. | That another scalar correction or the same fitted features constitute a new method. |
| EXP076–093 and temporal/state-shadow syntheses: a history rule avoids one known recovery failure; learned short-history gates fail the declared transfer diagnostic. | Availability/recovery state matters in the inspected cases. | A need for an LSTM, a broadly predictive temporal quality signal, or an agent benefit. |
| EXP095–109: training software works; small RGB covariance-head adaptations fail expanded ranking transfer; saved selection exchanges explain the change algebraically. | Keep original checkpoints as reference; separate ranking from calibration and downstream pose utility. | Failure of all covariance learning, a causal optimization explanation, or an RGB–thermal method result. |

The 48-pair/48-update adaptations are bounded pilots. They neither justify more
capacity by themselves nor rule out properly designed uncertainty learning.

## What would justify the possible tools?

| Possible tool | Evidence needed before choosing it | Required simpler control |
|---|---|---|
| Covariance memory / LSTM | Past information predicts future relative geometric risk beyond current evidence, across complete recording splits and causal prefixes. Specify what persists and for how long. | Current-only model, explicit lag features or fixed smoothing, availability/recovery history. |
| Agent / sequential policy | Actions create a consequential sequential tradeoff: they change future information, estimator state, sensing/compute cost or recovery. Define actions and reward in estimation units. | Fixed policy, simple feedback policy, one-step selector with the same information and budget. |
| Reliability/calibration layer | A specific, repeatable mismatch between native measurement uncertainty and its use in a common estimator; features and correction target must be available at decision time. | Native covariance, fixed weighting, existing mass-preserving/channel controls and task-matched calibration controls. |
| New representation / larger learner | Relevant predictive signal or domain deficiency that the simpler representation misses, with sufficient development data and prospective evaluation. | Frozen original experts and capacity-matched simpler inputs/models. |

These are admission questions, not declarations that the required evidence exists.
The named tool is not the research contribution by itself. Recurrent/selective
multimodal fusion and learned sensor covariance already appear in the literature;
a targeted review is needed before any novelty claim. Relevant primary starting
points include [DeepTIO](https://arxiv.org/abs/1909.07231),
[Selective Sensor Fusion](https://arxiv.org/abs/1903.01534), and
[Learned Metrics-Aware Covariance for Visual-Inertial Fusion](https://publications.ri.cmu.edu/learned-metrics-aware-covariance-for-visual-inertial-fusion).
These references are orientation, not a completed systematic review.

## Grounded next milestone

Prepare one RGB–thermal problem/evidence/experiment definition before another fit:

1. Identify one observed cross-modal estimation failure and its exact population.
   Separate availability, measurement error, covariance weighting, selection and
   reference/solver effects. Identify what prior controls already resolved.
2. Specify the common estimation target and baseline. A shared replay of two
   independent experts or a whole-motion selector is not automatically joint
   measurement-level fusion. State which actual baseline the intervention changes.
3. State one unresolved explanation and a test that can falsify it. Existing
   fixed-input covariance, channel/mass and solver studies are prerequisites;
   simply repeating them is not the new experiment.
4. Use a reference adequate for that target. Current TD3 nominal-frame/recorded-SLAM
   proxy results and unresolved MS²/CATS conventions retain their documented
   limits. Select a feasible target/reference combination, rather than treating
   unavailable physical evidence as verified or endlessly auditing it.
5. Choose the smallest method justified by the test, then define its evaluation
   across RGB/thermal conditions and its explicit route to another sensor type.

This milestone deliberately does not choose LSTM/agent/layer, train a new model,
open reserved recordings, or manufacture a positive fusion claim. It is not a
complete executable experiment freeze: the specific unresolved failure and
adequate source/reference case must be named before a scientific run.

## Sensor-generalization criterion

The candidate contribution should state how each sensor's measurement uncertainty
is interpreted and propagated into a shared estimation task, what assumptions it
requires, and what part of the procedure transfers. Different sensors may retain
different measurement models, residual dimensions, timing and covariance units.
Raw covariance magnitudes need not be directly comparable.

RGB–thermal is the first empirical case. Generalization to a third sensor requires
an explicit additional measurement-type case and its own reference/evaluation;
it cannot be inferred from more RGB datasets or from using a generic layer name.
No third sensor is selected in this steering note.

## Source anchors and status

- [Project scope and hypothesis](../../MS_RESEARCH_HANDOFF.md)
- [EXP011 precision-mass control](../../experiments/EXP-011-precision-mass.md)
- [EXP012 channel controls](../../experiments/EXP-012-channel-mass.md)
- [Cross-modal assessment through EXP070](ms2-fall-decision-2026-10-28.md)
- [Temporal-gate synthesis](temporal-gate-findings-2026-10-31.md)
- [State-shadow proxy results](state-shadow-proxy-findings-2026-11-01.md)
- [Reference readiness](evaluation-readiness-2026-10-30.md)
- [Expanded RGB findings](expanded-fixed-model-findings-2026-11-02.md)
- [Selection-exchange diagnostic](selection-exchange-findings-2026-11-02.md)

Read-only review plus this scope update: no inference, fitting, data acquisition,
reserve access, default changes or architecture implementation. Historical results
are cited, not rerun. The existing exposure ledger and source/reference boundaries
remain authoritative.
