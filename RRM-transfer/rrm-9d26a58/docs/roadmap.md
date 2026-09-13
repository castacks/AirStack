# RRM-1 Roadmap

Anchored to **IROS 2027 — 1 March 2027**. 197 days from a working mock loop to submission.

## Why not ICRA 2027

ICRA 2027's main track closes **15 September 2026**. With Isaac Sim unwired, that is not a
full paper. Treat it as a possible workshop outlet for the architecture (matching the
brief's §23 "early stage: architecture + simulation results") and aim the complete result
at IROS. Secondary: CoRL 2027, ~April 2027.

## Schedule

```mermaid
gantt
    title RRM-1 to IROS 2027
    dateFormat YYYY-MM-DD
    axisFormat %b

    section W1 Isaac backend
    Provision + verify environment      :2026-08-16, 4d
    USD scene, Panda + annotations      :2026-08-20, 11d
    Deterministic episode reset  (GATE) :crit, 2026-08-31, 10d
    observe() poses + relations         :2026-09-05, 10d
    apply() trajectory + sim step       :2026-09-12, 9d

    section W2 GR00T policy
    Load N1.7-3B, LIBERO_PANDA          :2026-09-21, 5d
    EmbodimentAdapter                   :2026-09-26, 7d
    Obs dict, chunk to JointTrajectory  :2026-10-03, 8d
    Numeric safety, swept vol   (GATE)  :crit, 2026-10-08, 11d

    section W3 Learned reasoner
    vLLM + guided_json smoke test       :2026-10-19, 3d
    Prompt from VERB_TABLE, versioned   :2026-10-22, 6d
    LocalReasoner parser + repair       :2026-10-28, 7d
    Oracle vs learned comparison        :2026-11-04, 5d

    section W4 Tasks and metrics
    T3, T4, T7 scenes                   :2026-11-09, 12d
    T5 ambiguity, T10 held-out          :2026-11-21, 8d
    Safety-labelled scenarios   (GATE)  :crit, 2026-11-23, 10d
    Close metric instrumentation gaps   :2026-11-30, 7d

    section W5 Baseline arms
    Baseline A, VLA direct              :2026-12-07, 7d
    Baseline B, no world model          :2026-12-14, 7d
    Baseline C, no predictive layer     :2026-12-21, 7d
    Arm-switching harness               :2026-12-28, 7d

    section W6 Benchmark
    Seeded sweeps                       :2027-01-04, 14d
    Ablations                           :2027-01-18, 7d
    Analysis, results frozen            :2027-01-25, 7d

    section W7 Paper
    Draft                               :2027-02-01, 14d
    Figures from traces                 :2027-02-08, 11d
    Review and revision                 :2027-02-15, 11d
    IROS submission             (GATE)  :crit, 2027-02-25, 5d
```

## Gates

Slippage here propagates to the submission date rather than being absorbed. Each is
scheduled early inside its stream for that reason.

| When | Gate | Why it blocks |
|---|---|---|
| 31 Aug – 9 Sep | **Deterministic episode reset** | Isaac physics is not reproducible by default and every reported number needs seeded episodes. Nothing in W4–W6 is publishable without it — which is why it precedes `observe()` despite being the less interesting task. |
| 8 – 18 Oct | **Swept-volume collision** | Safety #2 checks scalars against placeholder ranges today: structurally correct, numerically fake. Collision rate and safety violation rate are both meaningless until it is real. Most likely item to be underestimated. |
| 23 Nov – 2 Dec | **Safety-labelled scenarios** | Verifier precision, recall and false-negative rate cannot come from traces — traces record what the verifier *decided*, never whether it was *right*. Scenarios must carry known-unsafe actions labelled in advance. Retrofitting invalidates every episode collected before the change. |
| 25 Feb – 1 Mar | **Submission** | Four days of buffer is thin. The mitigation is freezing results on 31 January and refusing to re-run sweeps during write-up. |

## Metric instrumentation status

45 target metrics (`docs/evaluation_matrix.md`) against 10 instrumented. Grouped by what
unblocks them, since that ordering is what the schedule is built from.

| Group | Count | Unblocked by |
|---|---:|---|
| Live now — computed from traces, needs naming only | 7 | — |
| Unlocked by the Isaac backend | 6 | W1, W2 |
| Free from the Oracle — reference plans, no labelling | 4 | W3 |
| New instrumentation, straightforward | 6 | W3, W4 |
| **Needs labelled ground truth** | 5 | **W4 gate** |
| Deferred past submission | 4+ | Phase 6 / post-IROS |

Two things worth carrying forward:

**The Oracle pays for itself here.** Task Decomposition Accuracy, Step Ordering Accuracy,
Action Efficiency and Plan Validity all become computable by diffing a learned reasoner's
task graph against the Oracle's on the same scenario — four reasoning metrics with no
hand-labelling, because the Oracle was kept as a permanent reference ceiling.

**No weighted composite until the weights are validated.** `docs/evaluation_matrix.md`
specifies 25/15/15/15/10/10/10; those are a hypothesis. A single number from unvalidated
weights invites a reviewer to reject the framing instead of engaging with the result.
Report the per-metric table. Safety is never averaged away — any hard violation fails the
run regardless of task success.

## Beyond submission

Phase 6 perception (real detector + depth, and the VRAM upgrade that implies), sim-to-real
on a physical Panda, then cross-embodiment via `UNITREE_G1` — which needs only an
`EmbodimentAdapter` swap, since it is pre-registered in GR00T and requires no fine-tuning.
