# Sec. VI-C positioning scenarios for the external-platform arms (E1 UAS, E2 Aerostack2)

Written 2026-09-11 21:35 EDT, before the v7-external matrix is complete, so the
narrative is chosen by a rule fixed now rather than fitted to the outcome.
Companion to `analysis_v7_output.md` (numbers) and `tab_agents_external.tex`
(table rows). Paper context: `ICRA_2027_AirStack_Paper/paper_positioning.md`,
`main.tex` Sec. VI-C (`sec:eval:agents`), Sec. III-E (`sec:principles:agents`),
Sec. VII open question "whether the method can compare across platforms".

## Tally at time of writing (5 of 20 scored, 1 of record but not counted here)

| Cell | Scored | Final-state rung |
|---|---|---|
| E2 Aerostack2 / opus-5 | 2 / 5 | R8, R8 |
| E2 Aerostack2 / sonnet-5 | 3 / 5 | R6, R8, R3 |
| E1 UAS / opus-5 | 1 / 5 | R8 |
| E1 UAS / sonnet-5 | 0 / 5 | (attempt 1 was an infra failure, re-queued) |

v6 reference (n=10 per arm): A1 8/10 R8 (sonnet 4/5, opus 4/5) · A2 6/10
(sonnet 1/5, opus 5/5) · A3 open-loop 3/10 · A4 bare parts 3/10.

## What is the same and what is different about the E arms (state these in the paper in either case)

Held fixed vs. v6: prompt v4, the two planners and their contract, judge
parameters (byte-identical `judge:` block), caps (20 judge calls, 4 h), models,
practice/eval obstacle split, top-down final-state scoring on a fresh route.

Different, and each is a confound that cuts in a known direction:

1. **Simulator.** E arms fly Gazebo (Fortress/Harmonic) with the platform's
   own vehicle model; A1 flies Isaac Sim + PX4 SITL. Isaac bring-up is the
   heavier R1/R2 (GPU, licence, longer start). A4 was also Gazebo and scored
   3/10, so Gazebo alone does not explain a high E score.
2. **The verifier travelled with us.** E arms were judged closed-loop by
   AirStack's pytest harness in host mode, with 20 in-session calls, exactly
   like A4. Neither Aerostack2 nor UAS ships a full-stack flight test
   (Table I). So an E cell measures "that platform + AirStack's closed-loop
   judge", not that platform as released. A3 (same repo, loop opened) fell
   from 8/10 to 3/10; the E arms never ran open-loop.
3. **Training-data exposure.** Aerostack2 has been public since 2022 and is
   widely forked; UAS is a 2026 release; AirStack's own exposure is partly
   controlled by A2. No equivalent control exists for E1/E2.
4. **Provisioning parity.** Platform images pre-pulled/pre-built and sources
   cloned at pins, mirroring A1's pre-pulled images; workspace compile left to
   the agent in both. Recorded per trial.
5. **Substrate.** OSMO pods (RTX PRO 5000, 48 GB), not the v6 workstation;
   separate campaign hash; reported beside v6, never pooled.
6. **Task provenance.** The ladder was miniaturised from AirStack case
   studies. A high E score is evidence the ladder is not AirStack-shaped,
   which helps the method's validity in BOTH scenarios.
7. **Sample size.** n=5 per cell, 10 per platform. Wilson 95% intervals for
   8/10 and 6/10 overlap almost entirely; nothing between 6/10 and 10/10 is
   separable from A1. Voice rule: report counts, no ratios, no "faster".

## Decision rule (fixed now)

Per platform, n=10, final-state R8 count:

- **Scenario A, on par or better:** R8 ≥ 7/10 on a platform (A1 is 8/10).
- **Scenario B, a little worse:** R8 in 4–6/10.
- (Not asked, noted for completeness: ≤ 3/10 ≈ bare parts; then the story is
  that "released platform" is not one category and design principles decide,
  which the internal ablations already carry.)

The two platforms may land in different scenarios; the paragraph then reads
per platform, with the same skeleton.

---

## Scenario A — Aerostack2 and/or UAS on par with or better than AirStack

**Thesis.** The platform effect is real and shared by released platforms:
released platforms cluster at the top of the ladder, bare parts at the
bottom, and AirStack's closed-loop judge scores them all. AirStack's
platform-specific advantage is what the internal ablations support, not a
ceiling above its peers: consistency for the smaller model (A1–A2) and the
verification loop (A1–A3). The method (C-4) gains external validity, and the
open question in Sec. VII ("can the method compare across platforms") closes.

**What changes in the paper.**

- *Abstract / Intro / Conclusion.* Extend the three-number sentence to four:
  "8 of 10 on the platform, 3 of 10 from bare parts, and X of 10 and Y of 10
  on two other released platforms judged by the same harness." Put the
  external number next to A1, not after A4, so the reader sees the cluster.
- *Sec. III-E Agent legibility.* Add one sentence: agent legibility has three
  ingredients (scaffolding, command conventions, a verifiable success
  signal); Sec. VI-C lends the third to two other platforms. This pre-empts
  the "then Aerostack2 is equally agent-native" review by defining the term
  before the number appears.
- *Sec. VI-C Arms.* Add E1/E2 as a second campaign paragraph: what was held
  fixed, the seven differences above compressed to one sentence (simulator,
  closed-loop judge lent, no ablation control), and "reported beside, not
  pooled".
- *Sec. VI-C Results.* Lead with the ordering as counts. Then the one
  sentence that carries the reframing: "Both external platforms ran under
  AirStack's closed-loop judge, which neither releases; the same repository
  with the loop opened (A3) fell to 3/10, so the verifier, not the codebase,
  is the ingredient the external arms borrowed." Then the internal-ablation
  claims unchanged. Then any secondary metric that actually separates
  (judge calls per rung, hours, cost, sonnet/opus split); if none does, say
  "indistinguishable at n=10" and stop.
- *Table `tab:agents`.* Add a rule and two rows (E1, E2), caption notes the
  campaign split and simulator. If sonnet/opus diverge on an E arm the way
  A2 did, report it per model like A2.
- *Fig. `rung_survival`.* Add E1/E2 curves (already produced as
  `rung_survival_ext.pdf`); keep A-arm styling, dashed for the v7 campaign.
- *Sec. II Related Work.* Aerostack2 sentence gains "and agents found it
  comparably workable under our harness (Sec. VI-C)". Generous, and it makes
  the Table I comparison read as measured rather than asserted.
- *Sec. VII.* Replace "whether the method can compare across platforms,
  which needs a stack-neutral task suite and judge" with: it did, on two
  platforms, with the confounds named (simulator, lent verifier, exposure);
  the open question becomes running the external arms open-loop.
- *Title.* Keep "Agent-Native". The word describes the design (Sec. III-E),
  which now also covers lending the verifier. Do not soften it to a
  comparative.

**If E is strictly better (e.g., 10/10).** Say it plainly. Then locate where
A1 lost its two: A1/sonnet #5 shipped a final state with the swap not
persisted, A1/opus #5 lost corridor order on the fresh R7 route. Both are
final-state/generalisation failures, not bring-up. The Isaac R7 field is a
genuinely different obstacle problem from the Gazebo one (different
vehicle dynamics, different planner behaviour), so the honest sentence is
"the ceiling on a Gazebo platform under the same judge is at least as high".

**Optional disambiguating run (decision for the lead).** E2/opus open-loop,
n=5, judge once at the end. Cost ~5 × 2 h × 1 pod, ~US$100, one afternoon on
4 pods. If it falls toward A3's 3/10 it converts confound (2) into a
finding: the closed-loop harness is the transferable ingredient. If it
stays high, the paper says so and the platform effect is Aerostack2's own.
Feasible before 2026-09-15 only if the 11 queued trials finish by 09-12.

**Risks in A.** (i) A reviewer reads "8 vs 9" as AirStack losing; the counts
paragraph must state the interval overlap once. (ii) Someone is tempted to
go fishing in secondary metrics; pre-commit to the four in the table and
the per-rung median judge cycles already reported for A-arms. (iii) The
"Agent-Native" title needs the III-E sentence or it reads as a claim the
data contradicts.

---

## Scenario B — Aerostack2 and/or UAS a little worse (4–6/10)

**Thesis.** All released platforms beat bare parts; on top of that,
AirStack's agent-facing layer buys consistency, and the external platforms
look like the ablated arm A2. The ordering A1 > E ≈ A2 > A3 ≈ A4 is
reported as counts and left for the reader; the mechanism claim comes from
the failure taxonomy, not from the gap.

**What changes in the paper.**

- *Abstract / Intro / Conclusion.* Same four-number sentence as Scenario A,
  ordered A1, E, A4. No adjective.
- *Sec. VI-C Results.* Lead with counts. Then two sentences that do the
  work: (1) "At n=10 the AirStack–external gap is not separable (intervals
  overlap); the comparison that is separable is released platform vs bare
  parts." (2) "The external arms' sub-ceiling final states fall in the same
  failure classes as A2's: <taxonomy from the trials>." Fill (2) from the
  actual trials. E2/sonnet #1 already fits: in-session R1–R6 passed, the
  shipped state boots with the beta planner configured but alpha not
  running and takeoff timing out, i.e. a working system broken during the
  swap and never re-verified, the class already seen in two A2/sonnet
  trials. E2/sonnet #2 is an R7 clearance failure (no avoidance chain),
  the A4 class. If E failures are simulator flake or infra, they are not
  platform findings and must be excluded or labelled.
- *Per-model split.* This is the clean version of B. A2 lost only with
  sonnet (1/5 vs opus 5/5). If E's losses are also concentrated in sonnet
  (currently E2/sonnet 1/3, E2/opus 2/2, E1/opus 1/1), the existing sentence
  "agent-facing scaffolding matters most for the smaller model" gains an
  external replication: platforms without agent scaffolding behave like
  AirStack with its scaffolding removed. Report E rows per model like A2.
- *Table / Figure.* As in A. Order rows by campaign, not by score.
- *Sec. III-E.* Same lent-verifier sentence as A; it is true in both
  scenarios and stops the confound from being discovered by a reviewer.
- *Sec. II Related Work.* Aerostack2: "agents completed the same ladder on
  it in X/10 trials under our harness (Sec. VI-C)". No comparative.
- *Sec. VII.* Same closure of the cross-platform question, plus the
  exposure confound stated explicitly (Aerostack2's longer public history
  would bias toward it, so it does not explain a lower E score; UAS's
  short history could).

**Risks in B.** (i) Over-claiming superiority from 8 vs 6; the interval
sentence is mandatory and "NX" ratios are banned by the voice rules.
(ii) Attributing E failures to the platform when they are Gazebo/pod
infrastructure; rule 4 reruns cover pod failures, but a Gazebo timing
flake inside a scored trial stays scored, so name it. (iii) Reading the
sonnet effect from 5 trials; say "consistent with", not "shows".

---

## Shared across both scenarios

- Never pool v6 and v7. Never compute A1/E ratios. Counts, intervals once,
  reader computes.
- The E arms borrowed the verifier; say it in III-E and VI-C in both cases.
- The ladder is not AirStack-shaped; a high E score proves it, a middling
  E score with A2-like failure classes is consistent with it. Both help C-4.
- Release the E transcripts and final-state patches with the A-arm
  artifacts (both platforms are public; the private `helicopter_bridge`
  repo is recorded as un-importable in the provisioning log).
- Amendment 4 must be lead-approved and its analysis-time labels fixed
  before the paragraph is written (raw ids E1/E2 stay in artifacts).
- Budget: VI is at 1.75 pages with ~0.15 page free on page 8. The E
  paragraph plus two table rows costs roughly 0.12 page; the two
  `\todo{verify}` markers in Related Work about Aerostack2/UAS features can
  be resolved from the trial ENVIRONMENT.md facts and shortened to pay for
  it.
