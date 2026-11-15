# Handover mechanism comparison and decision

14 November 2026 America/New_York. Targeted primary method reading; no code
execution, baseline reproduction or new numerical experiment.

## Comparison

| Method | State across missing visual input | Activation and recovery | Difference from our buffered expert implementation |
|---|---|---|---|
| Magnabosco/Breckon2013, sections4.3/5.4 | Shared EKF robot/landmark state; wheel-encoder and GPS inputs also described | Optical observations suppressed over a predetermined contiguous section, then restored; thermal observations sustain combined estimation | Not two independently paused native VO graphs. Synthetic missing-camera continuity and shared-state handover are already demonstrated. |
| Fukuda/Fukuda2026, sections3–4 | Parallel visual trackers plus RGB-map global localization loop | Each modality computes tracking; spatial-inlier reliability suppresses weak output and weights fusion | Described reliability decision follows tracking; not evidence of skipping the expensive inactive tracker beforehand. Recovery memory/deadline budget not established in inspected method. |
| Yang/Chen/Kim2022, section3 and figure2 | Pose-regression LSTM continues with inertial features; absent visual feature input replaced with zeros | Policy uses current inertial features and previous hidden state; selected adjacent image pair invokes visual encoder | Continuous recurrent state with IMU covers skipped visual updates. Does not establish two camera-specific graph reactivation under retained-image bounds. |
| Pan et al2026, sections3.2–3.6 | Corrected IMU propagation and fused state continue; invoked DPVO-inspired backend maintains pose/patch variables | IMU-only agent invokes complete VO, fusion agent incorporates visual observation | Already selects expensive stateful VO; paper description does not fully specify skipped-frame queue/recovery semantics. Do not claim it lacks a buffer or cannot recover. |
| MSO thesis Chapter5, existing full review | Shared sliding-window visual/thermal/inertial BA | Reliability-weighted factors and adaptive removal | Strong direct multispectral baseline; sparse expert reactivation implementation unresolved, not proven absent. |
| ore-ms EXP122–125 | Separate native camera histories, encoded predecessor retention; one bounded catch-up | Probe feedback and source continuity determine processing; explicit native writeback/source-group deadlines | Mechanism is implemented. Benefit is not established across conditions: night67/95 with137calls vs thermal71/95 with83calls. |

## Conclusion

No inspected source establishes the identical bounded encoded-predecessor
catch-up implementation. That is an implementation distinction, not a verified
research gap or useful advantage. Shared-state multisensor estimators can avoid
some of the separate-expert reactivation problem architecturally. We must not
introduce separate experts, repair their avoidable state discontinuity, and count
that alone as progress over shared-state alternatives.

Treat current buffering/lease work as reusable engineering. Do not launch the
proposed combined replay as validation of a new research solution. Evidence does
not currently justify prioritizing a new handover method: generic overlap is
strong and later local utility is negative. This closes prioritization of this
specific proposal on current evidence, not RGB–thermal research or recovery as a
field. A strong application-specific empirical improvement can still constitute
a contribution even if every component exists; we currently lack that comparison.

Next useful work is baseline selection for the actual VO objective: identify a
reproducible multispectral estimator or clearly labeled method-inspired baseline,
and compare its accuracy/coverage/cost against our current pipeline. Prefer an
existing solution if it meets the application. A new method needs a concrete
remaining failure, not absence of an identical combination of software details.

## Reading depth and sources

- https://breckon.org/toby/publications/papers/magnabosco13slam.pdf
  Targeted sections4.3 and5.4, shared state and handover execution. Author-code not
  inspected. Paper sensor description includes encoder/GPS aids: do not label it
  a camera-only baseline.
- https://www.ecva.net/papers/eccv_2022/papers_ECCV/papers/136980229.pdf
  Targeted figure2 and section3.1–3.2, adjacent pairs, LSTM and zero padding.
- https://arxiv.org/html/2511.21083v2
  Targeted sections3.2–3.6; queue/reset/retention details remain unqualified.
- https://www.mdpi.com/2227-7080/14/1/74
  Prior-turn targeted sections3–6 reading retained; no fresh implementation review.
- mso-method-overlap-review-2026-11-12.md and research-position-2026-11-03.md:
  Existing thesis review and complete local transfer assessment retained.
