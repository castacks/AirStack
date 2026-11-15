# RGB–thermal handover: prior-art and continuation decision

14 November 2026, America/New_York. Targeted primary-source search and local
inventory review; no new experiment, author-code execution or novelty certification.

## Decision

Do not advance the broad claim that thermal fallback, RGB–thermal handover or
selective VO computation is new. Direct same-modality prior work exists. Applying
it to MAC-VO/MACThermal is an integration unless a distinct mechanism and measured
benefit over applicable prior/simple controls are established. Application to a
particular dataset or replacement of the backbone is insufficient by itself.

The narrower question of deadline-aware reactivation of stateful RGB/thermal
experts under bounded retained-input and compute budgets remains unverified as
a research gap. This search did not establish an identical implementation, but
absence of an exact match in a targeted search is not evidence of novelty.
Ordinary buffering, recovery confirmation and ownership enforcement are not
individually new research contributions.

## Direct and related sources

- Magnabosco and Breckon, Cross-spectral visual simultaneous localization and
  mapping (SLAM) with sensor handover, RAS 61(2), 2013, online 2012:
  https://breckon.org/toby/publications/papers/magnabosco13slam.pdf
  Primary author-hosted abstract/introduction inspected from indexed text; full
  handover method/code not inspected. Explicit optical/thermal mobile-robot SLAM
  and sensor handover across illumination changes. This directly defeats a generic
  RGB–thermal handover novelty claim. Related 2015 Multispectral Stereo Odometry
  author-hosted PDF introduction/reference27 independently identifies this work.
- Fukuda and Fukuda, Robustness Enhancement of Self-Localization for Drone-View
  Mixed Reality via Adaptive RGB-Thermal Integration, Technologies14(1),74,
  22January2026: https://www.mdpi.com/2227-7080/14/1/74
  Targeted sections3–6 inspected. Parallel RGB/thermal tracking, spatial-inlier
  reliability gating, extrinsic frame conversion, translation interpolation and
  quaternion SLERP; thermal-led tracking when RGB degrades. RGB-map VPS supplies
  global localization. Demonstrated MR registration is not a directly comparable
  VO ATE benchmark. No released implementation inspected. No bounded sparse
  reactivation mechanism established by inspected sections; a missing text term
  does not prove absence of an equivalent mechanism.
- Yang, Chen and Kim, Efficient Deep Visual and Inertial Odometry with Adaptive
  Visual Modality Selection, ECCV2022:
  https://arxiv.org/abs/2205.06187
  Primary abstract inspected. Learned policy disables expensive visual feature
  extraction using motion/IMU information. Compute-aware modality selection is
  established; RGB/IMU differs from two stateful RGB/thermal experts.
- Pan et al., Dual-Agent Reinforcement Learning for Adaptive and Cost-Aware
  Visual–Inertial Odometry, CVPR2026, arXiv2511.21083v2:
  https://arxiv.org/html/2511.21083v2
  Targeted sections3.2–3.4 inspected. IMU-only scheduling skips complete VO
  invocations; separate adaptive fusion agent. No thermal expert is described in
  inspected mechanism. This blocks generic cost-aware invocation novelty.
- MSO remains a direct RGB–thermal–inertial adaptive-estimation comparator.
  Existing local full thesis Chapter5 review remains authoritative:
  mso-method-overlap-review-2026-11-12.md. Fresh author publication-page recheck:
  https://shibowing.github.io/ . Exact later MSO code/manuscript still unverified;
  do not infer missing capabilities from unavailable code.

## Correction to preceding experiment summary

The prior conversational recommendation emphasized EXP074–077 and EXP122–124 but
omitted later EXP125–127. That omission made the candidate look stronger than the
complete inventory supports. EXP125 night transfer gives67/95 selected deliveries
with137calls, versus fixed thermal71/95 with83calls. EXP126 finds no RGB-only
availability opportunity in those histories. EXP127's18dense natural windows
from7exposed recordings have zero availability headroom over the best fixed expert.
See research-position-2026-11-03.md and EXP-125-night-transfer.md.

The bounded handover mechanism works in its synthetic engineering condition;
this does not establish utility across the later conditions. Do not rerun the
same closed failure as a newly proposed experiment or claim that its positive day
fixture alone establishes a new multisensor VO solution.

## Next work before numerical execution

Compare the original handover method and closest selective-compute methods at
mechanism level: shared versus separate estimator state, activation timing,
retained observations, reinitialization, deadlines and measured cost. Identify
one consequential unsupported case in our intended deployment. If an existing
method handles it, adopt/adapt that baseline. Only pursue a new mechanism if its
difference predicts a measurable advantage and existing evidence can test it.
No new numerical pipeline experiment or new algorithm is selected by this review.
