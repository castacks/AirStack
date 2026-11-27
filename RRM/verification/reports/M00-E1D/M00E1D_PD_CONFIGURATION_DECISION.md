# PD configuration-change decision package

**DESIGN REVIEW ONLY — NO CONSTRUCTION, DEPLOYMENT, OR TIER B EXECUTION AUTHORIZED**

## A. Decision requested

Whether to accept the E1C diagnostic guard host as part of an independent startup-only design, accept a reviewed alternative for further engineering, defer pending evidence, or reject. This packet requests a design disposition only. Target designation remains UNVERIFIED. Engineering recommends **Prefer separated safety and diagnostic components (C)**; no PD selection is supplied.

## B. Technical rationale

The original guard must execute to investigate its deadline/transaction/latch behavior. Source verifies its real state subscriptions, readiness/status and guarded parameter Get/Set; full production interface also creates command clients/publishers. A minimal guard process can avoid loading those production producers but changes integration/executor context. Physical/device/network/DDS enforcement is a separate platform function in all designs. No evidence requires an extra physical diagnostic host or inline proxy. C makes this allocation explicit and retains only necessary instrumentation. Benefit is a proposed reduction in coupling and clearer evidence claims, not measured latency/safety improvement. E1C already separates these boundaries; C refines rather than repudiates that design.

## C. Alternatives and trade sensitivity

A retains the E1C independent minimal guard/observer/hooks/recorder proposal. B uses verified platform controls and passive instrumentation, omitting the guard/full interface; it addresses external startup phases but cannot supply the missing internal guard timing/latch result. C keeps the minimal original guard subject, necessary native events and separate independent safety/stop and read-only evidence roles, without a diagnostic gateway. The alternatives report and eight-row qualitative trade CSV contain detailed criteria; no approved weights or measured scores are implied.

Sensitivity: if the question narrows to external simulator/PX4 phases, B becomes preferable for lower guard code change, requiring explicit scope approval. If full production integration fidelity dominates, none of A/B/C establishes it; defer and review a separately isolated production-context alternative. If A already meets C's minimal separation and supported executor constraints, their technical ranking is a tie; choose based on owner evidence/maintenance rather than component count. If platform deny policy/GPU-only capability or independent stop/evidence support fails, defer or redesign all candidates; no score can trade away safety prerequisites. If necessary internal hooks impose unacceptable measured interference, revise capture/claims before selecting, rather than infer a correction from historical intervals.

## D. Configuration impact

Material deltas include a new guard process/build/executor, smaller MAVROS/plugin/API set, isolated runtime/network/devices/mounts, independent P1 observer/launcher, native hooks/clocks/queues and outside-executor recorder/stop ownership. Source/image/PX4/Pegasus/scene differences already prevent complete frozen equivalence. Exact full manifests, actual effective settings, permitted startup transaction and diagnostic claim limits need reviewer/PD disposition. All resulting evidence would concern the declared diagnostic environment; CFG-SIL-01 equivalence requires an approved complete argument, and qualification additionally requires authorized applicable evidence and decision chain. No protected change, deadline extension/latch bypass or acceptance change is proposed.

## E. Safety implications

SF01–SF12 retain physical actuation/FC, privileges/mounts, MAVLink/DDS/network, guard/instrumentation/recorder loss, restarts and new dependencies as OPEN. IC01–IC12 are proposed controls, not enforcement proof. Independently enforce physical/device/network boundaries and exact-job stop regardless of guard or recorder health. Domain labels, passive monitoring, absent serial FD, SITL labels or READY cannot establish isolation. Common host/kernel/storage failure and unknown effective APIs remain. No quantitative probabilities or safe-to-operate finding is assigned.

## F. Measurement implications

MR01–MR11 cover every phase, executor/guard semantics, capture/discovery/resource/clock effects and evidence censoring. Native producer events, valid clock maps and bounded async collection still have unmeasured overhead. Separately authorized source/executor review, inert clock/loss/stop evidence and eventual exactly scoped matched startup characterization are required. Preserve historical19.276162179s and74.228771713s, n=1, as observations only. No effectiveness, distribution or new threshold is derived; no24-row experiment alteration or extra characterization starts are approved.

## G. Platform dependencies

OG01–OG06 consolidate RQ01–RQ14: named delegated owners/permitted workflows; GPU-only nonprivileged and physical exclusion; namespace/MAVLink/DDS/API deny capabilities; minimal guard/executor and native hooks; allocation/clocks/independent recorder/stop; immutable artifacts/assets and F00 authority/claim limits. All responses await owners and no external request is sent. No candidate has verified feasibility, effective isolation or designation.

## H. Recommended decision and offered PD options

Technical preference is C **for further engineering only**, conditional on owner evidence. Design selection can be deferred while collecting the six existing-evidence responses. The PD alone chooses; all options remain UNSELECTED:

- [ ] APPROVE GUARD-HOST DESIGN FOR FURTHER ENGINEERING
- [ ] APPROVE ALTERNATIVE DESIGN FOR FURTHER ENGINEERING
- [ ] DEFER DECISION PENDING EVIDENCE
- [ ] REJECT PROPOSED DESIGN

## I. Approval record

PD decision: ____________________

Approved scope: ____________________

Conditions: ____________________

Date (America/New_York or stated UTC): ____________________

Authorization reference: ____________________

No approval fields have been populated. A design decision does not authorize construction, deployment, activation, target designation, policy/fault testing, simulator/service start/restart, Tier B/C, physical commands, WP01, qualification or pushing. Any subsequent action needs its own exact reviewed authority. RA01 remains pending separately.
