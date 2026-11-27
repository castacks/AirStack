# PD architecture decision package — M00-E1C

**DESIGN PROPOSAL ONLY — NO DEPLOYMENT OR TIER B EXECUTION AUTHORIZED**

## A. Problem statement

The existing workspace has not demonstrated simulator-only isolation. E1B's dated evidence showed privileged containers, ttyS0–31/broad devices, shared writable source/cache mounts, shared bridge/published ports, DDS router command-service paths1↔0/99 and command-capable processes. Those observations establish exposure and candidate limits, not an active physical connection/mission or current E1C runtime state. No named owner, safe exact target, complete native instrumentation, independent stop/recorder proof or signed operational authority is available. G0 remains blocked.

## B. Candidates

Existing workspace CAND01 is an inspectable historical reference with installed hashes/logs but unresolved physical/network/mission and interference paths. Source workflow templates CAND02/03 privileged/autolaunch broad stacks and are not evidence of deployed independent rigs. CAND04 independent startup-only architecture would exclude physical vehicle paths, GCS/Cosmos/routers and operational mission components by intended artifact/network/device boundary. CAND04 is not constructed, deployed, verified or designated.

## C. Recommended architecture

Prefer CAND04 **for further design engineering only**: owner-attested independent host/boundary, native non-privileged runtime without DinD, minimal Isaac/Pegasus/PX4 SITL + MAVROS + diagnostic guard host + dependency observer, independent durable evidence supervision. This reduces unnecessary control/workload surfaces and makes startup dependencies observable. It carries higher initial build/security/asset/instrumentation effort. Non-privileged GPU/Isaac, offline assets, middleware policy and evidence failure-domain capability remain unverified. If unsupported, return for redesign; no privileged/shared-host fallback is approved.

The diagnostic host excludes full robot_interface arming/mode/setpoint publishers/clients while retaining exact original guard semantics. This is a material integration/configuration delta requiring reviewer/PD acceptance, source/build/executor/overhead comparison and development-only claim limits; it does not establish production behavior or CFG-SIL-01 equivalence. No original24-row trial/schedule modification is adopted here; bind any future approved candidate explicitly before a startup.

## D. Isolation controls

IC01–IC12 specify exposure, mitigation, enforcement, verification evidence, failure/residual risk and proposed owner role for physical/device exclusion, privileges/mounts, MAVLink/DDS/command boundaries, external/mission/arming paths, operator and emergency/evidence/resource controls. Every EX01–EX12 E1B exposure is mapped. Strong controls are artifact/device/network exclusion and minimal process/API permissions, not labels or procedural claims. Target isolation remains UNVERIFIED; proposed controls are not implementation proof.

## E. Configuration and owner dependencies

Manifest spec contains12 inherited reference facts and17 null future-target groups: source/build/image/scene/PX4/MAVROS/DDS/network/devices/limits/instrumentation/commands/cache/supervision/operator/authority remain to be exactly verified. RQ01–RQ14 formally request owner identity/exports/full provenance/capability/approval evidence through PD routing; no external owner request has been sent and no named owner response is claimed. Original FR-01–FR-20/A3/applicable E9/startup allocation still missing; only supplied excerpts and dated investigation records available. No missing digest/workflow/setting invented.

## F. Minimum implementation scope — requires a later exact directive

1. Resolve named environment/security/hardware/evidence ownership, platform feasibility, development claim limits and target-selection/designation through owner records and PD decisions.
2. Prepare reviewed exact source/build/package/plugin/guard-host/hook/launcher/asset/cache/security/mount/resource manifests and concrete diffs in a separately authorized engineering scope. Define finite P1 wait and operational stop/storage/capture/overhead bounds. No protected edits or build/deployment performed in E1C.
3. Under a separately authorized **construction** scope, create dedicated direct-runtime job/profile/private storage/network/device policy and minimal diagnostic binaries, excluding broad autolaunch/mission/GCS/router/physical interfaces. Actual artifacts/commands must be reviewable before that decision; this package provides design specifications only.
4. Require separate exact **deployment/activation** authority and implementation attestation; no construction decision implicitly grants a start. Then separately authorize bounded inert policy/clock/recorder/stop validation (E1B AV01–AV04) only after ClassD safety prerequisites.
5. Only if evidence supports readiness, submit a new explicit TierB decision for exact instrumented startup scope/commands/count/checkpoint/owner/limits/retention. Keep all failed/aborted/incomplete/setup starts, no retries/substitution/hidden cache warm-up. WP01/qualification/HIL/REAL remain separate and unauthorized.

## G. Safety and operational risks

Proposed structural exclusions could eliminate selected serial/bridge/mission/control paths **if correctly implemented**; host/GPU/kernel policy, remote assets, transitive plugins/automatic PX4 traffic, DDS permissions, residual command APIs, mutable mounts/cache, recorder/job failure domains, clock/drop/overhead and target-only emergency-stop ownership remain implementation/active-verification dependencies. No active test may be used as a substitute for unresolved physical boundary ownership. Design review does not declare a rig safe, close REC-01/BOUNDARY or change F00 acceptance.

## H. Requested PD architecture decision — all options UNSELECTED

- [ ] APPROVE DESIGN FOR FURTHER ENGINEERING
- [ ] RETURN FOR REVISION
- [ ] REJECT PROPOSED DESIGN

Selected option: ____________________

PD name/signature: ____________________

Decision date/time (America/New_York or stated UTC), reference and conditions: ____________________

Scope/claim-limit and guard-host delta disposition: ____________________

Owner-evidence routing/design revisions required: ____________________

No actual PD decision supplied. Approval of design for further engineering **does not authorize construction, deployment, activation, active verification, TierB trials, target designation, WP01 or protected implementation**. Each later activity needs its own exact reviewed scope/decision; the software pending record leaves selection null and every operational authority false. RA01 remains pending independently. Submit for ChatGPT review, then stop after E1C.
