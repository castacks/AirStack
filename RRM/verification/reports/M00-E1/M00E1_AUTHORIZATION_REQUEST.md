# M00-E1 PD decision request

**PREPARED — NOT AUTHORIZED FOR EXECUTION**

## 1. Purpose and justification

Request review of a future isolated simulator startup investigation. The single historical episode missed readiness dependencies before guard expiry and retained terminal STARTUP_DEADLINE. The 19.276162179-second deadline-error→MAVROS interval and 74.228771713-second app-complete→graph interval localize observations without identifying exclusive scene/resource/transport subcauses. No new startup or corrective runtime action occurred in E1.

## 2. Exact requested scope

Proposed M00-E2 Tier B development diagnostics only: one simulated Iris/PX4 SITL vehicle, no mission/arming/motion commands, no physical connections, no WP01 or qualification. Review package now; execution remains blocked until exact configuration, safe target and controls are supplied. If PD chooses to authorize preparation of a missing target/candidate first, that must be a separate explicit bounded scope, not inferred approval of these trials.

## 3. Number and type of trials

At most 24 proposed startups: concurrent P0 versus dependency-aware P1 × fresh dedicated versus pre-existing pinned warm cache × six repetitions. The CSV freezes seeded paired order. All setup/failed/aborted/incomplete starts count; no replacement to reach successful n. Exploratory within-pair phase contrasts, medians, observed ranges and complete outcome counts; no power, reliability, p95 or requirement qualification claim. M00-D design is preserved.

## 4. Identified environment

Logical proposal ENV-M00E2-ISOLATED-01-PROPOSED / CFG-M00E2-STARTUP-01-PROPOSED is not a provisioned environment. OSMO workflow UID, pool, host, containers, resource limits, exact images/source/hooks, stage/assets, endpoint allowlist and command manifest are PENDING. The configuration register separates verified reference metadata from target proposals/unknowns/approvals. Existing privileged shared-network containers are not an approved target and are not equivalent to CFG-SIL-01.

## 5. Required service operations

OP01 provision independent workflow/resources/network; OP02 stage reviewed pinned candidate; OP03 clone dedicated cache/evidence volumes; OP04 start isolated simulator/PX4; OP05 start policy-assigned MAVROS/interface; OP06 target-only stop/abort and scheduled subsequent starts; OP07 bounded independent evidence hooks and the normal guard startup Set/Get double1.0 transaction. **Every operation requires separate written PD authorization** tied to exact commands and target. No manual repair, gate bypass, timeout extension, unrestricted restart or active resource stress is requested. None was performed in E1.

## 6. Safety/isolation evidence

Verified reference Docker metadata, GPU identity and visible CPU/RAM are captured in passive-environment.json. Privilege, shared networking and published ports establish limitations; device isolation is unproven. No target safety control is certified. Independent device/network/process ownership attestation and a named stop authority are mandatory before startup. Safety review describes required evidence and target-only recovery.

## 7. Risks and mitigations

Hardware/network escape: deny physical paths and audit effective endpoints/capabilities. Cross-workflow effects: dedicated ownership/volumes/routes and bounded resource allocation. Unintended autostart/commands: freeze reviewed entrypoints and process allowlist, omit GCS/mission stack. Evidence loss/observer effect: independent supervised bounded recorder, clock maps/drop counters/overhead review. Cache/order confounds: paired randomized schedule and pinned independent cache copies. Mitigations are proposed and unverified, not completed controls.

## 8. Preconditions

CHK01–CHK18 checklist is binding for a later decision. Only CHK17/18 document preparation/integrity are verified as documents. Target configuration, isolation, hooks, clock mapping, cache artifacts, independent recorder, operational limits, stop/recovery and operator ownership remain unresolved. No blank item may be read as approval. Technical review must reconcile all candidate differences, including the P1 predicate and unchanged guard semantics. Missing original startup requirement limits HD06 adjudication and qualification even if diagnostic scope is later authorized.

## 9. Abort and recovery

Immediate abort for unknown/physical device or peer, arming/motion/mission traffic, configuration drift, unrelated workflow impact, recorder/clock loss, limit breach, uncontrolled restart or safety doubt. Exact numeric time/storage/resource limits and named escalation owner must be approved before operation; historical timings supply no limit. Stop exact target-owned command producers/process tree; preserve independent evidence if safe. Seal partial attempts, quarantine leftovers, re-attest isolation and obtain disposition before continuing; no automatic retry or replacement. See safety review and execution procedure.

## 10. Expected engineering evidence

All attempted startup/config/operator/authority manifests, raw logs, clock-domain mappings, I01–I06 event coverage, complete event/resource/network records and gaps, outcomes/anomalies/recorder exit status, exact reproduction commands, SHA-256 manifests and archive receipts. Compare the eight existing HD01–HD08 hypotheses with explicit missing/confounded channels. This evidence may guide corrective work; it does not itself qualify the system or close REC-01/BOUNDARY.

## 11. Unresolved uncertainties / recommendation

**Package prepared for technical review; not ready for execution approval as an exact runnable experiment.** Target identity/configuration, safety attestation, command manifest, P1 implementation/predicate, hook installation/overhead, independent recorder, clock maps, warm-cache provenance, resource/stop bounds and named operator remain unresolved. PD should defer execution pending those proofs or authorize only a separately defined preparation scope. G0 remains BLOCKED BY EXTERNAL INPUT with engineering blockers; M00-B01–B10 remain open, RA01 PENDING PD REVIEW, WP01 unauthorized.

## 12. Explicit decision record — to be completed only by PD

Decision (APPROVE exact scope / REJECT / DEFER): **PENDING**

PD name: ____________________

Signature: ____________________

Decision UTC / authorization reference / expiry: ____________________

Approved target workflow/host/container identity manifest: ____________________

Approved source/image/scene/instrumentation/command manifest hashes: ____________________

Approved operation IDs and maximum startups (proposed 24 inclusive of setup): ____________________

Approved safety attestation, operator/stop owner, evidence root and numerical operational limits: ____________________

Preconditions accepted with evidence references: ____________________

Rejection/defer reasons or required amendments: ____________________

All fields are blank/PENDING. No signature or approval is inferred from this document, Git commit, technical review or generator output. `M00E1_PENDING_DECISION.json` is a machine-readable pending record only. A later signed decision must be independently authenticated; E1 scripts cannot grant authority. Stop after E1 and submit to ChatGPT for technical review.
