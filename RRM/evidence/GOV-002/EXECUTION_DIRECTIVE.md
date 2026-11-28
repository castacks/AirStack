# RRM-C001-F00-GOV-002
## Organizational Authority Discovery and Controlled Evidence-Request Release Readiness

**Project:** RRM-C001 Autonomous UAS
**Repository:** `https://github.com/castacks/AirStack`
**Branch:** `ore_proj`
**Starting commit:** `381db1c4dcdd1bb71d047176e681bd18c72e342d`
**Program Director:** ChatGPT — project-level engineering direction
**Execution engineer:** Codex
**Previous milestone:** GOV-001B
**Authorization:** Passive investigation and offline documentation only

**G0:** BLOCKED BY EXTERNAL INPUT, with engineering blockers
**RA01:** Pending
**WP01:** Unauthorized
**Architecture C:** Conditionally approved for offline design
**D2:** Material configuration change deferred
**D4:** Accountable organizational ownership unverified
**Tier B target:** Undesignated
**Tier B execution:** NOT AUTHORIZED

---

## 1. Mission

Identify a credible, evidence-supported route to the organizational authority responsible for the AirStack/OSMO execution environment and its infrastructure and safety controls.

The objective is to move from an unsent evidence request to a **release-ready external coordination package** without fabricating ownership or bypassing organizational authorization.

This milestone must produce one of two outcomes:

**Outcome A — Release-ready:** A specific recipient or authorized organizational channel is identified with sufficient supporting evidence to request a separate PD delivery authorization.

**Outcome B — External-input hold:** No sufficiently supported recipient or channel can be identified. Record the exact missing authority information and stop.

Do not create another broad architecture or platform-feasibility study.

## 2. Mandatory startup checks

Before work:

1. Verify branch `ore_proj`.
2. Verify HEAD equals `381db1c4dcdd1bb71d047176e681bd18c72e342d`.
3. Verify the working tree is clean.
4. Read `RRM/ROADMAP_MANDATE.md`.
5. Read `RRM/PROJECT_STATUS.md`.
6. Read `RRM/handoffs/GOV001B_HANDOFF.md`.
7. Review the GOV-001B organizational request, owner-record search, minimum ownership structure, evidence-intake protocol, and external hold register.
8. Review the GOV-001A owner discovery and escalation action sheet.
9. Review M00-E1E's owner authority matrix and decision dependencies.
10. Review relevant AirStack and OSMO documentation.
11. Consult `RRM-transfer/rrm-9d26a58/README.md` as read-only historical reference.

If branch, HEAD, or repository state differs from expectations, stop and report.

Do not reset, rebase, or switch branches.

## 3. GOV2-01 — Targeted authority discovery

Perform a bounded investigation of existing records that may identify the organizational authority for:

- AirStack repository governance.
- OSMO execution infrastructure.
- Isaac Sim environment management.
- PX4/MAVROS integration.
- ROS 2/DDS networking.
- Host and container security.
- Physical hardware isolation.
- Configuration approval.
- Operational safety authorization.

Prioritize sources that may contain explicit ownership or delegation statements:

- CODEOWNERS.
- Maintainer and governance documentation.
- Existing workflow manifests.
- Repository organization documentation.
- Existing project contacts and approval records.
- Previously retained integration documentation.
- Existing local organizational records.

Record each source and its evidentiary limitations.

Do not equate code authorship, commit history, or repository membership with authority to approve safety or infrastructure changes.

Do not conduct active network discovery or service interrogation.

## 4. GOV2-02 — Recipient and channel qualification

For each plausible organizational contact or channel, evaluate:

- Identity or organizational unit.
- Connection to the relevant infrastructure.
- Evidence of responsibility.
- Evidence of communication-channel authenticity.
- Whether authority to appoint owners is established.
- Whether authority to approve platform changes is established.
- Whether authority to review safety isolation is established.
- Whether the contact can route the request to the correct authority.
- Remaining verification steps.

Classify each candidate as:

**VERIFIED AUTHORITY:** Supported by an explicit, applicable organizational authority record.

**VERIFIED ROUTING CHANNEL:** An authentic channel capable of routing the request, without assuming decision authority.

**CANDIDATE — UNVERIFIED:** Plausible but insufficiently supported.

**UNSUITABLE:** Evidence indicates the contact or channel is not appropriate.

A verified routing channel may be suitable for a request to identify the responsible owner, but it does not automatically establish that the channel itself has approval authority.

Do not invent names, email addresses, organizational roles, or authority records.

## 5. GOV2-03 — Minimum-contact strategy

Develop the smallest defensible external coordination sequence.

Prefer:

1. Identify an authentic organizational routing channel.
2. Request confirmation of the accountable infrastructure/safety authority.
3. Obtain explicit ownership or delegation evidence.
4. Confirm which authority may release existing platform/security records.
5. Request those records through an approved channel.
6. Review the records using the GOV-001B evidence-intake protocol.

Avoid sending detailed infrastructure-security requests to unverified individuals.

Do not request passwords, private keys, credentials, or unnecessary sensitive network information.

Do not contact anyone during this milestone.

## 6. GOV2-04 — External request release assessment

Review the prepared GOV-001B organizational request.

Determine whether it should be:

- Released unchanged after authorization.
- Narrowed to an initial ownership-confirmation request.
- Divided into separate infrastructure, configuration, and safety requests.
- Held pending recipient verification.

Recommend the appropriate sequence.

For each proposed communication, record:

- Intended recipient or channel.
- Evidence supporting its selection.
- Purpose.
- Information requested.
- Sensitivity classification.
- Delivery mechanism.
- Required authorization.
- Expected response.
- Follow-up owner.
- Stop or escalation condition.

Do not mark any request as sent.

## 7. GOV2-05 — PD release decision sheet

Prepare a concise PD decision sheet with the following alternatives:

**R1 — AUTHORIZE OWNERSHIP-CONFIRMATION REQUEST**

Applicable only when an authentic recipient or routing channel has been established.

**R2 — AUTHORIZE PLATFORM-EVIDENCE REQUEST**

Requires verified recipient authority, appropriate information-handling controls, and separately approved scope.

**R3 — DEFER REQUEST DELIVERY**

Use when recipient authority or channel authenticity remains insufficient.

**R4 — ESCALATE ORGANIZATIONAL AUTHORITY DISCOVERY**

Use when the available records cannot establish an appropriate route.

For each alternative, specify:

- Preconditions.
- Evidence.
- Scope.
- Risks.
- Limitations.
- Recommended disposition.

Leave the PD approval field unselected.

**This directive does not itself authorize R1 or R2.**

## 8. GOV2-06 — External dependency and G0 continuity

Preserve the original 20 technical prerequisite identifiers and their current dispositions:

- 3 satisfied.
- 8 unsatisfied.
- 9 unverified.

No prerequisite may be closed merely because a contact or communication channel has been identified.

Maintain:

- D1 — Architecture C conditionally approved for offline design.
- D2 — Deferred.
- D3 — Diagnostic claim-limit policy approved.
- D4 — Pending verified ownership.
- D5 — Passive/offline engineering only.
- G0 — Blocked.
- RA01 — Pending.
- WP01 — Unauthorized.
- Tier B target — Undesignated.
- Tier B execution — Not authorized.

Update `RRM/PROJECT_STATUS.md` with the outcome and next required external action.

## 9. Required deliverables

Create under `RRM/`:

1. `governance/GOV-002/GOV002_AUTHORITY_DISCOVERY.md`
2. `governance/GOV-002/GOV002_AUTHORITY_SOURCE_REGISTER.csv`
3. `governance/GOV-002/GOV002_RECIPIENT_CHANNEL_ASSESSMENT.csv`
4. `governance/GOV-002/GOV002_MINIMUM_CONTACT_STRATEGY.md`
5. `governance/GOV-002/GOV002_REQUEST_RELEASE_ASSESSMENT.md`
6. `governance/GOV-002/GOV002_PD_RELEASE_DECISION.md`
7. `governance/GOV-002/GOV002_EXTERNAL_HOLD_DISPOSITION.md`
8. `evidence/GOV-002/GOV002_EVIDENCE_INDEX.csv`
9. `evidence/GOV-002/GOV002_PREREQUISITE_CONTINUITY.csv`
10. `evidence/GOV-002/SHA256SUMS`
11. `handoffs/GOV002_HANDOFF.md`
12. Updated `PROJECT_STATUS.md`

Do not duplicate entire previous milestone reports.

## 10. Validation requirements

Validate that:

- Every claimed authority has traceable supporting evidence.
- Repository maintainership is not misrepresented as safety authority.
- Routing-channel authenticity is distinguished from approval authority.
- No recipient or email address is invented.
- No external request is falsely recorded as sent.
- All 20 prerequisites remain traceable.
- D1–D5 dispositions remain accurate.
- G0, RA01, WP01, and Tier B governance is preserved.
- Historical evidence is unchanged.
- Protected files remain unchanged.
- No prohibited active operations occurred.

Produce machine-readable validation results.

## 11. Git controls

Remain on `ore_proj`.

Modify only `RRM/`.

Do not modify `RRM-transfer/`, existing AirStack files outside `RRM/`, or historical evidence.

Stage only:

`git add -- RRM/`

Inspect the staged diff.

Commit locally with:

`RRM-F00 GOV-002: assess organizational authority and request release`

Report the full SHA, changed-file count, validation results, authority findings, and release recommendation.

**Do not push.**

## 12. Completion and stop conditions

GOV-002 is complete when:

1. A bounded authority discovery has been performed.
2. Candidate recipients and channels have been evaluated.
3. A minimum-contact strategy is documented.
4. The organizational request has a release recommendation.
5. The PD decision sheet is prepared.
6. G0 and prerequisite continuity is preserved.
7. Offline validation is complete.
8. Changes are committed locally.

The final handoff must answer:

- Was a verified organizational authority identified?
- Was an authentic routing channel identified?
- Is an ownership-confirmation request release-ready?
- Is a platform-evidence request release-ready?
- What exact authorization is needed next?
- Does the external-input hold remain in force?

**Stop after GOV-002.**

No external communications, owner appointments, rig construction, deployment, simulator startup, service restart, active network probing, hardware interaction, actuation, Tier B/C testing, or WP01 execution is authorized.

**Begin GOV-002 now.**
