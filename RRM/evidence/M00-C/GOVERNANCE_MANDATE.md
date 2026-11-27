# RRM-C001-GOV-001 — Engineering Roadmap and Progress Control Mandate

**Applies to:** F00 and all subsequently authorized RRM development baselines
**Repository:** `castacks/AirStack`
**Branch:** `ore_proj`
**Workspace:** `RRM/`
**Effective milestone:** M00-C

## 1. Governing requirement

Codex shall establish and maintain two permanent project-control documents:

- `RRM/ROADMAP_MANDATE.md`
- `RRM/PROJECT_STATUS.md`

These documents shall govern engineering execution, milestone tracking, verification progress, and handoffs between Codex, ChatGPT, and the RRM Program Director.

The roadmap is a planning and execution-control document. It shall not supersede controlled requirements, approved acceptance criteria, or formal qualification records.

## 2. Roadmap mandate

`ROADMAP_MANDATE.md` shall define:

1. Project objectives and engineering principles.
2. Current controlled baseline and source authority.
3. F00 work packages WP00–WP08.
4. Milestones and verification gates G0–G8.
5. Entry and exit criteria for each milestone.
6. Dependencies between milestones.
7. Required technical evidence and deliverables.
8. Codex and ChatGPT engineering responsibilities.
9. Program Director approval authority.
10. Repository protection and commit/push rules.
11. Blocker escalation and corrective-action procedures.
12. Future development phases beyond F00.

Future phases shall be explicitly marked **PROVISIONAL** until their scope, requirements, and acceptance criteria are approved.

## 3. Live project status

`PROJECT_STATUS.md` shall contain a concise, continuously updated status register.

At minimum, record:

| Field | Required information |
|---|---|
| Current baseline | F00 or subsequently approved baseline |
| Current milestone | Active task identifier |
| Current gate | G0–G8 and disposition |
| Last completed milestone | Identifier and result |
| Last local commit | Full SHA |
| Remote synchronization | Pushed / Not pushed / Unknown |
| Requirements coverage | Verified, failed, blocked, not tested |
| Open blockers | IDs, severity, and closure conditions |
| Current engineering activity | Authorized work |
| Next milestone | Proposed next task and dependencies |
| Required approvals | Outstanding Program Director decisions |
| Last update | Date, author, and evidence reference |

A milestone shall not be considered administratively complete until the project status is updated.

## 4. Mandatory milestone workflow

For every subsequent milestone, Codex shall:

1. Read `ROADMAP_MANDATE.md` and `PROJECT_STATUS.md` before beginning work.
2. Verify the active branch, repository status, and current authorization.
3. Identify the milestone's relationship to the approved roadmap.
4. Execute only the authorized scope.
5. Produce traceable technical evidence.
6. Update milestone and blocker dispositions.
7. Update `PROJECT_STATUS.md`.
8. Check that the roadmap and status agree.
9. Run applicable validation and integrity checks.
10. Commit the milestone and status updates together.
11. Report the local commit SHA to ChatGPT and the Program Director.
12. Stop at the required review or approval gate.

ChatGPT shall assess whether the milestone results support the reported progression and whether the proposed next task is consistent with the roadmap.

## 5. Roadmap change control

Codex may propose roadmap amendments but shall not silently change approved objectives, phase sequencing, acceptance criteria, or authorization boundaries.

Every material roadmap change shall include:

- Proposed change.
- Technical justification.
- Affected requirements and milestones.
- Dependency and schedule implications.
- Risks.
- Approval status.

Material changes require Program Director approval.

Routine progress updates to `PROJECT_STATUS.md` do not require separate approval, provided they accurately reflect the evidence.

## 6. Initial project state

Initialize the roadmap and status using the actual committed M00 and M00-B records.

The reported starting state is:

- **Baseline:** F00.
- **M00:** Completed; G0 HOLD.
- **M00-B:** Completed; G0 BLOCKED BY EXTERNAL INPUT, with engineering blockers.
- **Current authorized milestone:** M00-C.
- **Current primary investigation:** PX4 `STARTUP_DEADLINE` and readiness-state failure.
- **WP01:** Not authorized.
- **Remaining dependencies:** Missing frozen records, configuration equivalence, qualification findings, release assets, technical decisions, and technical review.

Verify this starting state against the repository evidence before publishing the initial status.

## 7. Repository protection

All roadmap and status files shall be created under `RRM/`.

`RRM-transfer/` and all other protected AirStack work shall remain unchanged.

Codex shall commit locally on `ore_proj` but shall not push. Only the Program Director shall push to GitHub.

## 8. M00-C integration

As part of M00-C, Codex shall:

1. Create the two governing documents.
2. Populate the roadmap from the approved F00 campaign.
3. Reconcile the milestone history against M00 and M00-B evidence.
4. Record the current G0 blockers.
5. Identify M00-C as the active milestone.
6. Implement a lightweight consistency check for the roadmap and status, if practical.
7. Include both documents in the M00-C local commit.
8. Report any inconsistency requiring Program Director review.

**This mandate remains active for every future RRM milestone unless formally superseded.**
