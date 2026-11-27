# RRM-C001-F00-ENG-001 — Repository Control Amendment

**Repository:** https://github.com/castacks/AirStack
**Authorized branch:** `ore_proj`
**New engineering workspace:** `AirStack/RRM/`
**Protected reference workspace:** `AirStack/RRM-transfer/`
**Commit responsibility:** Codex
**Push responsibility:** RRM Program Director (user)
**Effective scope:** M00–M08 and subsequent approved F00 engineering work

## 1. Mandatory repository restrictions

Codex shall perform all engineering development against the existing `ore_proj` branch of the AirStack repository.

Codex shall not create another development branch, switch to another project branch for implementation, or push commits to GitHub.

Codex shall create a new top-level directory named `RRM` at the same directory level as the existing `RRM-transfer` directory.

All newly developed F00 engineering software, scripts, configurations, documentation, verification matrices, and reports shall be maintained under `RRM/`.

The following are protected:

- `RRM-transfer/` and everything beneath it.
- Existing AirStack source code outside `RRM/`.
- Existing AirStack configuration and launch files.
- Existing work belonging to other development efforts.
- Existing Git history and unrelated working-tree changes.

Codex may inspect these resources but must not modify, move, remove, rename, reformat, or overwrite them.

If integration requires changes outside `RRM/`, Codex shall document the proposed change and request Program Director approval before implementation.

## 2. Reference documentation

Before developing new AirStack integrations, Codex shall read:

`RRM-transfer/rrm-9d26a58/README.md`

Codex shall use this README to understand previously established AirStack configurations, integration fixes, and operational limitations.

Particular attention shall be given to:

1. Isaac Sim and robot startup order.
2. Existing NumPy and Isaac Sim compatibility corrections.
3. Camera, LiDAR, and Foxglove sensor-visualization issues.
4. Foxglove WebSocket connectivity and OSMO tunnels.
5. Isaac Sim WebRTC livestream configuration and port forwarding.
6. OSMO workflow identification and lifecycle management.
7. Isaac Sim scene-loading configuration.
8. Existing safety and readiness checks.

Codex shall first determine whether documented fixes are already present in the current execution environment.

Do not blindly reapply fixes, restart running services, change pinned container images, or modify operational configurations.

Existing documentation is reference evidence, not proof that the current runtime is correctly configured.

## 3. Required new engineering workspace

Create the following structure under `AirStack/RRM/` as needed:

```text
RRM/
├── README.md
├── governance/
│   ├── ENGINEERING_DIRECTIVE.md
│   ├── BASELINE_REGISTER.md
│   └── DECISION_REGISTER.md
├── requirements/
├── verification/
│   ├── matrices/
│   ├── procedures/
│   └── reports/
├── simulation/
│   ├── sil/
│   ├── hil/
│   └── correlation/
├── integration/
├── scripts/
├── tests/
├── configurations/
├── evidence/
│   └── M00/
└── handoffs/
```

Directories may be expanded as engineering needs develop, provided all new tracked deliverables remain under `RRM/`.

Large raw sensor recordings, container images, temporary caches, and other unsuitable Git artifacts shall remain outside Git unless explicitly approved. Their locations and integrity hashes shall be recorded in the appropriate evidence manifests.

## 4. Controlled F00 requirements baseline

Codex shall inspect the authoritative package:

`~/Downloads/RRM-C001-F00-EXT/`

The package shall be treated as a read-only engineering input.

Codex shall reconcile its controlled requirements, acceptance thresholds, verification matrices, and qualification blockers with the new engineering campaign.

The repository `RRM/` directory is the development and evidence workspace. It does not replace or supersede the authoritative F00 requirements.

## 5. Git execution policy

Before making changes, Codex shall:

1. Verify that the working repository is `castacks/AirStack`.
2. Verify that the checked-out branch is `ore_proj`.
3. Record the current HEAD commit SHA.
4. Inspect `git status` and identify any pre-existing changes.
5. Preserve unrelated changes without staging, resetting, or cleaning them.
6. Confirm that `RRM-transfer/` is not being changed.

Codex shall commit only files belonging to the authorized `RRM/` workspace.

Use explicit staging, such as:

```bash
git add -- RRM/
git diff --cached --name-status
git diff --cached --check
```

Before committing, verify that every staged path begins with `RRM/`.

Codex shall not use indiscriminate staging commands such as `git add -A` or `git add .`.

Suggested commit format:

`RRM-F00 M00: establish baseline and requirements assessment`

For each completed milestone, Codex shall:

- Run applicable validation checks.
- Record failures and limitations.
- Stage only authorized RRM files.
- Create a local commit.
- Record the new commit SHA.
- Report the committed file list and validation results.
- State explicitly that the commit has not been pushed.

**Only the Program Director shall push commits to `origin/ore_proj`.**

Codex shall not execute `git push`, force-push, automated remote synchronization, or remote branch updates.

Codex shall not amend, reset, rebase, or otherwise rewrite existing history without explicit approval.

## 6. Initial milestone — M00

Codex shall begin by establishing the new RRM workspace and performing the controlled F00 package assessment.

Required M00 activities:

1. Inspect repository status and branch identity.
2. Read the existing RRM README and relevant AirStack documentation.
3. Create `RRM/` without disturbing existing files.
4. Inspect the controlled F00-EXT package.
5. Reconcile controlled requirements with the proposed KPI and IV&V structures.
6. Identify software dependencies and existing operational fixes.
7. Produce the M00 engineering reports and matrices.
8. Verify that no files outside `RRM/` were changed by the task.
9. Commit the completed M00 deliverables locally.
10. Return the commit SHA, verification results, and G0 readiness recommendation.

Do not proceed to WP01 implementation before the M00 technical review.

## 7. Completion criteria

M00 is complete only when:

- The correct branch and repository are confirmed.
- The new `RRM/` directory exists.
- The existing `RRM-transfer/` directory remains unchanged.
- The authoritative F00 package has been assessed or an access blocker documented.
- The requirements, KPI, and IV&V reconciliation outputs have been produced.
- All task-related tracked changes are confined to `RRM/`.
- Validation results and limitations are documented.
- Codex has created a local commit.
- No GitHub push has occurred.

**Begin M00 under these repository restrictions.**
