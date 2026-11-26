# Workflow YAML review

The Mac-local file is inaccessible to this workspace. The Project Director's supplied configuration and the previously supplied workflow text are reference material. They are not a readback of the current Mac file.

| Setting | Tracked template | Intended Mac workflow |
|---|---|---|
| Workspace task | One lead privileged task of group `airstack` | Same task shape |
| Outer image | `airstack-osmo-workspace:latest` | Same tag in the supplied workflow text |
| Command | Direct baked entrypoint | rclone initialization followed by the baked entrypoint |
| Git ref | `main` | `ore-ms-orb-development` |
| Isaac image | Compose default | Qualified digest under the task's `environment:` |
| Resources | 16 CPU, one GPU, 64 GiB memory, 500 GiB storage | 10 CPU, one GPU, 48 GiB memory, 500 GiB storage |
| Credential references | `airlab-nucleus`, `airlab-docker-login`, registry credential through the profile | Same, plus `ore-ms-box` for the supplied rclone setup |

The location of the digest is correct in the supplied design. The startup guard makes a separate no-build line optional. Verify the task-level mapping and prerequisite checks in the [deployment handoff](USER_DEPLOYMENT_HANDOFF.md). Preserve local rclone commands and credential references. No YAML file was edited. OSMO schema acceptance, quota and access on the Mac remain unverified.

Forwarding assumes SSH TCP 22, Isaac signaling TCP 49100 and media UDP 49099, and pod Foxglove TCP 8766. The port mappings come from the repository and prior working-pod evidence. A new workflow's network and client behavior remain to be checked.
