# Workflow provenance

On 26 November 2026 UTC, the workspace-local OSMO login metadata identified workflow `airstack-dev-314` and task `workspace` in its `refresh_url` field. Only the identifier matches were retained. The workspace ran `osmo_exec` as PID 1. The AirStack reflog records a fresh clone at 18:24:20 UTC, and the Docker daemon initialized earlier in the same workspace. The earlier remediation evidence described running containers in a separate session.

- The outer workspace image reference and immutable identity were unavailable from inside the task. The supplied Phase A reference used `airstack-osmo-workspace:latest`. The baked `/usr/local/bin/entrypoint.sh` SHA-256 is `d75e199301afc31bc9c5bdb384a54efe12256cf53be200a678ab28a4a3772bc9`. That is a file hash.
- The checkout was clean on `ore-ms-orb-development` at full Phase A SHA `19d1cb0d2bb4633abc1e102b6ad6b248236294b7`, matching the live origin branch before evidence delivery.
- Compose resolved `isaac-sim-livestream` to `airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697`. The created Isaac container's configured image and the local image's RepoDigests matched this reference. Its Docker image ID was `sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd`.
- Robot and GCS resolved to their Phase A references `v0.20.8_robot-x86-64_dev` and `v0.20.8_gcs` in the same registry repository. GCS was created with that tag and image ID `sha256:9e44be1c0c38e297ba5b9fc463f472ca5c224091969f50e417825a412e61aa7e`. The robot pull failed before container creation.
- The cloned CLI contains the digest-triggered `--no-build` guard, the PID 1 launch environment contained the digest, and daemon history showed pulls and container creation. The actual Compose invocation was unavailable after startup, so the no-build gate stays conditional.

Commands and selected observations are preserved in [sanitized diagnostics](evidence/diagnostics.txt).
