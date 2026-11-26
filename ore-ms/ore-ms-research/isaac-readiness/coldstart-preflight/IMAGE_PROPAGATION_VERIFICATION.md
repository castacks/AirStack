# Image propagation verification

The qualified manifest remains `airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697`. Image provenance and the verified Linux amd64 pull are recorded in the [remediation manifest](../numpy-remediation/IMAGE_MANIFEST.md). Phase A did not pull or publish an image.

The launch-only section of the committed entrypoint was exercised with the intended task environment and the real cloned CLI in a temporary isolated fixture. Docker Compose v5.1.3 parsed actual repository YAML without reading service env files. A stub replaced daemon operations, image checks and `compose up`. The fixture used no credential values and made no registry, SSH, daemon or OSMO calls.

| Case | What the offline test established |
|---|---|
| Default launch | With no pin or flag, the default Isaac release reference survived and the launch arguments stayed `up -d`. |
| Pinned launch | The digest survived into the CLI and exact livestream Compose image. The launch arguments became `up --no-build -d`. |
| Failed pinned launch | A simulated nonzero Docker exit produced the entrypoint's failure warning. The CLI did not log a successful start or attempt a build. |
| Explicit no-build flag | One `--no-build` argument was passed. |

All four cases retained the robot `v0.20.8_robot-x86-64_dev` and GCS `v0.20.8_gcs` references, one robot replica and one Isaac GPU request. Compose can reuse the same digest locally or pull it, subject to registry access. A failed pull cannot trigger a local build on this path. The cold-pod pull, access control and runtime were not executed. [Docker Compose up reference](https://docs.docker.com/reference/cli/docker/compose/up/) · [Compose build specification](https://docs.docker.com/reference/compose-file/build/)

From the repository root, run `python3 ore-ms/ore-ms-research/isaac-readiness/coldstart-preflight/evidence/check_startup.py` with the Docker Compose CLI installed. [Offline check](evidence/check_startup.py) · [Recorded results](evidence/startup-cases.jsonl)
