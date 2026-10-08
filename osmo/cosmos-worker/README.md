# RRM Cosmos worker image

This image runs one warm, private `rrm_cosmos_worker.py` service. It is separate from
the Isaac Docker-in-Docker workspace because loading a VLM and running Isaac are
independent GPU workloads.

Build it from the **AirStack repository root** after the RRM changes are available in
the build context:

```bash
docker buildx build --platform linux/amd64 \
  -f osmo/cosmos-worker/Dockerfile \
  -t airlab-docker.andrew.cmu.edu/airstack/airstack-rrm-cosmos-worker:latest \
  --push .
```

The image intentionally excludes the gated Cosmos checkpoint. The live-replan workflow
downloads the approved `nvidia/Cosmos-Reason2-8B` revision
`a9fae2cf89dc64db96b12860417f0eb403013bb9` into the worker task's ephemeral storage
on every new workflow. It then loads the model once and keeps it warm for the life of
that worker task.

Before submitting, accept the model terms in Hugging Face and add a read-only token to
your own OSMO profile. Run this in a private terminal (the token is neither printed nor
stored in the repository):

```bash
read -rsp 'Hugging Face token: ' HF_TOKEN; echo
osmo credential set rrm-huggingface-read --type GENERIC \
  --payload hf_token="$HF_TOKEN"
unset HF_TOKEN
```

The workflow maps that credential only into the `cosmos-worker` task as `HF_TOKEN`,
uses it for `hf download`, then unsets it before starting the HTTP service. The model
and Hugging Face cache disappear when the workflow ends; no checkpoint is copied to
Git, the image registry, or the Isaac workspace.

Do not expose worker port 8090 through `osmo workflow port-forward`; only the workspace
task in the same OSMO group should call it.

## Entity-capable source overlay — 2026-10-08

The branch-scoped `airstack-live-replan-ore-proj.yaml` now pins the published image
`airlab-docker.andrew.cmu.edu/airstack/airstack-rrm-cosmos-worker@sha256:6a6f1ea7233003a2e44c05b32041832a61af8a5957ef37be84c104edd24f2e83`
(tag `rrm-entities-20261008-4db36377`). It overlays the working RRM source from
`ore_proj` commit `4db36377` plus the capability-endpoint changes onto immutable
runtime parent `sha256:ddd2fcbfa57a0b981beca5f66a294c7288c264f0082f3b8852e028553564828c`.
This is not a clean-commit build. No model weights were added.

Rebuild from the repository root with a new unique tag:

```bash
docker build --platform linux/amd64 \
  -f osmo/cosmos-worker/Dockerfile.source-overlay \
  -t airlab-docker.andrew.cmu.edu/airstack/airstack-rrm-cosmos-worker:<unique-tag> .
```

Seven boundary/service tests passed inside the published candidate, including actual
HTTP `/v1/capabilities` and `/v1/verify-entities` routing with a fake model. This does
**not** qualify GPU inference, model accuracy, scene registration or visual grounding.
Capabilities report only an entrypoint-source hash and declared routes, not dependency
or checkpoint identity.

Editing the workflow does not change an existing worker. Submit the updated YAML
from an authenticated OSMO control host for a new workflow; do not kill the current
worker main process to attempt a reload. `ignoreNonleadStatus: false` can terminate
the whole workflow when that task exits. Preserve uncommitted workspace changes and
artifacts before retiring it. Updating only a Mac checkout's YAML selects this worker
image; updating only the remote checkout does not update the Mac launch file.
