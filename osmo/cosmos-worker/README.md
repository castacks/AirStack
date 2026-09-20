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
