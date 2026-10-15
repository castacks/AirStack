# RRM Cosmos worker image

## Editable development worker

The development variant uses [Dockerfile.dev](Dockerfile.dev) and the separate
[development workflow](../workflows/airstack-live-replan-dev.yaml). The existing
immutable workflow is unchanged. Preparation/unit testing is not a published-image
or successful OSMO launch claim: build/publish and an authenticated Mac submission
are still required. Do not shut down the current workspace before backing it up.

The supervisor owns task lifetime; the model server is a replaceable child:

```text
workspace dev client -- private authenticated :8091 --> supervisor
                                                        └─ Python model server :8090
```

Uploads create content-addressed source releases without activating them. Explicit
activation stops the current child, then starts the selected release; startup failure
leaves the supervisor alive and visibly degraded. Prior releases remain available
for manual rollback. No automatic file watching, retry loop or dual GPU model load.
Restarting reloads weights and rehashes checkpoint files, so expect startup latency.
Changes to supervisor, dependencies, CUDA or the base image still require a new image
and workflow. Source releases, logs and model cache remain ephemeral at teardown.

### Security and operational boundaries

This is trusted-development tooling, not production isolation. The development token
grants arbitrary Python execution as the worker user through uploaded source. Only
trusted developers should have it. HTTP is private-network plaintext, not TLS; do not
use on an untrusted/shared-sniffable network without adding TLS or an SSH tunnel.
Never publicly expose or OSMO-port-forward ports8090/8091. The client checks/pins
RFC1918/ULA/loopback destinations, bypasses proxies and refuses redirects; those checks
do not authenticate the server or encrypt traffic. A malicious worker running as the
same user is not isolated from the supervisor/token. Rotate the credential when
access changes; a running supervisor must be relaunched to adopt a rotated token.

Stop all inference/GUI campaign clients before activation; do not restart during
robot operation. `--quiesced` records the caller's assertion, not an enforced global
drain or a permission grant. In-flight requests can be interrupted, including a
forced kill after a30-second termination timeout. No robot control was added.

### Mac handoff: before stopping the old workflow

1. Ensure all intended source changes have actually been committed/pushed; a new
   workspace clones `ore_proj`, not this workspace's uncommitted filesystem.
2. Download ignored evidence separately. With the existing `airstack-osmo` SSH alias
   and its current tunnel still running, from a Mac directory where you want backups:

   ```bash
   mkdir -p airstack-backup/rrm-artifacts airstack-backup/notebook
   rsync -a airstack-osmo:/root/AirStack/.rrm-artifacts/ airstack-backup/rrm-artifacts/
   rsync -a airstack-osmo:/root/AirStack/notebook/ airstack-backup/notebook/
   ```

   Preserve any remaining uncommitted/untracked files too. Verify the backup before
   canceling anything; git push does not include these ignored artifacts.
3. Build and publish the dev image from an up-to-date checkout on an authenticated
   Docker build host (Mac Docker Desktop/buildx can target linux/amd64):

   ```bash
   docker login airlab-docker.andrew.cmu.edu
   docker buildx build --platform linux/amd64 \
     -f osmo/cosmos-worker/Dockerfile.dev \
     -t airlab-docker.andrew.cmu.edu/airstack/airstack-rrm-cosmos-worker:<unique-dev-tag> \
     --push .
   ```

   Replace `<unique-dev-tag>`. Record the resulting **registry digest**, not the
   local image ID. Dockerfile-specific ignore rules restrict context to Python
   sources and control packaging; no weights, credentials or notebook artifacts.
   The runtime base is digest-pinned. Import/compile checks run during the build.
4. Create a strong random development token outside chat and register it privately
   in your authenticated OSMO profile; never commit it:

   ```bash
   read -rs 'RRM_DEV_TOKEN?Paste a newly generated random token (at least32 characters): '
   echo
   osmo credential set rrm-worker-dev --type GENERIC --payload token="$RRM_DEV_TOKEN"
   unset RRM_DEV_TOKEN
   ```

   This prompt uses Mac's default zsh. Use a password manager to generate/save the
   token. Run with shell tracing disabled. OSMO receives the token through its CLI
   arguments; use a private trusted Mac session. Existing Nucleus, registry and
   `rrm-huggingface-read` credentials are still needed.
5. Submit the separate dev workflow, substituting the published dev-image digest:

   ```bash
   osmo workflow submit osmo/workflows/airstack-live-replan-dev.yaml \
     --pool airstack \
     --set-string 'worker_dev_image=airlab-docker.andrew.cmu.edu/airstack/airstack-rrm-cosmos-worker@sha256:<published-dev-digest>' \
     --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)"
   ```

   Use your actual public-key filename. The image placeholder intentionally has no
   working default. This does not automatically retire the old workflow. If GPU
   quota prevents overlap, back up first, cancel the old workflow, then submit the
   new one. Otherwise verify the new workflow before retiring the old one.

### Routine changes from the next workspace

The dev workflow writes `/run/rrm-worker-dev.token` with mode0600 in the **outer
workspace**, not the robot/Isaac nested containers. Run from that workspace shell;
the client uses only standard-library Python. Substitute the private control URL
shown in the submitted workflow (`http://<cosmos-worker-host>:8091`); do not assume
environment variables from the entrypoint automatically appear in an SSH shell.

```bash
python3 osmo/cosmos-worker/dev_client.py --url http://<cosmos-worker-host>:8091 status
python3 osmo/cosmos-worker/dev_client.py --url http://<cosmos-worker-host>:8091 \
  upload --source RRM-transfer/rrm-9d26a58
```

Save the returned `release_id`. After quiescing all clients:

```bash
python3 osmo/cosmos-worker/dev_client.py --url http://<cosmos-worker-host>:8091 \
  activate --release <release_id> --quiesced
python3 osmo/cosmos-worker/dev_client.py --url http://<cosmos-worker-host>:8091 \
  verify --worker-url http://<cosmos-worker-host>:8090
```

An activation response means a process was launched, not that the model is ready.
Retry **read-only** verification while weights load. `LOADED_SOURCE_MATCHED` requires
a responding model service, matching loaded source/release identity and stable child
state. Save verification JSON with each experiment. For manual rollback, activate
the previously recorded release ID after quiescing clients, then verify again.

Status reports child exit code and active/previous release. Model diagnostics remain
in `/var/lib/rrm/dev/worker.log`; inspect privately through OSMO exec on the Mac if
necessary. The control API does not expose logs or a general shell. Syntax-valid
code can still fail imports or model startup; no automatic rollback masks failure.

### Provenance and validation

The worker reports startup Python-source manifests, nonhidden checkpoint-artifact
byte hashes (including model/tokenizer/generation config files), package versions,
declared revision and explicit generation arguments. These are self-reported file
identities, not remote attestation. Files must not be edited in place after startup.
The response's semantic prompt SHA hashes the semantic user prompt only; it does not
hash the complete system/chat-template/media-expanded model input. Inherited effective
generation defaults are represented by hashed config files, not fully expanded.

CPU validation uses disposable fake children/models, not Cosmos GPU inference:

```bash
python -m unittest discover -s tests -p test_worker_dev.py -v
```

Run from the RRM root in its configured Docker runtime. OSMO CLI validation, real
image build/import, GPU loading and private-network connectivity must be qualified
on the actual build/control hosts before calling the deployment operational.

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

The branch-scoped `airstack-live-replan-ore-proj.yaml` originally pinned the published image
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
That earlier candidate's capabilities reported only an entrypoint-source hash and
declared routes, not dependency or checkpoint identity. The v4 worker additionally
reports its visual prompt revision and visual builder source hash; neither is a
checkpoint or model-accuracy attestation.

Editing the workflow does not change an existing worker. Submit the updated YAML
from an authenticated OSMO control host for a new workflow; do not kill the current
worker main process to attempt a reload. `ignoreNonleadStatus: false` can terminate
the whole workflow when that task exits. Preserve uncommitted workspace changes and
artifacts before retiring it. Updating only a Mac checkout's YAML selects this worker
image; updating only the remote checkout does not update the Mac launch file.

## Visual identity v4 source overlay — 2026-10-14

The RRM visual contract now admits image-observable `exists` and `kind` facts only.
The previously running worker predates that revision, so the console capability
check reports `BLOCKED`. The source-overlay image from commit `d10c4e9b` was built,
import-checked and published as tag `visual-v4-d10c4e9b` at registry digest
`sha256:8ffbf30a140d3565d8c95510aeb0f9762fe6566cfcae39cc1b3f0b9e146cbe52`.
The branch-scoped workflow selects this digest for its **next submission**. Existing
OSMO workers do not change when the YAML changes; submit a new workflow from an
authenticated OSMO control host after preserving current workspace artifacts. The
GUI then needs a matching active scene and fresh camera image before its read-only
identity check can run. Visual identity is not map localization.
