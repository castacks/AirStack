# Model and artifact persistence

RRM's semantic contracts and traces must survive a compute session, but an OSMO
workspace is not itself a durable artifact store.  The checked-in AirStack workflow
requests `storage: 500Gi`; that is task-local ephemeral storage.  It has no verified
PVC/volume mount.  The AirStack OSMO guide explicitly lists a persistent workspace as
a pool-policy follow-up, so it must not be assumed to exist.

This record is deliberately independent of robot body, simulator and model vendor.
It supports the body-agnostic RRM claim: a reasoner is evaluated from durable inputs
and evidence at the C01--C09 boundaries, while an embodiment adapter supplies its
own observations and action translation.

## Current model status

`nvidia/Cosmos-Reason2-*` is the remembered candidate family.  The prior report of a
roughly 33 GB temporary download is consistent with an 8B-class, higher-precision
checkpoint, but is **not** enough to identify an exact checkpoint, revision, dtype,
or license acceptance.  A search of the current workspace's `/tmp` and Hugging Face
cache found no recoverable Cosmos/Reason2 files.  Do not infer that Cosmos-Reason2-2B
is the prior model from the older prototype documentation, and do not treat a model
family as a selected model.

PSC feasibility validation on 2026-09-17 allocated one GPU-shared H100 session and
observed `NVIDIA H100 80GB HBM3`, 81,559 MiB VRAM, driver `610.57.04`.  Cosmos
Reason2-8B is therefore hardware-eligible on this allocation (NVIDIA documents a
32 GB minimum); it was not selected until its exact revision, terms and manifest were
recorded.  This feasibility test did not download or execute a learned model.

The selection and cache gate subsequently completed on the same date.  The approved
shadow-only candidate is `nvidia/Cosmos-Reason2-8B`, revision
`a9fae2cf89dc64db96b12860417f0eb403013bb9`.  Its 19-file cache was downloaded directly
to PSC Ocean (15.3 GB transfer, 17.5 GB reconstructed), with a JSON provenance record
and SHA-256 file list in `checkpoints/cosmos/manifests/`.  This replaces the old
repeated temporary download workflow; the model remains excluded from Git and any
control/dispatch path.

Before a model is downloaded or integrated, create a non-secret manifest containing:

| Field | Required record |
| --- | --- |
| Identity | Hub/repository ID, immutable revision/commit, files and SHA-256 digests |
| Terms | License name/version and proof that the responsible user accepted any gated terms |
| Runtime | dtype/quantization, tokenizer/processor version, GPU model/VRAM, driver, container image and measured memory/latency |
| RRM role | Inputs from the RRM world/task contract, structured output schema, no-control shadow status, and failure handling |
| Evaluation | Selected scene tier, teacher/ground-truth source, metric set, baseline and artifact run ID |
| Location | Durable root, relative cache path, owner/retention policy; never credentials or access tokens |

The first learned increment should remain **shadow-only**: it consumes the same C01,
C02 and C03 inputs as the deterministic reference and produces C04/C05 candidates.
Schema validation, evidence freshness, capability checks and safety/permission gates
remain deterministic.  No model output is a command, and no embodiment-specific
control policy is required to test RRM reasoning.

## Artifact classes and layout

Once AirLab supplies a verified persistent mount or storage endpoint, expose it in a
workspace as a single configured `RRM_PERSIST_ROOT`.  The mount location itself is an
infrastructure decision; `/tmp`, the OSMO workspace overlay, and the Git checkout are
not valid values.  Use a layout such as:

```text
$RRM_PERSIST_ROOT/
  models/hf/                    # content-addressed Hugging Face cache; immutable after verification
  models/manifests/              # small, Git-compatible manifests and checksums
  datasets/<dataset-id>/         # labelled frames/video or scene-derived evaluation input
  runs/<run-id>/
    inputs/                      # task and world-snapshot references / sampled frames
    traces/                      # C01--C09 JSONL and metrics
    reports/                     # compact evaluation summary
    complete.json                # checksums, versions, time range, producer and completion marker
  exports/                       # explicitly selected bundles for collaborator transfer
```

| Artifact | Source of truth | Persistence rule |
| --- | --- | --- |
| RRM source, schemas, test fixtures and docs | Git remote | Commit and push; never put weights or raw recordings in Git. |
| Model weights/tokenizers | Provider download into `models/hf`, addressed by manifest revision/hash | Download once only after the gate passes; mount or point `HF_HOME`/the model cache to this root.  Treat verified files as immutable. |
| USD scenes/assets | The existing Nucleus scene service, when the scene is referenced there | Keep a versioned scene URL/identifier in the run manifest; do not duplicate large shared stages into RRM storage unless a controlled offline package is needed. |
| Raw frames, videos and simulator captures | Durable dataset/run storage | Retain only data needed for replay, learning or audit; use named dataset versions and checksums. |
| RRM evidence, metrics and sampled visual inputs | `runs/<run-id>` | Persist append-only events plus a compact report; write `complete.json` only after checksums pass. |
| Local builds, Docker layers and temporary downloads | OSMO ephemeral workspace | Rebuild/recreate; never present these as evidence or durable model storage. |

## Transfer paths

### Preferred: AirLab-provided persistent mount or approved object store

Ask the AirLab/OSMO owner for the supported mechanism, path/endpoint, quota, access
control, backup/retention policy and whether it can be mounted into interactive OSMO
workspaces.  Configure that provider outside source control and inject only the
non-secret mount path as `RRM_PERSIST_ROOT`.

Then:

1. Validate write/read/rename and available quota with a small test file.
2. Download the exact manifest-pinned model once into `models/hf` on that storage;
   verify all file digests before marking the manifest usable.
3. Mount the same root read-only for inference when practical.  A model process never
   writes into the verified cache; fine-tuning/checkpoint outputs use a new run path.
4. At run end, create an export bundle from compact traces/reports and record hashes
   in `complete.json`.  Promote raw captures only when their evaluation role requires
   them.

Nucleus is already known here as the authenticated source for Omniverse/USD assets.
That does **not** establish it as an approved generic model or dataset store.  Do not
upload weights to Nucleus until the AirLab owner confirms the policy, quota and an
appropriate project location.

### Temporary fallback: export before the OSMO task ends

AirStack documents `osmo workflow rsync download <workflow> <pod-path>:<local-path>`
from the workstation before teardown for bags and recordings.  Use this only for
selected evidence or a verified backup when no durable mount exists.  It transfers
data from the ephemeral pod to the workstation; it does not make a persistent AirLab
store or solve a recurring model cache.  The workstation must have sufficient space,
and the downloaded directory must be checksum-verified against its run manifest.

Do not rely on a local `osmo` binary in the workspace: the current remote does not
have the CLI installed.  The command belongs on the operator's machine, where the
active workflow identity and OSMO credentials exist.

### Verified PSC route (2026-09-17)

The project has persistent PSC Ocean storage at
`/ocean/projects/eng260004p/oabolade/physical-ai` (200 GiB quota; about 196 GiB free
at validation).  It is the selected initial `RRM_PERSIST_ROOT` for model caches and
RRM evaluation data.  Its existing project layout uses `checkpoints/`, `datasets/`,
`runs/`, `containers/` and `src/`; RRM uses `checkpoints/cosmos/hf`, `datasets/rrm`
and `runs/rrm` beneath it.

A 28-byte smoke file was transferred successfully, without a workstation data copy,
from the OSMO workspace to
`runs/rrm/transfer-smoke/rrm-psc-smoke.txt` using password-authenticated rsync to
PSC's data-transfer service.  The transfer hostname resolved to two addresses during
testing: `192.231.243.27` reset during SSH key exchange, while `192.231.243.28`
accepted the connection and completed the transfer.  Treat that address pin as a
short-term diagnostic workaround, not a permanent PSC endpoint choice; use the PSC
data-transfer hostname by default and record any renewed connection failures.

Manually appending an OSMO-generated public key to `~/.ssh/authorized_keys` on a PSC
login node did **not** enable key authentication on the transfer node.  PSC requires
public keys to be registered through its SSH Key Management process and verified by
User Services.  Until that completes, prompt-based PSC password authentication is the
working direct-transfer mechanism.  Do not store that password, any private key, or a
Hugging Face token in this repository or in an artifact manifest.

## Operational gate for the next learned RRM increment

The next work item is not another manual checkpoint download.  It is a one-time
storage readiness check followed by a shadow reasoner deployment:

1. Confirm the exact Cosmos checkpoint (or select another candidate) by its manifest.
2. Confirm durable AirLab storage and its transfer/mount method.
3. Measure the currently allocated GPU's usable VRAM in the *actual inference
   container*, rather than relying on a historical size estimate.
4. Cache once, verify hashes, and record the model manifest.
5. Run the model against ground-truth labelled episodes in shadow mode, emitting only
   C04/C05 candidates that are scored against the teacher and deterministic baseline.
6. Persist the compact evidence bundle.  Only after that evaluation can perception/VLM
   inputs replace selected teacher facts; control remains a separately gated adapter
   concern.

This is how we move into full RRM development without confusing repeated bootstrap
work, VLM deployment, or flight control with the RRM contribution.
