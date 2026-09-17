# Model selection gate — Cosmos Reason2 8B

## Decision

| Field | Record |
| --- | --- |
| Status | Approved by the project user for cache and shadow evaluation on 2026-09-17; not approved for control or dispatch. |
| Candidate | `nvidia/Cosmos-Reason2-8B` |
| Role | Learned multimodal reasoning candidate: C01 task request + selected visual/teacher evidence represented through C02 + C03 capabilities → schema-validated C04 reasoning result and C05 plan candidate. |
| Non-role | Flight control, trajectory generation, direct ROS/action invocation, safety/permission decision, or replacement of embodiment adapters. |
| First evaluation | Ground-truth labelled episode → model shadow result → deterministic schema/evidence/capability/safety gates → score against teacher and deterministic reference. |
| Compute evidence | PSC `GPU-shared` probe: one NVIDIA H100 80GB HBM3, 81,559 MiB VRAM, driver 610.57.04. NVIDIA documents 32 GB as the Reason2-8B minimum. |
| Persistent artifact root | `/ocean/projects/eng260004p/oabolade/physical-ai` on PSC Ocean; set it as `RRM_PERSIST_ROOT` in PSC commands. |
| Cache target | `$RRM_PERSIST_ROOT/checkpoints/cosmos/hf`; no cache in OSMO `/tmp`, Docker layers, or PSC `$HOME`. |

## Preconditions before download

1. The responsible user must sign in to Hugging Face and accept the gated
   `nvidia/Cosmos-Reason2-8B` terms.  The model card requires contact-information
   sharing and carries the NVIDIA Open Model License Agreement.
2. Authenticate on PSC using `hf auth login` with a **read-only** Hugging Face token.
   Do not put the token in a shell history, repository, log, model manifest or chat.
3. Run `scripts/psc_cosmos_reason2_8b.sh preflight` with
   `RRM_PERSIST_ROOT` set to the PSC Ocean root.  It must report sufficient persistent
   storage and the authenticated account without downloading weights.
4. Invoke `download` only from a PSC context permitted for external data transfer by
   PSC policy.  If PSC denies outbound Hugging Face access, download through an
   approved transfer route; do not fall back to storing the model on a laptop or in
   OSMO temporary space.

## Exact revision and evidence

The exact revision is intentionally not guessed from a floating `main` reference.
After the first successful download, the bootstrap writes a non-secret manifest and a
SHA-256 file list under `$RRM_PERSIST_ROOT/checkpoints/cosmos/manifests/`.  The
manifest revision becomes the only revision used by RRM shadow experiments until a
new gate record explicitly supersedes it.

## Download attempt record

On 2026-09-17, the PSC bootstrap first passed persistent-root and Hugging Face-account
authentication checks, then encountered Hugging Face's repository-approval gate.  The
responsible user accepted the terms and reran the unchanged bootstrap from an
allocated H100 session.  The cache acquisition then completed: 19 files, 15.3 GB
downloaded, 17.5 GB reconstructed; exact revision
`a9fae2cf89dc64db96b12860417f0eb403013bb9`.  The snapshot, non-secret JSON manifest
and SHA-256 file list all reside under the PSC persistent root.  This completes the
cache gate; it does not change the shadow-only/no-control boundary.

## Evaluation and safety boundary

The 8B model is a hypothesis about useful physical/world reasoning—not the RRM
architecture itself.  RRM's body-agnostic state/evidence semantics remain the shared
interface.  A model may be stochastic, uncertain or wrong; deterministic validators
must reject malformed output, insufficient evidence, unknown/contradictory facts,
unsupported capability and safety/permission failures.  The current integration is
read-only/shadow-only and may never send an AirStack task.
