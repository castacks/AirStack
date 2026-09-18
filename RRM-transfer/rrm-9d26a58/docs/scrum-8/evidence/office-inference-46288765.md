# Office Cosmos inference and offline import — job 46288765

Verified at 2026-09-17 23:27:19 UTC against actual retrieved artifacts.
Branch `ore_proj`, base commit `7e4da153` plus canonical-ID prompt working changes.

PSC bundle: `/ocean/projects/eng260004p/oabolade/physical-ai/runs/rrm/office/46288765/`.
OSMO copy: `/root/AirStack/.rrm-artifacts/psc-office-46288765.FCRcPE/bundle/`.
Outputs: sibling `imported/decision.json` and `imported/proposal.json`.

The actual result is ACCEPTED. Goal: `near($self, blue_marker)`.
One action: `NAVIGATE_TO`, ID `NAVIGATE_TO`, target `blue_marker`, no dependencies.
Grounded entities include both markers; recovery budget 0. The adapter's reviewed
fixture maps the blue marker to `(3.2, 0, 1.5)` in `map`, tolerance 0.3 m.

Checks performed:

- SHA256SUMS: input.json, input.png, scene_manifest.json, result.json all pass.
- Both recorded inference-source hashes match the current corresponding files.
- Existing importer verifies input/media hashes, exact prompt, raw-response reparse,
  stored candidate equality and local scene binding, then compiles the actual plan.
- Fresh import returns READY and exactly matches both saved output JSON records.
- Result explicitly records `execution_dispatch=false`.

Source hashes:

```text
47867f0ae8b06ec219ba3affb43abd60944746489c8b9c65b4e994c2091b5552  rrm/cosmos_reason2.py
d4938be5efc8fdab2b538f06eb359eeeb8ed5ecb7b578fca163901f6f498aec5  scripts/rrm_cosmos_reason2.py
```

Model: pinned Cosmos-Reason2-8B revision
`a9fae2cf89dc64db96b12860417f0eb403013bb9`. Console reports H100 80GB,
Torch 2.9.0+cu128, torchvision 0.24.0+cu128, Transformers 4.57.3.
Checkpoint loading: 14m39s. Recorded inference wall time including loading:
900.1816659809556 s. CPU import verification used isolated Pydantic 2.13.5 in
`airstack-robot-desktop-1`.

This establishes ground-truth-assisted Office reasoning and offline adapter
compilation. It does not establish visual localization, collision-free travel,
live freshness, execution authority, physical stop, or flight. Orange-marker
avoidance is acknowledged in model prose; independent path verification remains
future work. Original PSC artifacts remain persistent; OSMO copies are ephemeral.
Full simulator connection and the visual command-entry interface remain deferred.
