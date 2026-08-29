# `full_droan_cpu` — trunk reference stack

Full autonomy with the **CPU DROAN local planner** (`droan_local_planner` +
a live `disparity_expansion` world model) instead of the GPU `droan_gl` node.
Local-planner variants are expressed as named stacks a few include lines
apart — rather than as launch-file arguments — so each variant is directly
selectable and carries its own observed wiring baseline.

## What it launches

Identical to [`full_default`](../full_default/README.md) except the DROAN
include lines — `droan_local_planner.launch.xml` plus
`disparity_expansion.launch.xml` instead of `droan_gl.launch.xml`. Everything
else — interface, sensors, perception, the other local modules, global,
behavior, logging, DDS router, gossip — matches `full_default` exactly.

## How to run

```bash
airstack up --stack full_droan_cpu --sim isaac --robots 1
airstack ready
```

## Known limits

- Every layer is composed module-by-module in `stack.launch.xml`, except
  `interface.launch.py` (wrapped by design — the safety boundary) and
  `logging.launch.xml` (already a single self-contained module).
- CPU DROAN is the slower planner path — use `full_default` (GPU `droan_gl`)
  unless you need to run without the GPU planner or are debugging
  `disparity_expansion`.
- `modules.repos` pins no external modules yet; `docker-compose.yaml` is a
  stub (trunk compose profiles provide all services).

## wiring.md

This stack's observed wiring diagram is committed at [wiring.md](wiring.md);
CI drift-checks the running graph against it. Regenerate via
`airstack test -m wiring --stack full_droan_cpu`.
