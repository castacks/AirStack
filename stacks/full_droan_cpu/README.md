# `full_droan_cpu` — trunk reference stack

Full autonomy with the **CPU DROAN local planner** (`droan_local_planner` +
a live `disparity_expansion` world model) instead of the GPU `droan_gl` node.
One of the presets absorbing the `local_*.launch.xml` variant explosion into
named stacks a few include lines apart (RFC #379 §3).

## What it launches

Wrap form (E1): identical to [`full_default`](../full_default/README.md)
except the Local layer include is
`local_bringup/launch/local_droan_cpu.launch.xml`. Everything else — interface,
sensors, perception, global, behavior, logging, DDS router, gossip — matches
`full_default` exactly.

## Equivalence claim

`airstack up --stack full_droan_cpu` produces a ROS graph identical to the
legacy `AUTONOMY_ROLE=full` dispatch with
`local_launch_file:=local_droan_cpu.launch.xml` passed to
`onboard_autonomy_all.launch.xml`.

## How to run

```bash
airstack up --stack full_droan_cpu --sim isaac --robots 1
airstack ready
```

## Known limits

- Wrap form: cross-module remaps still live inside the layer bringup launch
  files, not in `stack.launch.xml`.
- CPU DROAN is the slower planner path — use `full_default` (GPU `droan_gl`)
  unless you need to run without the GPU planner or are debugging
  `disparity_expansion`.
- `modules.repos` pins no external modules yet; `docker-compose.yaml` is a
  stub (trunk compose profiles provide all services).

## wiring.md

Generated — the commit arrives with the first snapshot run of
`airstack test -m wiring --stack full_droan_cpu`. Once committed, CI
drift-checks the running graph against it.
