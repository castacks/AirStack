# `full_droan_cpu` — trunk reference stack

Full autonomy with the **CPU DROAN local planner** (`droan_local_planner` +
a live `disparity_expansion` world model) instead of the GPU `droan_gl` node.
One of the presets absorbing the `local_*.launch.xml` variant explosion into
named stacks a few include lines apart (RFC #379 §3).

## What it launches

Local layer flattened (E2): identical to
[`full_default`](../full_default/README.md) except the DROAN include lines —
`droan_local_planner.launch.xml` plus `disparity_expansion.launch.xml`
instead of `droan_gl.launch.xml`. Everything else — interface, sensors,
perception, the other local modules, global, behavior, logging, DDS router,
gossip — matches `full_default` exactly. (This stack absorbed the deleted
`local_bringup/launch/local_droan_cpu.launch.xml` variant.)

## Equivalence claim

`airstack up --stack full_droan_cpu` produces a ROS graph identical to the
legacy `AUTONOMY_ROLE=full` dispatch with
`local_launch_file:=local_droan_cpu.launch.xml` passed to
`onboard_autonomy_all.launch.xml` (that variant file was deleted in P5-E2;
this folder's committed `wiring.md`, captured from exactly that legacy
configuration, remains the observed baseline the flat form must match).

## How to run

```bash
airstack up --stack full_droan_cpu --sim isaac --robots 1
airstack ready
```

## Known limits

- Partially flattened: the Local layer is composed module-by-module in
  `stack.launch.xml`; the other layers are still wrapped bringup includes
  (they flatten in E3).
- CPU DROAN is the slower planner path — use `full_default` (GPU `droan_gl`)
  unless you need to run without the GPU planner or are debugging
  `disparity_expansion`.
- `modules.repos` pins no external modules yet; `docker-compose.yaml` is a
  stub (trunk compose profiles provide all services).

## wiring.md

Generated — the commit arrives with the first snapshot run of
`airstack test -m wiring --stack full_droan_cpu`. Once committed, CI
drift-checks the running graph against it.
