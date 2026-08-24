# `full_default` — trunk reference stack

The current full-autonomy topology, as a self-contained stack folder (RFC #379 §3).
This is the stack most users start from and the baseline other stacks are copied from.

## What it launches

The entry point `launch/stack.launch.xml` composes the **Local layer as a
flat set of module-launch includes** (flattened in E2: takeoff/land task
server, fixed-trajectory task server, GPU DROAN planner, trajectory
controller, PID controller — each module launch file declares its topic
endpoints as args with canonical defaults, so bare includes mean canonical
wiring), and includes the remaining layer bringups — interface, sensors,
perception, global, behavior, logging — exactly as
`onboard_autonomy_all.launch.xml` does with default args, plus the two
role-`full` extras (DDS-router domain bridge to the GCS, gossip coordination
layer). Flattening the remaining layers arrives in E3.

## Equivalence claim

`airstack up --stack full_default` produces a ROS graph identical to the legacy
`AUTONOMY_ROLE=full` dispatch. Verify with the wiring snapshot test:

```bash
airstack test -m wiring --stack full_default --sim isaacsim --num-robots 1
```

## How to run

```bash
airstack up --stack full_default --sim isaac --robots 1
airstack ready
```

The shared per-robot preamble (ROBOT_NAME namespace, `use_sim_time`,
`robot_state_publisher`, world→map static TF) still runs in
`autonomy_bringup/launch/robot.launch.xml`, which dispatches to this stack when
`AIRSTACK_STACK_DIR` is set.

## Known limits

- Partially flattened: the Local layer is composed module-by-module in
  `stack.launch.xml`; the other layers' cross-module remaps still live inside
  their bringup launch files until E3. Grep those bringups (or read
  `wiring.md`) for their actual topic wiring.
- `modules.repos` pins no external modules yet; every package is trunk-resident.
- `docker-compose.yaml` is a stub — per-stack image composition arrives with
  the first module pins; trunk compose profiles provide all services.

## wiring.md

Generated — the commit arrives with the first snapshot run of
`airstack test -m wiring --stack full_default` (the harness writes
`observed_full_default.md` under the run directory; validate it and copy it
here as `wiring.md`). Once committed, CI drift-checks the running graph
against it.
