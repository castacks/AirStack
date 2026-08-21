# `lite_default` — onboard-lite reference stack

The compute-lite topology, unsplit, as a self-contained stack folder
(RFC #379 §3). This is the stack equivalent of the legacy
`AUTONOMY_ROLE=onboard` dispatch: everything a small vehicle runs on its own
compute, with the heavy global layer left out entirely.

## What it launches

The single entry point `launch/stack.launch.xml` composes:

- **Interface, Sensors, Perception, Behavior** — the layer bringups, exactly
  as `onboard_autonomy_local.launch.xml` composes them (wrap form).
- **Local layer, flat** (module launch files with canonical defaults, copied
  from `full_default`): takeoff/land task server, fixed-trajectory task
  server, GPU DROAN planner, trajectory controller, PID controller.
- **Role-onboard extras**: the robot↔GCS DDS router (lean onboard allowlist)
  and the gossip coordination layer.

**Deliberately absent:** the global layer (vdb_mapping, random_walk) and the
logging layer — the `AUTONOMY_ROLE=onboard` subtractions.

## When to use it

- Compute-constrained vehicles (VOXL, Jetson lite) flying task-driven
  missions (takeoff, land, fixed trajectory, direct navigate goals) with no
  onboard global planning.
- As the **onboard half** reference when authoring a split stack — the
  [`lite_offload_global`](../lite_offload_global/README.md) split stack's
  `onboard.launch.xml` is this topology paired with an offboard global half
  and an explicit `bridge.yaml`.

## Equivalence claim

`airstack up --stack lite_default` produces a ROS graph identical to the
legacy `AUTONOMY_ROLE=onboard` dispatch. Verify with the wiring snapshot test:

```bash
airstack test -m wiring --stack lite_default --sim isaacsim --num-robots 1
```

## How to run

```bash
airstack up --stack lite_default --sim isaac --robots 1
airstack ready
```

The shared per-robot preamble (ROBOT_NAME namespace, `use_sim_time`,
`robot_state_publisher`, world→map static TF) still runs in
`autonomy_bringup/launch/robot.launch.xml`, which dispatches to this stack
when `AIRSTACK_STACK_DIR` is set.

## Known limits

- **No global planner**: `global_plan` has no publisher in this stack.
  DROAN's navigate task still works with direct goals; exploration-style
  missions need `full_default` or the `lite_offload_global` split.
- Partially flattened: the Local layer is composed module-by-module; the
  other layers' cross-module remaps still live inside their bringup launch
  files (grep those bringups, or read `wiring.md` once committed, for their
  actual topic wiring).
- `modules.repos` pins no external modules yet; every package is
  trunk-resident.
- `docker-compose.yaml` is a stub — per-stack image composition arrives with
  the first module pins; trunk compose profiles provide all services.

## wiring.md

This stack's observed wiring diagram is committed at [wiring.md](wiring.md).

Generated — the commit arrives with the first snapshot run of
`airstack test -m wiring --stack lite_default` (the harness writes
`observed_lite_default.md` under the run directory; validate it and copy it
here as `wiring.md`). Once committed, CI drift-checks the running graph
against it. Until a GPU/sim runner flies this stack, `wiring.md` is absent
and the layout contract skips it (bootstrap rule).
