# `lite_default` — onboard-lite reference stack

The compute-lite topology, unsplit, as a self-contained stack folder
(RFC #379 §3). This is the stack equivalent of the removed legacy
`AUTONOMY_ROLE=onboard` role: everything a small vehicle runs on its own
compute, with the heavy global layer left out entirely. (Historical note:
the desktop-profile `onboard` role was unreachable in practice —
`robot-desktop` hardcoded `AUTONOMY_ROLE=full` — so this stack is the first
reachable lite topology on desktop.)

## What it launches

The single entry point `launch/stack.launch.xml` composes:

- **Interface** (wrapped by design) plus **Sensors, Perception, Local,
  Behavior — all flat**: module launch files with canonical defaults, the
  same blocks as `full_default` (LiDAR filter, stereo_image_proc + topic
  keepalive, takeoff/land task server, fixed-trajectory task server, GPU
  DROAN planner, trajectory controller, PID controller, safety monitor).
- **Cross-domain extras**: the robot↔GCS DDS router (the shared
  `autonomy_bringup/config/dds_router.yaml` allowlist) and the gossip
  coordination layer.

**Deliberately absent:** the global layer (vdb_mapping, random_walk) and the
logging layer.

## When to use it

- Compute-constrained vehicles (VOXL, Jetson lite) flying task-driven
  missions (takeoff, land, fixed trajectory, direct navigate goals) with no
  onboard global planning.
- As the **onboard half** reference when authoring a split stack — the
  [`lite_offload_global`](../lite_offload_global/README.md) split stack's
  `onboard.launch.xml` is this topology paired with an offboard global half
  and an explicit `bridge.yaml`.

## Equivalence

`airstack up --stack lite_default` produces the topology the removed legacy
`AUTONOMY_ROLE=onboard` dispatch launched. Verify with the wiring snapshot
test:

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
- All layers are flattened in `launch/stack.launch.xml` (same layout as
  `full_default`); the interface stays a wrapped include by design, so read
  [wiring.md](wiring.md) for the observed MAVROS wiring.
- `modules.repos` pins no external modules yet; every package is
  trunk-resident.
- `docker-compose.yaml` is a stub — per-stack image composition arrives with
  the first module pins; trunk compose profiles provide all services.

## wiring.md

This stack's observed wiring diagram is committed at [wiring.md](wiring.md);
CI drift-checks the running graph against it. Regenerate via
`airstack test -m wiring --stack lite_default`.
