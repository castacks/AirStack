# `full_default` — trunk reference stack

The current full-autonomy topology, as a self-contained stack folder (RFC #379 §3).
This is the stack most users start from and the baseline other stacks are copied from.

## What it launches

The entry point `launch/stack.launch.xml` composes **every layer as a flat
set of module-launch includes** (Local flattened in E2; sensors, perception,
global, and behavior in E3): LiDAR near-range filter, stereo
disparity/point-cloud pair, topic keepalive, takeoff/land task server,
fixed-trajectory task server, GPU DROAN planner, trajectory controller, PID
controller, VDB mapping, random-walk global planner, and the drone safety
monitor — each module launch file declares its topic endpoints as args with
canonical defaults, so bare includes mean canonical wiring. Plus the two
role-`full` extras (DDS-router domain bridge to the GCS, gossip coordination
layer). Two blocks stay wrapped by design: `interface.launch.py` (the safety
boundary; flattens with RFC #380 Part 2's platform modules) and
`logging.launch.xml` (already a single self-contained module).

## Equivalence

`airstack up --stack full_default` produces a ROS graph identical to the
removed legacy `AUTONOMY_ROLE=full` dispatch — **machine-proven** by the
wiring-snapshot diff (this folder's `wiring.md` is the committed baseline).
This stack is also the default: with no stack selected, `robot.launch.xml`
launches it. Verify with the wiring snapshot test:

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

- The interface layer is a wrapped include (`interface.launch.py`) — its
  MAVROS wiring is not visible in `stack.launch.xml`; read `wiring.md` for
  the observed graph. It flattens with the platform-module refactor
  (RFC #380 Part 2).
- `modules.repos` pins no external modules yet; every package is trunk-resident.
- `docker-compose.yaml` is a stub — per-stack image composition arrives with
  the first module pins; trunk compose profiles provide all services.

## wiring.md

This stack's observed wiring diagram is committed at [wiring.md](wiring.md).

Generated — the commit arrives with the first snapshot run of
`airstack test -m wiring --stack full_default` (the harness writes
`observed_full_default.md` under the run directory; validate it and copy it
here as `wiring.md`). Once committed, CI drift-checks the running graph
against it.
