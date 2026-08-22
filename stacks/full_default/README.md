# `full_default` — trunk reference stack

The full-autonomy topology, as a self-contained stack folder. This is the
stack most users start from, the default when no stack is selected, and the
baseline other stacks are copied from.

## What it launches

The entry point `launch/stack.launch.xml` composes **every layer as a flat
set of module-launch includes**: LiDAR near-range filter, stereo
disparity/point-cloud pair, topic keepalive, takeoff/land task server,
fixed-trajectory task server, GPU DROAN planner, trajectory controller, PID
controller, VDB mapping, random-walk global planner, and the drone safety
monitor — each module launch file declares its topic endpoints as args with
canonical defaults, so bare includes mean canonical wiring. Two cross-domain
extras run alongside: the DDS-router domain bridge to the GCS and the gossip
coordination layer. Two blocks stay wrapped by design: `interface.launch.py`
(the safety boundary) and `logging.launch.xml` (already a single
self-contained module).

## Baseline

This stack is the default: with no stack selected, `robot.launch.xml`
launches it. Its committed [wiring.md](wiring.md) is the observed-graph
baseline that other full stacks are compared against. Verify with the wiring
snapshot test:

```bash
airstack test -m wiring --stack full_default --sim isaacsim --num-robots 1
```

## How to run

```bash
airstack up --stack full_default --sim isaac --robots 1
airstack ready
```

The shared per-robot preamble (ROBOT_NAME namespace, `use_sim_time`,
`robot_state_publisher`, world→map static TF) runs in
`autonomy_bringup/launch/robot.launch.xml`, which dispatches to this stack when
`AIRSTACK_STACK_DIR` is set.

## Known limits

- The interface layer is a wrapped include (`interface.launch.py`) — its
  MAVROS wiring is not visible in `stack.launch.xml`; read `wiring.md` for
  the observed graph.
- `modules.repos` pins no external modules yet; every package is trunk-resident.
- `docker-compose.yaml` is a stub — per-stack image composition arrives with
  the first module pins; trunk compose profiles provide all services.

## wiring.md

This stack's observed wiring diagram is committed at [wiring.md](wiring.md);
CI drift-checks the running graph against it. Regenerate via
`airstack test -m wiring --stack full_default`.
