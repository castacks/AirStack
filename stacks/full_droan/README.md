# `full_droan` — full autonomy with the GPU DROAN local planner

The full-autonomy topology with the **DROAN** reactive local planner
(`droan_gl`, GPU/OpenGL) from the external
[asm_droan](https://github.com/castacks/asm_droan) module in place of the
default MIGHTY planner. DROAN plans directly in the stereo disparity image
with no persistent map, so this is the stack for depth-camera-only vehicles,
minimal compute budgets, and comparisons against AirStack's long-standing
default behaviour — it is the 0.20.x `full_default` topology, kept as a
named stack.

## What it launches

Identical to [`full_default`](../full_default/README.md) except the
local-planner include: `droan_gl.launch.xml` (module-provided, canonical
defaults, zero remaps) instead of the MIGHTY module's
`mighty_module.launch.xml`. Everything else — interface, sensors, perception,
takeoff/land and fixed-trajectory task servers, trajectory controller, PID
controller, VDB mapping, random-walk global planner, safety monitor, logging,
DDS router, gossip — matches `full_default` exactly.

The CPU variant of the same planner (`droan_local_planner` + a live
`disparity_expansion` world model) is [`full_droan_cpu`](../full_droan_cpu/README.md).

## How to run

```bash
airstack up --stack full_droan --sim isaac --robots 1
airstack ready
```

`airstack up` adds the `asm_droan` pin from this stack's `modules.repos` to
the checkout and syncs it when it is missing, and composes the module's
dependency layer (`droan_gl`'s assimp/EGL/GLFW/GLM link deps) when it is
absent. Explicitly:

```bash
airstack module add https://github.com/castacks/asm_droan --version v0.1.0
airstack module lock --build
```

## Known limits

- Requires a GPU with OpenGL/EGL in the robot container (the planner runs its
  expansion and collision checks in shaders). Machines without one use
  `full_droan_cpu`.
- DROAN keeps no map: accumulated collision votes cannot be erased by looking
  again, so dense clutter can become an absorbing hover state, and its
  forward stereo field of view produces close-quarters near-contacts. The
  default `full_default` (MIGHTY) is the better choice where clearance
  margins matter; the judged comparison is in the
  [asm_mighty README](https://github.com/castacks/asm_mighty#why-mighty-replaced-droan).
- The interface layer is a wrapped include (`interface.launch.py`) — read
  `wiring.md` for the observed MAVROS wiring.

## wiring.md

This stack's observed wiring diagram is committed at [wiring.md](wiring.md);
CI drift-checks the running graph against it. Regenerate via
`airstack test -m wiring --stack full_droan`.
