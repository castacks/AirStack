# full_mighty

`full_default` with the local planner swapped: the DROAN GPU planner
(`droan_gl`) is replaced by the **MIGHTY** Hermite-spline planner from the
external **asm_mighty** module (MIT ACL, RA-L 2026), together with its
acl-mapping voxel world model (fed by the filtered Ouster cloud) and a
`mighty_bridge` adapter that serves the same `tasks/navigate` NavigateTask
action and publishes `trajectory_controller/trajectory_segment_to_add`
segments — so the rest of the stack (trajectory controller, PID, safety
monitor, takeoff/landing, GCS) is unchanged from `full_default`.

This stack is the module-swap demonstration for the modular architecture:
the only difference vs `full_default` is one include in
`launch/stack.launch.xml` plus the `asm_mighty` pin in `modules.repos`.

Bring-up:

```bash
airstack module sync            # pulls asm_mighty per this stack's modules.repos
airstack up --stack full_mighty --sim isaac
```

Notes:

- MIGHTY is CPU-only (no GPU contention with Isaac).
- The planner needs the module's `nlohmann-json3-dev` dep layer:
  `airstack module lock --build` before `airstack up`.
- Planner/world-model tuning lives in the module
  (`mighty_bridge/config/*_airstack.yaml`).
