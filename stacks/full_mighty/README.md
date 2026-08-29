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
- The [asm_mighty repo](https://github.com/castacks/asm_mighty) is **private
  until the AirStack agent study concludes** (then public) — until the flip,
  `airstack module sync` needs castacks-member credentials. Registry entry:
  [modules/mighty.yaml](https://github.com/castacks/airstack-modules-index/blob/main/modules/mighty.yaml);
  catalog page: [mighty](../../docs/modules/mighty.md).
- Validation at the pinned version (Isaac Sim, judged on ground truth):
  44/44 vendored gtests, empty-world NavigateTask route (goal error 0.14 m),
  7/7 pillar-field traversals, and 5/5 judged obstacle-route flights with
  min clearances 1.59–1.65 m against a 1.0 m gate — the motivating DROAN
  comparison (figures + numbers) is in the module README.
