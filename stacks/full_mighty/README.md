# full_mighty

`full_default` with the local planner swapped: the DROAN GPU planner
(`droan_gl`) is replaced by the **MIGHTY** Hermite-spline planner from the
external **asm_mighty** module (MIT ACL, RA-L 2026), together with its
acl-mapping voxel world model (fed by the filtered Ouster cloud) and a
`mighty_bridge` adapter that serves the same `tasks/navigate` NavigateTask
action and publishes receding-horizon `trajectory_controller/trajectory_override`
trajectories — so the rest of the stack (trajectory controller, PID, safety
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
- The [asm_mighty repo](https://github.com/castacks/asm_mighty) is public.
  Registry entry:
  [modules/mighty.yaml](https://github.com/castacks/airstack-modules-index/blob/main/modules/mighty.yaml);
  catalog page: [mighty](../../docs/modules/mighty.md).
- **Pin v0.1.3 or newer on AirStack 0.20.x.** `takeoff_landing_task` leaves
  the trajectory controller in ROBOT_POSE, where `trajectory_override` is
  merged but never flown; the v0.1.1 bridge assumed no mode management, so
  MIGHTY committed one trajectory and idled in GOAL_SEEN ("never replans").
  v0.1.2 sets TRACK before forwarding overrides, bounds and preempts
  NavigateTask goals, drops stale `global_plan` routes, and turns the vehicle
  to the goal pose's yaw on arrival (MIGHTY drops the goal orientation;
  `arrival_yaw` param). v0.1.3 adds launch args for the mapper->planner grid
  seam (`mighty_*_grid_topic`, to interpose a constraint node) and the
  altitude band (`mighty_z_min`, `mighty_mapper_z_ground`,
  `mighty_mapper_z_min_unknown`, for missions below the takeoff point);
  defaults unchanged. Changelog in the module README.
- Validation: the judged campaign ran at v0.1.0 (code-identical to v0.1.1;
  Isaac Sim, judged on ground truth): 44/44 vendored gtests, empty-world
  NavigateTask route (goal error 0.14 m), 7/7 pillar-field traversals, and
  5/5 judged obstacle-route flights with min clearances 1.59–1.65 m against a
  1.0 m gate — the motivating DROAN comparison (figures + numbers) is in the
  module README. The v0.1.2 bridge fixes were verified in closed-loop Isaac
  flights (NavigateTask goal reached within 0.1 m; arrival yaw within 10°);
  the judged campaign has not been re-run at v0.1.3.
