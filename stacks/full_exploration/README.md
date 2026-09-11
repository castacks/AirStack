# full_exploration

`full_default` with the global planner swapped: the random-walk planner is
replaced by the **frontier-based geometric exploration planner** from the
external **asm_exploration_planner** module — the planner the Construction
Project flew on a real construction site. It integrates the filtered Ouster
cloud into its own OpenVDB occupancy map, extracts and clusters frontiers,
samples (optionally bounded) viewpoints around them, and publishes a
shortened, collision-checked RRT path on `global_plan`. Because that planner
only *publishes* plans, the stack also runs trunk's
`global_plan_navigate_bridge`, which turns each new plan into a `NavigateTask`
goal for `droan_gl` (a task executor that plans only while a goal is active).
Everything else — local planner, trajectory controller, PID, safety monitor,
takeoff/landing, VDB mapping, GCS bridge — is unchanged from `full_default`.

The only differences vs `full_default` are the global-planner block in
`launch/stack.launch.xml` (two includes replacing one) and the
`asm_exploration_planner` pin in `modules.repos`.

Bring-up:

```bash
airstack module sync            # pulls asm_exploration_planner per this stack's modules.repos
airstack up --stack full_exploration --sim isaac
airstack ready
```

Then, after takeoff, enable exploration from the GCS Tasks panel (the
`behavior/global_plan_toggle` service). Exploration runs until toggled off.

Notes:

- The exploration planner needs no extra image dependencies (OpenVDB, PCL,
  Eigen ship in the trunk robot image), so no `airstack module lock --build`.
- Planner tuning lives in the module: `exploration_planner/config/
  exploration_planner.yaml` (open-world defaults) and
  `exploration_planner_construction_site.yaml` (bounded exploration inside
  site extents, conservative unknown-as-occupied collision checks). Select
  the latter by passing `exploration_config` on the module include.
- `path_end_threshold_m` (default 3.0 m) must stay above the bridge's
  `goal_tolerance_m` (1.0 m): `droan_gl` stops short of the goal, and the
  planner only replans once the vehicle is inside that radius.
- `vdb_mapping` still runs: the GCS/RViz layouts and the topic keepalive
  consume its map, even though the planner does not.
- `wiring.md` is not committed yet (bootstrap-deferred): generate it from the
  first validated run with
  `airstack test -m wiring --stack full_exploration --sim isaacsim --num-robots 1`
  and commit the observed graph.
- Module repo: [castacks/asm_exploration_planner](https://github.com/castacks/asm_exploration_planner).
