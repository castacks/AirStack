# full_exploration

`full_mighty` with the global planner swapped: the random-walk planner is
replaced by the **frontier-based geometric exploration planner** from the
external **asm_exploration_planner** module — the planner the Construction
Project flew on a real construction site. It integrates the filtered Ouster
cloud into its own OpenVDB occupancy map, extracts and clusters frontiers,
samples (optionally bounded) viewpoints around them, and publishes a
shortened, collision-checked RRT path on `global_plan`. That planner only
*publishes* plans; the MIGHTY bridge (`asm_mighty`) follows the `global_plan`
topic directly, so no Path→NavigateTask adapter is needed (a droan_gl-based
variant would add trunk's `global_plan_navigate_bridge`). Everything else —
MIGHTY local planner + acl-mapping world model (LiDAR-based, so it works in
unlit scenes), trajectory controller, PID, safety monitor, takeoff/landing,
VDB mapping, GCS bridge — is unchanged from `full_mighty`.

The only differences vs `full_mighty` are the global-planner include in
`launch/stack.launch.xml` and the `asm_exploration_planner` pin in
`modules.repos`.

Bring-up:

```bash
airstack module sync            # pulls asm_exploration_planner per this stack's modules.repos
airstack up --stack full_exploration --sim isaac
airstack ready
```

Then, after takeoff, enable exploration from the GCS Tasks panel (the
`behavior/global_plan_toggle` service). Exploration runs until toggled off.

Notes:

- `asm_mighty` v0.1.1 has a known fixed-yaw-at-goal bug (the vehicle does not
  turn toward its goal); a fix is being upstreamed — re-pin `modules.repos`
  when it lands.

- The exploration planner needs no extra image dependencies (OpenVDB, PCL,
  Eigen ship in the trunk robot image); MIGHTY needs its `nlohmann-json3-dev`
  layer, so run `airstack module lock --build` before `airstack up` (as for
  `full_mighty`).
- Planner tuning lives in the module: `exploration_planner/config/
  exploration_planner.yaml` (open-world defaults) and
  `exploration_planner_construction_site.yaml` (bounded exploration inside
  site extents, conservative unknown-as-occupied collision checks). Select
  the latter by passing `exploration_config` on the module include.
- `path_end_threshold_m` (default 3.0 m) is the radius around the plan's
  final pose inside which the planner considers the plan complete and picks
  the next frontier; keep it above the local planner's arrival tolerance
  (MIGHTY's follower stops ~1–2 m short of the route end).
- `vdb_mapping` still runs: the GCS/RViz layouts and the topic keepalive
  consume its map, even though the planner does not.
- `wiring.md` is not committed yet (bootstrap-deferred): generate it from the
  first validated run with
  `airstack test -m wiring --stack full_exploration --sim isaacsim --num-robots 1`
  and commit the observed graph.
- Module repo: [castacks/asm_exploration_planner](https://github.com/castacks/asm_exploration_planner).
