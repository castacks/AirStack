# Vendored from castacks/asm_mighty

Upstream: https://github.com/castacks/asm_mighty @ 5547954 ("conflating
override throttle: never drop the last trajectory of a burst"), 2026-08-28.

Lives under `robot/ros_ws/src/modules/` — the IN-TREE module directory of
the RFC #379 module system (`airstack module create --in-tree`, upstream
develop / the airstack-paper branch), which is where a fork keeps a module it
carries local patches to; `stacks/full_mighty/modules.repos` records the
upstream pin. Copied in as plain packages rather than added with `airstack
module add`:
the module manifest (`module.yaml`) wants an AirStack trunk with the module
CLI (`airstack_compat: >=0.20.0-alpha.16`) and this branch — and upstream
`main` at 0.19.0 — has no `airstack module` command. Wiring that the module
system would do from a stack file is done by hand instead:
`local_bringup/launch/local.launch.xml` (`local_planner:=mighty`, from
`LOCAL_PLANNER`), `local_bringup/config/mighty_disaster.yaml` +
`global_mapper_disaster.yaml` (our overrides of the module's own configs),
and the apt deps in `robot/docker/Dockerfile.robot`.

Local patches (keep this list current so the module can be re-pulled):

* `mighty_bridge/mighty_bridge/bridge_node.py` — `NavigateTask.Goal.
  max_speed_mps` (our task_msgs field) is read and LOGGED (`speed cap
  requested N m/s`) but NOT applied: throttling the forwarded waypoints
  below MIGHTY's v_max drifts the vehicle behind MIGHTY's replan anchor and
  its 4 m start guard drops every trajectory (measured 2026-08-28). MIGHTY's
  own `v_max` is the speed until a live v_max is wired into mighty_node. An
  EMPTY-plan goal is accepted as a speed-only "activator" (succeeds at once,
  the global_plan follower keeps driving) instead of being rejected. Debug
  lines: `follower gate: ...` (until airborne), `override -> controller:
  ...` (5 s throttle), `trajectory controller mode -> TRACK: ok`. Waypoint
  yaw is the direction of travel (droan's convention), not MIGHTY's spline
  yaw, which was 0 and flew the drone sideways.
* `mighty_bridge/mighty_bridge/bridge_node.py` — NavigateTask poses and
  global_plan poses stamped in a frame other than `world_frame` are
  transformed through TF (droan_gl did this); a goal whose frame cannot be
  transformed is ABORTED, never flown as raw numbers (a 'world' goal was
  flown as map coordinates, 39 m off, 2026-08-28).
* `mighty/include/mighty/mighty.hpp`, `mighty/src/mighty/mighty.cpp`,
  `mighty/include/mighty/mighty_node.hpp`, `mighty/src/mighty/mighty_node.cpp`
  — `v_max` is a LIVE parameter: `MIGHTY::updateVmax` updates the optimizer
  params, the feasibility bounds and the HGP front end; `mighty_node`
  accepts it through a set-parameters callback (0 < v <= 20). mighty_bridge
  pushes the NavigateTask cap into it (`_push_vmax`) instead of throttling
  waypoints, and scales its start guard with the effective v_max
  (`start_guard_s` x v_max, floor `start_guard_min_m`).
* `mighty/src/mighty/mighty_node.cpp` — "Vehicle Type" log printed a
  std::string with %d.

* `mighty_bridge/mighty_bridge/bridge_node.py` — the follower / NavigateTask
  set the trajectory controller to TRACK when they engage: after a takeoff
  it sits in ROBOT_POSE and never walks an override.
* `mighty_bridge/launch/mighty_module.launch.xml` — `mighty_follow_min_climb_m`
  arg wired to the bridge's `follow_min_climb_m` param.

`tools/smoke_sim.py` was not copied.
