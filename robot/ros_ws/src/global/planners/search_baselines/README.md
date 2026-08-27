# search_baselines

One launch file per search method, so a run is named after the METHOD:

```bash
ros2 launch search_baselines vlfm.launch.xml      scene_params_file:=<scene>.yaml
ros2 launch search_baselines conavgpt2.launch.xml scene_params_file:=<scene>.yaml
ros2 launch search_baselines nearest.launch.xml   scene_params_file:=<scene>.yaml
ros2 launch search_baselines frontier.launch.xml  scene_params_file:=<scene>.yaml
ros2 launch search_baselines lawnmower.launch.xml scene_params_file:=<scene>.yaml
```

## Why one node, three methods

The executable these launch is `search_planner`, and it lives HERE. It is built
around the **vendored upstream Co-NavGPT2** library in the `conavgpt2` package
(`Global_Map_Proc`, `Frontier_Det`, `ROS_Agent`), so the package name says what
the code IS. VLFM and the geometric control reuse that machinery — the RGBD
occupancy map, the frontier extraction, the search-area and altitude filters,
the droan actuation — and differ ONLY in how a frontier is chosen:

| method | selection |
|---|---|
| `conavgpt2` | a generative VLM picks from numbered top-down BEV images — one image per candidate of the SAME 3D list, cut by label |
| `vlfm` | BLIP-2 ITM scores the live RGB into a value map; argmax value |
| `nearest` | closest frontier, no model at all |
| `frontier` | information gain minus visit cost — VLFM without the VLM |
| `lawnmower` | no frontier at all: boustrophedon lanes over the sector, the floor every semantic method must beat |

Sharing the map is not a shortcut, it is the experiment: because the three build
the SAME map from the SAME frames and act through the SAME controller, a
difference in the result is attributable to the selection policy and to nothing
else. Two separate implementations would each need their own mapping to be
argued equivalent before any comparison meant anything.

`conavgpt2` is a LEAF: it holds the vendored third-party code its name refers
to and depends on nothing in this stack.

## What each method needs running

Every model the fleet talks to lives in ONE container — `offboard-compute` —
and no planner loads a model of its own. Start it once, before any planner:

```bash
./airstack.sh up offboard-compute                       # detector only (default)
START_ITM_SERVER=true ./airstack.sh up offboard-compute  # + the VLFM scorer
docker exec offboard-compute curl -s localhost:8200/health
docker exec offboard-compute tail -f /tmp/offboard/detector_server.log
```

| service | port | who needs it | VRAM |
|---|---|---|---|
| `detector_server` — YOLO(-World) + MobileSAM | 8200 | **every method** (`detector_mode: 'server'`) | ~0.6 GiB |
| `itm_server` — BLIP-2 ITM | 8100 | `vlfm` | ~2.5 GiB |
| `vlm_server` — generative, OpenAI-compatible | 8000 | `conavgpt2` / `paper` | ~5.9 GiB at 7B nf4 |

| method | services it contacts per tick |
|---|---|
| `conavgpt2` | detector + generative VLM |
| `vlfm` | detector + ITM (two HTTP calls to the same host) |
| `nearest` | detector |
| `frontier` | detector |
| `lawnmower` | detector |

`vlfm` does **not** preflight or contact the generative endpoint, so a VLFM run
needs only the detector and the ITM service.

### Why the detector is a service, not a model per planner

Detection is **stateless** — one frame in, boxes and masks out, no history — so
requests from different robots interleave with no cross-talk. That is the same
property that made the ITM scorer shareable, and it matters more here because
every arm detects: at `NUM_ROBOTS=3`, three planner processes each loading YOLO
*and* MobileSAM is three copies of the same weights on one card producing
identical answers (~1.6 GiB each, against ~0.6 GiB for one shared instance
serving all three). It also keeps the comparison controlled — no arm can win by
having a different detector, because there is only one.

Measured across the docker bridge, 480x300 frame, PNG (233 KB on the wire):
**190 ms round trip** — 41 ms YOLO, 114 ms SAM, ~35 ms transport and codec. The
first request with a new open-vocabulary vocabulary costs ~2.9 s extra because
`set_classes` re-runs the CLIP text encoder; the server caches the vocabulary
and `/metrics` reports `vocab_switches`, which should stop climbing after the
first call of a run. If it keeps climbing, two planners disagree about
`detection_classes` and are thrashing the text tower.

The transport is plain HTTP on the docker bridge (`http://offboard-compute:8200`),
**not DDS** — frames travel in the request body, so none of this traffic touches
`dds_router.yaml` or a ROS domain, and `offboard-compute` joins no domain at all.

`detector_mode: 'local'` restores the old in-process behaviour for a bench with
no shared host. There is no automatic fallback: `'server'` with an unreachable
service is fatal at startup, because a detector that quietly answers "nothing
here" is indistinguishable from a search that found nobody.

`tests/test_detector_api.py` drives the REAL client over REAL HTTP against the
REAL server with a stubbed checkpoint (no GPU, no ultralytics): frame and mask
bit-exactness in both encodings, `mask=None` when nothing is found, the conf
threshold, the open-vocab vocabulary switch, closed-set vs open-vocab probing,
and a server that is down raising something an operator can act on.

## Co-NavGPT2 on the 3D frontiers

`conavgpt2.launch.xml` loads `config/conavgpt2.yaml`: `nav_mode: gpt`, the
paper's own cadence and threshold, and `frontier_source: voxel3d` — the same
source block as `vlfm.yaml` / `frontier.yaml`, so all three arms are offered
one candidate list. (`paper.yaml`, the previous default, sets no frontier
source and inherits `slab2d`; it is kept as the 2D-faithful overlay.)

Co-NavGPT is a 2D method: the VLM sees one top-down BEV per candidate and
nothing of its height. What the 3D source changes for it is (a) candidates
exist at every height the camera has seen through instead of one slab, and
(b) the waypoint flies at the chosen frontier's own z
(`_frontier_altitude`), held for the whole round even after the frontier is
carved away on approach. Two things had to hold for the VLM's `frontier_k` to
mean `target_point_list[k]`:

- **One offered candidate per grid cell.** The candidate images are cut from
  `target_edge_map` by label (`edge == k+1`), and z-stratified candidates over
  the same ground share a cell — the later disc used to erase the earlier
  label and the VLM was shown a blank map for that id. `_voxel_frontiers` now
  stamps only free pixels, drops a fully-covered duplicate, and refills the
  cap from lower ranks. Lossless for the other arms: value, rank gain and
  visit cost are all per-cell.
- **The source-level sector filter in the map frame** (`_to_map_arr`), like
  the pick-side one. This arm has no pick-side filter of its own, so with a
  grid→map offset it was being offered the wrong sector.

When the list is empty — t=0, or every frontier out of bounds — every arm now
steers to the sector centroid (`_fallback_goal`) instead of upstream's random
cell of the whole grid.

`tests/test_conavgpt2_baseline.py` runs the real `_voxel_frontiers`,
`_assign`, `_record_round`, `_resolve_frontier`, `_fallback_goal` and
`_frontier_altitude`, AND the real vendored `get_all_candidate_maps` image
loop (cv2 dilate and the PIL renderer replaced by numpy stand-ins that count
red pixels), against a scripted VLM: N images for N candidates, every image
drawn, `frontier_k → list[k]`, bad ids → `frontier_0`, the 3D height flown
and held, the map-frame filter, the centroid fallback, and a carved
`VoxelMap` end to end. No sim, no GPU, no ROS.

## How the lawnmower drives the drone

The lanes are a path hundreds of metres long, and droan_gl steers by the
closest point of whatever Path it is given (`cost = deviation - path_distance`,
`droan_gl_node.cpp`). Handing it the whole path, or handing it the lane ENDS
one at a time, is the same mistake at two scales: a single pose 260 m out that
it flies straight at, with nothing in between to be judged on and nothing to
skip if that pose is unreachable. The frontier arms never do that — a frontier
is a few tens of metres out, and the next one is chosen when it is reached.

So the lawnmower is fed to the drone the same way (`lawnmower.Sweep`, wired in
`_lawnmower_pick`):

1. **Transit.** From wherever the drone is, straight legs of at most
   `lawnmower_leg_m` (25 m) to the entry point of the sweep — the lane end
   nearest the drone. Flown once.
2. **Sweep.** The lane loop cut into legs of the same length, until the sim
   budget runs out; the wrap at the end goes back to the start of the loop,
   never through the transit.

Each tick publishes ONE goal through the same `_command` as every other arm:
the activator NavigateTask plus a two-pose Path on `/global_plan` — the goal,
then the **next lane point** (not a 2 m extension of the approach), so droan
has the lane's heading through the goal and progress credit along it. A leg
is done when the drone is within `lawnmower_reach_radius_m` of it OR has
passed it (ahead of it along the lane and within the reach radius laterally —
droan cuts onto the segment past the goal without entering the reach disc). A
leg the drone makes no progress on for `lawnmower_stall_s` is skipped, which
is the lawnmower's version of the frontier arms' lock/swap.

Frames: the lanes are authored in the MAP frame (`search_area_xy`), the agent
and `_goal_points` in the GRID frame, and both conversions go through the same
offset `_build_path` applies (`_to_map` / `_map_to_grid`). A goal past the map
edge is clamped to the grid and reach is judged against the clamped point, so
a sector that does not fit `map_extent_m` is warned about rather than stalled
on at every lane end — but fix the config: `search_area_pad_m` is 0 for this
arm so it flies exactly the area the others do.

Seeing it: the whole planned path is drawn on `/{robot}/search/markers` (and
republished by the GCS as `/gcs/{robot}/search/markers`) under five namespaces
— `search_lanes` (the full plan, dim), `search_lanes_done` (solid up to the
current goal, reset each lap), `search_lane_transit` (white; the one-time walk
to the entry lane end, which covers nothing), `search_lane_goal` (amber sphere
at `lawnmower_reach_radius_m`) and `search_lanes_id` (phase / index / lap /
spacing). `/global_plan` only ever carries the current ≤ `lawnmower_leg_m`
leg, so without these an on-plan sweep and a drone wandering look identical —
and the dim-vs-solid split is the only view of the ground the sweep has NOT
reached yet, which is where an undetected survivor would be.

`tests/test_lawnmower_baseline.py` runs the real `_lawnmower_pick`,
`_build_path` and `_command` against a teleporting drone with no ROS or sim:
two poses per publish, every hop ≤ `lawnmower_leg_m`, transit once then the
lanes, every lane point becomes a goal, offset-invariant, detours resume
without a skip, stalls skip, clipped lane ends do not stall.

### The 3D lawnmower: known obstacles, stall lifts, xy-only reach

A lane goal at cruise height inside a tree canopy is a goal droan can never
reach; the sweep used to sit on it for the stall timer and then leave that
ground unswept. A coverage path is planned over a KNOWN map, so this arm reads
the layout generator's obstacle boxes — `annotations/<scene>_obstacles.json`
(houses, trees, cars; written by the launcher next to the people-only GT
`<scene>.json`, found via `$RESULTS_SCENE`) — and flies each leg at
`max(flight_altitude_m, tallest box along the leg + obstacle_clearance_m)`,
clamped to the altitude band (`search_baselines/clearance.py`,
`_lm_leg_z_for`). Over open ground it stays at cruise, where the detector
works. The two-pose path carries THIS leg's height on the goal and the NEXT
leg's on the through pose, and `Sweep` judges reach in xy only, so a drone
that popped over a canopy descends toward the next goal instead of dropping
back onto the one it is already above. A stall (`lawnmower_stall_s`, 12 s)
first LIFTS the goal by `lawnmower_stall_lift_m` (twice), and only then skips
it. The frontier / VLFM / CoNavGPT arms do not get the obstacle file: their
map is what they have seen — state that asymmetry when the arms are compared.

### A fleet: one planner per drone, one sector each

Every robot container runs its own planner on its own ROS domain. The launch
passes `num_robots` = NUM_ROBOTS, and outside `team_mode` the node drives ONE
robot (its own) while cutting the search area into NUM_ROBOTS sectors
(`sector_partition: strips`) and taking the slice `ROBOT_NAME` names
(`robot_N` -> slice N-1). Detected people OUTSIDE the sector are ignored —
another drone's to visit — so two drones never converge on one person.
