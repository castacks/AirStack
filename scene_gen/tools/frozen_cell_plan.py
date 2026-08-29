#!/usr/bin/env python3
"""Plan a benchmark run on a FROZEN dataset cell: sectors, spawns, map sizing.

    python3 scene_gen/tools/frozen_cell_plan.py --robots 8
    python3 scene_gen/tools/frozen_cell_plan.py --cell Fire/Suburban/level_2/1
    python3 scene_gen/tools/frozen_cell_plan.py --robots 8 --emit-yaml OUTDIR

WHY THIS IS OFFLINE AND NOT IN THE PLANNER. Everything below is a pure function
of the cell's own `GT_hints.json`, so it can be computed once, READ, and pasted
into a mission — where a reviewer can see the eight coordinates a drone will be
put at instead of trusting a number the pod produced at 3 a.m. It imports the
planner's OWN `sector.py`, so the sectors printed here are bit-identical to the
ones `search_planner` cuts at run time; anything else would be a second
implementation to keep in step.

WHAT IT ANSWERS, in the order the answers depend on each other:

1. THE SEARCH AREA. `frozen_annotations.search_polygon` — the convex hull of
   every damaged object in the cell, grown by `--pad` metres and CLIPPED TO
   THE PLATE. That is the whole of what a mission is allowed to search: the
   disaster-affected area plus a working margin, and nothing else.

   The pad is applied HERE rather than by the planner, and the emitted overlay
   therefore sets `search_area_pad_m: 0.0`. The planner pads and THEN
   partitions, which on a 1 km plate gives the two end drones a bounding
   rectangle made mostly of the ring hanging over the edge of the ground
   sheet — measured before this was fixed, sector 8 was 57,000-82,000 m2 of
   which only 7,600-12,300 m2 was on the plate.

2. THE SECTORS. `sector.sector_for(poly, n, i, mode='rect', axis='principal',
   margin_m=0)` — the same call the node makes on the same already-padded
   polygon, so sector i here IS robot i+1's area.

3. WHERE THE DRONES GO. One spawn INSIDE EACH SECTOR, on free ground.

   Not a cluster on one kerb, which is what every earlier mission did and what
   the 1 km wildfire file still does. Two reasons, and the second is the one
   that decides it:

     * the search area has to contain the spawn — a drone that starts outside
       its own sector spends the beginning of a sim-time-bounded run flying to
       work, and that transit is charged to the method;
     * MIGHTY flies its own `v_max` (1.5 m/s) and ignores the speed cap the
       NavigateTask carries (benchmark-disaster-dataset 4c). At 1.5 m/s the
       500 m from a central kerb to an outer sector is 330 s of a 600 s
       budget. The transit argument that made a cluster acceptable at 7 m/s
       does not survive the local planner this eval actually runs.

   "Free ground" is: at least `--clear` metres from every obstacle box in
   `GT_hints.json` (houses, trees, cars, debris, toppled furniture) and from
   every survivor in `GT_people.json` — a drone must not be spawned on top of
   the answer key. Among the candidates that qualify, the one CLOSEST TO THE
   SECTOR CENTROID wins: that is the point that minimises both the transit
   inside the sector and the map extent below.

   Constraints are relaxed in a fixed order, and the plan says which one each
   spawn used: `in-damage` (inside the damage hull itself), `in-pad`
   (anywhere in the drone's own padded sector), `near-seam` (the sector's
   boundary inset dropped), `roomiest` (the best clearance the sector has,
   whatever it is). Clearance is never traded for staying inside the hull —
   both are the drone's own search area, and only one of them decides whether
   it clears the canopy on the way up.

4. HOW BIG THE MAP MUST BE. `map_cells` is fixed at 480, so `map_extent_m` sets
   CELL SIZE, not cell count. Two constraints, and the file has to satisfy
   both because the four arms share one scene overlay:

     * the three SECTORED arms run `frame_mode: 'local'` — the grid is centred
       on that robot's takeoff point, so its half-extent must reach the far
       corner of its own sector FROM ITS SPAWN;
     * the CoNavGPT2 team arm runs `frame_mode: 'global_enu'` with
       `map_origin_xy: [0, 0]` (conavgpt2_team.yaml, which loads after the
       scene overlay) — its half-extent must reach the far corner of the WHOLE
       search area from the world origin.

   The extent printed is 2x the larger of the two, rounded up, plus a margin.
"""

import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "simulation", "isaac-sim", "utils"))
sys.path.insert(0, os.path.join(
    _REPO, "robot", "ros_ws", "src", "global", "planners", "search_baselines",
    "search_baselines"))

import frozen_annotations as fa          # noqa: E402
import sector as sect                    # noqa: E402  (planner's own copy)

DEFAULT_ROOT = os.path.expanduser("~/SEI-COA/final_disaster_dataset")
CONTAINER_ROOT = "/isaac-sim/final_disaster_dataset"

# The six suburban cells that exist today, people placement 1. Fire and tornado
# levels 1-3; the urban locales and the other two disasters are not built.
DEFAULT_CELLS = [
    "Fire/Suburban/level_1/1", "Fire/Suburban/level_2/1",
    "Fire/Suburban/level_3/1",
    "Tornado/Suburban/level_1/1", "Tornado/Suburban/level_2/1",
    "Tornado/Suburban/level_3/1",
]

MAP_CELLS = 480.0          # fixed upstream; map_extent_m therefore sets cell size
FRONTIER_BOUNDARY_M = 20.0  # what frontier_threshold_points should mean, in metres


def scene_name(cell):
    """`Fire/Suburban/level_1/1` -> `FireSuburbanL1V1`.

    RESULTS_SCENE names the annotation files and the results directory, so it
    has to be a single token, stable, and unmistakable about which cell it is.
    """
    dis, loc, lvl, var = cell.strip("/").split("/")
    return "{0}{1}L{2}V{3}".format(dis, loc, lvl.split("_", 1)[1], var)


def usd_name(cell):
    dis, loc, lvl, var = cell.strip("/").split("/")
    return "{0}_{1}_lvl{2}_{3}.usd".format(
        dis.lower(), loc.lower(), lvl.split("_", 1)[1], var)


def _poly_list(poly):
    return [[float(p[0]), float(p[1])] for p in poly]


def _centroid(poly):
    a = 0.0
    cx = cy = 0.0
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        cr = x0 * y1 - x1 * y0
        a += cr
        cx += (x0 + x1) * cr
        cy += (y0 + y1) * cr
    if abs(a) < 1e-9:
        return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)
    a *= 0.5
    return (cx / (6.0 * a), cy / (6.0 * a))


CLEARANCE_CAP_M = 25.0   # far enough that anything past it is "wide open"


def clearance_field(boxes, people, cap_m=CLEARANCE_CAP_M):
    """A `clearance(x, y)` function: metres to the nearest obstacle, capped.

    NOT a blocked/free predicate. A 1 km suburb has ~6,000 trees and ~250
    houses, so on most of the plate the honest answer to "is there 10 m of
    free ground here" is no — the streets are ~10 m wide and the kerb trees
    stand in them. A boolean at a fixed radius therefore reports whole sectors
    as unusable when what is actually true is that the best spot in them has
    6 m of room. Returning the distance lets the caller take the best point a
    sector HAS and say what it got.

    Survivors are in the field as well as the geometry: a drone must not be put
    on top of the answer key.

    Bucketed on a `cap_m` lattice, so a 7,000-object cell costs nine bucket
    lookups per candidate instead of 7,000 distance tests.
    """
    cell = max(1.0, float(cap_m))
    grid = {}

    def add(cx, cy, hx, hy):
        for gx in range(int(math.floor((cx - hx) / cell)),
                        int(math.floor((cx + hx) / cell)) + 1):
            for gy in range(int(math.floor((cy - hy) / cell)),
                            int(math.floor((cy + hy) / cell)) + 1):
                grid.setdefault((gx, gy), []).append((cx, cy, hx, hy))

    for b in boxes:
        bw = b["bbox_world"]
        add(bw["center_xyz_m"][0], bw["center_xyz_m"][1],
            abs(bw["size_xyz_m"][0]) / 2.0, abs(bw["size_xyz_m"][1]) / 2.0)
    for r in people:
        try:
            add(float(r["x"]), float(r["y"]), 0.4, 0.4)
        except (KeyError, TypeError, ValueError):
            continue

    def clearance(x, y):
        gx0, gy0 = int(math.floor(x / cell)), int(math.floor(y / cell))
        best = cap_m
        for gx in (gx0 - 1, gx0, gx0 + 1):
            for gy in (gy0 - 1, gy0, gy0 + 1):
                for cx, cy, hx, hy in grid.get((gx, gy), ()):
                    dx = max(abs(x - cx) - hx, 0.0)
                    dy = max(abs(y - cy) - hy, 0.0)
                    d = math.hypot(dx, dy)
                    if d < best:
                        best = d
                        if best <= 0.0:
                            return 0.0
        return best

    return clearance


def _inset(poly, m):
    """`poly` shrunk by `m` metres, via the planner's own outward padder."""
    if m <= 0.0:
        return _poly_list(poly)
    return _poly_list(sect.pad_polygon(poly, -float(m)))


def pick_spawn(sector_poly, clearance, want_m=10.0, step_m=2.5, edge_m=15.0,
               must_be_in=(), bounds=None, strict=False):
    """A spawn point inside `sector_poly`: `(x, y, clearance_m)`, or None.

    Two passes over the same lattice, in this order:

      1. among the points with at least `want_m` of free ground, the one
         CLOSEST TO THE SECTOR CENTROID — the point that minimises both the
         drone's transit inside its own sector and the map half-extent it
         needs;
      2. if the sector has no such point (a wooded band on a suburban plate
         often has none at 10 m), the point with the MOST clearance it has,
         ties broken toward the centroid — unless `strict`, which returns None
         instead so the caller can relax a DIFFERENT constraint first. Trading
         11 m of rotor clearance for "inside the damage hull rather than inside
         its 50 m pad" is the wrong trade: both are in the drone's own search
         area, and only one of them decides whether it clears the canopy on the
         way up.

    So the answer is always the roomiest usable spot the sector actually
    contains, and the achieved clearance is returned for the caller to print
    rather than being silently traded away.

    `edge_m` keeps the drone off the sector boundary: a spawn exactly on the
    seam is judged OUTSIDE the sector by `sector.points_in_polygon`'s half-open
    rule (the upper and right edges do not belong to the polygon), and the
    planner would spend its first ticks nudging the drone back in.

    `must_be_in` is every polygon the point must ALSO lie inside — in practice
    the UNPADDED damage hull, so a drone starts on ground the disaster actually
    touched rather than in the 50 m pad ring around it. `bounds` is the plate
    rectangle inset by a margin: the sectors are cut from the PADDED polygon
    and can therefore reach past the edge of the ground sheet, and a drone
    spawned at x = -505 m on a plate that ends at -500 m has nothing under it.
    """
    inner = _inset(sector_poly, edge_m)
    if len(inner) < 3:
        inner = _poly_list(sector_poly)
    xs = [p[0] for p in inner]
    ys = [p[1] for p in inner]
    cx, cy = _centroid(inner)
    near = None      # (dist_to_centroid, x, y, clearance) with clearance >= want
    roomy = None     # (-clearance, dist_to_centroid, x, y)
    y = math.floor(min(ys) / step_m) * step_m
    while y <= max(ys):
        x = math.floor(min(xs) / step_m) * step_m
        while x <= max(xs):
            if (fa.point_in_polygon(x, y, inner)
                    and (bounds is None
                         or (bounds[0] <= x <= bounds[2]
                             and bounds[1] <= y <= bounds[3]))
                    and all(fa.point_in_polygon(x, y, q) for q in must_be_in)):
                c = clearance(x, y)
                if c > 0.0:
                    d = math.hypot(x - cx, y - cy)
                    key = (-c, d)
                    if roomy is None or key < roomy[:2]:
                        roomy = (-c, d, x, y)
                    if c >= want_m and (near is None or d < near[0]):
                        near = (d, x, y, c)
            x += step_m
        y += step_m
    if near is not None:
        return (round(near[1], 1), round(near[2], 1), round(near[3], 2))
    if strict:
        return None
    if roomy is not None:
        return (round(roomy[2], 1), round(roomy[3], 1), round(-roomy[0], 2))
    return None


def plan_cell(root, cell, n_robots, pad_m, clear_m, sector_mode, sector_axis,
              step_m, edge_m, margin_m):
    args_pad = pad_m
    usd = os.path.join(root, cell, usd_name(cell))
    people_doc, hints_doc = fa.load_cell(usd)
    if hints_doc is None:
        raise SystemExit("{0}: no GT_hints.json".format(cell))
    rect = fa.region_m(hints_doc)
    hull = fa.affected_polygon(hints_doc, rect)
    if len(hull) < 3:
        raise SystemExit("{0}: no damaged objects — no search area".format(cell))
    # THE PLANNER GETS A POLYGON THAT IS ALREADY PADDED AND ALREADY ON THE
    # PLATE, and the overlay sets search_area_pad_m: 0.0 (frozen_annotations.
    # pad_convex explains why padding after the partition is wrong).
    poly = fa.search_polygon(hints_doc, rect, pad_m)
    people = fa.people_records(people_doc)
    boxes = fa.obstacle_boxes(hints_doc)
    clearance = clearance_field(boxes, people)

    sectors = [sect.sector_for(poly, n_robots, i, mode=sector_mode,
                               axis=sector_axis, margin_m=0.0)
               for i in range(n_robots)]

    # A spawn must be ON THE PLATE (the sectors are cut from the padded polygon
    # and can hang over the edge of the ground sheet) and INSIDE THE UNPADDED
    # DAMAGE HULL (the drone starts on ground the disaster touched, not in the
    # pad ring). Each fallback below drops one of those, hardest first, and the
    # plan records which one was used so a relaxed spawn is visible rather than
    # silent.
    bounds = (rect[0] + edge_m, rect[1] + edge_m,
              rect[2] - edge_m, rect[3] - edge_m)
    attempts = (
        ("in-damage", dict(edge_m=edge_m, must_be_in=(hull,), bounds=bounds,
                           strict=True)),
        ("in-pad", dict(edge_m=edge_m, must_be_in=(), bounds=bounds,
                        strict=True)),
        ("near-seam", dict(edge_m=0.0, must_be_in=(), bounds=bounds,
                           strict=True)),
        ("roomiest", dict(edge_m=0.0, must_be_in=(), bounds=bounds,
                          strict=False)),
    )
    spawns = []
    clears = []
    modes = []
    for i, s in enumerate(sectors):
        sp = mode = None
        for mode, kw in attempts:
            sp = pick_spawn(_poly_list(s), clearance, want_m=clear_m,
                            step_m=step_m, **kw)
            if sp is not None:
                break
        if sp is None:
            raise SystemExit(
                "{0}: sector {1}/{2} has no point on the plate clear of every "
                "obstacle — lower --step".format(cell, i + 1, n_robots))
        spawns.append((sp[0], sp[1]))
        clears.append(sp[2])
        modes.append(mode)

    # Map sizing (see the module docstring, item 4).
    local_half = max(
        fa.distance_to_farthest_vertex(sp[0], sp[1], _poly_list(s))
        for sp, s in zip(spawns, sectors))
    team_half = fa.distance_to_farthest_vertex(0.0, 0.0, poly)
    half = max(local_half, team_half) + margin_m
    extent = 20.0 * math.ceil(2.0 * half / 20.0)
    cell_m = extent / MAP_CELLS

    inside = sum(1 for r in people
                 if fa.point_in_polygon(float(r["x"]), float(r["y"]), poly))
    return {
        "cell": cell,
        "scene": scene_name(cell),
        "usd": usd_name(cell),
        "region_m": list(rect),
        "damage_polygon": hull,
        "damage_area_m2": fa.polygon_area(hull),
        "polygon": poly,
        "polygon_area_m2": fa.polygon_area(poly),
        "plate_frac": fa.polygon_area(hull)
        / max(1e-9, (rect[2] - rect[0]) * (rect[3] - rect[1])),
        "search_frac": fa.polygon_area(poly)
        / max(1e-9, (rect[2] - rect[0]) * (rect[3] - rect[1])),
        "sectors": [_poly_list(s) for s in sectors],
        "sector_areas": [sect.polygon_area(s) for s in sectors],
        "spawns": spawns,
        "spawn_clearance_m": clears,
        "spawn_mode": modes,
        "pad_m": pad_m,
        "people": len(people),
        "people_inside_padded": inside,
        "obstacles": len(boxes),
        "local_half_m": local_half,
        "team_half_m": team_half,
        "map_extent_m": extent,
        "cell_m": cell_m,
        "frontier_threshold_points": max(4, int(round(FRONTIER_BOUNDARY_M
                                                      / cell_m))),
    }


def spawn_configs_json(spawns, poly, indent="     "):
    """SPAWN_CONFIGS for the mission `env:`, one entry per robot.

    Each drone is YAWED TOWARD THE CENTROID of the whole search area rather
    than left at identity. The first thing a run does is take off and look, and
    a 30 deg-down camera pointed at the middle of the damage sees the scene on
    its first frame instead of the plate edge — which is also the first frame
    VLFM's value map is built from.
    """
    cx, cy = _centroid(poly)
    rows = []
    for x, y in spawns:
        yaw = math.atan2(cy - y, cx - x)
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        rows.append('{{"x_m": {0:.1f}, "y_m": {1:.1f}, "orient": '
                    '[0.0, 0.0, {2:.4f}, {3:.4f}]}}'.format(x, y, qz, qw))
    return "[" + (",\n" + indent).join(rows) + "]"


SCENE_YAML = '''\
# SCENE OVERLAY — {cell}, the FROZEN dataset cell.
#
# GENERATED by `scene_gen/tools/frozen_cell_plan.py --emit-yaml`. Every number
# below was measured off this cell's own GT_hints.json; re-run the tool rather
# than editing them by hand, and re-run it if the cell is ever re-exported.
#
# ONE file serves all four arms — frontier / lawnmower / vlfm / conavgpt2 —
# loaded LAST (after planner.yaml then the method overlay), so the scene has
# the final word on geometry while nav_mode / the scorer / frontier_source
# come from the METHOD overlay and are deliberately NOT set here. The one
# exception is conavgpt2_team.yaml, which loads AFTER this file because the
# team arm has to override frame_mode; see map_extent_m below.
#
#   damage hull   {damage_area:,.0f} m2 ({plate_frac:.0%} of the {plate_w:.0f} x {plate_h:.0f} m plate)
#   search area   {search_area:,.0f} m2 ({search_frac:.0%}), {npts} pts
#   survivors     {people} total, {people_in} inside the search area
search_planner:
  ros__parameters:

    # ── map geometry ────────────────────────────────────────────────────────
    # map_cells is FIXED at 480, so map_extent_m sets CELL SIZE, not cell
    # count: {extent:.0f} m -> {cell_m:.2f} m/cell. It has to satisfy the LARGER of two
    # demands, because the four arms share this file:
    #   * the three SECTORED arms are frame_mode 'local' — the grid is centred
    #     on that robot's takeoff point and must reach the far corner of its
    #     own sector: {local_half:.0f} m here, measured from the spawns in the mission;
    #   * the CoNavGPT2 TEAM arm is frame_mode 'global_enu' with
    #     map_origin_xy [0, 0] (conavgpt2_team.yaml) — its grid is centred on
    #     the plate and must reach the far corner of the WHOLE search area:
    #     {team_half:.0f} m.
    # Every arm therefore rasterises at the same cell size, which is the point:
    # a difference in results has to come from the selection policy.
    map_extent_m: {extent:.1f}
    frame_mode: 'local'

    # ── the search area: the damage, and nothing more ───────────────────────
    # `<RESULTS_SCENE>_region.json`, written by the Isaac launcher from this
    # cell's GT_hints.json (simulation/isaac-sim/utils/frozen_annotations.py).
    # The key is 'search': the convex hull of every damaged object, grown by
    # {pad:.0f} m and clipped to the plate. NOT 'burn' or 'affected' — those names
    # belong to the fire MODEL in scene_gen/disaster/region.py and cannot be
    # computed for a tornado cell at all.
    #
    # THE PAD IS ALREADY IN THE POLYGON, so search_area_pad_m stays 0.0 (the
    # method overlays' value). Padding here instead would pad AFTER the sector
    # split, and on a 1 km plate that hands the two end sectors a bounding
    # rectangle that is mostly the ring hanging over the edge of the ground
    # sheet — measured before this was fixed: sector 8 was 57,000-82,000 m2 of
    # which 7,600-12,300 m2 was on the plate.
    search_area_source: 'scene'
    search_area_scene_key: 'search'
    search_area_pad_m: 0.0
    # The damage runs as a band/corridor, so the per-robot rect bands are cut
    # along the polygon's OWN major axis; axis-aligned bands would hand the
    # middle drone a rectangle spanning the diagonal.
    sector_axis: 'principal'

    # WORLD frame: the region file is authored in scene coordinates and the
    # planner subtracts its measured map origin once, on the first tick with a
    # GPS fix. Every robot then cuts the SAME sectors regardless of where it
    # spawned. `search_area_xy` below is the same polygon inline — the
    # fallback for a `search_area_source: 'config'` run on a machine with no
    # annotation file.
    search_area_frame: 'world'
    search_area_xy: [{search_xy}]

    # frontier_threshold_points counts CELLS ALONG THE FRONTIER BOUNDARY, so
    # its physical meaning moves with cell size. {threshold} cells x {cell_m:.2f} m is
    # ~{threshold_m:.0f} m of boundary, the same order as the 250 m bench's 10 m and the
    # 1.7 km plat's 21 m. The default 20 would discard every frontier shorter
    # than {default20:.0f} m, which is most of them.
    frontier_threshold_points: {threshold}
    # From the air the SMALL frontier regions are gaps between houses; the big
    # ones are open ground nobody has covered.
    frontier_order: 'largest'
    # Finer than the grid it rasterises into would be pointless, and the
    # per-tick cloud is the cost that scales on a 1 km scene.
    scene_voxel_m: 1.0

    # ── run budget ──────────────────────────────────────────────────────────
    # SIM seconds, counted from the planner's FIRST data tick — i.e. after
    # takeoff — so neither takeoff nor the scene load is charged to the method.
    # 600 s matches every other baseline in this stack, so a run covers the
    # same amount of SIMULATED search whatever the real-time factor was.
    max_sim_seconds: 600.0

    # The mount tilt is NOT set here: camera_pitch_rad defaults to
    # ZED_PITCH_DEG, the same variable the Isaac launch script tilts the
    # physical camera by, so the two cannot disagree.

    # ── search bounds / actuation ───────────────────────────────────────────
    # Suburban canopy and damaged structures top out near 15 m; the drone flies
    # the band above them and descends only to confirm a detection. The
    # FRONTIER band is a separate question from the FLIGHT band: frontiers may
    # sit from head height up over the rooflines, while the drone is clamped to
    # 15-40 m AGL. Sizing the voxel map to the flight band alone puts it in
    # empty air and degenerates voxel3d back into a 2D slab.
    obstacle_min_z_m: 2.0
    obstacle_max_z_m: 15.0
    frontier_z_min_m: 3.0
    frontier_z_max_m: 20.0
    min_altitude_agl_m: 15.0
    max_altitude_agl_m: 40.0
    flight_altitude_m: 20.0
    inspect_altitude_m: 10.0
    depth_max_m: 80.0

    # Arrival radii on a {cell_m:.2f} m grid. frontier_unlock_radius_m must be
    # >= goal_tolerance_m or the lock releases before the local planner
    # considers the goal reached.
    goal_tolerance_m: 5.0
    frontier_unlock_radius_m: 8.0

    # ── detector ────────────────────────────────────────────────────────────
    # ONE gate, measured, never inherited: 0.65 keeps the confident real people
    # and sits above the ~0.32 false-positive band. It matches planner.yaml and
    # every other layer, so the DETECTOR is not what differs between arms.
    sem_threshold: 0.65
    goal_name: 'person'
    detection_classes: ['person', 'car', 'truck', 'house', 'building', 'tree',
                        'smoke', 'fire', 'road', 'rubble']
    # Open-vocab weights. In SERVER mode the offboard detector's own
    # --yolo-weights is what actually runs, so this must name the SAME file
    # (CONAVGPT2_YOLO_WORLD_WEIGHTS in the mission) or the planner logs
    # DETECTOR MISMATCH.
    yolo_world_weights: '/root/.cache/conavgpt2_weights/yolov8l-world.pt'
    mobile_sam_weights: '/root/.cache/conavgpt2_weights/mobile_sam.pt'
    ultralytics_weights_dir: '/root/.cache/conavgpt2_weights'

    # The mcap carries the map render; PNGs would be one per agent per tick.
    save_debug_images: false

    # ── fleet sizing: this overlay is generated for NUM_ROBOTS = {robots} ────
    # The spawns, and therefore map_extent_m above, are already {robots}-specific;
    # so is this. VLFM scores a keyframe in ~69 ms on the ONE shared BLIP-2 ITM
    # server, which is ~7% duty per robot at 1 Hz, ~14% at 2 Hz and ~35% at
    # 5 Hz — so the shipped 0.2 s serves about three robots and {robots} would ask
    # {duty:.0f}% of one scorer. {period} s is the rate that fits this fleet.
    #
    # Ignored by every arm but vlfm, which is why it can live in the shared
    # scene layer instead of forking the overlay: a scene layer that differed
    # BY ARM would be the one thing this file exists to prevent.
    #
    # It is an ASSUMPTION until the run says otherwise. The mission's poller
    # reads /metrics on :8100 every 15 s; if mean_ms or errors climb through
    # the flight, the scorer is still saturated and this number is too low.
    vlfm_keyframe_period_s: {period}
'''


def scene_yaml_name(cell):
    dis, loc, lvl, _var = cell.strip("/").split("/")
    return "frozen_{0}_{1}_lvl{2}.yaml".format(
        dis.lower(), loc.lower(), lvl.split("_", 1)[1])


def itm_keyframe_period_s(n_robots):
    """Keyframe period for `n_robots` sharing ONE BLIP-2 ITM scorer.

    From the measured budget (69 ms per keyframe at the shipped 2x3 tiling):
    ~35% duty per robot at 0.2 s, ~14% at 0.5 s, ~7% at 1.0 s. Three thresholds
    rather than a formula, because those are the three numbers that were
    actually measured and an interpolated 0.63 would be a fiction.
    """
    if n_robots <= 3:
        return 0.2
    return 0.5 if n_robots <= 7 else 1.0


def emit_scene_yaml(plan, out_dir, n_robots):
    rect = plan["region_m"]
    xy = []
    for i, (x, y) in enumerate(plan["polygon"]):
        xy.append("{0:.1f}, {1:.1f}".format(x, y))
    # The comma at the end of every wrapped line is load-bearing: without it
    # YAML folds the two numbers either side of the break into ONE string
    # ("374.7 -454.8"), the polygon silently loses a vertex, and the planner
    # reshapes what is left into a different shape. Caught by
    # tests/test_frozen_scene_overlays.py, which round-trips every emitted file.
    wrapped = (",\n" + " " * 22).join(
        [", ".join(xy[i:i + 3]) for i in range(0, len(xy), 3)])
    body = SCENE_YAML.format(
        cell=plan["cell"],
        damage_area=plan["damage_area_m2"], plate_frac=plan["plate_frac"],
        plate_w=rect[2] - rect[0], plate_h=rect[3] - rect[1],
        search_area=plan["polygon_area_m2"], search_frac=plan["search_frac"],
        npts=len(plan["polygon"]), people=plan["people"],
        people_in=plan["people_inside_padded"],
        extent=plan["map_extent_m"], cell_m=plan["cell_m"],
        local_half=plan["local_half_m"], team_half=plan["team_half_m"],
        pad=plan["pad_m"], search_xy=wrapped,
        threshold=plan["frontier_threshold_points"],
        threshold_m=plan["frontier_threshold_points"] * plan["cell_m"],
        default20=20 * plan["cell_m"],
        robots=n_robots, period=itm_keyframe_period_s(n_robots),
        duty=100.0 * n_robots * 0.069 / 0.2)
    path = os.path.join(out_dir, scene_yaml_name(plan["cell"]))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(body)
    return path


def emit_env_entries(plans, robots):
    """The `environments:` block for the OSMO mission, one entry per
    (cell, method). Printed rather than written: the mission file carries
    prose around it that no generator should own."""
    methods = (("frontier", "", "false", "false"),
               ("lawnmower", "", "false", "false"),
               ("vlfm", "8100", "true", "false"),
               ("conavgpt2_team", "8000", "false", "true"))
    out = []
    for p in plans:
        for name, port, itm, vlm in methods:
            out.append(
                "  - name: {0}_{1}\n"
                "    method: {1}\n"
                "    model_port: \"{2}\"\n"
                "    scene_yaml: {3}\n"
                "    START_DETECTOR_SERVER: \"true\"\n"
                "    START_ITM_SERVER: \"{4}\"\n"
                "    START_VLM_SERVER: \"{5}\"\n"
                "    FROZEN_SCENE: {6}\n"
                "    RESULTS_SCENE: {7}\n"
                "    SPAWN_CONFIGS: >-\n"
                "      {8}\n".format(
                    p["scene"].lower(), name, port,
                    scene_yaml_name(p["cell"]), itm, vlm,
                    p["cell"], p["scene"],
                    spawn_configs_json(p["spawns"], p["polygon"],
                                       indent="       ")))
    return "".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default=DEFAULT_ROOT,
                    help="dataset root on THIS machine (default %(default)s)")
    ap.add_argument("--cell", action="append",
                    help="<Disaster>/<Locale>/level_<n>/<k>; repeatable. "
                         "Default: the six built suburban cells")
    ap.add_argument("--robots", type=int, default=8)
    ap.add_argument("--pad", type=float, default=50.0,
                    help="search_area_pad_m the planner will apply (default 50)")
    ap.add_argument("--clear", type=float, default=10.0,
                    help="metres of free ground a spawn needs (default 10)")
    ap.add_argument("--sector-mode", default="rect")
    ap.add_argument("--sector-axis", default="principal")
    ap.add_argument("--step", type=float, default=2.5,
                    help="spawn search lattice, metres (default 5)")
    ap.add_argument("--edge", type=float, default=15.0,
                    help="keep spawns this far inside the sector (default 15)")
    ap.add_argument("--margin", type=float, default=40.0,
                    help="metres of map half-extent beyond what is required")
    ap.add_argument("--emit-yaml", metavar="DIR",
                    help="write one scene overlay per cell into DIR (normally "
                         "robot/ros_ws/src/global/planners/search_baselines/"
                         "config) and print the mission's environments: block")
    ap.add_argument("--json", action="store_true", help="dump the full plan")
    args = ap.parse_args()

    cells = args.cell or DEFAULT_CELLS
    plans = []
    for c in cells:
        p = plan_cell(args.root, c, args.robots, args.pad, args.clear,
                      args.sector_mode, args.sector_axis, args.step, args.edge,
                      args.margin)
        plans.append(p)
        print("=" * 78)
        print("{0}   {1}".format(p["cell"], p["scene"]))
        args_pad = args.pad
        print("  damage hull   {0:,.0f} m2 ({1:.0%} of the plate)".format(
            p["damage_area_m2"], p["plate_frac"]))
        print("  search area   {0:,.0f} m2 ({1:.0%}), {2} pts  "
              "= hull + {3:.0f} m pad, clipped to the plate".format(
                  p["polygon_area_m2"], p["search_frac"], len(p["polygon"]),
                  args_pad))
        print("  people        {0} total, {1} inside the search area"
              .format(p["people"], p["people_inside_padded"]))
        print("  obstacles     {0} boxes".format(p["obstacles"]))
        print("  sectors       {0} x {1}/{2}, {3:,.0f}-{4:,.0f} m2".format(
            args.robots, args.sector_mode, args.sector_axis,
            min(p["sector_areas"]), max(p["sector_areas"])))
        print("  map_extent_m  {0:.0f}  ({1:.2f} m/cell; local half {2:.0f} m, "
              "team half {3:.0f} m)".format(
                  p["map_extent_m"], p["cell_m"], p["local_half_m"],
                  p["team_half_m"]))
        print("  frontier_threshold_points {0}".format(
            p["frontier_threshold_points"]))
        for i, (x, y) in enumerate(p["spawns"], start=1):
            print("    robot_{0}  ({1:8.1f}, {2:8.1f})  clear {3:4.1f} m  "
                  "{4:<12s} sector {5:,.0f} m2".format(
                      i, x, y, p["spawn_clearance_m"][i - 1],
                      p["spawn_mode"][i - 1], p["sector_areas"][i - 1]))
        print("  SPAWN_CONFIGS: >-")
        print("    " + spawn_configs_json(p["spawns"], p["polygon"]))
    if args.emit_yaml:
        print("=" * 78)
        for p in plans:
            print("wrote {0}".format(
                emit_scene_yaml(p, args.emit_yaml, args.robots)))
        print("\n# --- environments: block for the OSMO mission ---")
        print(emit_env_entries(plans, args.robots))
    if args.json:
        print(json.dumps(plans, indent=1))


if __name__ == "__main__":
    main()
