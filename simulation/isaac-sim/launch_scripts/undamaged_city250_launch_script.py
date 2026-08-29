#!/usr/bin/env python
"""
UNDAMAGED CITY, 250 x 250 m — a downtown block grid built from the two new
asset packs: GreatAmericanCity's 31 whole buildings and CitySample's façade
kit composed into towers.

    CITY_SEED=7 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/city250u \
    ISAAC_SIM_SCRIPT_NAME=undamaged_city250_launch_script.py airstack up isaac-sim

No fire, no fracture, no physics, no settle — this is the clean plate the
damage passes will later run against, and the first look at both packs in a
real layout.

THE TWO PACKS ARRIVE IN COMPLETELY DIFFERENT STATES
-----------------------------------------------------
GREATAMERICANCITY is ready. 31 whole buildings, one mesh with 8-15
`GeomSubset`s each, 36k-823k triangles (median ~240k), 14-86 m footprints and
38.6-312 m tall — six of them over 150 m, taller than anything the library
had. Measured into `_plans/gac_buildings.json`. Every one is authored in
CENTIMETRES, so `SCALE_GAC` is not optional: referencing a cm asset into a
metres stage rescales nothing and one building would swallow the plate.

CITYSAMPLE is not. `All_Buildings_Lineup_Hero.usd` looks like 20 assembled
buildings and is 20 heaps of overlapping panels — Unreal's per-instance
transform arrays did not survive the export, so every module of a tower sits
at one origin (measured: each `InstancedStaticMesh` prim is exactly ONE copy
of its source module, ratio 1.00, and every hero building bounds 4-17 m
tall). `detail/citysample_building` authors the stacking this pack lost, from
the module boxes catalogued in `_plans/citysample_kit.json`; see its
docstring for the ring convention and the point budget.

WEIGHT, MEASURED, BECAUSE IT DECIDES THE MIX
---------------------------------------------
The CitySample modules are Nanite source meshes and the 20 hero buildings are
83.2M triangles between them — against 8.0M for all 31 GreatAmericanCity
buildings. What makes them usable anyway is that a composed tower references
only 2-48 DISTINCT modules however many bays it has, and every placement is
marked `instanceable`, so STORED geometry per composed building is 0.00-1.44M
points. Drawn instances are another matter: `SFJ_A` and `CHG_A` tile a heavy
module tens of times and draw 44-58M points each — `CITY_CS_HEAVY=0` drops
those two if the render needs relief.

Env:
    CITY_REGION      plate size to TRY first, m (default 250)
    CITY_REGION_MAX  ceiling it may grow to (default 500)
    CITY_SEED        (default 7)
    CITY_CS_HEAVY    1 admits the SFJ_A / CHG_A families (default 1)
    CS_MAX_POINTS  per-module budget for the kit (default 60000)
    SNAP_DIR       captures, under /isaac-sim/.nvidia-omniverse/logs/
"""

import json
import math
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(n, d=""):
    v = os.environ.get(n)
    return d if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")

import omni.kit.app                                            # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade        # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

import scene_generator as sg                                   # noqa: E402
from detail import citysample_building as cs                   # noqa: E402
from disaster import damage                                    # noqa: E402

PARENT = "/World/city"
# THE ROOT IS NOT OPTIONAL. `_join_asset_root(path, "")` returns the path
# UNCHANGED, so a relative reference resolves against the anonymous authoring
# layer, fails to open, and leaves an empty prim behind — no error, just a
# building that is not there. It cost a whole run: 31 of the 49 buildings
# were invisible while the log happily printed their measured sizes, because
# the sizes come from the catalogue and never from the stage.
NUC = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
GAC_DIR = ("GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
SCALE_GAC = 0.01

REGION = float(_env("CITY_REGION", "250"))
REGION_MAX = float(_env("CITY_REGION_MAX", "500"))
SEED = int(_env("CITY_SEED", "7"))
CS_HEAVY = _env("CITY_CS_HEAVY", "1") not in ("0", "false")
PACKS = [q.strip() for q in _env("CITY_PACKS", "gac,cs").split(",") if q.strip()]
# a whitelist of GreatAmericanCity buildings, by number: "1,2,3,24"
ONLY = [q.strip().zfill(2) for q in _env("CITY_ONLY", "").split(",") if q.strip()]
CS_INSTANCE = _env("CITY_CS_INSTANCE", "0") not in ("0", "false")
CS_MAX_POINTS = int(_env("CS_MAX_POINTS", str(cs.MAX_POINTS)))
SNAP_DIR = _env("SNAP_DIR", "")

STREET = 13.0         # avenue between one block and the next
CROSS = 10.0          # cross street punched through a row
CROSS_EVERY = 95.0    # run of frontage before a cross street
GAP = 2.5             # gap between neighbours in a row
EDGE = 5.0            # margin at the plate edge
# LONGEST TERRACE. Without a cap the packer builds one row the full width of
# the plate, which swallows every head and tail candidate and strands the
# buildings that can only sit mid-terrace — 12 placed, 3 impossible. Shorter
# terraces mean more of them, each with its own ends, and it reads as blocks
# rather than one continuous 490 m wall.
ROW_M = float(_env("CITY_ROW_M", "240"))
# these two tile a module heavy enough to draw 44-58M points per building
CS_HEAVY_FAMS = ("SFJ_A", "CHG_A")


# ----------------------------------------------------------------- stock ---

def load_stock(rng):
    """[entry] — the pool the packer draws from, both packs interleaved."""
    pool = []
    gac = json.load(open(os.path.join(_SG, "_plans", "gac_buildings.json")))
    faces = {}
    try:
        for f in json.load(open(os.path.join(_SG, "_plans",
                                             "gac_faces.json"))):
            faces[f["name"]] = f
    except Exception:
        pass
    n_gac = 0
    if "gac" in PACKS:
        for r in gac:
            num = r["name"].replace("SM_Building_", "")[:2]
            if ONLY and num not in ONLY:
                continue
            fa = faces.get(r["name"], {})
            pool.append({"kind": "gac", "style": r["name"],
                         "usd": GAC_DIR + r["name"] + ".usd",
                         "W": r["W"], "D": r["D"], "H": r["H"],
                         "cx": r.get("cx", 0.0), "cy": r.get("cy", 0.0),
                         "z0": r.get("z0", 0.0), "tris": r["tris"],
                         "front": fa.get("front", "S"),
                         "blank": list(fa.get("blank_sides", []))})
            n_gac += 1
    kit = cs.load_kit()
    n_cs = 0
    for fam, var in (cs.families(kit) if "cs" in PACKS else []):
        if not CS_HEAVY and (fam + "_" + var) in CS_HEAVY_FAMS:
            continue
        W, D = cs.footprint(kit, fam, var)
        # a target height in the range the pack's own hero buildings occupy;
        # the stack lands on whatever its level heights sum to
        spec = cs.plan_building(kit, fam, var, rng.uniform(38.0, 96.0),
                                max_points=CS_MAX_POINTS)
        if spec is None:
            continue
        pool.append({"kind": "cs", "style": fam + "_" + var, "fam": fam,
                     "var": var, "spec": spec, "W": W, "D": D,
                     "H": spec["H"]})
        n_cs += 1
    return pool, kit, n_gac, n_cs


def _dims(e):
    return e["W"], e["D"], e["H"]


# ------------------------------------------------------------------ plan ---

# ---------------------------------------------------------------- faces ---
# Compass side -> the local outward direction it faces, before any rotation.
_SIDE_DIR = {"E": (1.0, 0.0), "N": (0.0, 1.0),
             "W": (-1.0, 0.0), "S": (0.0, -1.0)}


def face_rules(e, street_dir):
    """(yaw, needs_left, needs_right) for one building on a frontage.

    A KIT BUILDING IS ONLY MODELLED WHERE IT WAS MEANT TO BE SEEN. Measured
    over the pack (`_plans/gac_faces.json`, triangle density per elevation
    against that building's own best): most of these carry their detail on ONE
    elevation and are flat slabs on the other three — `SM_Building_04` runs
    77.8 tri/m2 on E and 0.14 on W. Dropped on an open corner that reads as a
    missing wall, so the layout has to place them the way the artist assumed.

    Two rules come out of it:

      1. ORIENTATION — turn the building so its densest elevation (`front`)
         faces the street. Because these are authored with the back on W and
         the front on E, that also swings the blank W slab into the block
         interior for free.
      2. ADJACENCY — after that rotation a blank elevation may still end up
         pointing ALONG the street. Those must be covered by a neighbour, so
         the building may not sit at that end of its row. A building blank on
         both flanks (7 of the 15 here) can only go mid-row.

    `street_dir` is the world direction the front must face.
    """
    fx, fy = _SIDE_DIR.get(e.get("front", "S"), (0.0, -1.0))
    yaw = math.degrees(math.atan2(street_dir[1], street_dir[0]) -
                       math.atan2(fy, fx))
    ca, sa = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    # along the frontage = street_dir turned +90 deg
    ax, ay = -street_dir[1], street_dir[0]
    need_l = need_r = False
    for b in e.get("blank", []):
        bx, by = _SIDE_DIR[b]
        wx, wy = bx * ca - by * sa, bx * sa + by * ca
        d = wx * ax + wy * ay
        if d > 0.6:
            need_r = True
        elif d < -0.6:
            need_l = True
    return yaw % 360.0, need_l, need_r


def _turned(e, front):
    """(W, D) of a building once `face_rules` has turned it for this row."""
    sd = (0.0, -1.0) if front == "S" else (0.0, 1.0)
    yaw, _, _ = face_rules(e, sd)
    return ((e["D"], e["W"]) if int(round(yaw)) % 180 == 90
            else (e["W"], e["D"]))


def rows_by_rule(ents, usable, rng):
    """Build TERRACES that are legal by construction. Returns [[entry]].

    Ordering a row after the fact does not work: a row can be dealt a hand
    with no legal arrangement at all — 7 of these 15 buildings are blank on
    BOTH flanks and can only ever sit mid-terrace, so a row of nothing but
    those has no valid head or tail. Instead each terrace is STARTED with a
    building that needs no left neighbour and CLOSED with one that needs no
    right neighbour, and the both-flanks buildings are filled in between.

    Semantics here are for a row facing SOUTH. A north-facing row is the same
    row REVERSED, because turning the frontage through 180 degrees swaps which
    flank is on the left.
    """
    pool = list(ents)
    rng.shuffle(pool)
    rows = []
    while pool:
        hi = next((i for i, e in enumerate(pool) if not e["need_l"]), None)
        if hi is None:
            break                      # nothing left that can open a terrace
        head = pool.pop(hi)
        ti = next((i for i, e in enumerate(pool) if not e["need_r"]), None)
        tail = pool.pop(ti) if ti is not None else None
        row, wid = [head], head["W"]
        reserve = (GAP + tail["W"]) if tail else 0.0
        while pool:
            e = pool[0]
            if wid + GAP + e["W"] + reserve > usable:
                break
            pool.pop(0)
            row.append(e)
            wid += GAP + e["W"]
        if tail is not None:
            row.append(tail)
        # CLOSING THE TERRACE IS NOT OPTIONAL. When no tail candidate was
        # left, the fill loop ended the row on whatever came next — and if
        # that building is blank on its right flank it stands exposed at the
        # end of the terrace (`SM_Building_06_Small` at index 0 of 3, once the
        # north row is reversed). Give the offenders back to the pool until
        # the row ends on something that can close it.
        while row and row[-1]["need_r"]:
            pool.insert(0, row.pop())
        if not row:
            break
        rows.append(row)
    # SECOND PASS: a building that needs BOTH flanks covered can never open
    # or close a terrace, so once the head/tail candidates run out it is left
    # over — but it is perfectly placeable in the MIDDLE of a terrace that
    # still has width. Seven of these fifteen are in that class, so without
    # this pass a third of the stock never gets placed.
    for e in list(pool):
        for row in rows:
            wid = sum(q["W"] for q in row) + GAP * (len(row) - 1)
            if wid + GAP + e["W"] > usable or len(row) < 2:
                continue
            row.insert(max(1, len(row) // 2), e)
            pool.remove(e)
            break
    return rows, pool


def _shelf(entries, usable):
    """Split `entries` into rows no wider than `usable`, cross-streets in.

    Returns [([(entry, x_offset_from_row_start)], run_length)].
    """
    rows, cur, run, since = [], [], 0.0, 0.0
    for e in entries:
        W = e["W"]
        sep = 0.0 if not cur else (CROSS if since >= CROSS_EVERY else GAP)
        if cur and run + sep + W > usable:
            rows.append((cur, run))
            cur, run, since, sep = [], 0.0, 0.0, 0.0
        if sep == CROSS:
            since = 0.0
        cur.append((e, run + sep))
        run += sep + W
        since += sep + W
    if cur:
        rows.append((cur, run))
    return rows


def plan_city(pool, rng, region):
    """Bands of back-to-back rows: a BLOCK SIZED TO ITS BUILDINGS.

    THE BLOCK IS NOT A FIXED RECTANGLE ANY MORE, and that is the whole change.
    The earlier packer cut the plate into four fixed blocks and then had to
    REJECT anything that would not fit one — which threw out `SM_Building_31`
    (60.3 x 142.2 m, deeper than a 113 m block) and would have thrown out more
    of GreatAmericanCity's big stock. Real downtowns have buildings that take
    a whole block, so the block is sized to what lands on it instead.

    A band is one city block: a row facing the street below it, a row facing
    the street above it, back to back, and the band is exactly as deep as
    those two rows need. A 142 m-deep building simply makes its band 142 m
    deep and takes that block on its own. Nothing is ever rejected for size.

    Rows are packed to width with a cross street every `CROSS_EVERY` metres of
    run, so the result reads as a grid rather than as one continuous terrace.

    Entries are sorted by DEPTH before packing so a row's buildings are of a
    like depth — a band takes its deepest member, so mixing a 142 m block with
    a 15 m one would leave 127 m of empty block behind the shallow one — and
    the ROW ORDER is then shuffled so the deep bands are spread through the
    city rather than stacked along one edge.
    """
    half = region / 2.0
    usable = region - 2.0 * EDGE
    # TURN EVERY BUILDING BEFORE PACKING. `face_rules` swings the good
    # elevation to the street, and for this stock that is a 90 or 270 degree
    # turn on most of them — which SWAPS width and depth. Shelving on the
    # un-turned width packed rows 515 m wide onto a 490 m plate.
    ent = []
    for e in pool:
        yaw, nl, nr = face_rules(e, (0.0, -1.0))
        tw, td = _turned(e, "S")
        ent.append(dict(e, W=tw, D=td, need_l=nl, need_r=nr))
    ent.sort(key=lambda e: -e["D"])
    rows, leftover = rows_by_rule(ent, min(usable, ROW_M), rng)
    rng.shuffle(rows)

    # TWO PASSES, SO THE CITY IS CENTRED ON THE PLATE. Laying bands from the
    # bottom edge left 110 m of bare ground along the top of a 500 m plate and
    # put the whole city off-centre under the nadir camera.
    bands, total, skipped = [], 0.0, 0
    for i in range(0, len(rows), 2):
        south = rows[i]
        north = list(reversed(rows[i + 1])) if i + 1 < len(rows) else []
        # after `face_rules` a building may stand at 90 or 270 deg, which
        # swaps the footprint it occupies; band depth has to use the turned one
        dS = max((e["D"] for e in south), default=0.0)
        dN = max((e["D"] for e in north), default=0.0)
        if total + dS + dN > (region - 2.0 * EDGE):
            skipped += len(south) + len(north)
            continue
        bands.append((south, dS, north, dN))
        total += dS + dN + STREET
    total = max(0.0, total - STREET)

    placed, y, n_row = [], -total / 2.0, [0]
    for (south, dS, north, dN) in bands:
        band = dS + dN
        for row, front in ((south, "S"), (north, "N")):
            sd = (0.0, -1.0) if front == "S" else (0.0, 1.0)
            ordered = [dict(e, yaw=face_rules(e, sd)[0]) for e in row]
            # NO CROSS STREETS IN A ROW UNDER THESE RULES. A blank flank is
            # only covered by a neighbour that is actually there, and a gap
            # punched mid-row exposes it again — so the row is continuous and
            # the cross streets are the ones between BANDS.
            wid = (sum(e["W"] for e in ordered)
                   + GAP * max(0, len(ordered) - 1))
            x0, t = -wid / 2.0, 0.0
            for k, e in enumerate(ordered):
                cx = x0 + t + e["W"] / 2.0
                cy = (y + e["D"] / 2.0 if front == "S"
                      else y + band - e["D"] / 2.0)
                # row id and index, so the placement can be AUDITED — a
                # building's flank neighbour is the one beside it IN ITS ROW,
                # and rows are aligned at the front, so comparing y centres
                # calls a legal pair exposed whenever their depths differ
                placed.append(dict(e, x=cx, y=cy, row=n_row[0], idx=k,
                                   row_n=len(ordered)))
                t += e["W"] + GAP
            n_row[0] += 1
        y += band + STREET
    if leftover:
        print("[city] {0} building(s) could not open or close a terrace: {1}"
              .format(len(leftover), ", ".join(e["style"] for e in leftover)))
    return placed, skipped + len(leftover)


# ----------------------------------------------------------------- build ---

def ground_and_light(stage, span):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.76, 0.81, 0.92))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(3400.0)
    key.CreateAngleAttr(0.7)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-34.0, 0.0, 28.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = max(500.0, span * 1.8)
    g.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                        Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateExtentAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, e, 0)])
    # A `displayColor` QUAD TAKES THE DEFAULT SPECULAR SURFACE and renders as
    # a mirror under a dome; bind a real material or the city floats on glass.
    try:
        m = damage._pbr(stage, "/World/Looks/asphalt", (0.030, 0.030, 0.031),
                        0.93)
        UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(m)
    except Exception:
        g.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.28, 0.27)])
    return damage._pbr(stage, "/World/Looks/roofslab", (0.045, 0.044, 0.042),
                       0.90)


def place_gac(stage, path, e):
    """One GreatAmericanCity building, centred on (x, y)."""
    holder = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    # A TYPELESS CHILD TAKES THE REFERENCE. Referencing an asset whose default
    # prim is a Mesh onto a prim already declared Xform leaves the local type
    # winning: the composed prim holds mesh attributes, is not a Mesh, and
    # bounds 0 x 0 x 0 — how every DownTown block measured as nothing.
    kid = stage.DefinePrim(Sdf.Path(path + "/asset"))
    kid.GetReferences().AddReference(sg._join_asset_root(e["usd"], NUC))
    stage.Load(Sdf.Path(path))
    xf = UsdGeom.Xformable(holder)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(e["x"]), float(e["y"]), 0.0))
    xf.AddRotateZOp().Set(float(e["yaw"]))
    xf.AddScaleOp().Set(Gf.Vec3f(SCALE_GAC, SCALE_GAC, SCALE_GAC))
    return holder.GetPrim()


def main():
    t0 = time.time()
    rng = random.Random(SEED)
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    w = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(w.GetPrim())
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))

    pool, kit, n_gac, n_cs = load_stock(rng)
    print("[city] stock: {0} GreatAmericanCity + {1} CitySample families"
          .format(n_gac, n_cs))
    # THE PLATE GROWS UNTIL THE STOCK FITS. The 49 distinct assets carry
    # 116,764 m2 of footprint between them — 187% of a 250 m plate before a
    # single street is drawn, and 47% of a 500 m one, which is about right
    # for a dense downtown.
    region = REGION
    placed, skipped = plan_city(pool, rng, region)
    while skipped and region < REGION_MAX:
        region = min(REGION_MAX, region * 2.0)
        print("[city] {0} would not fit a {1:.0f} m plate — growing to {2:.0f} m"
              .format(skipped, region / 2.0, region))
        placed, skipped = plan_city(pool, rng, region)
    span = region
    print("[city] placing {0} buildings on a {1:.0f} m plate ({2} left out)"
          .format(len(placed), region, skipped), flush=True)
    roof_mat = ground_and_light(stage, span)

    n_prim = n_pts = n_miss = 0
    tall = 0.0
    for i, e in enumerate(placed):
        t1 = time.time()
        path = "{0}/b{1}".format(PARENT, i)
        if e["kind"] == "gac":
            pr = place_gac(stage, path, e)
            # AN UNRESOLVED REFERENCE IS SILENT, so check rather than trust.
            nres = sum(1 for q in Usd.PrimRange(pr) if q.IsA(UsdGeom.Mesh))
            n_miss += 0 if nres else 1
            note = "{0:.0f}k tris{1}".format(e["tris"] / 1000.0,
                                             "" if nres else "  << NOT LOADED")
            H = e["H"]
        else:
            r = cs.build(stage, path, kit, e["spec"], e["x"], e["y"],
                         e["W"], e["D"], yaw=e["yaw"], tag="m",
                         material=roof_mat)
            n_prim += r["prims"]
            n_pts += r["points"]
            H = r["H"]
            note = "{0} lvl, {1} prims, {2:.1f}M pts drawn".format(
                r["levels"], r["prims"], r["points"] / 1e6)
        tall = max(tall, H)
        print("  {0:<22} {1:<4} {2:5.1f} x {3:5.1f} x {4:6.1f} m   {5}  ({6:.1f}s)"
              .format(e["style"][:22], e["kind"], e["W"], e["D"], H, note,
                      time.time() - t1), flush=True)
        if i % 6 == 0:
            omni.kit.app.get_app().update()

    for _ in range(90):
        omni.kit.app.get_app().update()

    n_gac_p = sum(1 for e in placed if e["kind"] == "gac")
    print("\n" + "=" * 74)
    print("UNDAMAGED CITY  {0:.0f} x {0:.0f} m".format(span))
    print("  {0} buildings: {1} GreatAmericanCity, {2} CitySample composed"
          .format(len(placed), n_gac_p, len(placed) - n_gac_p))
    if n_miss:
        print("  !! {0} building(s) FAILED TO LOAD".format(n_miss))
    print("  {0} distinct assets   tallest {1:.0f} m   built in {2:.0f} s"
          .format(len({e["style"] for e in placed}), tall, time.time() - t0))
    print("  CitySample: {0} module placements, {1:.1f}M points drawn "
          "(instanced)".format(n_prim, n_pts / 1e6))
    print("=" * 74 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            vspan = span * 1.25
            # NADIR MUST CLEAR THE TALLEST ROOF — a 312 m tower on the plate
            # otherwise turns the plumb shot into a close-up of one roof.
            sn.place_camera(stage, (0.0, 0.0, tall + vspan / 1.164),
                            (0.0, 0.0, 0.0))
            sn.snapshot(os.path.join(SNAP_DIR, "city_top.png"))
            for nm, (ax, ay) in (("sw", (-1, -1)), ("se", (1, -1)),
                                 ("ne", (1, 1)), ("nw", (-1, 1))):
                d = span * 1.05
                sn.place_camera(stage, (ax * d, ay * d, 0.55 * d + tall * 0.4),
                                (0.0, 0.0, tall * 0.30))
                sn.snapshot(os.path.join(SNAP_DIR, "city_" + nm + ".png"))
            for nm, eye, tgt in (
                    ("street_ew", (-span * 0.62, 0.0, 4.0),
                     (span * 0.4, 0.0, 16.0)),
                    ("street_ns", (0.0, -span * 0.62, 4.0),
                     (0.0, span * 0.4, 16.0))):
                sn.place_camera(stage, eye, tgt)
                sn.snapshot(os.path.join(SNAP_DIR, nm + ".png"))
            print("[city] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[city] snapshots FAILED: {0}".format(exc))

    print("UNDAMAGED CITY DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
