"""fence_png.py — top-down plan of the LOT FENCES, with every defect marked.

    python3 tools/fence_png.py --seed 3 --out _plans/fences.png
    python3 tools/fence_png.py --seed 3 --zoom -640,500,90 --out _plans/zoom.png

WHY THIS EXISTS
---------------
Fences are the one thing in the suburb that is judged by whether two objects a
metre apart line up, and that is not a question a flythrough answers reliably —
you see a mess, you cannot say which of four causes it is. So every module is
drawn as its TRUE ORIENTED RECTANGLE, not as a line, and anything that fails one
of the four invariants is coloured:

    red      the module is on the carriageway (`_RoadIndex`, the positive test)
    magenta  the module is inside a cul-de-sac turnaround disc
    orange   the module CROSSES another module — hairline cores intersect
    cyan     the module is DOUBLED — a second fence parallel to it, within a
             metre, overlapping it along the line
    pink     the module stands ACROSS A DRIVEWAY — its core intersects the
             paving that `apply_ground` will lay for that lot (the house
             plan's drive when the kit supplies one, else the plat's)
    violet   a SEATING PROP standing in a back yard that is neither fenced
             nor tree-screened — see THE YARD LAYER below
    red dot  a DANGLING END — a module end that meets nothing: no other
             module, no wall, no garage. Two of these a metre apart on one
             line are the gap the eye reads between neighbours' fences.

Drives are drawn as blue-grey ribbons and front walks as thinner ones, so a
fence that ought to break for them can be seen not to.

A clean plan is grey-green fences on white lots, drives passing through
openings, and nothing else.

THE YARD LAYER
--------------
A FENCE IS ONLY EVER JUDGED BY WHAT IT ENCLOSES, and for most of this tool's
life `build` stopped one call short of being able to show that. It ran the plat
and `build_placements` and then returned — so the plate showed the fence and not
the garden, and the defect the user actually reported (a bench standing in a
back yard wide open to the block behind it) was invisible on every plate this
tool had ever drawn. `suburb_yardplan` imports `math`, `suburb_yards` and
`suburb_net` and nothing else — no pxr, no stage — so the whole yard pass runs
on the host as-is, and `build` now calls it.

What that buys the picture, drawn under the fences:

    pale green wash    the lot's enclosure — the region behind its BUILDING
                       LINE, which contains the house — closed by fence alone
    pale olive wash    ...closed by a treeline instead
    no wash            a back garden that is open ground, or a lot with no
                       garden behind its back wall at all
    green discs        every canopy the yard pass planted, at the crown radius
                       the SCREEN WALK itself used, so a wash you can see is a
                       wash you can check against the trees drawing it
    grey-green discs   the parcel pass's own verge and lot trees. Drawn paler
                       because they are real trees that the seating gate does
                       NOT count — `suburb_yardplan` screens a yard with the
                       canopies it planted itself and indexes nothing else — so
                       a yard ringed in grey-green and washed as open is the
                       code behaving as written, not a plate that disagrees
                       with itself.
    blue squares       a seating prop — table set, bench or bin
    violet squares     ...standing in a yard that is neither.

DRAWING 4,700 CANOPIES WITHOUT LOSING THE FENCE. The two tree passes put ~3,500
and ~1,200 discs on a 1600 x 1200 m plate, at 1.5-13 m radius; drawn as opaque
patches they cover the lot lines, the drives and every fence module on the
plate. They go down as one `EllipseCollection` each, at low alpha, with no edge
in the overview, UNDER the houses and the drives and well under the fences —
the fence stays the top thing on the plate, which is the point of the plate.

MODULE SIZES ARE MEASURED, NOT DECLARED. `_plans/fence_modules_measured.json`
(written by tools/measure_fences.py from the USD files themselves, under a
headless Isaac python) is read first; the asset-set comments and the objaverse
manifest are the fallback, and the title says which was used.

WHY IT RUNS ON THE HOST
-----------------------
Same trick as `tools/plan_png.py`: `suburb_scene` imports `pxr` at module scope
but the fence path never touches a USD API, so the module is stubbed and the
footprint questions are answered from measurements instead of from a stage.
:func:`build` is the shared entry point — `tools/fence_check.py` asserts against
exactly the scene this draws, so the picture and the test can never disagree.

WHAT IT IS NOT
--------------
A render. Heights are read but not drawn, and the modules are shown at their
measured plan footprint, which for a picket is a 9 cm ribbon — visible in the
overview only as a line. Use ``--zoom`` for anything about how two runs meet.
"""

import argparse
import collections
import glob
import json
import math
import os
import random
import struct
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN)
sys.path.insert(0, _HERE)

# `suburb_scene` and `scene_generator` import pxr at module scope; the fence
# path never calls into it. Same stub as plan_png.py, and for the same reason.
for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom",
           "pxr.UsdShade", "pxr.UsdSkel", "pxr.Vt", "pxr.UsdPhysics"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdShade", "UsdSkel", "Vt",
           "UsdPhysics"):
    setattr(sys.modules["pxr"], _n, types.ModuleType(_n))

import yaml                                                        # noqa: E402
import scene_generator as sg                                       # noqa: E402
import suburb_scene as ss                                          # noqa: E402
from compile_disaster import (resolve_config_path, compile_spec,   # noqa: E402
                              DEFAULT_BASE)
from layout import suburb_net as sn                                # noqa: E402
from detail import suburb_parcel as sp                             # noqa: E402
from detail import suburb_yardplan as yp                           # noqa: E402
from plan_png import measured_sizes, StubResolver                  # noqa: E402


# ---------------------------------------------------------------------------
# how big is a fence module, without a stage
# ---------------------------------------------------------------------------

def _glb_size(uid, target_m, fit="footprint"):
    """Predict an Objaverse asset's converted bbox from its SOURCE glb.

    `plan_png.measured_sizes` scrapes `# a x b x c m` comments and only matches
    entries written as a `.usd` path, so it never sees an `objaverse://<uid>`
    one; those are read from `assets/objaverse/manifest.yaml` instead. This is
    the third case: an entry the asset set names but `prepare_assets.py` has not
    converted yet, which is exactly the state the repo is in the moment a new
    fence is added to `lot_fences`. Without it the tool sizes a brand-new panel
    from `fallback_sizes` and draws a fiction.

    Pure Python — the build host has neither trimesh nor pxr. glTF accessors
    carry POSITION min/max, so the bbox is those corners pushed through each
    node's matrix. Validated against the two cached fences: it reproduces their
    manifest bboxes exactly. Source glTF is Y-up and the converter writes Z-up,
    hence the axis swap on the way out.
    """
    hits = glob.glob(os.path.expanduser(
        "~/.objaverse/hf-objaverse-v1/glbs/*/%s.glb" % uid))
    if not hits:
        return None
    with open(hits[0], "rb") as fh:
        if struct.unpack("<III", fh.read(12))[0] != 0x46546C67:
            return None
        ln, _ty = struct.unpack("<II", fh.read(8))
        g = json.loads(fh.read(ln))

    def mul(a, b):
        return [sum(a[i * 4 + k] * b[k * 4 + j] for k in range(4))
                for i in range(4) for j in range(4)]

    def trs(node):
        if "matrix" in node:                    # glTF matrices are column-major
            m = node["matrix"]
            return [m[j * 4 + i] for i in range(4) for j in range(4)]
        t = node.get("translation", [0, 0, 0])
        x, y, z, w = node.get("rotation", [0, 0, 0, 1])
        s = node.get("scale", [1, 1, 1])
        rot = [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0,
               2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0,
               2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0,
               0, 0, 0, 1]
        return mul([1, 0, 0, t[0], 0, 1, 0, t[1], 0, 0, 1, t[2], 0, 0, 0, 1],
                   mul(rot, [s[0], 0, 0, 0, 0, s[1], 0, 0, 0, 0, s[2], 0,
                             0, 0, 0, 1]))

    lo, hi = [1e30] * 3, [-1e30] * 3

    def walk(ni, mat):
        node = g["nodes"][ni]
        mat = mul(mat, trs(node))
        if "mesh" in node:
            for prim in g["meshes"][node["mesh"]]["primitives"]:
                acc = g["accessors"][prim["attributes"]["POSITION"]]
                for bits in range(8):
                    p = [(acc["max"] if (bits >> k) & 1 else acc["min"])[k]
                         for k in range(3)]
                    for i in range(3):
                        v = (mat[i * 4] * p[0] + mat[i * 4 + 1] * p[1]
                             + mat[i * 4 + 2] * p[2] + mat[i * 4 + 3])
                        lo[i] = min(lo[i], v)
                        hi[i] = max(hi[i], v)
        for c in node.get("children", ()):
            walk(c, mat)

    ident = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    for ni in g["scenes"][g.get("scene", 0)]["nodes"]:
        walk(ni, ident)
    ext = [hi[i] - lo[i] for i in range(3)]
    ref = {"footprint": max(ext[0], ext[2]), "height": ext[1],
           "max": max(ext)}[fit]
    if ref <= 0:
        return None
    k = float(target_m) / ref
    return (ext[0] * k, ext[2] * k, ext[1] * k)      # Y-up source -> Z-up world


MEASURED_JSON = os.path.join(_SCENE_GEN, "_plans", "fence_modules_measured.json")
SIZE_SOURCE = {"measured": 0, "declared": 0}


def _sizes(cfg):
    """`{basename: (sx, sy, sz)}` for everything the scene can ask about."""
    sets = os.path.join(_SCENE_GEN, "config", "asset_sets")
    out = measured_sizes([os.path.join(sets, f) for f in os.listdir(sets)
                          if f.endswith(".yaml")])
    SIZE_SOURCE["declared"] = len(out)
    man_path = os.path.join(_SCENE_GEN, "assets", "objaverse", "manifest.yaml")
    man = {}
    if os.path.exists(man_path):
        man = (yaml.safe_load(open(man_path)) or {}).get("assets") or {}
    for uid, e in man.items():
        s = e.get("size_m")
        if s:
            out[uid + ".usdc"] = (float(s[0]), float(s[1]), float(s[2]))
    # Objaverse entries the asset set names but the cache has not converted.
    for raw in (cfg.get("usds", {}) or {}).values():
        for entry in (raw if isinstance(raw, list) else []):
            if not isinstance(entry, dict):
                continue
            u = str(entry.get("usd", ""))
            if not u.startswith("objaverse://"):
                continue
            uid = u.split("//", 1)[1]
            if uid + ".usdc" in out or not entry.get("target-size-m"):
                continue
            sz = _glb_size(uid, float(entry["target-size-m"]),
                           entry.get("fit", "footprint"))
            if sz:
                out[uid + ".usdc"] = sz
                print(f"[fence_png] {uid[:8]}… not converted yet — sized "
                      f"{sz[0]:.2f} x {sz[1]:.2f} x {sz[2]:.2f} m from its "
                      f"source glb (run prepare_assets.py)")
    # THE REAL THING, when it exists: bboxes read off the USD files under a
    # headless Isaac python (tools/measure_fences.py). Overrides the comments
    # and the manifest for every fence module it covers.
    if os.path.exists(MEASURED_JSON):
        meas = (json.load(open(MEASURED_JSON)) or {}).get("modules") or {}
        for u, rec in meas.items():
            # `footprint` IS `scene_generator._measure_footprint`'s answer at the
            # entry's own scale and axis-up — the same call the build makes — so
            # it is the only field worth reading. The old `resolver` field it
            # replaced was written by a `SizeResolver` that had fallen back, and
            # every module in it read 4.00 x 4.00 x 3.00 m: `fallback_sizes`
            # ["fence"], stamped "MEASURED from USD" in this tool's own title.
            # A fallback is not a measurement, so an entry that has not got the
            # real thing is SKIPPED here rather than trusted.
            fp = rec.get("footprint") or {}
            if all(isinstance(fp.get(k), (int, float)) for k in ("sx", "sy", "sz")):
                sz = (float(fp["sx"]), float(fp["sy"]), float(fp["sz"]))
            elif (rec.get("raw") or {}).get("size_m"):
                sz = tuple(float(v) for v in rec["raw"]["size_m"])
            else:
                continue
            name = os.path.basename(str(u))
            if name in out and out[name] == sz:
                continue                      # already recorded under its twin key
            out[name] = sz
            SIZE_SOURCE["measured"] += 1
            print(f"[fence_png] {name[:44]}: "
                  f"{sz[0]:.3f} x {sz[1]:.3f} x {sz[2]:.3f} m MEASURED from USD")
    return out


# ---------------------------------------------------------------------------
# the scene
# ---------------------------------------------------------------------------

def build(seed=None, config_name="suburb_net", house_instances=None,
          region_m=None):
    """The suburb, up to and including the YARD PASS, on the host.

    Mirrors `suburb_scene.generate_suburb_on_stage` step for step down to the
    rng seeding, minus everything that needs a stage: the park, the frontage
    props and the ground. Those cannot move a fence.

    THE YARD PASS IS NOT ONE OF THEM, and leaving it out is what made the plate
    unable to show the defect that started this work. `suburb_yardplan` is
    geometry and the standard library — `math`, `suburb_yards`, `suburb_net` —
    so it runs here exactly as it runs in the build, and it is what plants the
    trees that screen a garden and the seating that must not stand in an open
    one.

    IT RUNS LAST, ON THE SAME `rng`, AND THAT ORDERING IS THE WHOLE CARE OF IT.
    `yp.plan` draws thousands of times from the shared generator; called before
    `build_placements`, or handed a generator of its own, every downstream draw
    in the suburb moves and the plate stops being a picture of the scene it
    claims to check. Called here — after the last placement is laid, with
    nothing after it — it cannot move a fence, and the fence module count is
    unchanged to the module: 2846 / 3262 / 3478 / 3896 on seeds 1 / 3 / 5 / 7,
    before and after.

    WHAT IT STILL IS NOT THE REAL SCENE'S YARD. The build runs the PARK between
    `build_placements` and `yp.plan`, and the park draws from the same rng — so
    the species and the stations here are a different sample from the ones the
    stage gets. Every question this tool and `fence_check` ask is a question
    about a suburb, not about that suburb: "does a seated lot have an
    enclosure" has to hold for every draw or it does not hold at all. A plate
    that had to match the stage tree for tree would have to build the park, and
    the park needs a stage.

    `pool_holes` IS CAPTURED RATHER THAN THROWN AWAY. `build_placements` fills
    it with every swimming pool the kit stamped, and `generate_suburb_on_stage`
    hands that same list to `yp.plan` as `keepout_rings`; passing `[]` here
    instead would plant 452 trees (seed 3) inside water that the build keeps
    them out of, and each of those is a canopy that can screen a yard the real
    one does not.

    *region_m* OVERRIDES THE PRESET'S OWN EXTENT, for a caller that wants the
    real code path at a size a test suite can afford. `suburb_net.generate` is
    14.4 s of the 16 s this function costs on the shipped 1600 x 1200 m preset —
    the street network is the expensive half and it is superlinear in area — so
    `tests/` builds a few hundred metres of suburb and gets the same passes,
    the same records and the same invariants in about a second. Nothing else is
    touched: at the default `None` this is the shipped preset exactly, which is
    what `fence_check` and the plates run.
    """
    path = resolve_config_path(config_name)
    cfg = compile_spec(yaml.safe_load(open(path)),
                       yaml.safe_load(open(DEFAULT_BASE)))
    cfg = sg.resolve_asset_set(cfg, path)
    cfg["measure_usds"] = False
    if seed is not None:
        cfg["seed"] = int(seed)
    res = StubResolver(_sizes(cfg), cfg.get("fallback_sizes"))

    rng = random.Random(int(cfg.get("seed", 0)) + 7717)
    if region_m is not None:
        cfg.setdefault("layout", {})["region_m"] = [float(region_m[0]),
                                                    float(region_m[1])]
    region = (cfg.get("layout", {}) or {}).get("region_m") or [1600.0, 1200.0]
    net, blocks, info = sn.generate(float(region[0]), float(region[1]), rng,
                                    dict(cfg.get("suburb_net") or {}))
    buildable = [b for b in blocks if not b.get("undeveloped")]

    pools = ss.AssetPools(cfg)
    yaw_off = float((cfg.get("suburb_parcel") or {}).get("house_yaw_offset_deg", -90.0))
    catalogue = ss.modular_catalogue(cfg)
    # THE SAME CONFIG THE BUILD RUNS THE PLAT WITH — see `parcel_config`. This
    # tool used to carry its own copy of that block, and the copy never
    # installed `front_openings`, so it drew every front fence unbroken.
    pcfg = ss.parcel_config(cfg, net, catalogue)
    parcels = sp.parcel_blocks(buildable, rng, pcfg)
    # `house_instances` IS THE ASSEMBLY CONTRACT, passed straight through.
    # Given a list, `build_placements` records each house's (style, pose) into
    # it and does NOT emit that house's ~28 kit modules — which is exactly
    # what a host-side tool reasoning about BUILDINGS wants, and the only way
    # to count them correctly: the modules carry no per-building id, so
    # counting placements counts modules and over-reports by more than an
    # order of magnitude. Default None keeps every existing caller unchanged.
    # `apply_ground` discs every lollipop end at this radius; the fence pass
    # and this tool both have to disc the same ground or a module the build
    # keeps out of a turnaround is drawn standing in one (and vice versa).
    bulb_r = float((cfg.get("suburb_net") or {}).get(
        "bulb_radius_m", sn.DEFAULTS["bulb_radius_m"]))
    pool_holes = []
    placements = ss.build_placements(cfg, res, parcels, rng, pools,
                                     yaw_off=yaw_off, catalogue=catalogue,
                                     pool_holes_out=pool_holes, net=net,
                                     house_instances=house_instances)
    yard, ystats = yp.plan(cfg, parcels, rng, resolver=res,
                           keepout_discs=pcfg.get("keepout_discs"),
                           keepout_rings=pool_holes)
    return {"cfg": cfg, "res": res, "pools": pools, "net": net,
            "house_instances": house_instances or [],
            "blocks": blocks, "info": info, "parcels": parcels,
            "placements": placements, "yard": yard, "ystats": ystats,
            "pool_holes": pool_holes,
            "discs": [((float(c[0]), float(c[1])), float(r))
                      for (c, r) in pcfg["keepout_discs"]],
            "bulb_r": bulb_r, "seed": int(cfg.get("seed", 0))}


def modules(scene):
    """Every fence placement as ``{box, x, y, ang, L, T, usd}``.

    The box is the module's TRUE oriented rectangle: measured length and
    thickness, both multiplied by the `fit` scale `_fence_run` folded into the
    placement, at the geometric run angle with the asset's own `yaw-offset`
    taken back off. Anything less than that and an overlap test is guessing.
    """
    res, pools = scene["res"], scene["pools"]
    for t in ("low", "privacy"):                    # register scale/yaw/axis
        pools.load_tagged(ss._raw_pool(scene["cfg"], "lot_fences"), t)
    out = []
    for p in scene["placements"]:
        if p.get("category") != "fence":
            continue
        u = p["usd"]
        ln, th, _h, fix = ss._fence_module(res, pools, u)
        fit = p["scale"] / max(pools.scale_of(u), 1e-9)
        # The run direction, recovered. `build_placements` writes
        # `fyaw + fix - yaw_of` and `place` adds `yaw_of` back, so the stored
        # yaw is `fyaw + fix` and the module's LONG axis is what `fix` turned
        # onto the run — subtract it to get the boundary back.
        ang = math.radians(p["yaw_deg"] - fix)
        out.append({"x": p["x_m"], "y": p["y_m"], "ang": ang,
                    "L": ln * fit, "T": th * fit, "usd": u,
                    "box": sp._corners(p["x_m"], p["y_m"], ln * fit, th * fit,
                                       math.cos(ang), math.sin(ang))})
    return out


def _ends(m):
    """The module's centre LINE — the two points its axis runs between."""
    hx, hy = math.cos(m["ang"]) * m["L"] / 2.0, math.sin(m["ang"]) * m["L"] / 2.0
    return ((m["x"] - hx, m["y"] - hy), (m["x"] + hx, m["y"] + hy))


def _core(m, eps=None):
    """The hairline ribbon `_FenceGrid` reasons about. See its docstring."""
    eps = ss._FenceGrid._CORE_M if eps is None else eps
    return sp._corners(m["x"], m["y"], max(0.1, m["L"] - 2 * eps), eps,
                       math.cos(m["ang"]), math.sin(m["ang"]))


def classify(scene, mods, cell=8.0):
    """Index sets for the four invariants, keyed by defect name.

    The thresholds for `crossing` and `doubled` are `_FenceGrid`'s own, imported
    rather than restated: this is a regression test on a stated guarantee, and a
    second copy of the numbers is a second thing to keep in step.
    """
    road = ss._RoadIndex(scene["net"])
    bulb_r = scene["bulb_r"]
    bad = {"on_road": set(), "in_bulb": set(), "crossing": set(),
           "doubled": set(), "on_drive": set(on_drive(scene, mods))}

    for i, m in enumerate(mods):
        if road.on_road((m["x"], m["y"])):
            bad["on_road"].add(i)
        for (c, _r) in scene["discs"]:
            if math.hypot(m["x"] - c[0], m["y"] - c[1]) < bulb_r:
                bad["in_bulb"].add(i)
                break

    grid = collections.defaultdict(list)
    reach = ss._FenceGrid._DOUBLE_M
    for i, m in enumerate(mods):
        box = sp._corners(m["x"], m["y"], m["L"] + 2 * reach, 2 * reach,
                          math.cos(m["ang"]), math.sin(m["ang"]))
        for gx in range(int(math.floor(min(q[0] for q in box) / cell)),
                        int(math.floor(max(q[0] for q in box) / cell)) + 1):
            for gy in range(int(math.floor(min(q[1] for q in box) / cell)),
                            int(math.floor(max(q[1] for q in box) / cell)) + 1):
                grid[(gx, gy)].append(i)
    cores = [_core(m) for m in mods]
    seen = set()
    for ids in grid.values():
        for ii in range(len(ids)):
            for jj in range(ii + 1, len(ids)):
                i, j = ids[ii], ids[jj]
                if (i, j) in seen:
                    continue
                seen.add((i, j))
                a, b = mods[i], mods[j]
                if sp._obb_overlap(cores[i], cores[j]):
                    bad["crossing"].add(i)
                    bad["crossing"].add(j)
                    continue
                ca, sa = math.cos(a["ang"]), math.sin(a["ang"])
                cb, sbn = math.cos(b["ang"]), math.sin(b["ang"])
                if abs(ca * cb + sa * sbn) < ss._FenceGrid._DOUBLE_COS:
                    continue
                dx, dy = b["x"] - a["x"], b["y"] - a["y"]
                # Symmetric, exactly as `_FenceGrid.free` computes it — see the
                # comment there on why a frame-relative overlap is order-
                # dependent and therefore untestable.
                along = min(abs(dx * ca + dy * sa), abs(dx * cb + dy * sbn))
                if ((a["L"] + b["L"]) / 2.0 - along
                        <= ss._FenceGrid._DOUBLE_OVERLAP_M):
                    continue
                if sn.seg_seg_dist(_ends(a)[0], _ends(a)[1], _ends(b)[0],
                                   _ends(b)[1]) < ss._FenceGrid._DOUBLE_M:
                    bad["doubled"].add(i)
                    bad["doubled"].add(j)
    return bad


def drives(scene):
    """Every drive and front walk the ground pass will lay, as ribbons.

    THE SAME RULE `apply_ground` / `_paving_keepout` use: the house plan's
    drive when the kit stamped one (an attached garage), else the plat's own
    `drives[di]`. Whatever is drawn here is what gets paved.
    """
    out = []
    for p in scene["parcels"]:
        hs = p.get("houses") or []
        for di, d in enumerate(p.get("drives") or ()):
            h = hs[di] if di < len(hs) else None
            plan = (h or {}).get("plan")
            if plan and plan.get("drive"):
                run, src = plan["drive"], "plan"
            else:
                run, src = (d["a"], d["b"]), "plat"
            out.append({"a": tuple(run[0]), "b": tuple(run[-1]),
                        "w": float(d.get("w", 3.0)), "src": src,
                        "house": h, "kind": "drive"})
            if plan and plan.get("path"):
                out.append({"a": tuple(plan["path"][0]),
                            "b": tuple(plan["path"][-1]), "w": ss.WALK_W_M,
                            "src": "plan", "house": h, "kind": "walk"})
    return out


def _ribbon_box(r):
    ax, ay = r["a"]
    bx, by = r["b"]
    L = math.hypot(bx - ax, by - ay)
    if L < 1e-6:
        return None
    return sp._corners((ax + bx) / 2.0, (ay + by) / 2.0, L, r["w"],
                       (bx - ax) / L, (by - ay) / L)


def on_drive(scene, mods):
    """Indices of modules whose hairline core crosses a drive ribbon."""
    boxes = [(_ribbon_box(r), r) for r in drives(scene) if r["kind"] == "drive"]
    boxes = [(b, r) for b, r in boxes if b is not None]
    cores = [_core(m) for m in mods]
    grid = collections.defaultdict(list)
    for i, (b, _r) in enumerate(boxes):
        for gx in range(int(math.floor(min(q[0] for q in b) / 16.0)),
                        int(math.floor(max(q[0] for q in b) / 16.0)) + 1):
            for gy in range(int(math.floor(min(q[1] for q in b) / 16.0)),
                            int(math.floor(max(q[1] for q in b) / 16.0)) + 1):
                grid[(gx, gy)].append(i)
    hit = {}
    for k, m in enumerate(mods):
        for i in grid.get((int(math.floor(m["x"] / 16.0)),
                           int(math.floor(m["y"] / 16.0))), ()):
            if sp._obb_overlap(cores[k], boxes[i][0]):
                hit[k] = boxes[i][1]
                break
    return hit


def dangling(scene, mods, tol=0.35):
    """Module ends that meet nothing within *tol* — and that OUGHT to.

    An end is closed by another module's centre line, a house or garage wall
    (the fence is cut AT the wall, so the end sits on it), or the road edge
    (a run trimmed off the carriageway ends at the kerb margin).

    TWO MORE ENDINGS ARE LEGITIMATE, and counting them was what kept this
    number pinned near 350 while the fences themselves got better — a metric
    that cannot go down is a metric nobody reads:

      A PAVED OPENING.  The front run breaks for the drive and the walk, by
      design (`_front_runs`). Both sides of that break are ends in mid-air and
      both are correct; the gap is the gate. Any end sitting on a drive or walk
      ribbon is one of them.

      THE BUILDING LINE.  A lot whose front yard is left open — its asset is
      over `_FRONT_FENCE_MAX_H_M`, or its package never had a front run — has
      its side fences cut back level with the front of the house
      (`suburb_scene._trim_to_building_line`). That is where a real privacy
      fence ends and where the gate goes. The two points are computed from the
      lot rectangle and the house depth, so this agrees with the trim by
      construction rather than by a tolerance.

      THE GATE OPENING.  `suburb_scene._gate_returns` turns each side run in to
      the house's front corner, and the return crosses the drive by
      construction — the drive runs up the side yard to a garage door standing
      ON the building line. So the return is broken by the lot's own
      `front_gaps`, exactly as the front run is, and BOTH SIDES OF THAT BREAK
      are ends in mid-air that are correct. Uncounted, that is what made the
      gate returns look like a regression: 376 returns on seed 3 took the free
      ends from 37 to 44 while closing 116 of the 121 fenced yards, and the
      report said the fence had got worse. It had not — 17 of those 44 sit on
      an opening the plat put there deliberately.

      AT THE EDGE OF THE OPENING, NOT ANYWHERE INSIDE IT. `_front_runs` cuts a
      run at `offset +- half_width` and nowhere else, so a legitimate end sits
      ON one of those two lines; an end in the MIDDLE of a drive corridor is a
      run stopping on asphalt, which is a different defect and one `on_drive`
      already colours. Measured on seeds 1/3/5/7 the two readings differ by a
      single end (75 -> 35 against 75 -> 36 on seed 7), so this costs nothing
      and refuses to launder the case it should not.

      This is a paved opening's twin and not a duplicate of it: the paving test
      asks about the RIBBON `apply_ground` lays, which is the plat's drive or
      the kit plan's, and it is trimmed by `pad=-0.30`; `front_gaps` is the
      union of the KIT's own door and garage-door gaps with the plat's drive,
      and the return is cut on that union. Where the two agree the end is
      excused twice, which costs nothing.

    Everything else is a fence stopping in mid-air. Returned as points, with
    what each is nearest to for the report.
    """
    # The legal front termini: where each lot's two side lines cross the front
    # of its house. `lot_corners` is (front_left, front_right, rear_right,
    # rear_left), so the side lines are 0->3 and 1->2.
    stops = []
    for par in scene["parcels"]:
        for h in par["houses"]:
            lc, p0, n, c, d = (h.get("lot_corners"), h.get("frontage"),
                               h.get("n"), h.get("c"), h.get("d"))
            if not (lc and p0 and n and c and d):
                continue
            depth = (c[0] - p0[0]) * n[0] + (c[1] - p0[1]) * n[1] - float(d) / 2.0
            for (f, r) in ((lc[0], lc[3]), (lc[1], lc[2])):
                ln = math.hypot(r[0] - f[0], r[1] - f[1])
                if ln < 1e-6:
                    continue
                t = depth / ln
                stops.append((f[0] + (r[0] - f[0]) * t,
                              f[1] + (r[1] - f[1]) * t))
    pave = [b for b in (_ribbon_box(r) for r in drives(scene)) if b is not None]
    # THE GATE OPENINGS, in each lot's own frame. Kept as the frame rather than
    # resolved to world points because a `front_gaps` entry is a LINE, not a
    # point: the run that stops on it may be the front fence at the 2.5 m
    # inset, a gate return at the building line, or anything between, and the
    # depth band below is what covers all of them. 1,046 openings on seed 3.
    gates = []
    for par in scene["parcels"]:
        for h in par["houses"]:
            p, u, n, d = h.get("frontage"), h.get("u"), h.get("n"), h.get("d")
            stop = ss._building_line_depth(h)
            if stop is None or not (p and u and n) or not d:
                continue
            # The deepest a `front_gaps` cut can reach: `_gate_returns` refuses
            # a side run whose front end is past `back + 0.5`, and the front run
            # is shallower still. Past that depth an end on the drive's bearing
            # is a fence in the back garden, not a fence at the gate.
            deep_max = stop + float(d) + 0.5
            for (o, hw) in (h.get("front_gaps") or ()):
                gates.append((p, u, n, float(o), float(hw), deep_max))

    walls = []
    for par in scene["parcels"]:
        for h in par["houses"]:
            walls.append(h["corners"])
            if h.get("garage"):
                walls.append(h["garage"]["corners"])
    wgrid = collections.defaultdict(list)
    for i, b in enumerate(walls):
        for gx in range(int(math.floor((min(q[0] for q in b) - 1) / 16.0)),
                        int(math.floor((max(q[0] for q in b) + 1) / 16.0)) + 1):
            for gy in range(int(math.floor((min(q[1] for q in b) - 1) / 16.0)),
                            int(math.floor((max(q[1] for q in b) + 1) / 16.0)) + 1):
                wgrid[(gx, gy)].append(i)
    mgrid = collections.defaultdict(list)
    lines = [_ends(m) for m in mods]
    for i, (p0, p1) in enumerate(lines):
        for gx in range(int(math.floor((min(p0[0], p1[0]) - 1) / 16.0)),
                        int(math.floor((max(p0[0], p1[0]) + 1) / 16.0)) + 1):
            for gy in range(int(math.floor((min(p0[1], p1[1]) - 1) / 16.0)),
                            int(math.floor((max(p0[1], p1[1]) + 1) / 16.0)) + 1):
                mgrid[(gx, gy)].append(i)
    road = ss._RoadIndex(scene["net"])

    def near_wall(pt):
        for i in wgrid.get((int(math.floor(pt[0] / 16.0)),
                            int(math.floor(pt[1] / 16.0))), ()):
            b = walls[i]
            for k in range(4):
                if sn.seg_seg_dist(pt, pt, b[k], b[(k + 1) % 4]) <= tol + 0.15:
                    return True
        return False

    out = []
    for i, (p0, p1) in enumerate(lines):
        for pt in (p0, p1):
            closed = False
            for j in mgrid.get((int(math.floor(pt[0] / 16.0)),
                                int(math.floor(pt[1] / 16.0))), ()):
                if j == i:
                    continue
                if sn.seg_seg_dist(pt, pt, lines[j][0], lines[j][1]) <= tol:
                    closed = True
                    break
            if closed or near_wall(pt):
                continue
            if any(abs(pt[0] - q[0]) <= tol + 0.5 and abs(pt[1] - q[1]) <= tol + 0.5
                   for q in stops):
                continue                      # ends at its own building line
            if any(sp._obb_overlap(sp._corners(pt[0], pt[1], 0.02, 0.02, 1.0, 0.0),
                                   b, pad=-0.30) for b in pave):
                continue                      # ends at a drive or walk opening
            if any(abs(abs((pt[0] - p[0]) * u[0] + (pt[1] - p[1]) * u[1] - o)
                       - hw) <= tol + 0.5
                   and -tol <= ((pt[0] - p[0]) * n[0]
                                + (pt[1] - p[1]) * n[1]) <= dmax + tol
                   for (p, u, n, o, hw, dmax) in gates):
                continue                      # ends at its own gate opening
            if road.on_road(pt, margin=ss._FENCE_ROAD_MARGIN_M + tol) \
                    if hasattr(road, "on_road") else False:
                out.append((pt, i, "kerb"))
                continue
            out.append((pt, i, "open"))
    return out


def house_assets(scene):
    """`{n distinct fence assets: n houses}` — one is the only legal answer."""
    out = collections.Counter()
    for par in scene["parcels"]:
        for h in par["houses"]:
            pick = h.get("_fence_pick")
            if not pick:
                continue
            vals = set(pick.values()) if isinstance(pick, dict) else {pick}
            out[len(vals)] += 1
    return out


def continuity(scene, mods, tol=0.25):
    """Gaps between consecutive modules along one collinear fence line.

    Grouped by LINE rather than by run, because within a run `_fence_run` tiles
    exactly by construction and a per-run measure is 0.0 however bad the fence
    looks. What is worth measuring is whether two runs that lie on the same line
    — a lot's and its neighbour's — meet. Pairs more than 8 m apart are two
    different fences on one bearing, not a break, and are reported separately.
    """
    groups = collections.defaultdict(list)
    for m in mods:
        a = m["ang"] % math.pi
        dx, dy = math.cos(a), math.sin(a)
        perp = -m["x"] * dy + m["y"] * dx
        groups[(round(math.degrees(a) / 3.0), round(perp / 0.5))].append(
            (m["x"] * dx + m["y"] * dy, m["L"]))
    pairs = breaks = far = 0
    for items in groups.values():
        items.sort()
        for i in range(len(items) - 1):
            gap = (items[i + 1][0] - items[i + 1][1] / 2.0
                   - items[i][0] - items[i][1] / 2.0)
            if gap > 8.0:
                far += 1
                continue
            pairs += 1
            breaks += int(gap > tol)
    return {"pairs": pairs, "breaks": breaks,
            "break_frac": (breaks / pairs) if pairs else 0.0,
            "separate_runs": far}


# ---------------------------------------------------------------------------
# the yard — what the fence is there to enclose
# ---------------------------------------------------------------------------

def houses(scene):
    """Every house record in the suburb, flat and in plat order."""
    return [h for par in scene["parcels"] for h in par["houses"]]


def _yard_cfg(scene):
    """`suburb_yardplan`'s effective config, resolved the way `plan` resolves
    it — its `DEFAULTS` under the preset's own `suburb_yardplan` block, with
    `suburb_yards` accepted as the legacy key. Written out here because `plan`
    keeps the merged dict local and this tool has to ask it two questions.
    """
    cfg = dict(yp.DEFAULTS)
    cfg.update(scene["cfg"].get("suburb_yardplan")
               or scene["cfg"].get("suburb_yards") or {})
    return cfg


def canopies(scene):
    """The yard pass's trees as ``(x, y, radius)`` — THE GATE'S OWN POPULATION.

    RECONSTRUCTED, BECAUSE `plan` DOES NOT PUBLISH THEM. It builds this list
    internally (`canopies`, fed to `_Canopies` for the seating gate) and returns
    only the placements, so the radius has to be recovered here. It is recovered
    from `yp._crown_radii` — the shipped function, the same one the screen walk
    spaces its stations with — under the same two floors `plant_tree` applies
    when it records a trunk:

        max(crown_r.get(usd, screen_step / 2), tree_min_separation_m / 2)

    Those two floors are the only thing restated here, and they are restated
    rather than re-invented: a radius guessed differently from the one the
    screen walk used would draw a wash the trees on the plate do not support,
    and would let `fence_check` pass a yard the build refused to seat.
    Measured on seed 3: 3,470 canopies, 1.51 m to 12.71 m.

    THE PARCEL PASS'S TREES ARE NOT IN HERE, and that is the point of the
    function existing separately from `parcel_canopies`. `suburb_yardplan`
    indexes the canopies IT planted and nothing else, so those are the only
    trees that can qualify a seat — and a checker judging enclosure over a
    larger set than the gate used would wave through exactly the props the gate
    would have refused.
    """
    lib = yp._Lib(scene["cfg"])
    pool = lib.pool("yard_trees") or lib.pool("trees")
    crown = yp._crown_radii(pool, scene["res"])
    ycfg = _yard_cfg(scene)
    step = max(1.0, float(ycfg.get("screen_step_m", 4.5)))
    sep = float(ycfg["tree_min_separation_m"])
    return [(p["x_m"], p["y_m"],
             max(crown.get(p["usd"], step / 2.0), sep / 2.0))
            for p in scene["yard"] if p.get("category") == "tree"]


def parcel_canopies(scene):
    """The PARCEL pass's trees as ``(x, y, radius)`` — for the plate only.

    `suburb_parcel` stations a verge tree and a lot tree of its own and
    `build_placements` plants them, ~1,200 a seed. They are real trees standing
    on real ground and a plate that omitted them would show gardens barer than
    they are — but they are drawn paler and they are NOT handed to any
    enclosure test, because the seating gate cannot see them either. Radius is
    half the measured crown width, `_crown_radii`'s own definition, asked of the
    resolver per placement rather than per pool entry: these come from several
    pools and the placement is the only record of which entry was drawn.
    """
    out = []
    for p in scene["placements"]:
        if p.get("category") != "tree":
            continue
        fp = scene["res"].get(p["usd"], "tree", scale=p.get("scale", 1.0),
                              axis_up=p.get("axis_up", "Z"))
        out.append((p["x_m"], p["y_m"],
                    0.5 * max(float(fp.get("sx", 0.0) or 0.0),
                              float(fp.get("sy", 0.0) or 0.0))))
    return out


def seating(scene):
    """Every seating-group prop the yard pass placed.

    BY THE POOL'S OWN LITERAL TAG TEST, not by category: `emit` charges the
    table set, the bench, the bin, the mailboxes and every shrub as `"plant"`,
    so category cannot tell a garden bench from a foundation shrub. `plan`
    selects its patio pool as the `yard_props` entries tagged `patio` or `shed`
    and this asks the identical question of the identical pool — deliberately
    NOT `_tagged`, which falls back to the WHOLE pool when no entry carries the
    tag and would hand this function every mailbox in the suburb.

    THE BIN IS LEFT OUT, and that is not an oversight. It is tagged `bin`, it is
    placed against the back wall INSIDE the group, and it is emitted only after
    the group's centrepiece is down — so it can never stand in a yard the group
    did not already qualify, and counting it would inflate the seating count
    with an object that is not seating. Measured on seed 3: 392 props by this
    test across 199 groups, plus 122 bins.
    """
    _pp = yp._Lib(scene["cfg"]).pool("yard_props")
    usds = {e["usd"] for e in _pp
            if "patio" in e["tags"] or "shed" in e["tags"]}
    return [p for p in scene["yard"]
            if p.get("category") == "plant" and p["usd"] in usds]


class _LotIndex:
    """Which lots a world point stands in, by each lot's OWN LOCAL FRAME.

    NOT BY NEAREST HOUSE CENTRE, and that is the whole reason this class is
    here rather than a two-line loop. A bench sits at the side of a garden, and
    on a narrow lot the nearest house centre to it is frequently the
    NEIGHBOUR's — the two centres are one lot width apart and the bench is
    offset `patio_side_off_frac` (0.55) of a half width toward the boundary. Two
    of the props on seed 3 score to the wrong lot that way, and during Phase 4
    both were reported as seating in an open yard when the yard they were
    actually in was fenced. The lot frame cannot make that mistake: `frontage`,
    `u`, `n`, `lot_width` and `lot_depth` are the numbers the lot was ISSUED on,
    the same five `suburb_scene._lot_lines` strikes the enclosure edges from.

    A POINT CAN BE IN TWO LOTS AT ONCE and the answer is a LIST. A lot is a
    rectangle hung off one frontage, so two lots hung off two frontages of the
    same block genuinely overlap in the corner between them — this is the same
    fact `fence_check.trespass` measures, and on seed 3 it puts 8 of 392 seating
    props inside two lot rectangles. The caller decides what to do with two
    owners; inventing a single winner here would be inventing the very
    nearest-centre guess this exists to avoid.

    Hash-gridded on the lot bounding box: 392 props against 578 lots is a
    quarter of a million frame transforms as a scan, and this tool is run in a
    loop over four seeds.
    """

    CELL = 40.0

    def __init__(self, hs):
        self.hs = list(hs)
        self.g = collections.defaultdict(list)
        for i, h in enumerate(self.hs):
            lc = h.get("lot_corners")
            if not lc:
                continue
            xs = [q[0] for q in lc]
            ys = [q[1] for q in lc]
            for gx in range(int(math.floor(min(xs) / self.CELL)),
                            int(math.floor(max(xs) / self.CELL)) + 1):
                for gy in range(int(math.floor(min(ys) / self.CELL)),
                                int(math.floor(max(ys) / self.CELL)) + 1):
                    self.g[(gx, gy)].append(i)

    def at(self, x, y):
        """Indices into the house list of every lot containing ``(x, y)``."""
        out = []
        for i in self.g.get((int(math.floor(x / self.CELL)),
                             int(math.floor(y / self.CELL))), ()):
            h = self.hs[i]
            p, u, n = h.get("frontage"), h.get("u"), h.get("n")
            lw, ld = h.get("lot_width"), h.get("lot_depth")
            if not (p and u and n) or not lw or not ld:
                continue
            dx, dy = x - p[0], y - p[1]
            along = dx * u[0] + dy * u[1]
            deep = dx * n[0] + dy * n[1]
            if abs(along) <= float(lw) / 2.0 and 0.0 <= deep <= float(ld):
                out.append(i)
        return out


def drawn_fences(scene):
    """Every fence span that actually became modules, as ``(a, b)`` pairs.

    `h["fence_drawn"]` and not `h["fence_segs"]`: the second is what the PLAT
    proposed and three cuts stand between it and the ground. Flat over the whole
    suburb because a lot's left-hand boundary is very often its neighbour's
    right-hand one, drawn once with the neighbour's asset — an enclosure test
    handed only this lot's own spans would call half the fenced lots open.
    """
    return [(a, b) for h in houses(scene)
            for (a, b, _t) in (h.get("fence_drawn") or ())]


def enclosure_state(scene, trees=None):
    """``{id(house): state}`` for every lot, state being one of

        ``"fenced"``    the back yard closes on fence alone
        ``"screened"``  it closes, but a treeline is doing some of it
        ``"open"``      it has a back garden and nothing encloses it
        ``None``        it has no back garden behind its back wall to enclose

    THE SHIPPED PREDICATE DECIDES, NOT A COPY OF IT. `_yard_enclosed` is
    `suburb_scene`'s, it is what `build_placements` gated the all-or-nothing
    fence sweep on, and calling it here is what stops this tool from being the
    fifth pass in the tree with its own opinion about what an enclosed yard is.
    The fence half is read off `h["enclosure"]["closed"]`, which that same
    function wrote.

    IT IS NOT BIT-FOR-BIT `suburb_yardplan`'s SEATING GATE, and the difference
    is worth knowing before reading a plate. The gate asks "all three edges
    fenced" OR "all three edges screened"; `_yard_enclosed` asks per EDGE and
    ORs there, so a yard fenced down one side and screened down the other
    passes here and is refused a seat there. That makes this the WEAKER test,
    which is the safe direction for a checker — anything the gate seated passes
    it — and on seeds 1/3/5/7 the two answers are in fact identical on every
    lot, so the wash on the plate is the gate's answer in practice as well as
    in principle.
    """
    trees = canopies(scene) if trees is None else trees
    fences = drawn_fences(scene)
    out = {}
    for h in houses(scene):
        if not h.get("rear_edges"):
            out[id(h)] = None
        elif bool((h.get("enclosure") or {}).get("closed")):
            out[id(h)] = "fenced"
        elif ss._yard_enclosed(h, fences=fences, trees=trees)[0]:
            out[id(h)] = "screened"
        else:
            out[id(h)] = "open"
    return out


def seating_unenclosed(scene, trees=None, state=None):
    """Seating props standing in a yard nothing encloses — ``[(pt, usd)]``.

    THE ONE NUMBER THE WHOLE ENCLOSURE REWORK IS JUDGED ON. Before it, 294 of
    the 358 patio props on seed 3 stood in a back garden open to the block
    behind it; the target is zero and `fence_check` gates on zero.

    A PROP IN TWO LOTS IS EXCUSED BY EITHER OF THEM. `_LotIndex` returns every
    lot whose rectangle contains the prop and two lot rectangles genuinely
    overlap in a block corner, so "which lot was this placed for" has no answer
    from the record — `plan` does not publish the owner. Flagging a prop only
    when EVERY containing lot is open is the reading that cannot produce a false
    positive, and a regression that removed the gate would put props in lots
    that are open on all counts, so it cannot hide one either. On seed 3 it is 8
    props of 392 that have two owners at all.

    A PROP IN NO LOT AT ALL IS FLAGGED. It is a different defect — furniture
    that escaped every lot rectangle — and it has never been seen; folding it in
    here is cheaper than a counter nobody reads, and the point carries its own
    coordinates for the plate.
    """
    hs = houses(scene)
    state = enclosure_state(scene, trees) if state is None else state
    ix = _LotIndex(hs)
    out = []
    for p in seating(scene):
        pt = (p["x_m"], p["y_m"])
        owners = ix.at(pt[0], pt[1])
        if not any(state.get(id(hs[i])) in ("fenced", "screened")
                   for i in owners):
            out.append((pt, p["usd"]))
    return out


def fence_fragment(scene):
    """Lots carrying fence modules whose back yard is not closed by them.

    A FENCE THAT ENCLOSES NOTHING is what the eye reads as "the fences are
    broken", and it was 76 of the 155 fenced lots on seed 3 before the rework.
    `build_placements` now sweeps them — a lot that cannot complete its
    perimeter is stripped of fence entirely, iterated to a fixed point because
    deleting one lot's fence can un-close the neighbour whose side line it was —
    so this asserts the result.

    STATED OVER EVERY LOT WITH A MODULE, WITH NO EXCEPTION. It could not be,
    until Phase 2 also dropped the gardenless-fenced-lot carve-out: a lot with
    under 4 m of garden behind its back wall has no yard that can ever close,
    and while those kept their fence this had to except them by name — which is
    an exception that hides exactly the defect it is stated over. They are
    stripped too now (55 of them on seed 3), so `_rear_yard_edges` returning
    `[]` makes `_yard_enclosed` answer False and a lot like that shows up here
    rather than being waved past.

    RECOMPUTED, NOT READ OFF `h["enclosure"]["closed"]`. That flag is written by
    the same function in the same pass that does the stripping; a check that
    read it would be asserting that the pass agrees with itself. Measuring the
    modules against the edges again, from the outside, is what makes this a test.
    """
    fences = drawn_fences(scene)
    return [h for h in houses(scene)
            if h.get("fence_drawn") and not ss._yard_enclosed(h, fences)[0]]


# ---------------------------------------------------------------------------
# the picture
# ---------------------------------------------------------------------------

_DEFECT_COLOUR = [("on_road", "#e02020"), ("in_bulb", "#c020c0"),
                  ("crossing", "#ff8800"), ("doubled", "#00b8d0"),
                  ("on_drive", "#ff2090")]

# THE YARD LAYER'S PALETTE. Two rules held it together: nothing in it may be a
# colour already spent on a fence defect above, and nothing that is CORRECT may
# be a saturated colour — the plate's job is that a defect is the only thing on
# it that shouts. Hence washes at 15% and canopies at 30%, and one violet that
# is nowhere else in the plate.
#
# Violet rather than a sixth alarm red: `on_road` red is already the colour of
# the small dangling-end dots, and a seating prop is drawn as a SQUARE of about
# the same size — two different defects that are both small red marks in a
# garden is exactly the ambiguity the per-defect colouring exists to prevent.
_SEAT_COLOUR = "#3f5fa8"          # a prop in a yard something encloses
_SEAT_BAD_COLOUR = "#7a00cc"      # ...in a yard nothing does
_ENCLOSURE_WASH = {"fenced": "#7fb069", "screened": "#b6ae5a"}
_CANOPY_COLOUR = "#6ea364"         # the yard pass's trees — these screen a yard
_PARCEL_CANOPY_COLOUR = "#a9b6a5"  # the parcel pass's — these do not


def draw(scene, mods, bad, out_path, zoom=None, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import (PolyCollection, LineCollection,
                                        EllipseCollection)
    from matplotlib.patches import Circle

    if zoom:
        zx, zy, zr = zoom
        x0, y0, x1, y1 = zx - zr, zy - zr, zx + zr, zy + zr
    else:
        x0, y0, x1, y1 = scene["info"]["region"]
    span = max(x1 - x0, y1 - y0)
    fig, ax = plt.subplots(figsize=(16, 16 * (y1 - y0) / max(x1 - x0, 1e-6)),
                           dpi=110)
    ax.set_facecolor("#ffffff")

    # Carriageways at TRUE width, because "on the road" is a claim about the
    # kerb and drawing centrelines would make every red module look wrong.
    segs, widths = [], []
    for e in scene["net"].edges.values():
        if e.road_class == "boundary":
            continue
        for i in range(len(e.pts) - 1):
            segs.append((e.pts[i], e.pts[i + 1]))
            widths.append(e.half_w * 2.0)
    if segs:
        pts_per_m = (16 * 110) / max(span, 1e-6)
        ax.add_collection(LineCollection(
            segs, colors="#d9d9d9", zorder=1, capstyle="round",
            linewidths=[w * pts_per_m * 72.0 / 110.0 for w in widths]))

    ax.add_collection(PolyCollection(
        [b["poly"] for b in scene["blocks"]], facecolors="none",
        edgecolors="#b8ccb8", linewidths=0.6, zorder=2))
    for (c, r) in scene["discs"]:
        ax.add_patch(Circle(c, scene["bulb_r"], facecolor="#f0e0f0",
                            edgecolor="#c8a0c8", lw=0.7, zorder=1.5))

    hrecs = houses(scene)
    lots, house_boxes = [], []
    for h in hrecs:
        if h.get("lot_corners"):
            lots.append(h["lot_corners"])
        house_boxes.append(h["corners"])
    # -- the yard, under the lot lines ------------------------------------
    # ORDER IS THE READABILITY. Wash, then the two tree layers, then the lot
    # lines on top of all three: the lot line is 0.4 pt of near-white and a
    # canopy drawn over it erases the boundary the wash is a claim about.
    # Everything here stays below the houses (3), the paving (3.1) and the
    # fence (4) — a plate where the trees hide the fence is not a fence plate.
    trees_y = canopies(scene)
    trees_p = parcel_canopies(scene)
    state = enclosure_state(scene, trees_y)
    wash = collections.defaultdict(list)
    for h in hrecs:
        st = state.get(id(h))
        if st in _ENCLOSURE_WASH and h.get("enclosure_poly"):
            wash[st].append(h["enclosure_poly"])
    for st, polys in wash.items():
        ax.add_collection(PolyCollection(
            polys, facecolors=_ENCLOSURE_WASH[st], edgecolors="none",
            alpha=0.15, zorder=2.2))
    # ONE COLLECTION PER TREE PASS, NOT ONE PATCH PER TREE. 4,663 `Circle`
    # patches on seed 3 is the difference between a plate that renders in
    # seconds and one that does not; `EllipseCollection` in `xy` units takes
    # the true measured diameter and still scales with the axes on a zoom.
    for discs, colour in ((trees_p, _PARCEL_CANOPY_COLOUR),
                          (trees_y, _CANOPY_COLOUR)):
        if not discs:
            continue
        d = [2.0 * t[2] for t in discs]
        ax.add_collection(EllipseCollection(
            d, d, [0.0] * len(d), units="xy",
            offsets=[(t[0], t[1]) for t in discs],
            offset_transform=ax.transData, facecolors=colour,
            edgecolors=(colour if zoom else "none"),
            linewidths=0.3, alpha=(0.42 if zoom else 0.30),
            zorder=(2.3 if colour == _PARCEL_CANOPY_COLOUR else 2.35)))
    ax.add_collection(PolyCollection(lots, facecolors="none",
                                     edgecolors="#e2e2e2", linewidths=0.4,
                                     zorder=2.5))
    ax.add_collection(PolyCollection(house_boxes, facecolors="#efe6d8",
                                     edgecolors="#c8bba4", linewidths=0.4,
                                     zorder=3))

    # Drives and walks, under the fences: what the ground pass will pave.
    rib_d, rib_w = [], []
    for r in drives(scene):
        b = _ribbon_box(r)
        if b is None:
            continue
        (rib_d if r["kind"] == "drive" else rib_w).append(b)
    ax.add_collection(PolyCollection(rib_d, facecolors="#b9c6d6",
                                     edgecolors="#8fa3bd", linewidths=0.3,
                                     zorder=3.2))
    ax.add_collection(PolyCollection(rib_w, facecolors="#d8dde4",
                                     edgecolors="none", zorder=3.1))

    flagged = {}
    for name, colour in _DEFECT_COLOUR:
        for i in bad.get(name, ()):
            flagged.setdefault(i, colour)      # first in the list wins
    # A module's plan footprint is 9-33 cm across, invisible on a 1600 m plate,
    # so a defective one is also ringed. The ring is what you actually see when
    # the whole plat is in frame; the rectangle is what you check on a zoom.
    good = [m["box"] for i, m in enumerate(mods) if i not in flagged]
    ax.add_collection(PolyCollection(good, facecolors="#4a6b4a",
                                     edgecolors="#4a6b4a", linewidths=0.3,
                                     zorder=4))
    for i, colour in flagged.items():
        ax.add_collection(PolyCollection([mods[i]["box"]], facecolors=colour,
                                         edgecolors=colour, linewidths=0.6,
                                         zorder=5))
        ax.add_patch(Circle((mods[i]["x"], mods[i]["y"]), span / 260.0,
                            facecolor="none", edgecolor=colour, lw=1.0,
                            alpha=0.85, zorder=6))

    # -- the seating, over the fence -------------------------------------
    # DRAWN AT A FLOOR ON TRUE SIZE. A table set is 2.43 m square and a bench
    # 2.62 x 0.79 m; at 1600 m across the plate that is under two pixels, so
    # the marker is `max(true, span/220)` — 7.3 m of plate at the overview,
    # 2.5 m on a 90 m zoom, i.e. the real object once you are close enough to
    # judge it. Over the fence rather than under it because the question this
    # marker answers is "what is this prop standing in", and a prop hidden by
    # the fence module beside it answers nothing.
    seat_bad = {pt for (pt, _u) in seating_unenclosed(scene, trees_y, state)}
    side = max(2.5, span / 220.0)
    ok_sq, bad_sq = [], []
    for pr in seating(scene):
        pt = (pr["x_m"], pr["y_m"])
        (bad_sq if pt in seat_bad else ok_sq).append(
            sp._corners(pt[0], pt[1], side, side, 1.0, 0.0))
    ax.add_collection(PolyCollection(ok_sq, facecolors=_SEAT_COLOUR,
                                     edgecolors="none", alpha=0.85, zorder=6))
    ax.add_collection(PolyCollection(bad_sq, facecolors=_SEAT_BAD_COLOUR,
                                     edgecolors=_SEAT_BAD_COLOUR,
                                     linewidths=0.8, zorder=6.5))
    for pt in seat_bad:
        ax.add_patch(Circle(pt, span / 260.0, facecolor="none",
                            edgecolor=_SEAT_BAD_COLOUR, lw=1.0, alpha=0.85,
                            zorder=6.6))

    ends = dangling(scene, mods)
    open_ends = [e for e in ends if e[2] == "open"]
    if open_ends:
        ax.scatter([e[0][0] for e in open_ends], [e[0][1] for e in open_ends],
                   s=(9.0 if zoom else 3.0), c="#d01010", marker="o",
                   linewidths=0, zorder=7)

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.axis("off")
    counts = ", ".join(f"{n} {len(bad.get(n, ()))}" for n, _c in _DEFECT_COLOUR)
    src = ("sizes MEASURED from USD" if SIZE_SOURCE["measured"]
           else "sizes from asset-set comments / manifest")
    # THE YARD LINE OF THE TITLE, and `seating_unenclosed` is reported in the
    # same breath as `on_drive` on purpose: both are gated at zero in
    # `fence_check`, and a plate whose title does not carry the number is a
    # plate you have to go and run the checker to trust.
    n_frag = len(fence_fragment(scene))
    n_fenced = sum(1 for h in hrecs if h.get("fence_drawn"))
    st = collections.Counter(state.values())
    ax.set_title(
        f"{title}  {len(mods)} fence modules on {n_fenced} lots — {counts}, "
        f"dangling ends {len(open_ends)}, fence_fragment {n_frag}  [{src}]"
        f"\nyards: {st['fenced']} fenced, {st['screened']} tree-screened, "
        f"{st['open']} open, {st[None]} with no back garden  |  "
        f"{len(ok_sq) + len(bad_sq)} seating props, seating_unenclosed "
        f"{len(bad_sq)}  |  {len(trees_y)} yard canopies "
        f"(+{len(trees_p)} parcel)",
        fontsize=11, color="#333333")
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".",
                exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--config", default="suburb_net")
    ap.add_argument("--out", default="_plans/fences.png")
    ap.add_argument("--zoom", default="", metavar="X,Y,R",
                    help="centre and half-extent in metres, e.g. -640,500,90")
    # What this plate is OF, in the title. A zoom is worthless in a report
    # without it: two 90 m squares of suburb look alike, and the reader cannot
    # tell a before from an after or a garage-less lot from a curved face.
    ap.add_argument("--title", default="",
                    help="prefix for the figure title")
    args = ap.parse_args()

    scene = build(args.seed, args.config)
    mods = modules(scene)
    bad = classify(scene, mods)
    zoom = None
    if args.zoom:
        zx, zy, zr = (float(v) for v in args.zoom.split(","))
        zoom = (zx, zy, zr)
    draw(scene, mods, bad, args.out, zoom=zoom,
         title=(f"{args.title}  " if args.title else "")
         + f"seed {scene['seed']}")

    print(f"[fence_png] seed {scene['seed']}: {len(mods)} modules")
    for name, _c in _DEFECT_COLOUR:
        print(f"  {name:>9}: {len(bad[name])}")
    print(f"  houses by distinct fence assets: {dict(house_assets(scene))}")
    print(f"  continuity: {continuity(scene, mods)}")
    ends = dangling(scene, mods)
    n_open = sum(1 for e in ends if e[2] == "open")
    print(f"  dangling ends: {n_open} open ({len(ends) - n_open} at a kerb) "
          f"of {2 * len(mods)} module ends")
    hit = on_drive(scene, mods)
    by_style = collections.Counter()
    by_src = collections.Counter()
    for r in hit.values():
        h = r.get("house") or {}
        by_src[r["src"]] += 1
        by_style[(h.get("archetype"), h.get("art_garage"))] += 1
    lots = {id(r.get("house")) for r in hit.values()}
    print(f"  on_drive: {len(hit)} modules across {len(lots)} drives "
          f"(drive from {dict(by_src)}; by (archetype, art_garage) "
          f"{dict(by_style)})")
    fenced = [h for par in scene["parcels"] for h in par["houses"]
              if h.get("fence_segs")]
    front = [h for h in fenced if any(t == "low" for (_a, _b, t) in h["fence_segs"])]
    print(f"  fenced houses {len(fenced)}, with a front run {len(front)}; "
          f"of those art_garage=False {sum(1 for h in front if not h.get('art_garage'))}")
    # THE YARD PASS'S OWN REPORT, printed rather than swallowed. `build` runs
    # `yp.plan` and `plan` says nothing on its own — the seating gate's split
    # (`seat_fenced` / `seat_screened` / `patio_open`) is the line that says
    # WHY a garden is bare, and it is the first thing to read when the count of
    # seating props on the plate looks wrong.
    yp.report(scene["ystats"])
    trees_y = canopies(scene)
    state = enclosure_state(scene, trees_y)
    st = collections.Counter(state.values())
    print(f"  yards: {st['fenced']} closed by fence, {st['screened']} by a "
          f"treeline, {st['open']} open, {st[None]} with no back garden "
          f"(of {len(houses(scene))} lots)")
    print(f"  fence on {sum(1 for h in houses(scene) if h.get('fence_drawn'))} "
          f"lots; fence_fragment {len(fence_fragment(scene))}")
    print(f"  seating props {len(seating(scene))}, seating_unenclosed "
          f"{len(seating_unenclosed(scene, trees_y, state))}")
    print(f"  canopies: {len(trees_y)} from the yard pass (these can screen a "
          f"yard), {len(parcel_canopies(scene))} from the parcel pass "
          f"(these cannot)")
    print(f"[fence_png] wrote {args.out}")


if __name__ == "__main__":
    main()
