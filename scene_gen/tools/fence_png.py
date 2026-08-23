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

A clean plan is grey-green fences on white lots, and nothing else.

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


def _sizes(cfg):
    """`{basename: (sx, sy, sz)}` for everything the scene can ask about."""
    sets = os.path.join(_SCENE_GEN, "config", "asset_sets")
    out = measured_sizes([os.path.join(sets, f) for f in os.listdir(sets)
                          if f.endswith(".yaml")])
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
    return out


# ---------------------------------------------------------------------------
# the scene
# ---------------------------------------------------------------------------

def build(seed=None, config_name="suburb_net"):
    """The suburb, up to and including `build_placements`, on the host.

    Mirrors `suburb_scene.generate_suburb_on_stage` step for step down to the
    rng seeding, minus everything that needs a stage: the park, the yard
    planting, the frontage props and the ground. Those cannot move a fence.
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
    region = (cfg.get("layout", {}) or {}).get("region_m") or [1600.0, 1200.0]
    net, blocks, info = sn.generate(float(region[0]), float(region[1]), rng,
                                    dict(cfg.get("suburb_net") or {}))
    buildable = [b for b in blocks if not b.get("undeveloped")]

    pcfg = dict(cfg.get("suburb_parcel") or {})
    bulb_r = float(sn.DEFAULTS["bulb_radius_m"])
    pcfg.setdefault("keepout_discs",
                    [(e.pts[-1], bulb_r + float(pcfg.get("bulb_margin_m", 3.0)))
                     for e in net.edges.values()
                     if e.street_type == "lollipop"])
    pools = ss.AssetPools(cfg)
    yaw_off = float(pcfg.get("house_yaw_offset_deg", -90.0))
    catalogue = ss.modular_catalogue(cfg)
    if catalogue and pcfg.get("house_sizes") is None:
        pcfg["house_sizes"] = [(e["w"], e["d"]) for e in catalogue]
        tight = min(d["lot"] for d in sp.DENSITY.values())
        need = (min(e["w"] for e in catalogue)
                + float(pcfg.get("house_gap_m", 4.0)) * 0.5)
        lw = sp._rng_pair(pcfg.get("lot_width_m", [21.0, 30.0]), (21.0, 30.0))
        if lw[0] < need / tight:
            pcfg["lot_width_m"] = [need / tight,
                                   max(lw[1], need / tight * 1.35)]
    parcels = sp.parcel_blocks(buildable, rng, pcfg)
    placements = ss.build_placements(cfg, res, parcels, rng, pools,
                                     yaw_off=yaw_off, catalogue=catalogue,
                                     pool_holes_out=[], net=net)
    return {"cfg": cfg, "res": res, "pools": pools, "net": net,
            "blocks": blocks, "info": info, "parcels": parcels,
            "placements": placements,
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
           "doubled": set()}

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
# the picture
# ---------------------------------------------------------------------------

_DEFECT_COLOUR = [("on_road", "#e02020"), ("in_bulb", "#c020c0"),
                  ("crossing", "#ff8800"), ("doubled", "#00b8d0")]


def draw(scene, mods, bad, out_path, zoom=None, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection, LineCollection
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

    lots, houses = [], []
    for par in scene["parcels"]:
        for h in par["houses"]:
            if h.get("lot_corners"):
                lots.append(h["lot_corners"])
            houses.append(h["corners"])
    ax.add_collection(PolyCollection(lots, facecolors="none",
                                     edgecolors="#e2e2e2", linewidths=0.4,
                                     zorder=2.5))
    ax.add_collection(PolyCollection(houses, facecolors="#efe6d8",
                                     edgecolors="#c8bba4", linewidths=0.4,
                                     zorder=3))

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

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.axis("off")
    counts = ", ".join(f"{n} {len(bad.get(n, ()))}" for n, _c in _DEFECT_COLOUR)
    ax.set_title(f"{title}  {len(mods)} fence modules — {counts}",
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
    args = ap.parse_args()

    scene = build(args.seed, args.config)
    mods = modules(scene)
    bad = classify(scene, mods)
    zoom = None
    if args.zoom:
        zx, zy, zr = (float(v) for v in args.zoom.split(","))
        zoom = (zx, zy, zr)
    draw(scene, mods, bad, args.out, zoom=zoom,
         title=f"seed {scene['seed']}")

    print(f"[fence_png] seed {scene['seed']}: {len(mods)} modules")
    for name, _c in _DEFECT_COLOUR:
        print(f"  {name:>9}: {len(bad[name])}")
    print(f"  houses by distinct fence assets: {dict(house_assets(scene))}")
    print(f"  continuity: {continuity(scene, mods)}")
    print(f"[fence_png] wrote {args.out}")


if __name__ == "__main__":
    main()
