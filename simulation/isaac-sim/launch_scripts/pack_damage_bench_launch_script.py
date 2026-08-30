#!/usr/bin/env python
"""
Fire damage ACROSS THE ASSET PACKS — one row per pack, one column per asset.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/pack_bench \
    ISAAC_SIM_SCRIPT_NAME=pack_damage_bench_launch_script.py airstack up isaac-sim

WHY THIS EXISTS
---------------
`urban_fire_bench_launch_script.py` puts seven KIT buildings in a row and is
the right bench for judging the fire ladder. It cannot answer the question
that actually blocks the urban scene, which is: **the city draws from six
different asset packs, and only one of them can be damaged properly.** That is
invisible on a bench built entirely from the one pack that works.

So: rows are PACKS, columns are DIFFERENT ASSETS INSIDE that pack, and every
building is damaged. Read it down a column to compare packs at the same
severity; read it along a row to see whether a pack behaves consistently
across its own stock.

THE TWO DAMAGE PATHS, AND WHY THE ROWS LOOK DIFFERENT
------------------------------------------------------
KIT row (`bld_*_DG0`, from ModernCityEnvironment01 / Downtown_West /
CivilianArea):  `urban_building.build_building` re-assembles the style from
its façade pieces, `quake_flow.describe` turns those into elements carrying a
side / storey / role, and the FULL urban-fire ladder runs — gutted interiors,
windows out to a black void, floors and roof burnt through, per-storey smoke
wash and char, partial collapse. This is the reference for what damage should
look like.

EVERY OTHER ROW is a single whole-asset mesh with no elements. There is
nothing to take apart, so the structural damage comes from
`disaster/monolith_damage.py`, which CUTS the shell itself with Shapely
(preserving the source UVs) on a stepped storey/bay profile, then puts a
recess, storey slabs, buckled columns and a rubble field behind the cut:
`roof_collapse`, `partial_collapse`, `corner_loss`, `soft_storey`,
`mid_storey`. That is real geometry loss rather than paint — but it has no
windows, no storey plates and no modules to work with, so it will never reach
the kit row.

There is no whole-asset damage path any more. `urban_fire.burn_monolith`
used to be one, and it was REMOVED (2026-08-29) because its entire repertoire
was one flat multiplier over every material on the asset, which is what made a
burnt 500 m downtown render as a field of uniformly grey boxes. A whole-asset
building now routes through `disaster/kit_substitute.route()` instead: to its
kit twin where one exists, to the slicer where it does not, and refused where
neither is possible.

Env:
    PB_PACKS      comma list of rows (default: all) — kit, gac, downtowncity,
                  muyang_downtown, moderncity, brownstone
    PB_COLS       assets per row (default 5)
    PB_LEVELS     kit-row fire levels (default F1,F2,F3,F4,F5)
    PB_RECIPES    monolith_damage recipes for the other rows
                  (default roof_collapse,partial_collapse,corner_loss,
                   soft_storey,mid_storey)
    PB_COL_M      column pitch, m (default 85)
    PB_ROW_M      row pitch, m (default 105)
    PB_MAX_H      skip assets taller than this, m (default 70) — a 300 m
                  tower in a row of 20 m sheds makes the row unreadable
    PB_SEED       (default 5)
    PB_FLOW       1 authors NVIDIA Flow on the kit row (default 0 — geometry
                  is what this bench is for, and Flow costs GPU the 500 m
                  scene showed we do not have spare)
    SETTLE_STEPS  physics ceiling for the kit row's loose pieces (default 900)
    SNAP_DIR      captures, under /isaac-sim/.nvidia-omniverse/logs/
    KEEP_OPEN     1 keeps a headless run up after the captures
"""

import math
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING, so
    `os.environ.get(name, default)` never reaches its default. Empty is
    absent."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
# FRACTIONAL CUTOUT OPACITY. This bench calls `urban_fire.burn_building`,
# which authors `wall_overlay`'s soot mask and `_glass_pane`'s smoke deposits
# — both FRACTIONAL CUTOUT opacity, which RTX discards unless this is set, so
# they render as hard binary stamps instead of graded staining. The startup
# flag alone does not survive stage composition and the carb form alone is too
# late for startup, so the launchers that get this right do BOTH; see
# `urban_fire_bench_launch_script.py` and `disaster/ground.py:KIT_ARGS`.
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                              "extra_args": KIT_ARGS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux                  # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                   # noqa: E402
import yaml                                                    # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from compile_disaster import (resolve_config_path, compile_spec,  # noqa: E402
                              DEFAULT_BASE)
from detail import urban_building as ub                        # noqa: E402
from disaster import fracture, settle                          # noqa: E402
from disaster import monolith_damage as md                     # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

PARENT = "/World/bench"
COLS = int(_env("PB_COLS", "5"))
COL_M = float(_env("PB_COL_M", "85"))
ROW_M = float(_env("PB_ROW_M", "105"))
MAX_H = float(_env("PB_MAX_H", "70"))
SEED = int(_env("PB_SEED", "5"))
FLOW = _env("PB_FLOW", "0") not in ("0", "false", "no")
SETTLE_STEPS = int(_env("SETTLE_STEPS", "900"))
SNAP_DIR = _env("SNAP_DIR")
LEVELS = [v.strip() for v in _env("PB_LEVELS", "F1,F2,F3,F4,F5").split(",") if v.strip()]
RECIPES = [v.strip() for v in _env(
    "PB_RECIPES",
    "roof_collapse,partial_collapse,corner_loss,soft_storey,mid_storey"
).split(",") if v.strip()]

# Which pack an asset path belongs to. Ordered: the first match wins, and
# `bld_` is checked before anything else because a kit bake lives under
# `scene_gen/assets/`, which no other rule should claim.
PACKS = [
    ("kit",             ("/bld_",)),
    ("gac",             ("GreatAmericanCity",)),
    ("downtowncity",    ("downtowncity", "FactoryDistrict")),
    ("muyang_downtown", ("Muyang/DownTown",)),
    ("moderncity",      ("ModernCityEnvironment/",)),
    ("brownstone",      ("aec/brownstone",)),
]
PACK_LABEL = {
    "kit": "KIT (MCE01 / Downtown_West / CivilianArea) — full fire ladder",
    "gac": "GreatAmericanCity", "downtowncity": "downtowncity + FactoryDistrict",
    "muyang_downtown": "Muyang DownTown", "moderncity": "ModernCityEnvironment",
    "brownstone": "AEC brownstone",
}


def _pack_of(usd):
    for name, keys in PACKS:
        if any(k in usd for k in keys):
            return name
    return None


def library():
    """Every building the CITY can draw, bucketed by pack.

    Read out of the live asset set rather than hardcoded, so this bench is
    always showing the stock the city would actually place — including the
    kit entries just added to the pools.
    """
    path = resolve_config_path("downtown_gac")
    cfg = compile_spec(yaml.safe_load(open(path)), yaml.safe_load(open(DEFAULT_BASE)))
    cfg = sg.resolve_asset_set(cfg, path)
    # RESOLVE AGAINST `asset_root` HERE, ONCE. Most entries in these pools are
    # RELATIVE ("Muyang/DownTown/Assets/BG_Building_A.usd") and only mean
    # anything joined to the set's `asset_root`
    # (omniverse://.../Library/Stages/). Referencing the raw string composes a
    # prim that resolves to nothing — no error, no warning, just an EMPTY
    # BOUND, which this bench first reported as "unmeasurable" for every
    # Muyang / Dmytro / ModernCityEnvironment asset while GAC (whose entries
    # are absolute URLs) measured fine.
    root = str(cfg.get("asset_root") or "")
    pools = (cfg.get("usds") or {}).get("buildings") or {}
    seen, out = set(), {}
    for pool, items in pools.items():
        for e in (items or []):
            usd = e.get("usd") if isinstance(e, dict) else e
            if not usd:
                continue
            full = sg._join_asset_root(usd, root)
            if full in seen:
                continue
            seen.add(full)
            pk = _pack_of(usd)
            if pk:
                out.setdefault(pk, []).append(
                    {"usd": full,
                     "scale": float(e.get("scale", 1.0)) if isinstance(e, dict) else 1.0,
                     "name": os.path.basename(usd).rsplit(".", 1)[0]})
    return out


def ground_and_light(stage, w, d):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(1100.0)
    dome.CreateColorAttr(Gf.Vec3f(0.80, 0.84, 0.90))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2600.0)
    key.CreateAngleAttr(0.8)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.90))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-42.0, 0.0, 28.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    hw, hd = w * 0.62, d * 0.62
    g.CreatePointsAttr([Gf.Vec3f(-hw, -hd, 0), Gf.Vec3f(hw, -hd, 0),
                        Gf.Vec3f(hw, hd, 0), Gf.Vec3f(-hw, hd, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.30, 0.29)])
    g.CreateExtentAttr([Gf.Vec3f(-hw, -hd, 0), Gf.Vec3f(hw, hd, 0)])


def place_and_measure(stage, cell, usd, scale):
    """Reference the asset under `cell/shell`, seat it, and measure it.

    THE SHELL'S LOCAL FRAME IS WHAT `monolith_damage` WORKS IN.
    `cut_shell` expresses every point in `source_root`'s frame, and its cut
    profile assumes a building CENTRED on that frame's origin with its base at
    z=0 and its front on -Y. So the centring translate goes on the ASSET
    child, and `cell/shell` itself is left at identity — put the translate on
    the shell instead and the geometry is centred in the wrong frame, which
    silently moves the cut off the building.
    """
    shell = UsdGeom.Xform.Define(stage, Sdf.Path(cell + "/shell"))
    kid = stage.DefinePrim(Sdf.Path(cell + "/shell/asset"))
    kid.GetReferences().AddReference(usd)      # already absolute, see library()
    # A prim composed into a RUNNING stage does not auto-load nested payloads
    # the way Usd.Stage.Open does; without this a payloaded asset lands with a
    # correct bbox and no visible geometry.
    stage.Load(Sdf.Path(cell + "/shell"))
    xf = UsdGeom.Xformable(kid)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp()
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(shell.GetPrim()).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    # world -> this cell's frame; the cell carries only a translate
    cxf = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(cell))
    ox, oy, oz = cxf.ExtractTranslation()
    cx = 0.5 * (mn[0] + mx[0]) - ox
    cy = 0.5 * (mn[1] + mx[1]) - oy
    z0 = mn[2] - oz
    tr.Set(Gf.Vec3d(-cx, -cy, -z0))
    return {"W": float(mx[0] - mn[0]), "D": float(mx[1] - mn[1]),
            "H": float(mx[2] - mn[2])}


def burn_kit(stage, cell, style, level, rng, nrng, mats, tag, ssf,
             flow_root, cache):
    """One kit building, assembled from pieces and put through the ladder."""
    pls = ub.build_building(style, 0.0, 0.0, 0.0, random.Random(SEED + hash(tag) % 997))
    sg.apply_placements(stage, pls, cell, ssf)
    ub.apply_glass_tint(stage, pls)
    n_st = max(1, len(qf._mass_specs(style, 0.0, 0.0, 0.0)[0]["levels"]))
    origin = max(0, min(n_st - 1, int(round(0.22 * (n_st - 1)))))
    sides = ("S",) if level in ("F1", "F2") else ("S", "E")
    return uf.burn_building(stage, cell, style, pls, 0.0, 0.0, 0.0, level,
                            rng, nrng, mats, tag, flow_root=flow_root,
                            origin=origin, sides=sides, mat_cache=cache)


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))
    _, ssf = get_stage_meters_per_unit(stage)
    t0 = time.time()

    # shapely for the shell cuts, vtk/manifold for the kit's fire_collapse
    fracture.ensure_deps()
    fracture.ensure_vtk(verbose=False)

    lib = library()
    want = [p.strip() for p in _env(
        "PB_PACKS", ",".join(n for n, _k in PACKS)).split(",") if p.strip()]
    rows = [p for p in want if lib.get(p)]
    print("[pack_bench] library: " + ", ".join(
        "{0}={1}".format(k, len(v)) for k, v in sorted(lib.items())))

    ground_and_light(stage, COLS * COL_M + 120.0, len(rows) * ROW_M + 140.0)
    flow_root = None
    if FLOW:
        from disaster import fire as fx
        fx.setup_flow_stack(stage, density_cell_size_m=0.35, max_blocks=8192,
                            scene_scale_factor=ssf)
        flow_root = fx.FLOW_ROOT
    mats = uf.materials(stage, PARENT)
    mat_cache = {}
    loose, static, vel = [], ["/World/ground"], {}
    placed = 0
    cells = []          # (name, x, y, height) for the per-building captures

    for ri, pack in enumerate(rows):
        y = (ri - (len(rows) - 1) / 2.0) * ROW_M
        # WALK THE CANDIDATES, DO NOT PRE-PICK COLS OF THEM. Height is only
        # known after the asset is on the stage, and pre-picking by name-spread
        # grabbed SM_Building_16 (312 m) and _31 (302 m) — both over PB_MAX_H —
        # so the GAC row came out with two buildings and three holes. Spread
        # the ORDER, then take the first COLS that actually fit.
        order = sorted(lib[pack], key=lambda e: e["name"])
        n = len(order)
        cand = [order[int(round(i * (n - 1) / max(1, min(COLS, n) * 3 - 1)))]
                for i in range(min(COLS, n) * 3)]
        cand = list({e["usd"]: e for e in cand}.values()) + order
        cand = list({e["usd"]: e for e in cand}.values())
        print("[pack_bench] {0:<16} {1} candidate(s)".format(pack, len(cand)))
        ci = 0
        for ent in cand:
            if ci >= COLS:
                break
            x = (ci - (COLS - 1) / 2.0) * COL_M
            tag = "{0}{1}".format(pack[:3], ci)
            cell = "{0}/{1}_{2}".format(PARENT, pack, ci)
            cxf = UsdGeom.Xform.Define(stage, Sdf.Path(cell))
            cxf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
            rng = random.Random(SEED + 31 * ri + ci)
            nrng = np.random.default_rng(SEED + 31 * ri + ci)
            tb = time.time()
            try:
                if pack == "kit":
                    style = ent["name"][4:-4]        # bld_<style>_DG0
                    lvl = LEVELS[ci % len(LEVELS)]
                    res = burn_kit(stage, cell, style, lvl, rng, nrng, mats,
                                   tag, ssf, flow_root, mat_cache)
                    loose += res["loose"]
                    static += res["static_extra"]
                    vel.update(res["velocity"])
                    label = "{0} {1}".format(style, lvl)
                else:
                    dims = place_and_measure(stage, cell, ent["usd"], ent["scale"])
                    if not dims or dims["H"] > MAX_H:
                        print("      skip {0} ({1})".format(
                            ent["name"],
                            "unmeasurable" if not dims
                            else "{0:.0f} m > PB_MAX_H".format(dims["H"])))
                        stage.RemovePrim(Sdf.Path(cell))
                        continue
                    recipe = RECIPES[ci % len(RECIPES)]
                    desc = md.Descriptor(dims["W"], dims["D"], dims["H"],
                                         construction="rc")
                    md.author(stage, cell + "/damage", desc, recipe,
                              seed=SEED + ci, source_root=cell + "/shell")
                    label = "{0} {1}".format(ent["name"], recipe)
                placed += 1
                cells.append(("{0}_{1}_{2}".format(ri, pack, label.replace(" ", "_")),
                              x, y, (dims or {}).get("H", 30.0)
                              if pack != "kit" else 30.0))
                ci += 1
                print("      {0:<44} {1:.0f} s".format(label, time.time() - tb))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("      FAILED {0}: {1}".format(ent["name"], exc))
                stage.RemovePrim(Sdf.Path(cell))
        for _ in range(3):
            omni.kit.app.get_app().update()

    for _ in range(10):
        omni.kit.app.get_app().update()
    if loose:
        settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.08,
                   rng=random.Random(SEED), bake_result=True,
                   velocity_map=vel, density=1600.0, max_speed=6.0,
                   converge=True, max_steps=int(SETTLE_STEPS * 2.2),
                   quiet_steps=50)
    for _ in range(10):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 76)
    print("PACK DAMAGE BENCH   {0} row(s) x {1} column(s), {2} building(s)"
          .format(len(rows), COLS, placed))
    for ri, pack in enumerate(rows):
        print("  row {0}: {1}".format(ri, PACK_LABEL.get(pack, pack)))
    print("  kit row  : full urban_fire ladder ({0})".format(", ".join(LEVELS)))
    print("  other rows: monolith_damage shell cuts ({0})".format(
        ", ".join(RECIPES)))
    print("  {0:.0f} s".format(time.time() - t0))
    print("=" * 76 + "\n")

    app = omni.kit.app.get_app()
    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            for _ in range(120):
                app.update()
            span_x = COLS * COL_M
            span_y = len(rows) * ROW_M
            sn.overview(stage, (0.0, 0.0), max(span_x, span_y) * 1.15,
                        os.path.join(SNAP_DIR, "grid_top.png"), ssf)
            # PER ROW, LOOKING STEEPLY DOWN. The first cut put the eye level
            # with the row and 0.52 x the row's own LENGTH behind it, which
            # aims the lens straight along +Y through every row standing
            # behind this one — all six stacked up in one frame and none of
            # them was legible. A steep look-down puts the horizon, and
            # therefore the other rows, out of frame entirely.
            for ri, pack in enumerate(rows):
                y = (ri - (len(rows) - 1) / 2.0) * ROW_M
                d, h = span_x * 0.30, span_x * 0.42
                sn.place_camera(stage, (0.0, (y - d) * ssf, h * ssf),
                                (0.0, y * ssf, 10.0 * ssf))
                sn.snapshot(os.path.join(SNAP_DIR,
                                         "row_{0}_{1}.png".format(ri, pack)))
            # AND ONE PAIR PER BUILDING. A row shot cannot show whether a
            # façade reads; that judgement is always made on a single
            # building, which is what `urban_fire_bench` captures and why its
            # output is usable and the first cut of this one was not.
            sn.views_around(stage, {c[0]: (c[1], c[2]) for c in cells},
                            SNAP_DIR, ssf,
                            top_h=max(70.0, 1.5 * max([c[3] for c in cells] or [40.0])),
                            obl_dist=58.0, obl_h=30.0)
            print("[pack_bench] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[pack_bench] snapshots FAILED: {0}".format(exc))

    if _env("KEEP_OPEN") == "1" or not _HEADLESS:
        while simulation_app.is_running():
            app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
