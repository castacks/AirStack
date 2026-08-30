#!/usr/bin/env python
"""kit_burn_probe — `disaster.urban_fire.burn_building` end to end on a bare
USD stage (no Kit, no Flow, no physics): place a ModernCityEnvironment KIT
building, fit its interior, run the whole fire ladder on it. This is exactly
the `GF_EXTRA_KIT=style:level[:sides]` path
`simulation/isaac-sim/launch_scripts/gac_fire_bench_launch_script.py` takes
for its extra KIT columns (build_building -> apply_placements ->
apply_glass_tint -> burn_building, see that file's `main()` around
line 347), mirrored call-for-call here so a launch is not the first time the
chain runs on a kit building.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/kit_burn_probe.py apartment F5c"

With NO arguments, runs BOTH of `gac_fire_bench_launch_script.py`'s default
`GF_EXTRA_KIT` columns in sequence, each on a fresh in-memory stage.
`GF_EXTRA_KIT` itself defaults to `""` (no extra kit columns on that bench),
so the two run here are one `urm` style and one `rc` style
(`quake_flow.FAMILY_TYPE`) at F5c — `uf.LADDER`'s partial-collapse level —
picked from `urban_building.STYLES`: `apartment` (family "01", urm) and
`office` (family "02", rc).
"""
import random
import resource
import sys
import time
import traceback
from collections import Counter

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                        # noqa: E402
import scene_generator as sg                                  # noqa: E402
from detail import urban_building as ub                       # noqa: E402
from disaster import fracture, quake_flow as qf, urban_fire as uf  # noqa: E402

# The two F5c kit columns run with no arguments: one urm style, one rc style
# (see module docstring). `sides=None` in both cases — `plan_fire` draws its
# own venting elevations, exactly as an un-pinned `GF_EXTRA_KIT` entry would.
DEFAULTS = [("apartment", "F5c", None), ("office", "F5c", None)]


def _sides(spec):
    """`"S"` / `"S,E"` / `"S/E"` -> a tuple of elevation letters, or None —
    the same shape `GF_EXTRA_KIT`'s third field and `UF_SIDES` both parse to."""
    if not spec:
        return None
    parts = spec.replace("/", ",").split(",")
    out = tuple(q.strip().upper()[:1] for q in parts if q.strip())
    return out or None


def _bound_name(prim):
    """The NAME of the material bound to `prim`, or "(unbound)"."""
    m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    return m.GetPrim().GetName() if m else "(unbound)"


def _material_census(stage, cell):
    census = Counter()
    for p in Usd.PrimRange(stage.GetPrimAtPath(cell)):
        if not p.IsA(UsdGeom.Mesh):
            continue
        census[_bound_name(p)] += 1
        for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(p)):
            census[_bound_name(s.GetPrim())] += 1
    print("[kit_probe] MATERIAL CENSUS:")
    for name, n in sorted(census.items()):
        print("    {0:<28} {1}".format(name, n))
    return census


def _col_storey(name):
    """`col_<mass>_<storey>_<a>_<b>` -> storey (int) or None.

    Mirrors `urban_fire._storey_of_path`'s `part_<mass>_<storey>_<k>`
    parse, but a column name carries TWO trailing grid indices (`a`, `b`,
    `quake_flow.fit_interior`'s `cpath = ".../col_{mtag}_{i}_{a}_{b}"`), so
    the storey is the THIRD-from-last `_` field, not the second.
    """
    bits = name.split("_")
    if len(bits) < 5 or not all(b.isdigit() for b in bits[-3:]):
        return None
    return int(bits[-3])


def _is_box_mesh(prim):
    """A `quake_flow._box` mesh is a closed box: 6 quad faces, 8 points. A
    `_cyl` (`sides=7` default) has more of both."""
    mesh = UsdGeom.Mesh(prim)
    fc = mesh.GetFaceVertexCountsAttr().Get()
    pts = mesh.GetPointsAttr().Get()
    return fc is not None and pts is not None and len(fc) == 6 and len(pts) == 8


def _flags(ctx, stage, cell):
    info = ctx["info"]
    fire = ctx.get("fire") or {}

    # -- rebar-tone ----------------------------------------------------
    n_rebar = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(cell)):
        if not p.IsA(UsdGeom.Mesh):
            continue
        if p.GetName().startswith(("rebar", "sbar")) and _bound_name(p) == "rebar":
            n_rebar += 1
    print("FLAG rebar-tone: {0} prim(s) named rebar_*/sbar_* still bound to "
          "the quake 'rebar' material (expected 0 after today's change — "
          "they must be on steel)".format(n_rebar))

    # -- pale-columns ----------------------------------------------------
    touched = set(fire.get("storeys") or ())
    n_pale = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(cell)):
        if not p.IsA(UsdGeom.Mesh) or not p.GetName().startswith("col_"):
            continue
        storey = _col_storey(p.GetName())
        if storey is None:
            continue
        sev = uf._severity(ctx, storey)
        if sev >= 0.25 and _bound_name(p) == "concrete":
            n_pale += 1
    print("FLAG pale-columns: {0} fit column prim(s) (name starts with "
          "col_) bound to the quake 'concrete' material on storeys the "
          "fire touched (severity >= 0.25); touched storeys = {1}".format(
              n_pale, sorted(touched)))

    # -- cyl-rods ----------------------------------------------------
    n_cyl = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(cell)):
        if not p.IsA(UsdGeom.Mesh) or not p.GetName().startswith(("joist_", "rafter_")):
            continue
        if not _is_box_mesh(p):
            n_cyl += 1
    print("FLAG cyl-rods: {0} prim(s) named joist_*/rafter_* that are NOT "
          "boxes (a box is a Mesh with 6 faces / 8 points; a _cyl has "
          "more) — expected 0.".format(n_cyl))

    # -- floating-joists ----------------------------------------------------
    all_zs = sorted({z for m in (info.get("masses") or {}).values()
                     for z in (m.get("levels") or [])})
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    n_float = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(cell)):
        if not p.IsA(UsdGeom.Mesh) or not p.GetName().startswith("joist_"):
            continue
        rng_box = cache.ComputeWorldBound(p).ComputeAlignedRange()
        if rng_box.IsEmpty():
            continue
        zmin = rng_box.GetMin()[2]
        if not any(abs(zmin - z) <= 0.05 for z in all_zs):
            n_float += 1
    print("FLAG floating-joists: {0} joist_* prim(s) whose world-space "
          "bottom is more than 0.05 m from any floor slab z (levels = "
          "{1})".format(n_float, ["{0:.2f}".format(z) for z in all_zs]))


def run_one(style, level, sides, seed):
    t0 = time.time()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(stage, "/W/bench")
    cell = "/W/bench/k0"
    UsdGeom.Xform.Define(stage, cell)
    mats = uf.materials(stage, "/W/bench")
    rng = random.Random(seed)
    nrng = np.random.default_rng(seed)

    # -- mirror gac_fire_bench_launch_script.py's GF_EXTRA_KIT column build
    # exactly (simulation/isaac-sim/launch_scripts/gac_fire_bench_launch_
    # script.py:347-359): build the placed kit, apply it + its glass tint,
    # work out the same low-biased origin from the main mass's own storey
    # count, then burn it with `urban_fire.burn_building`.
    pls = ub.build_building(style, 0.0, 0.0, 0.0, random.Random(seed + 7))
    sg.apply_placements(stage, pls, cell + "/parts", 1.0)
    ub.apply_glass_tint(stage, pls)
    specs = qf._mass_specs(style, 0.0, 0.0, 0.0)
    main_spec = max(specs, key=lambda m: len(m["levels"]))
    n_st = max(1, len(main_spec["levels"]))
    origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
    ctx = uf.burn_building(stage, cell, style, pls, 0.0, 0.0, 0.0, level,
                           rng, nrng, mats, "k0", flow_root=None,
                           origin=origin, sides=sides, mat_cache={})

    for n in ctx["notes"]:
        print("   note:", n[:220])
    print("[kit_probe] {0} {1} sides={2}: loose {3}, static {4}, "
          "authored {5}; {6:.0f}s".format(
              style, level, ",".join(ctx["fire"].get("sides") or ()),
              len(ctx["loose"]), len(ctx["static_extra"]),
              len(ctx["authored"]), time.time() - t0))

    _material_census(stage, cell)
    _flags(ctx, stage, cell)
    return ctx


def main():
    args = sys.argv[1:]
    seed = 7
    if "--seed" in args:
        i = args.index("--seed")
        seed = int(args[i + 1])
        del args[i:i + 2]

    if not args:
        jobs = [(s, lv, sd, seed + 31 * i)
                for i, (s, lv, sd) in enumerate(DEFAULTS)]
    else:
        style = args[0]
        level = args[1] if len(args) > 1 else "F5c"
        sides = _sides(args[2]) if len(args) > 2 else None
        jobs = [(style, level, sides, seed)]

    fracture.ensure_deps(verbose=False)
    fracture.ensure_vtk(verbose=False)

    t0 = time.time()
    for style, level, sides, sd in jobs:
        print("=" * 78)
        print("[kit_probe] {0} {1} seed={2}".format(style, level, sd))
        run_one(style, level, sides, sd)
    print("=" * 78)
    rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    print("PEAK_RSS_MB {0:.0f} WALL_S {1:.0f}".format(rss_mb, time.time() - t0))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
