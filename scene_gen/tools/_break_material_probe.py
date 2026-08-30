#!/usr/bin/env python
"""_break_material_probe — WHAT IS BOUND TO THE PIECES A COLLAPSE BREAKS, and
whether the fit-out inside the hole actually falls.

Two questions, one build, bare USD in the container (no Kit, no Flow, no
physics — same shape as `kit_burn_probe.py` / `gac_burn_probe.py`):

1. **The subset census.** `quake_flow._break` / `_break_split` bind a
   fragment's OWN CLADDING at prim level (on the GAC path that is the sooted
   atlas `gac_fire.rebind_sooted` already put on the piece) and put every
   face the pipeline invented — the cut faces, the back, the reveals — into a
   `materialBind` GeomSubset named `core` (`quake_flow._t_core_bind`). So a
   broken piece is ALREADY two materials, and the only question is which of
   them the fire palette was written over. `fire_collapse.bind_break` now
   writes the char onto the `core` subset alone for a piece that STAYS,
   because charring the whole prim takes the façade off the outward faces and
   the piece boundary reads as a hard dark rectangle beside its intact
   neighbour ("the material of the broken/debris part is a much darker colour
   than the intact façade next to it", user, second-row review 2026-08-30).
   This prints, per fragment, whether it is loose or static, whether it has a
   `core` subset, and what the prim and the subset each bind — bucketed into
   FAÇADE / CHAR / other.

2. **The fit-out in the hole.** `quake_flow.fit_interior`'s partitions,
   columns and props are not in `static_extra` while the recipes run
   (`burn_building` folds `fit["all"]` in once, at the end), so
   `r_partial_collapse`'s position sweep cannot see them and a partition on a
   killed storey inside the lost region was left standing in mid-air
   (`part_main_6_1` / `part_main_6_0`, commercial_mid F5c). Step 4b of the
   recipe now sends them down. This prints every `part_`/`col_`/`prop_` prim
   with its storey, its world z, whether the plan's region covers it and
   whether it ended up in `ctx["loose"]`.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/_break_material_probe.py kit:commercial_mid:F5c:S,E"
    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/_break_material_probe.py gac:SM_Building_05:F5c"

Entries are `kind:name:level[:sides]`, exactly the shape `fire_bake.sh` uses.
"""
import random
import sys
import time
import traceback
from collections import Counter

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                        # noqa: E402
import scene_generator as sg                                  # noqa: E402
from detail import urban_building as ub                       # noqa: E402
from disaster import fracture, quake_flow as qf               # noqa: E402
from disaster import fire_collapse as fc, urban_fire as uf    # noqa: E402


# ---------------------------------------------------------------------------
# What KIND of material is this
# ---------------------------------------------------------------------------
# `damage.char_materials` authors under `<parent>/BurnLooks*`; the flat fire
# tones live in `urban_fire.materials()`'s own `FireLooks` scope but are
# NAMED, so the name is the reliable test for those.
_CHAR_NAMES = ("char_concrete", "soot", "soot_mid", "soot_light", "calcined",
               "burnt_metal", "void", "dark_concrete", "ash")
_FACADE_HINTS = ("/QuakeLooks/clad_", "/SootLooks/", "/src/", "/Looks/",
                 "/Materials/")


def _mat_of(prim):
    m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if not m or not m.GetPrim().IsValid():
        return None, "(unbound)", "unbound"
    p = m.GetPrim()
    path, name = str(p.GetPath()), p.GetName()
    if "/BurnLooks" in path:
        kind = "char"
    elif name in _CHAR_NAMES or name.startswith(("Char_", "Scorch_", "Ash_")):
        kind = "char"
    elif any(h in path for h in _FACADE_HINTS):
        kind = "facade"
    else:
        kind = "other"
    return path, name, kind


def _region_side(plan, m, lx, ly, pad=1.0):
    """`fire_collapse.region_side` when it exists (it is the shipped test),
    else the same arithmetic locally — so this probe still runs against a
    PRISTINE copy of the tree for a before/after comparison."""
    f = getattr(fc, "region_side", None)
    if f is not None:
        return f(plan, m, lx, ly, pad=pad)
    for sd, (lo, hi, dep) in plan["region"].items():
        t = fc.along_of(m, sd, lx, ly)
        d_in = -fc.outward_of(m, sd, lx, ly)
        if lo - pad <= t <= hi + pad and -1.5 <= d_in <= dep + pad:
            return sd
    return None


def _core_subset(prim):
    for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
        if s.GetPrim().GetName() == "core":
            return s.GetPrim()
    return None


def _census(stage, cell, ctx):
    loose = set(ctx.get("loose") or ())
    static = set(ctx.get("static_extra") or ())
    rows = Counter()
    examples = {}
    n_frag = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(cell)):
        if not p.IsA(UsdGeom.Mesh):
            continue
        path = str(p.GetPath())
        if "/brk_" not in path and "/lidbrk_" not in path:
            continue
        n_frag += 1
        where = ("loose" if path in loose else
                 "static" if path in static else "unlisted")
        core = _core_subset(p)
        _pp, pname, pkind = _mat_of(p)
        if core is None:
            key = (where, "no-core", pkind, "-")
        else:
            _cp, cname, ckind = _mat_of(core)
            key = (where, "core", pkind, ckind)
        rows[key] += 1
        examples.setdefault(key, (path, pname,
                                  _mat_of(core)[1] if core is not None else "-"))
    print("[break_probe] BREAK-FRAGMENT SUBSET CENSUS ({0} fragment prim(s))"
          .format(n_frag))
    print("    {0:<9} {1:<8} {2:<8} {3:<8} {4:>6}   example prim / prim-mat / core-mat"
          .format("where", "subsets", "prim", "core", "n"))
    for key in sorted(rows, key=lambda k: (-rows[k], k)):
        where, has, pkind, ckind = key
        path, pname, cname = examples[key]
        print("    {0:<9} {1:<8} {2:<8} {3:<8} {4:>6}   {5}  [{6} | {7}]".format(
            where, has, pkind, ckind, rows[key],
            path.rsplit("/", 2)[-1], pname, cname))
    # The one line that answers the review: a STATIC fragment must keep a
    # façade material on the prim and carry the char on its core subset.
    good = sum(n for k, n in rows.items()
               if k[0] == "static" and k[1] == "core" and k[2] == "facade"
               and k[3] == "char")
    bad = sum(n for k, n in rows.items() if k[0] == "static" and k[2] == "char")
    tot = sum(n for k, n in rows.items() if k[0] == "static")
    print("[break_probe] STATIC fragments: {0}/{1} keep a FAcADE material on "
          "the prim with the char on the cut faces; {2} are charred WHOLE "
          "(prim-level char) — that number is the rectangle the review saw."
          .format(good, tot, bad))
    return rows


def _fit_report(stage, cell, ctx):
    plan = ctx.get("partial_collapse")
    if not plan:
        print("[break_probe] no partial-collapse plan on the ctx — skipping "
              "the fit-out report")
        return
    m = ctx["info"]["masses"].get(plan["mass"]) or ctx["info"]["masses"]["main"]
    loose = set(ctx.get("loose") or ())
    static = set(ctx.get("static_extra") or ())
    killed = set(int(s) for s in plan["storeys"])
    xf = UsdGeom.XformCache()
    print("[break_probe] FIT-OUT vs the hole: mass {0}, killed storeys {1}, "
          "sides {2}, region {3}".format(
              plan["mass"], sorted(killed), "/".join(plan["sides"]),
              {k: tuple(round(q, 1) for q in v)
               for k, v in plan["region"].items()}))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    fit = ctx.get("fit") or {}
    # FROM THE CTX'S OWN LISTS, not a stage walk: `Usd.PrimRange` skips
    # DEACTIVATED prims, and `r_gut_interior` deactivates a share of the
    # partitions and props — a walk therefore cannot tell "gutted" from
    # "never authored", and the two prims the review named would simply be
    # absent from the table.
    rows = []
    for p in (fit.get("partitions") or ()):
        bits = str(p).rsplit("/", 1)[-1].split("_")
        try:
            rows.append((p, "part", int(bits[-2])))
        except (ValueError, IndexError):
            rows.append((p, "part", None))
    for (mt_, s_), cols in (fit.get("columns") or {}).items():
        for p in (cols or ()):
            rows.append((p, "col", int(s_)))
    for (mt_, s_), props in (fit.get("props") or {}).items():
        for p in (props or ()):
            rows.append((p, "prop", int(s_)))
    seen = Counter()
    for path, kind, storey in rows:
        p = stage.GetPrimAtPath(path)
        if not p or not p.IsValid():
            continue
        try:
            t = xf.GetLocalToWorldTransform(p).ExtractTranslation()
        except Exception:
            continue
        # THE SAME FOOTPRINT TEST THE RECIPE USES (step 4b): centre plus the
        # four plan corners of the world bbox. A partition frames INTO the
        # lost wall, so its end is in the region and its centre is not.
        pts = [(float(t[0]), float(t[1]))]
        try:
            bx = bc.ComputeWorldBound(p).ComputeAlignedRange()
            if not bx.IsEmpty():
                a, b = bx.GetMin(), bx.GetMax()
                pts += [(float(a[0]), float(a[1])), (float(b[0]), float(a[1])),
                        (float(b[0]), float(b[1])), (float(a[0]), float(b[1]))]
        except Exception:
            pass
        sd = None
        for wx, wy in pts:
            lx, ly = qf._to_local(m, wx, wy)
            sd = _region_side(plan, m, lx, ly)
            if sd is not None:
                break
        state = ("loose" if path in loose else
                 "static" if path in static else "unlisted")
        active = p.IsActive()
        should = (sd is not None and storey in killed)
        seen[(kind, should, state, active)] += 1
        if should or kind == "part":
            print("    {0:<40} storey {1!s:<4} z {2:7.2f}  region {3!s:<5} "
                  "-> {4}{5}".format(path.rsplit("/", 1)[-1], storey,
                                     float(t[2]), sd or "-", state,
                                     "" if active else " (deactivated)"))
    fitall = list(fit.get("all") or ())
    print("[break_probe] FIT-OUT SIZE: {0} storey(s) fitted out, {1} prim(s) "
          "in fit['all'] ({2} slab, {3} column, {4} partition, {5} prop)"
          .format(len({s for _p, _k, s in rows if s is not None}),
                  len(fitall), len(fit.get("slabs") or {}),
                  sum(len(v or ()) for v in (fit.get("columns") or {}).values()),
                  len(fit.get("partitions") or ()),
                  sum(len(v or ()) for v in (fit.get("props") or {}).values())))
    print("[break_probe] fit-out summary (kind, in-the-hole, list, active):")
    for k in sorted(seen):
        print("    {0!s:<40} {1}".format(k, seen[k]))
    wrong = sum(n for k, n in seen.items()
                if k[1] and k[2] != "loose" and k[3])
    print("[break_probe] {0} ACTIVE fit-out prim(s) inside the lost region on "
          "a killed storey are NOT in ctx['loose'] — expected 0.".format(wrong))


def run_kit(style, level, sides, seed):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(stage, "/W/bench")
    cell = "/W/bench/k0"
    UsdGeom.Xform.Define(stage, cell)
    mats = uf.materials(stage, "/W/bench")
    pls = ub.build_building(style, 0.0, 0.0, 0.0, random.Random(seed + 7))
    sg.apply_placements(stage, pls, cell + "/parts", 1.0)
    ub.apply_glass_tint(stage, pls)
    specs = qf._mass_specs(style, 0.0, 0.0, 0.0)
    main_spec = max(specs, key=lambda q: len(q["levels"]))
    n_st = max(1, len(main_spec["levels"]))
    origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
    ctx = uf.burn_building(stage, cell, style, pls, 0.0, 0.0, 0.0, level,
                           random.Random(seed), np.random.default_rng(seed),
                           mats, "k0", flow_root=None, origin=origin,
                           sides=sides, mat_cache={})
    return stage, cell, ctx


def run_gac(name, level, sides, seed):
    from disaster import gac_fire as gf
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(stage, "/W/bench")
    cell = "/W/bench/g0"
    UsdGeom.Xform.Define(stage, cell)
    mats = uf.materials(stage, "/W/bench")
    ctx = gf.burn_gac(stage, cell, name, level, random.Random(seed),
                      np.random.default_rng(seed), mats, "g0",
                      flow_root=None, mat_cache={}, ssf=1.0, sides=sides,
                      verbose=True)
    return stage, cell, ctx


def main():
    spec = sys.argv[1] if len(sys.argv) > 1 else "kit:commercial_mid:F5c:S,E"
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 7
    bits = spec.split(":")
    kind = bits[0]
    name = bits[1] if len(bits) > 1 else "commercial_mid"
    level = bits[2] if len(bits) > 2 else "F5c"
    sides = None
    if len(bits) > 3 and bits[3].strip():
        sides = tuple(q.strip().upper()[:1]
                      for q in bits[3].replace("/", ",").split(",") if q.strip())
    fracture.ensure_deps(verbose=False)
    fracture.ensure_vtk(verbose=False)
    t0 = time.time()
    print("=" * 78)
    print("[break_probe] {0} seed={1}".format(spec, seed))
    try:
        stage, cell, ctx = (run_gac(name, level, sides, seed) if kind == "gac"
                            else run_kit(name, level, sides, seed))
    except Exception:
        traceback.print_exc()
        return 1
    for n in ctx["notes"]:
        print("   note:", n[:240])
    print("[break_probe] loose {0}, static {1}, authored {2}; {3:.0f}s".format(
        len(ctx["loose"]), len(ctx["static_extra"]), len(ctx["authored"]),
        time.time() - t0))
    _census(stage, cell, ctx)
    _fit_report(stage, cell, ctx)
    return 0


if __name__ == "__main__":
    sys.exit(main())
