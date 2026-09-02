#!/usr/bin/env python
"""qc_edge_probe — did the QUAKE partial collapse tear every edge of its hole?

The quake twin of `tools/collapse_edge_probe.py`. Authors one archetype grade
on a bare USD stage — no Kit, no Flow, no physics, exactly the
`tools/kit_burn_probe.py` path — and reports, per edge class of the hole
(above / below / left / right / return), how many SURVIVING modules touch it
and how many of them `fire_collapse._tear_perimeter` actually tore. Anything
but 100 % is a straight kit seam left on the edge of the hole, which is the
2026-08-31 review's "the break-aways should be like the urban fire's".

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
        /isaac-sim/AirStack/scene_gen/tools/qc_edge_probe.py office_plain,apartment_long DG4"

Reproduces `bake_quake_archetypes_launch_script.py`'s own pose and seed
(`ARCH_SEED`, the level grid pitch) so the plan it prints is the plan that
baked archetype was made from.
"""
import os
import random
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pxr import Usd, UsdGeom                                   # noqa: E402
import scene_generator as sg                                   # noqa: E402
from detail import urban_building as ub                        # noqa: E402
from disaster import bake                                      # noqa: E402
from disaster import fire_collapse as fc                       # noqa: E402
from disaster import fracture, quake_flow as qf                # noqa: E402

SEED = int(os.environ.get("ARCH_SEED") or "4")
GRADES = ["DG0", "DG1", "DG2", "DG3", "DG4", "DG5", "SETTLE", "TILT", "OV"]


def _pose(style, level):
    spec = ub.STYLES[style]
    W, D = ub.footprint(spec)
    H = ub.height(spec)
    pitch = max(60.0, max(W, D) + 1.2 * H + 20.0)
    return GRADES.index(level) * pitch, 0.0


def run(style, level, export_dir=None):
    t0 = time.time()
    X, Y = _pose(style, level)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(stage, "/W/gen")
    parent = "/W/gen/a_{0}_{1}".format(style, level)
    UsdGeom.Scope.Define(stage, parent)
    mats = qf.materials(stage, "/W/gen")
    pls = ub.build_building(style, X, Y, 0.0, random.Random(SEED))
    sg.apply_placements(stage, pls, parent, 1.0)
    ub.apply_glass_tint(stage, pls)
    seed = SEED + (abs(hash((style, level))) % 100000)
    ctx = qf.wreck_building(stage, parent, style, pls, X, Y, 0.0, level,
                            random.Random(seed), np.random.default_rng(seed),
                            mats, "{0}_{1}".format(style, level), mat_cache={})
    print("=" * 84)
    print("[qc] {0} {1} type={2} {3:.0f}s".format(
        style, level, ctx["info"]["type"], time.time() - t0))
    for n in ctx["notes"]:
        if n.startswith("quake collapse") or n.startswith("[qc]"):
            print("   note:", n[:460])
    bad = 0
    for pl in (ctx.get("quake_collapse") or []):
        cen = fc.edge_census(pl["tears"])
        n_t = sum(1 for j in pl["tears"] if j.get("torn"))
        n_f = sum(1 for j in pl["tears"] if j.get("failed"))
        n_d = sum(1 for j in pl["tears"] if j.get("dropped"))
        n_ns = sum(1 for j in pl["tears"] if j.get("no_static"))
        print("  MASS {0} mode={1} sides={2} s0={3}: kill {4}, tears {5} "
              "(torn {6}, failed {7}, dropped {8}, no_static {9})".format(
                  pl["mass"], pl["mode"], ",".join(pl["sides"]), pl["s0"],
                  len(pl["kill"]), len(pl["tears"]), n_t, n_f, n_d, n_ns))
        for c in fc.EDGE_CLASSES:
            n, k = cen[c][0], cen[c][1]
            flag = "" if (not n or k == n) else "   <-- STRAIGHT SEAM"
            print("     {0:<7} {1}/{2}{3}".format(c, k, n, flag))
            bad += (n - k)
        for j in pl["tears"]:
            if j.get("torn") or j.get("dropped"):
                continue
            print("     UNTORN {0} {1} st{2} {3} {4}".format(
                j["name"], j["side"], j["storey"], j["classes"],
                "failed" if j.get("failed") else "skipped"))
    print("  [qc] straight seams left: {0}".format(bad))
    if export_dir:
        # THE AUTHORED BUILDING, NO SETTLE. `bake_quake_archetypes_launch_
        # script` runs PhysX between authoring and export; this path does not,
        # because a second `SimulationApp` in a container that is already
        # running a bench is not a look check, it is an outage. What the ring
        # changes is the AUTHORED tear geometry, which is fully visible
        # before anything falls.
        os.makedirs(export_dir, exist_ok=True)
        out = os.path.join(export_dir, "qc_{0}_{1}.usd".format(style, level))
        # ...AND WITHOUT WHAT FELL. `_break_split` writes the loose fragments
        # IN PLACE and they only move when the settle runs, so an unsettled
        # export shows a torn module as an intact one — the tear is there, the
        # near portion is simply still sitting in the hole it came out of.
        # Deactivating `ctx["loose"]` leaves exactly what STANDS, which is the
        # outline the review is about.
        n_off = 0
        for q in set(ctx.get("loose") or ()):
            pr = stage.GetPrimAtPath(q) if q else None
            if pr and pr.IsValid() and pr.IsActive():
                pr.SetActive(False)
                n_off += 1
        print("  [qc] deactivated {0} loose prim(s) before export".format(n_off))
        paths = [parent]
        m0 = ctx["info"]["masses"]["main"]
        ok = bake.export_object(stage, None, paths, out,
                                recenter=(float(m0["cx"]), float(m0["cy"]),
                                          0.0), merge=False)
        print("  [qc] exported {0} -> {1}".format(bool(ok), out))
    return ctx


def main():
    args = sys.argv[1:]
    export_dir = None
    if "--export" in args:
        i = args.index("--export")
        export_dir = args[i + 1]
        del args[i:i + 2]
    styles = (args[0].split(",") if args else ["office_plain", "apartment_long"])
    level = args[1] if len(args) > 1 else "DG4"
    fracture.ensure_vtk(verbose=True)
    for s in styles:
        try:
            run(s.strip(), level, export_dir=export_dir)
        except Exception:                                      # pragma: no cover
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()
